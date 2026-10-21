import cv2
import numpy as np


# 기존 ver2 설정 유지
HOLD_SECONDS = 0.20
WIDTH_LEARN_FRAMES = 3
SMOOTH_ALPHA = 0.35


def lane_edges(frame):
    """전체 Canny 영상과 차선 검출용 에지를 반환합니다."""
    height, width = frame.shape[:2]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    raw_edges = cv2.Canny(blurred, 50, 150)

    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    white = cv2.inRange(
        hls,
        (0, 150, 0),
        (180, 255, 150),
    )

    yellow = cv2.inRange(
        hsv,
        (15, 65, 70),
        (40, 255, 255),
    )

    colors = cv2.dilate(
        white | yellow,
        np.ones((5, 5), np.uint8),
    )

    edges = cv2.bitwise_and(raw_edges, colors)
    mask = np.zeros_like(edges)

    vertices = np.array([[
        (int(width * 0.05), int(height * 0.96)),
        (int(width * 0.40), int(height * 0.66)),
        (int(width * 0.60), int(height * 0.66)),
        (int(width * 0.95), int(height * 0.96)),
    ]], dtype=np.int32)

    cv2.fillPoly(mask, vertices, 255)
    lane_edge_image = cv2.bitwise_and(edges, mask)

    return raw_edges, lane_edge_image


class LaneTracker:
    def __init__(self):
        self.shape = None
        self.reset()

    def reset(self):
        self.pair = None
        self.last_seen = None
        self.lane_width = None
        self.width_count = 0

    def candidates(self, lines, height, width):
        """허프 선분들을 묶어서 좌우 차선 후보를 만듭니다."""
        groups = [[], []]

        if lines is None:
            return groups

        y_top = height * 0.76
        y_bottom = height * 0.94
        segments = []

        for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
            dx = float(x2 - x1)
            dy = float(y2 - y1)

            if abs(dy) < 6:
                continue

            a = dx / dy

            if abs(a) > 3:
                continue

            b = x1 - a * y1

            ends = np.array([
                a * y_top + b,
                a * y_bottom + b,
            ])

            if not -width * 0.10 < ends[1] < width * 1.10:
                continue

            if self.pair is None:
                side = 0 if ends[1] < width * 0.5 else 1

            else:
                distances = np.max(
                    np.abs(self.pair - ends),
                    axis=1,
                )

                side = int(np.argmin(distances))

                if distances[side] > width * 0.13:
                    continue

                if abs(distances[0] - distances[1]) < width * 0.02:
                    continue

            length = min(np.hypot(dx, dy), 80)

            segments.append((
                side,
                ends,
                [(x1, y1), (x2, y2)],
                length,
            ))

        for side in (0, 1):
            pool = [
                segment
                for segment in segments
                if segment[0] == side
            ]

            for seed in pool:
                support = [
                    segment
                    for segment in pool
                    if np.max(
                        np.abs(segment[1] - seed[1])
                    ) < width * 0.045
                ]

                points = np.array([
                    point
                    for segment in support
                    for point in segment[2]
                ], dtype=float)

                if len(points) < 2:
                    continue

                if np.ptp(points[:, 1]) < height * 0.035:
                    continue

                weights = np.repeat([
                    np.sqrt(segment[3])
                    for segment in support
                ], 2)

                a, b = np.polyfit(
                    points[:, 1],
                    points[:, 0],
                    1,
                    w=weights,
                )

                ends = np.array([
                    a * y_top + b,
                    a * y_bottom + b,
                ])

                score = sum(
                    segment[3]
                    for segment in support
                )

                if self.pair is not None:
                    delta = np.max(
                        np.abs(ends - self.pair[side])
                    )

                    if delta > width * 0.13:
                        continue

                    score -= delta * 2

                duplicate = any(
                    np.max(np.abs(ends - old[1])) < 10
                    for old in groups[side]
                )

                if not duplicate:
                    groups[side].append((score, ends))

            groups[side].sort(
                key=lambda item: item[0],
                reverse=True,
            )

            groups[side] = groups[side][:6]

        return groups

    def accept(self, measured, now):
        """검출 또는 추정 결과를 평활화해서 반영합니다."""
        if self.pair is None:
            self.pair = measured.copy()
        else:
            self.pair = (
                (1 - SMOOTH_ALPHA) * self.pair
                + SMOOTH_ALPHA * measured
            )

        self.last_seen = now

    def update(self, edges, now):
        height, width = edges.shape

        if self.shape != edges.shape:
            self.reset()
            self.shape = edges.shape

        if (
            self.last_seen is not None
            and now - self.last_seen > HOLD_SECONDS
        ):
            self.reset()

        # 기존 허프변환 설정 유지
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180,
            threshold=12,
            minLineLength=10,
            maxLineGap=20,
        )

        left_candidates, right_candidates = self.candidates(
            lines,
            height,
            width,
        )

        choices = []

        for left_score, left in left_candidates:
            for right_score, right in right_candidates:
                widths = right - left

                if not width * 0.12 < widths[1] < width * 0.95:
                    continue

                if not (
                    -width * 0.03
                    < widths[0]
                    < widths[1] * 0.85
                ):
                    continue

                if self.pair is None:
                    if not left[1] < width * 0.5 < right[1]:
                        continue

                if not (
                    -width * 0.15 < left.min()
                    and right.max() < width * 1.15
                ):
                    continue

                score = (
                    min(left_score, 100)
                    + min(right_score, 100)
                    - widths[1] * 0.8
                )

                choices.append((
                    score,
                    np.array([left, right]),
                ))

        # 양쪽 차선 실제 검출
        if choices:
            measured = max(
                choices,
                key=lambda item: item[0],
            )[1]

            measured_width = measured[1] - measured[0]

            # 실제 양쪽 검출 결과로만 폭 학습
            if np.all(measured_width > 0):
                if self.lane_width is None:
                    self.lane_width = measured_width.copy()
                else:
                    self.lane_width = (
                        0.9 * self.lane_width
                        + 0.1 * measured_width
                    )

                self.width_count += 1
            else:
                self.lane_width = None
                self.width_count = 0

            self.accept(measured, now)

            return self.pair.copy(), "TRACKING"

        ready = (
            self.pair is not None
            and self.lane_width is not None
            and self.width_count >= WIDTH_LEARN_FRAMES
        )

        only_one_side = (
            bool(left_candidates) != bool(right_candidates)
        )

        # 한쪽 검출 시 기억한 차선 폭으로 반대쪽 추정
        if ready and only_one_side:
            if left_candidates:
                left = left_candidates[0][1]

                measured = np.array([
                    left,
                    left + self.lane_width,
                ])

                status = "LEFT_ONLY"

            else:
                right = right_candidates[0][1]

                measured = np.array([
                    right - self.lane_width,
                    right,
                ])

                status = "RIGHT_ONLY"

            movement = np.max(
                np.abs(measured - self.pair)
            )

            if movement <= width * 0.13:
                self.accept(measured, now)

                return self.pair.copy(), status

        if self.width_count < WIDTH_LEARN_FRAMES:
            self.width_count = 0
            self.lane_width = None

        if self.pair is not None:
            return self.pair.copy(), "HELD"

        return None, "LOST"


def draw_result(frame, pair, status):
    """결과 이미지와 중심 오차를 반환하며 창은 띄우지 않습니다."""
    height, width = frame.shape[:2]
    camera_x = width // 2

    cv2.line(
        frame,
        (camera_x, int(height * 0.60)),
        (camera_x, height - 1),
        (100, 100, 100),
        1,
    )

    error = 0.0

    is_detected = status in (
        "TRACKING",
        "LEFT_ONLY",
        "RIGHT_ONLY",
    )

    if pair is not None:
        y_top = height * 0.76
        y_bottom = height * 0.94

        bottoms = (
            pair[:, 1]
            + (pair[:, 1] - pair[:, 0])
            * (height - 1 - y_bottom)
            / (y_bottom - y_top)
        )

        for side, (points, bottom) in enumerate(zip(pair, bottoms)):
            inferred = (
                (status == "LEFT_ONLY" and side == 1)
                or (status == "RIGHT_ONLY" and side == 0)
            )

            color = (
                (0, 165, 255)
                if inferred
                else (0, 255, 0)
            )

            if status == "HELD":
                color = (150, 150, 150)

            cv2.line(
                frame,
                (int(points[0]), int(y_top)),
                (int(bottom), height - 1),
                color,
                2,
                cv2.LINE_AA,
            )

        center_top = float(np.mean(pair[:, 0]))
        center_bottom = float(np.mean(bottoms))

        cv2.line(
            frame,
            (int(center_top), int(y_top)),
            (int(center_bottom), height - 1),
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        error = camera_x - center_bottom
        marker_y = height - 30

        cv2.line(
            frame,
            (camera_x, marker_y),
            (int(center_bottom), marker_y),
            (0, 0, 255),
            1,
        )

        cv2.circle(
            frame,
            (int(center_bottom), marker_y),
            5,
            (0, 255, 0),
            -1,
        )

        error_text = f"Error: {error:.1f} px"

        if status == "HELD":
            error_text += " (previous)"

    else:
        error_text = "Error: unavailable"

    cv2.circle(
        frame,
        (camera_x, height - 30),
        5,
        (255, 0, 0),
        -1,
    )

    status_color = (
        (0, 255, 0)
        if status == "TRACKING"
        else (0, 165, 255)
    )

    cv2.putText(
        frame,
        status,
        (15, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        status_color,
        2,
    )

    cv2.putText(
        frame,
        error_text,
        (15, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        1,
    )

    return frame, error, is_detected

# ============================================================
# main_integrated.py 기존 인터페이스 호환 함수
# main_integrated.py 변경 없이 새 차선 추적 알고리즘 사용
# ============================================================

import time

# 프레임 사이의 차선 정보와 학습한 폭을 계속 유지
_compat_tracker = LaneTracker()


def region_of_interest(img, vertices):
    """기존 main_integrated.py 호출을 위한 ROI 함수."""
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(img, mask)


def calculate_steering(frame, lines):
    """기존 API: (error, result_image, is_detected) 반환.

    새 추적기는 원본 컬러 영상에서 자체 색상 필터와 허프 변환을
    수행하므로, 기존 main의 ROI/Hough 결과 lines는 사용하지 않는다.
    HELD/LOST 시 is_detected=False를 유지해 기존 제어부가 정지한다.
    """
    _raw_edges, filtered_edges = lane_edges(frame)
    pair, status = _compat_tracker.update(filtered_edges, time.monotonic())
    result_image, error, is_detected = draw_result(frame, pair, status)
    return error, result_image, is_detected
