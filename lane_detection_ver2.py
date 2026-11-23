import cv2
import numpy as np


# 기존 ver2 설정 유지 (반응 지연 축소를 위해 SMOOTH_ALPHA만 0.35 -> 0.50 조정)
HOLD_SECONDS = 0.20
WIDTH_LEARN_FRAMES = 3
SMOOTH_ALPHA = 0.50

# 조향 오차를 계산할 전방 주시 높이 비율 (0.55 ~ 0.65 사이에서 조정)
LOOKAHEAD_Y_RATIO = 0.50

# 한쪽 차선 후보가 받을 수 있는 최대 색상 가산점
COLOR_BONUS = 40.0

# 차선으로 인정할 선분 주변 최소 색상(흰색/노란색) 비율 (0.0 ~ 1.0)
# 이 값 미만인 선분(도로 이음새 등)은 차선 후보에서 제외합니다.
MIN_COLOR_RATIO = 0.25

# 화면 아래로 연장한 x좌표의 허용 범위: 화면 폭 기준 -50% ~ 150%.
LANE_X_MIN_RATIO = -0.50
LANE_X_MAX_RATIO = 1.50
# 양쪽 차선 간격의 최소/최대 허용값 (화면 폭 비율, 검정 도로 실측 폭 ~1.34 기준)
MIN_LANE_WIDTH_RATIO = 0.80
MAX_LANE_WIDTH_RATIO = 2.00
# 학습된 차선 폭(self.lane_width) 대비 허용 오차 비율 (±25%)
LEARNED_WIDTH_TOLERANCE = 0.25


def lane_edges(frame):
    """전체 Canny 영상과 차선 검출용 에지를 반환합니다."""
    height, width = frame.shape[:2]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    raw_edges = cv2.Canny(blurred, 50, 150)

    mask = np.zeros_like(raw_edges)

    vertices = np.array([[
        (int(width * 0.01), int(height * 0.99)),  # 왼쪽 아래
        (int(width * 0.01), int(height * 0.20)),  # 왼쪽 위
        (int(width * 0.99), int(height * 0.20)),  # 오른쪽 위
        (int(width * 0.99), int(height * 0.99)),  # 오른쪽 아래
    ]], dtype=np.int32)

    cv2.fillPoly(mask, vertices, 255)
    lane_edge_image = cv2.bitwise_and(raw_edges, mask)

    return raw_edges, lane_edge_image


def _tape_center_segment(x1, y1, x2, y2, tape_mask):
    """경계 근처의 제한된 색상 띠를 찾아 테이프 중심 선분으로 변환합니다.

    두 Canny 경계가 모두 검출될 필요는 없습니다. 색상 띠의 양 끝이
    확인되지 않거나 너무 넓으면 원래 선분을 그대로 사용합니다.
    """
    original = (float(x1), float(y1), float(x2), float(y2))
    if tape_mask is None or abs(y2 - y1) < 6:
        return original
    height, width = tape_mask.shape
    max_tape_width = max(6, int(width * 0.10))
    samples = []
    for y in np.unique(np.rint(np.linspace(y1, y2, 16)).astype(int)):
        if not 0 <= y < height:
            continue
        x = int(round(x1 + (x2 - x1) * (y - y1) / (y2 - y1)))
        if not 0 <= x < width:
            continue
        row = tape_mask[y]
        lo, hi = max(0, x - 3), min(width, x + 4)
        hits = np.flatnonzero(row[lo:hi]) + lo
        if not len(hits):
            continue
        seed = int(hits[np.argmin(np.abs(hits - x))])
        left = right = seed
        while left > 0 and row[left - 1] and seed - left <= max_tape_width:
            left -= 1
        while right < width - 1 and row[right + 1] and right - seed <= max_tape_width:
            right += 1
        tape_width = right - left + 1
        if left == 0 or right == width - 1:
            continue
        if not 2 <= tape_width <= max_tape_width:
            continue
        # 색상 영역 내부의 바닥 이음새 등을 중심선으로 바꾸지 않습니다.
        if min(abs(x - left), abs(x - right)) > 4:
            continue
        samples.append((float(y), (left + right) * 0.5))
    if len(samples) < 8:
        return original
    samples = np.asarray(samples)
    if np.ptp(samples[:, 0]) < abs(y2 - y1) * 0.5:
        return original
    a, b = np.polyfit(samples[:, 0], samples[:, 1], 1)
    residual = np.abs(samples[:, 1] - (a * samples[:, 0] + b))
    if np.max(residual) > 3.0:
        return original
    return (a * y1 + b, float(y1), a * y2 + b, float(y2))


class LaneTracker:
    def __init__(self):
        self.shape = None
        self.reset()

    def reset(self):
        self.pair = None
        self.last_seen = None
        self.lane_width = None
        self.width_count = 0

    def candidates(self, lines, height, width, color_mask=None, tape_mask=None):
        """허프 선분들을 묶어서 좌우 차선 후보를 만듭니다."""
        groups = [[], []]

        if lines is None:
            return groups

        y_top = height * 0.76
        y_bottom = height * 0.94
        segments = []

        for x1, y1, x2, y2 in np.asarray(lines).reshape(-1, 4):
            x1, y1, x2, y2 = _tape_center_segment(
                x1, y1, x2, y2, tape_mask
            )
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

            if not width * LANE_X_MIN_RATIO < ends[1] < width * LANE_X_MAX_RATIO:
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

            # 선분 주변의 흰색/노란색 비율을 측정해 최소 기준 미만(도로 이음새 등)은 제외합니다.
            color_ratio = 0.0
            if color_mask is not None:
                xs = np.clip(np.rint(np.linspace(x1, x2, 24)).astype(int), 0, width - 1)
                ys = np.clip(np.rint(np.linspace(y1, y2, 24)).astype(int), 0, height - 1)
                color_ratio = float(np.mean(color_mask[ys, xs] > 0))
                if color_ratio < MIN_COLOR_RATIO:
                    continue

            segments.append((
                side,
                ends,
                [(x1, y1), (x2, y2)],
                length,
                color_ratio,
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

                total_length = sum(segment[3] for segment in support)
                color_bonus = COLOR_BONUS * sum(
                    segment[3] * segment[4] for segment in support
                ) / max(total_length, 1.0)

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
                    groups[side].append((score, ends, color_bonus))

            groups[side].sort(
                key=lambda item: item[0] + item[2],
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

    def update(self, edges, now, frame=None):
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

        color_mask = None
        tape_mask = None
        if frame is not None:
            if frame.shape[:2] != edges.shape:
                raise ValueError("frame과 edges의 해상도가 같아야 합니다.")
            hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            white = cv2.inRange(hls, (0, 150, 0), (180, 255, 150))
            yellow = cv2.inRange(hsv, (15, 65, 70), (40, 255, 255))
            # 중심 계산에는 팽창시키지 않은 원래 색상 영역을 사용합니다.
            tape_mask = white | yellow
            color_mask = cv2.dilate(tape_mask, np.ones((5, 5), np.uint8))

        left_candidates, right_candidates = self.candidates(
            lines,
            height,
            width,
            color_mask,
            tape_mask,
        )

        choices = []

        for left_score, left, left_bonus in left_candidates:
            for right_score, right, right_bonus in right_candidates:
                widths = right - left

                if not width * MIN_LANE_WIDTH_RATIO < widths[1] < width * MAX_LANE_WIDTH_RATIO:
                    continue

                # 폭 학습이 완료된 상태에서는 기존 차선 폭과 크게 다른 조합(한 차선 양 경계, 이음새)을 제외합니다.
                if (
                    self.lane_width is not None
                    and self.width_count >= WIDTH_LEARN_FRAMES
                ):
                    learned_width = self.lane_width[1]
                    if not (
                        learned_width * (1.0 - LEARNED_WIDTH_TOLERANCE)
                        <= widths[1]
                        <= learned_width * (1.0 + LEARNED_WIDTH_TOLERANCE)
                    ):
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
                    width * LANE_X_MIN_RATIO < left.min()
                    and right.max() < width * LANE_X_MAX_RATIO
                ):
                    continue

                score = (
                    min(left_score, 100)
                    + min(right_score, 100)
                    + left_bonus + right_bonus
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
    marker_y = int(height * LOOKAHEAD_Y_RATIO)

    cv2.line(
        frame,
        (camera_x, min(int(height * 0.60), marker_y)),
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
        # 차선 좌표의 계산 기준 높이: 기존 값 유지
        y_top = height * 0.76
        y_bottom = height * 0.94

        # 표시선의 위쪽 높이만 조절합니다.
        # 0.40: 화면 위에서 40% 지점까지 표시
        # 숫자를 줄이면 더 길게, 늘리면 더 짧게 표시됩니다.
        draw_y_top = int(height * 0.40)

        # 기존 기울기를 유지하며 위쪽 표시 좌표 계산
        draw_tops = (
            pair[:, 0]
            + (pair[:, 1] - pair[:, 0])
            * (draw_y_top - y_top)
            / (y_bottom - y_top)
        )

        # 화면 맨 아래의 차선 좌표: 기존 계산 유지
        bottoms = (
            pair[:, 1]
            + (pair[:, 1] - pair[:, 0])
            * (height - 1 - y_bottom)
            / (y_bottom - y_top)
        )

        # 전방 주시 높이(marker_y)에서의 좌우 차선 좌표
        lookaheads = (
            pair[:, 0]
            + (pair[:, 1] - pair[:, 0])
            * (marker_y - y_top)
            / (y_bottom - y_top)
        )

        for side, bottom in enumerate(bottoms):
            inferred = (
                (status == "LEFT_ONLY" and side == 1)
                or (status == "RIGHT_ONLY" and side == 0)
            )

            # 실제 차선: 초록색 / 가상 차선: 주황색
            color = (
                (0, 165, 255)
                if inferred
                else (0, 255, 0)
            )

            if status == "HELD":
                color = (150, 150, 150)

            cv2.line(
                frame,
                (int(draw_tops[side]), draw_y_top),
                (int(bottom), height - 1),
                color,
                2,
                cv2.LINE_AA,
            )

        draw_center_top = float(np.mean(draw_tops))
        center_bottom = float(np.mean(bottoms))
        center_lookahead = float(np.mean(lookaheads))

        # 노란색 중앙선도 동일한 높이까지 연장
        cv2.line(
            frame,
            (int(draw_center_top), draw_y_top),
            (int(center_bottom), height - 1),
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        # 전방 주시점 기준 조향 오차 계산 (횡오차 + 차체 방향각 동시 반영)
        error = camera_x - center_lookahead

        cv2.line(
            frame,
            (camera_x, marker_y),
            (int(center_lookahead), marker_y),
            (0, 0, 255),
            1,
        )

        cv2.circle(
            frame,
            (int(center_lookahead), marker_y),
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
        (camera_x, marker_y),
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