import cv2
import numpy as np


# 기존 ver2 설정 유지
HOLD_SECONDS = 0.20
WIDTH_LEARN_FRAMES = 3
SMOOTH_ALPHA = 0.35

# 추정 차선으로부터 실제 차선으로 복귀할 때 연속 확인할 영상 수
REACQUIRE_FRAMES = 5
# 유효 관측이 끊겨도 위치/폭 기억은 잠시 보관 (주행 허용 시간이 아님)
TRACK_MEMORY_SECONDS = 5.0
# 기억한 폭과 새 쌍의 폭 차이: 각 기준 높이에서 검사
WIDTH_CHANGE_RATIO = 0.12
WIDTH_CHANGE_FLOOR_RATIO = 0.025
# 보이는 차선을 따라 이동한 가상 차선과 새 후보의 허용 거리
REACQUIRE_POSITION_RATIO = 0.06
# 선분 방향 변화 허용치: 두 기준 높이 x 변화량의 차이 / 화면 폭
DIRECTION_CHANGE_RATIO = 0.05

# 한쪽 차선 후보가 받을 수 있는 최대 색상 가산점
COLOR_BONUS = 40.0

# 화면 아래로 연장한 x좌표의 허용 범위: 화면 폭 기준 -50% ~ 150%.
LANE_X_MIN_RATIO = -0.75
LANE_X_MAX_RATIO = 1.75
# 양쪽 차선 간격의 최대 허용값: 화면 폭의 200%.
MAX_LANE_WIDTH_RATIO = 2.50


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
        self.visible_side = None
        self.recovery_count = 0
        self.recovery_width = None
        self.recovery_time = None
        self.confirmed_color = np.zeros(2, dtype=float)

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

            # 에지를 삭제하지 않고, 선분 주변의 흰색/노란색 비율만 측정합니다.
            color_ratio = 0.0
            if color_mask is not None:
                xs = np.clip(np.rint(np.linspace(x1, x2, 24)).astype(int), 0, width - 1)
                ys = np.clip(np.rint(np.linspace(y1, y2, 24)).astype(int), 0, height - 1)
                color_ratio = float(np.mean(color_mask[ys, xs] > 0))

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
            and now - self.last_seen > TRACK_MEMORY_SECONDS
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

        return self._select_candidates(
            (left_candidates, right_candidates), width, now
        )

    def _clear_recovery(self):
        self.recovery_count = 0
        self.recovery_width = None
        self.recovery_time = None

    def _width_matches(self, widths, width):
        tolerance = np.maximum(
            np.abs(self.lane_width) * WIDTH_CHANGE_RATIO,
            width * WIDTH_CHANGE_FLOOR_RATIO,
        )
        return bool(np.all(np.abs(widths - self.lane_width) <= tolerance))

    def _candidate_matches(self, candidate, side, width):
        if self.pair is None:
            return True
        shift = candidate[1] - self.pair[side]
        if np.max(np.abs(shift)) > width * 0.13:
            return False
        if abs(shift[1] - shift[0]) > width * DIRECTION_CHANGE_RATIO:
            return False
        # 이전에 색상 근거가 확실했던 선이 갑자기 무색 이음새로 바뀌는 것 방지.
        # 처음부터 어두워 색상 근거가 약했던 선에는 강제 색상 필터를 적용하지 않음.
        reference = self.confirmed_color[side]
        return reference < 16.0 or candidate[2] >= reference * 0.5

    def _confirm_pair(self, measured, selected, now):
        measured_width = measured[1] - measured[0]
        if np.all(measured_width > 0):
            if self.lane_width is None:
                self.lane_width = measured_width.copy()
            else:
                self.lane_width = 0.9 * self.lane_width + 0.1 * measured_width
            self.width_count += 1
        else:
            self.width_count = 0
            self.lane_width = None
        colors = np.array([c[2] for c in selected])
        if self.width_count <= 1:
            self.confirmed_color = colors
        else:
            self.confirmed_color = 0.9 * self.confirmed_color + 0.1 * colors
        self.visible_side = None
        self._clear_recovery()
        self.accept(measured, now)
        return self.pair.copy(), "TRACKING"

    def _missing(self, now):
        self._clear_recovery()
        # 위치/폭은 보관하지만 HELD와 LOST 모두 draw_result에서 주행 불허.
        if self.pair is not None and now - self.last_seen <= HOLD_SECONDS:
            return self.pair.copy(), "HELD"
        return None, "LOST"

    def _infer(self, candidate, side, width, now):
        real = candidate[1]
        measured = np.array([real, real + self.lane_width]) if side == 0 else np.array([
            real - self.lane_width, real
        ])
        if np.max(np.abs(measured - self.pair)) > width * 0.13:
            return self._missing(now)
        self.visible_side = side
        # 가상 차선 상태에서는 폭 및 색상 학습을 하지 않는다.
        self.accept(measured, now)
        return self.pair.copy(), "LEFT_ONLY" if side == 0 else "RIGHT_ONLY"

    def _select_candidates(self, groups, width, now):
        ready = (self.pair is not None and self.lane_width is not None
                 and self.width_count >= WIDTH_LEARN_FRAMES)
        if ready:
            groups = tuple([
                c for c in candidates if self._candidate_matches(c, side, width)
            ] for side, candidates in enumerate(groups))
        choices = []
        for left in groups[0]:
            for right in groups[1]:
                measured = np.array([left[1], right[1]])
                widths = measured[1] - measured[0]
                if not width * 0.12 < widths[1] < width * MAX_LANE_WIDTH_RATIO:
                    continue
                if not -width * 0.03 < widths[0] < widths[1] * 0.85:
                    continue
                if self.pair is None and not left[1][1] < width * 0.5 < right[1][1]:
                    continue
                if not (width * LANE_X_MIN_RATIO < left[1].min()
                        and right[1].max() < width * LANE_X_MAX_RATIO):
                    continue
                if ready:
                    if not self._width_matches(widths, width):
                        continue
                    # 양쪽 차선의 공통 이동은 허용하고, 한쪽만 다른 선으로 튀는 변화는 검사.
                    left_shift = measured[0] - self.pair[0]
                    right_shift = measured[1] - self.pair[1]
                    if np.max(np.abs(left_shift - right_shift)) > width * REACQUIRE_POSITION_RATIO:
                        continue
                score = min(left[0], 100) + min(right[0], 100) + left[2] + right[2]
                choices.append((score, measured, (left, right)))

        gap = self.last_seen is not None and now - self.last_seen > HOLD_SECONDS
        recovering = self.visible_side is not None or self.recovery_count > 0 or gap
        if choices:
            _, measured, selected = max(choices, key=lambda item: item[0])
            if not ready or not recovering:
                return self._confirm_pair(measured, selected, now)
            # 연속 프레임의 폭을 비교하므로, 커브에서 두 선이 함께 이동해도 허용.
            widths = measured[1] - measured[0]
            consistent = (
                self.recovery_width is not None
                and self.recovery_time is not None
                and 0 < now - self.recovery_time <= HOLD_SECONDS
                and np.max(np.abs(widths - self.recovery_width)) <= width * 0.025
            )
            self.recovery_count = self.recovery_count + 1 if consistent else 1
            self.recovery_width = widths.copy()
            self.recovery_time = now
            if self.recovery_count >= REACQUIRE_FRAMES:
                return self._confirm_pair(measured, selected, now)
            if self.visible_side is not None:
                return self._infer(selected[self.visible_side], self.visible_side, width, now)
            # 양쪽 모두 잃었던 경우, 복귀 확인이 끝날 때까지 정지 상태 유지.
            return None, "LOST"

        self._clear_recovery()
        if ready:
            if self.visible_side is not None:
                # 기존에 보이던 쪽을 유지. 반대쪽 잡음으로 기준을 즉시 바꾸지 않는다.
                side = self.visible_side
                if groups[side]:
                    return self._infer(groups[side][0], side, width, now)
            elif bool(groups[0]) != bool(groups[1]):
                side = 0 if groups[0] else 1
                return self._infer(groups[side][0], side, width, now)
            elif groups[0] and groups[1]:
                # 두 후보가 있지만 폭이 맞지 않는 경우, 더 확실한 한쪽만 사용.
                best = (groups[0][0], groups[1][0])
                distances = [np.max(np.abs(best[i][1] - self.pair[i])) for i in (0, 1)]
                side = int(np.argmin(distances))
                if distances[1 - side] - distances[side] > width * 0.02:
                    return self._infer(best[side], side, width, now)
                color_side = 0 if best[0][2] > best[1][2] else 1
                if best[color_side][2] - best[1 - color_side][2] > 8.0:
                    return self._infer(best[color_side], color_side, width, now)
                # 어느 쪽이 진짜인지 불명확하면 추측 주행하지 않는다.
        elif self.width_count < WIDTH_LEARN_FRAMES:
            self.width_count = 0
            self.lane_width = None
        return self._missing(now)



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

        # 노란색 중앙선도 동일한 높이까지 연장
        cv2.line(
            frame,
            (int(draw_center_top), draw_y_top),
            (int(center_bottom), height - 1),
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        # 조향 오차 계산: 기존 방식 그대로 유지
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