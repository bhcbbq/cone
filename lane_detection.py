import cv2
import time
import numpy as np

# 차선 검출 모듈: 카메라/시리얼 연결 및 모터 명령 없음.
# 기존 main_integrated.py의 함수 호출과 오차 부호를 유지합니다.
# ROI와 Canny/Hough 설정은 기존 메인에서 관리합니다.
CONFIRM_FRAMES = 3
SINGLE_SIDE_MAX_SECONDS = 0.40
RESET_AFTER_FAILURES = 3
ERROR_SMOOTHING = 0.35


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(img, mask)


class LaneTracker:
    """Geometric/temporal checks, not semantic recognition of road markings."""

    def __init__(self):
        self.shape = None
        self.reset()

    def reset(self):
        self.widths = None
        self.last_pair_time = None
        self.last_center = None
        self.filtered_error = None
        self.confirm_count = 0
        self.failures = 0

    def fail(self, frame, reason):
        self.failures += 1
        self.confirm_count = 0
        self.filtered_error = None
        if self.failures >= RESET_AFTER_FAILURES:
            self.reset()
        cv2.putText(frame, 'LANE LOST: ' + reason, (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        return 0.0, frame, False

    @staticmethod
    def fit_side(candidates, width, height, y_bottom, y_top):
        if not candidates:
            return None
        # Each segment proposes x = a*y+b. Select mutually agreeing segments
        # instead of averaging every edge (including unrelated objects).
        anchors = np.array([[a * y_bottom + b, a * y_top + b]
                            for a, b, _, _ in candidates])
        weights = np.array([length for _, _, length, _ in candidates])
        tolerance = max(10.0, width * 0.035)
        best_mask, best_score = None, -1.0
        for anchor in anchors:
            mask = np.max(np.abs(anchors - anchor), axis=1) <= tolerance
            score = float(weights[mask].sum())
            if score > best_score:
                best_score, best_mask = score, mask
        selected = [c for c, keep in zip(candidates, best_mask) if keep]
        points = np.array([p for _, _, _, ends in selected for p in ends])
        if (best_score < height * 0.22 or
                np.ptp(points[:, 1]) < height * 0.18):
            return None
        point_weights = np.repeat([c[2] for c in selected], 2)
        matrix = np.column_stack((points[:, 1], np.ones(len(points))))
        root_weights = np.sqrt(point_weights)
        coeffs, _, rank, _ = np.linalg.lstsq(
            matrix * root_weights[:, None], points[:, 0] * root_weights,
            rcond=None)
        if rank < 2 or not np.all(np.isfinite(coeffs)):
            return None
        a, b = coeffs
        prediction = matrix @ coeffs
        residual = np.sqrt(np.average((points[:, 0] - prediction) ** 2,
                                      weights=point_weights))
        if residual > tolerance * 0.5:
            return None
        return np.array([a * y_bottom + b, a * y_top + b])

    def update(self, frame, lines, now=None):
        now = time.monotonic() if now is None else now
        height, width = frame.shape[:2]
        if self.shape != (height, width):
            self.reset()
            self.shape = (height, width)
        if self.last_pair_time is not None and (
                now - self.last_pair_time > SINGLE_SIDE_MAX_SECONDS):
            self.reset()
        if lines is None or np.size(lines) == 0:
            return self.fail(frame, 'no lines')

        y_bottom = height - 1
        y_top = int(height * 0.55)
        y_eval = int(height * 0.80)
        camera_center = width // 2
        groups = [[], []]
        for x1, y1, x2, y2 in np.asarray(lines, dtype=float).reshape(-1, 4):
            if not np.all(np.isfinite([x1, y1, x2, y2])):
                continue
            dx, dy = x2 - x1, y2 - y1
            length = float(np.hypot(dx, dy))
            # x(y) also supports vertical lines; reject horizontal edges.
            if abs(dy) < max(8, height * 0.025) or length < 20:
                continue
            a = dx / dy
            if abs(a) > 1.5:
                continue
            b = x1 - a * y1
            x_eval = a * y_eval + b
            xb, xt = a * y_bottom + b, a * y_top + b
            if not (-0.10 * width <= xb <= 1.10 * width and
                    -0.10 * width <= xt <= 1.10 * width):
                continue
            # Check image position AND perspective orientation.
            candidate = (a, b, length, ((x1, y1), (x2, y2)))
            if x_eval < camera_center - width * 0.03 and a <= 0.15:
                groups[0].append(candidate)
            elif x_eval > camera_center + width * 0.03 and a >= -0.15:
                groups[1].append(candidate)

        left, right = [self.fit_side(g, width, height, y_bottom, y_top)
                       for g in groups]
        inferred = None
        pair_widths = None
        if left is not None and right is not None:
            pair_widths = right - left
            wb, wt = pair_widths
            if not (0.25 * width <= wb <= 1.15 * width and
                    0.06 * width <= wt <= wb * 1.10):
                return self.fail(frame, 'invalid lane width')
            if self.widths is not None and np.any(
                    np.abs(pair_widths - self.widths) > width * 0.15):
                return self.fail(frame, 'lane width jumped')
        elif left is not None or right is not None:
            if (self.widths is None or self.last_pair_time is None or
                    self.confirm_count < CONFIRM_FRAMES):
                return self.fail(frame, 'need both lanes')
            # Only use a recent measured width; never assume a fixed 160px.
            if left is None:
                left = right - self.widths
                inferred = 'LEFT'
            else:
                right = left + self.widths
                inferred = 'RIGHT'
        else:
            return self.fail(frame, 'no reliable lane')

        centers = (left + right) / 2.0
        if not np.all((centers >= 0) & (centers < width)):
            return self.fail(frame, 'center outside image')
        if self.last_center is not None and np.any(
                np.abs(centers - self.last_center) > width * 0.12):
            return self.fail(frame, 'center jumped')

        self.last_center = centers.copy()
        self.failures = 0
        self.confirm_count += 1
        if pair_widths is not None:
            self.widths = (pair_widths.copy() if self.widths is None else
                           0.7 * self.widths + 0.3 * pair_widths)
            self.last_pair_time = now

        raw_error = float(camera_center - centers[0])
        self.filtered_error = (raw_error if self.filtered_error is None else
                               (1 - ERROR_SMOOTHING) * self.filtered_error +
                               ERROR_SMOOTHING * raw_error)
        confirmed = self.confirm_count >= CONFIRM_FRAMES

        for side, values in [('LEFT', left), ('RIGHT', right)]:
            color = (0, 165, 255) if inferred == side else (0, 255, 0)
            cv2.line(frame, (int(values[0]), y_bottom),
                     (int(values[1]), y_top), color, 6)
        cv2.line(frame, (int(centers[0]), y_bottom),
                 (int(centers[1]), y_top), (0, 255, 255), 3)
        cv2.circle(frame, (camera_center, y_bottom - 40), 8, (255, 0, 0), -1)
        cv2.circle(frame, (int(centers[0]), y_bottom - 40), 8, (0, 255, 0), -1)
        cv2.line(frame, (camera_center, y_bottom - 40),
                 (int(centers[0]), y_bottom - 40), (0, 0, 255), 4)
        cv2.putText(frame, f'Error: {self.filtered_error:.1f} px (raw {raw_error:.1f})',
                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        status = ('Lane: TRACKING' if confirmed else
                  f'Lane: CONFIRMING {self.confirm_count}/{CONFIRM_FRAMES}')
        if inferred:
            status += ' / ONE SIDE ESTIMATED'
        cv2.putText(frame, status, (20, 90), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 0) if confirmed else (0, 165, 255), 2)
        return (self.filtered_error if confirmed else 0.0), frame, confirmed


_tracker = LaneTracker()


def calculate_steering(frame, lines):
    """반환: (error, 표시 영상, 검출 성공). 양수 오차 = 중심이 화면 왼쪽.

    양쪽 차선 3회 확인 후 성공. 한쪽은 최근 측정 폭으로 최대 0.4초 추정.
    False는 검출 불확실을 의미하며, 실제 정지/회전 판단은 메인의 역할입니다.
    """
    return _tracker.update(frame, lines)

