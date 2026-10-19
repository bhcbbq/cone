import cv2
import time
import numpy as np

# 차선 검출 모듈: 카메라/시리얼 연결 및 모터 명령 없음.
# lane_detection_test.py의 수평 밴드 탐색 및 2차 곡선(x(y)) 피팅 알고리즘 적용.
CANNY_LOW = 50
CANNY_HIGH = 150
CONFIRM_FRAMES = 3
SINGLE_SIDE_MAX_SECONDS = 0.4
ERROR_SMOOTHING = 0.35
LOOKAHEAD_RATIO = 0.72
N_BANDS = 14


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, vertices, 255)
    return cv2.bitwise_and(img, mask)


def peaks(histogram, radius, limit):
    """Separate nearby double edges from unrelated curve candidates."""
    values = np.convolve(histogram.astype(float), np.ones(9), mode='same')
    result = []
    for _ in range(limit):
        x = int(np.argmax(values))
        if values[x] < 3:
            break
        result.append(x)
        values[max(0, x - radius):x + radius + 1] = 0
    return result


class CurveTracker:
    def __init__(self):
        self.shape = None
        self.reset()

    def reset(self):
        self.previous = None
        self.last_pair_time = None
        self.width_model = None
        self.confirm_count = 0
        self.failures = 0
        self.filtered_error = None

    def fail(self, reason):
        self.confirm_count = 0
        self.filtered_error = None
        self.failures += 1
        if self.failures >= 3:
            self.reset()
        return {'detected': False, 'reason': reason}

    @staticmethod
    def trace(edges, seed, top):
        height, width = edges.shape
        bounds = np.linspace(height, top, N_BANDS + 1).astype(int)
        points = []
        prediction = float(seed)
        gaps = 0
        margin = max(24, int(width * 0.07))
        for i in range(N_BANDS):
            hi, lo = bounds[i], bounds[i + 1]
            y = (lo + hi - 1) / 2
            if len(points) >= 2:
                px, py = points[-1]
                qx, qy = points[-2]
                slope = np.clip((px - qx) / (py - qy), -2.5, 2.5)
                prediction = px + slope * (y - py)
            elif points:
                prediction = points[-1][0]
            xlo = max(0, int(prediction - margin))
            xhi = min(width, int(prediction + margin + 1))
            if xhi <= xlo or hi <= lo:
                break
            band = edges[lo:hi, xlo:xhi] > 0
            histogram = band.sum(axis=0)
            choices = peaks(histogram, radius=12, limit=5)
            options = []
            for local in choices:
                # A horizontal stripe must not count as a lane boundary.
                a, b = max(0, local - 8), min(xhi - xlo, local + 9)
                ry, rx = np.nonzero(band[:, a:b])
                if len(ry) == 0 or len(np.unique(ry)) < max(3, (hi - lo) * .4):
                    continue
                x = float(np.median(rx + a + xlo))
                score = len(np.unique(ry)) / (hi - lo) - .6 * abs(x - prediction) / margin
                options.append((score, x, float(np.median(ry + lo))))
            if not options:
                gaps += 1
                if gaps > 2:
                    break
                continue
            gaps = 0
            _, x, observed_y = max(options)
            points.append((x, observed_y))

        if len(points) < 6:
            return None
        pts = np.array(points)
        # Require visible support near the robot, and across the ROI height.
        if pts[0, 1] < height - 2.2 * (height - top) / N_BANDS:
            return None
        if np.ptp(pts[:, 1]) < (height - top) * .4:
            return None
        t = pts[:, 1] / (height - 1)
        keep = np.ones(len(pts), dtype=bool)
        for _ in range(3):
            if keep.sum() < 6:
                return None
            coeff = np.polyfit(t[keep], pts[keep, 0], 2)
            residual = np.abs(np.polyval(coeff, t) - pts[:, 0])
            med = np.median(residual[keep])
            mad = np.median(np.abs(residual[keep] - med))
            keep = residual <= max(7.0, med + 3.0 * 1.4826 * mad)
        if keep.sum() < 6 or np.ptp(pts[keep, 1]) < (height - top) * .4:
            return None
        coeff = np.polyfit(t[keep], pts[keep, 0], 2)
        rms = float(np.sqrt(np.mean((np.polyval(coeff, t[keep]) - pts[keep, 0]) ** 2)))
        if not np.all(np.isfinite(coeff)) or rms > width * .025:
            return None
        return {'coeff': coeff, 'top': float(pts[keep, 1].min()),
                'score': float(keep.sum()) - rms / 5.0}

    def detect(self, edges, roi_top, now=None):
        now = time.monotonic() if now is None else now
        height, width = edges.shape
        if self.shape != edges.shape:
            self.reset()
            self.shape = edges.shape
        if self.last_pair_time is not None and now - self.last_pair_time > SINGLE_SIDE_MAX_SECONDS:
            self.reset()
        if np.count_nonzero(edges) < 20:
            return self.fail('no edge support')
        histogram = (edges[int(height * .90):] > 0).sum(axis=0)
        seeds = peaks(histogram, max(18, int(width * .035)), 12)
        if self.previous is not None:
            seeds += [int(np.polyval(c, .95)) for c in self.previous]
        tracks = []
        for seed in seeds:
            if not 0 <= seed < width:
                continue
            track = self.trace(edges, seed, roi_top)
            if track is None:
                continue
            duplicate = next((i for i, old in enumerate(tracks)
                              if np.max(np.abs(np.polyval(track['coeff'] - old['coeff'], [.75, 1.]))) < 18), None)
            if duplicate is None:
                tracks.append(track)
            elif track['score'] > tracks[duplicate]['score']:
                tracks[duplicate] = track
        tracks.sort(key=lambda x: np.polyval(x['coeff'], 1.0))
        choices = []
        for i, left in enumerate(tracks):
            for right in tracks[i + 1:]:
                common_top = max(left['top'], right['top'])
                if common_top > height * .80:
                    continue
                ys = np.linspace(common_top / (height - 1), 1., 20)
                lx, rx = [np.polyval(t['coeff'], ys) for t in (left, right)]
                widths = rx - lx
                if (widths.min() < width * .10 or widths.max() > width * 1.15 or
                        widths[-1] < width * .22 or widths[0] > widths[-1] * 1.3):
                    continue
                if not lx[-1] < width / 2 < rx[-1]:
                    continue
                if min(lx.min(), rx.min()) < -width * .1 or max(lx.max(), rx.max()) > width * 1.1:
                    continue
                score = left['score'] + right['score']
                if self.previous is not None:
                    delta = max(float(np.max(np.abs(np.polyval(t['coeff'] - old, ys))))
                                for t, old in zip((left, right), self.previous))
                    if delta > width * .18:
                        continue
                    score -= delta / 15.
                choices.append((score, left, right, common_top))

        inferred = None
        if choices:
            _, left, right, top = max(choices, key=lambda p: p[0])
            pair = (left['coeff'], right['coeff'])
            measured_pair = True
        elif (len(tracks) == 1 and self.previous is not None and
              self.confirm_count >= CONFIRM_FRAMES and self.width_model is not None):
            track = tracks[0]
            ys = np.linspace(max(track['top'] / (height - 1), .70), 1., 15)
            differences = [np.max(np.abs(np.polyval(track['coeff'] - p, ys)))
                           for p in self.previous]
            side = int(np.argmin(differences))
            if differences[side] > width * .10 or track['top'] > height * .80:
                return self.fail('single boundary ambiguous')
            if side == 0:
                pair = (track['coeff'], track['coeff'] + self.width_model)
                inferred = 'RIGHT'
            else:
                pair = (track['coeff'] - self.width_model, track['coeff'])
                inferred = 'LEFT'
            top = track['top']
            measured_pair = False
        else:
            return self.fail('need two consistent curves')

        ys = np.linspace(top / (height - 1), 1., 30)
        lx, rx = [np.polyval(c, ys) for c in pair]
        centers = (lx + rx) / 2
        if (np.any(rx - lx < width * .10) or centers.min() < 0 or
                centers.max() >= width or lx.min() < -width * .1 or rx.max() > width * 1.1):
            return self.fail('invalid curve geometry')
        # Evaluate only inside observed support, not high above the visible lane.
        target_y = int(max(height * LOOKAHEAD_RATIO, top))
        target_x = float(np.polyval((pair[0] + pair[1]) / 2, target_y / (height - 1)))
        raw_error = width // 2 - target_x
        self.filtered_error = (raw_error if self.filtered_error is None else
                               (1 - ERROR_SMOOTHING) * self.filtered_error + ERROR_SMOOTHING * raw_error)
        self.previous = tuple(c.copy() for c in pair)
        if measured_pair:
            self.width_model = pair[1] - pair[0]
            self.last_pair_time = now
        self.confirm_count += 1
        self.failures = 0
        detected = self.confirm_count >= CONFIRM_FRAMES
        return {'detected': detected, 'reason': 'TRACKING' if detected else 'CONFIRMING',
                'pair': pair, 'top': top, 'inferred': inferred,
                'target': (target_x, target_y), 'raw_error': raw_error,
                'error': self.filtered_error if detected else 0.0}


def draw_result(frame, result):
    height, width = frame.shape[:2]
    if 'pair' not in result:
        cv2.putText(frame, 'LANE LOST: ' + result['reason'], (20, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 2)
        return frame
    ys = np.arange(int(np.ceil(result['top'])), height)
    pair = result['pair']
    for name, coeff in [('LEFT', pair[0]), ('RIGHT', pair[1]), ('CENTER', (pair[0] + pair[1]) / 2)]:
        xs = np.polyval(coeff, ys / (height - 1))
        points = np.column_stack((xs, ys)).astype(np.int32).reshape(-1, 1, 2)
        color = (0, 255, 255) if name == 'CENTER' else (0, 255, 0)
        if name == result['inferred']:
            color = (0, 165, 255)
        cv2.polylines(frame, [points], False, color, 3 if name == 'CENTER' else 5)
    x, y = result['target']
    cv2.circle(frame, (width // 2, y), 7, (255, 0, 0), -1)
    cv2.circle(frame, (int(x), y), 7, (0, 255, 0), -1)
    cv2.line(frame, (width // 2, y), (int(x), y), (0, 0, 255), 3)
    cv2.putText(frame, f"Lookahead error: {result['error']:.1f} px (raw {result['raw_error']:.1f})",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 2)
    status = result['reason'] + (' / ONE SIDE ESTIMATED' if result['inferred'] else '')
    cv2.putText(frame, 'CURVE: ' + status, (20, 85), cv2.FONT_HERSHEY_SIMPLEX,
                .55, (0, 255, 0) if result['detected'] else (0, 165, 255), 2)
    return frame


_tracker = CurveTracker()


def calculate_steering(frame, lines=None, roi_top=None):
    """반환: (error, 표시 영상, 검출 성공). 양수 오차 = 중심이 화면 왼쪽.

    - main.py를 수정하지 않고 기존처럼 (frame, lines)를 전달하면,
      lane_detection_test.py의 Canny 및 확장 ROI 설정으로 에지를 추출해 동작합니다.
    - 만약 두 번째 인자로 2차원 이진 에지 영상(cropped_edges)을 직접 전달하면,
      재계산 없이 해당 에지 영상을 그대로 사용합니다.
    """
    height, width = frame.shape[:2]

    if (isinstance(lines, np.ndarray) and
            lines.ndim == 2 and
            lines.shape == (height, width)):
        cropped_edges = lines
        if roi_top is None:
            roi_top = int(height * 0.4)
    else:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)
        roi_vertices = [
            (0, height - 1),
            (int(width * 0.1), int(height * 0.4)),
            (int(width * 0.9), int(height * 0.4)),
            (width - 1, height - 1),
        ]
        cropped_edges = region_of_interest(edges, np.array([roi_vertices], np.int32))
        roi_top = min(y for x, y in roi_vertices)

    result = _tracker.detect(cropped_edges, roi_top)
    result_image = draw_result(frame, result)
    error = float(result.get('error', 0.0))
    return error, result_image, bool(result['detected'])