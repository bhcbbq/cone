"""Standalone video/CSI camera test: quadratic curves on camera and Canny views.
No motor or serial commands. The YouTube implementation is not available;
this implements a similar visualization using horizontal-band curve tracking.
"""
import cv2
import time
import numpy as np

# VSCode 실행 버튼: 이 영상 파일을 바로 재생합니다.
# 젯슨 CSI 카메라로 확인할 때는 VIDEO_PATH = "" 로 바꾸세요.
VIDEO_PATH = r"C:\Users\noah4\OneDrive\바탕 화면\KakaoTalk_20261019_184257213.mp4"

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
    # 차량/카메라 중앙 기준선. 검출 성공 여부와 관계없이 표시합니다.
    cv2.line(frame, (width // 2, int(height * .4)),
             (width // 2, height - 1), (0, 255, 0), 1)
    if 'pair' not in result:
        cv2.putText(frame, 'LANE LOST: ' + result['reason'], (20, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 255), 2)
        return frame
    ys = np.arange(int(np.ceil(result['top'])), height)
    pair = result['pair']
    for name, coeff in [('LEFT', pair[0]), ('RIGHT', pair[1]), ('CENTER', (pair[0] + pair[1]) / 2)]:
        xs = np.polyval(coeff, ys / (height - 1))
        points = np.column_stack((xs, ys)).astype(np.int32).reshape(-1, 1, 2)
        color = {'LEFT': (255, 0, 0), 'RIGHT': (0, 0, 255),
                 'CENTER': (0, 255, 255)}[name]
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


def gstreamer_pipeline(sensor_id=0, capture_width=1280, capture_height=720,
                       display_width=640, display_height=360, framerate=30,
                       flip_method=0):
    return (
        'nvarguscamerasrc sensor-id=%d ! '
        'video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, '
        'framerate=(fraction)%d/1 ! nvvidconv flip-method=%d ! '
        'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
        'videoconvert ! video/x-raw, format=(string)BGR ! '
        'appsink drop=true max-buffers=1 sync=false'
        % (sensor_id, capture_width, capture_height, framerate, flip_method,
           display_width, display_height))


def main():
    import argparse
    from pathlib import Path
    parser = argparse.ArgumentParser(description='Camera or MP4 lane detection test; no motor commands.')
    parser.add_argument('--video', default=VIDEO_PATH, help='MP4 video file path')
    parser.add_argument('--camera', action='store_true', help='Use Jetson CSI camera')
    args = parser.parse_args()
    if args.camera:
        args.video = None
    cap = None
    tracker = CurveTracker()
    try:
        if args.video:
            video_path = Path(args.video).expanduser()
            if not video_path.is_file():
                raise FileNotFoundError(f'영상 파일이 없습니다: {video_path}')
            cap = cv2.VideoCapture(str(video_path))
        else:
            cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            raise RuntimeError('영상 파일 또는 카메라를 열 수 없습니다.')
        fps = cap.get(cv2.CAP_PROP_FPS) if args.video else 30.0
        if not np.isfinite(fps) or fps <= 0:
            fps = 30.0
        frame_index = 0
        window_name = 'Future Makers - UGV02 Autonomous Driving'
        cv2.namedWindow(window_name)
        print('곡선 인식 화면 테스트: q / Ctrl+C 종료, Space 일시정지/재개.')
        print('파랑=왼쪽, 빨강=오른쪽, 노랑=중심곡선, 초록 세로선=카메라 중앙, 주황=추정 차선.')
        while cap.isOpened():
            frame_started = time.monotonic()
            ret, frame = cap.read()
            if not ret or frame is None:
                print('영상 재생 종료 또는 프레임 읽기 실패.' if args.video else '카메라 프레임 수신 실패: 종료합니다.')
                break
            # Match the camera test width while preserving the video aspect ratio.
            if args.video and frame.shape[1] != 640:
                new_height = max(2, round(frame.shape[0] * 640 / frame.shape[1]))
                frame = cv2.resize(frame, (640, new_height))
            height, width = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)
            # Wider upper ROI retains curves that bend toward the image sides.
            roi_vertices = [
                (0, height - 1),
                (int(width * 0.1), int(height * 0.4)),
                (int(width * 0.9), int(height * 0.4)),
                (width - 1, height - 1),
            ]
            cropped_edges = region_of_interest(edges, np.array([roi_vertices], np.int32))
            result = tracker.detect(cropped_edges, min(y for x, y in roi_vertices),
                                    now=frame_index / fps if args.video else None)
            frame_index += 1
            result_image = draw_result(frame.copy(), result)
            cv2.imshow(window_name, result_image)
            # Canny는 단일 채널이므로 컬러로 변환한 뒤 같은 곡선을 겹칩니다.
            edge_view = cv2.cvtColor(cropped_edges, cv2.COLOR_GRAY2BGR)
            edge_view = draw_result(edge_view, result)
            cv2.imshow('Canny Edges - Curve Tracking', edge_view)
            delay_ms = (max(1, round(1000 * (1.0 / fps - (time.monotonic() - frame_started))))
                        if args.video else 1)
            key = cv2.waitKey(delay_ms) & 0xFF
            if key == ord('q'):
                break
            if key == ord(' '):
                print('일시정지: Space 재개 / q 종료')
                while True:
                    key = cv2.waitKey(50) & 0xFF
                    if key in (ord(' '), ord('q')):
                        break
                if key == ord('q'):
                    break
    except KeyboardInterrupt:
        print('테스트 종료')
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
