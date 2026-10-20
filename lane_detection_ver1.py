import cv2
import numpy as np


ROAD_MODE = "highway"

HOLD_SECONDS = 0.20
N_BANDS = 12
MAX_DASH_GAPS = 4


def get_edges(frame, return_raw=False):
    h, w = frame.shape[:2]

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(gray, 40, 120)
    raw_edges = edges.copy()

    if ROAD_MODE == "black_track":
        mask = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            41,
            8,
        )
        top = int(h * 0.50)

    else:
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(
            hls,
            (0, 150, 0),
            (180, 255, 150),
        )
        mask |= cv2.inRange(
            hsv,
            (15, 65, 70),
            (40, 255, 255),
        )
        top = int(h * 0.70)

    mask = cv2.dilate(
        mask,
        np.ones((5, 5), np.uint8),
    )

    edges = cv2.bitwise_and(edges, mask)
    bottom = int(h * 0.96)

    edges[:top] = 0
    edges[bottom:] = 0
    edges[:, :int(w * 0.02)] = 0
    edges[:, int(w * 0.98):] = 0

    if return_raw:
        return raw_edges, edges, top, bottom

    return edges, top, bottom


def find_peaks(histogram, limit=6):
    values = np.convolve(
        histogram.astype(float),
        np.ones(7),
        mode="same",
    )

    result = []

    for _ in range(limit):
        x = int(np.argmax(values))

        if values[x] < 5:
            break

        result.append(x)
        values[max(0, x - 18):x + 19] = 0

    return result


class LaneTracker:
    def __init__(self):
        self.shape = None
        self.reset()

    def reset(self):
        self.pair = None
        self.width_model = None
        self.last_pair = None
        self.last_observed = None
        self.pair_count = 0

    def trace(self, edges, band, seed, bounds):
        h, w = edges.shape
        points = []

        guide = None
        seed_y = (
            bounds[band] + bounds[band + 1] - 1
        ) / 2

        if ROAD_MODE == "highway" and self.pair is not None:
            distances = [
                abs(
                    np.polyval(c, seed_y / (h - 1))
                    - seed
                )
                for c in self.pair
            ]

            side = int(np.argmin(distances))

            if (
                distances[side] < w * 0.06
                and abs(distances[0] - distances[1]) > w * 0.05
            ):
                guide = self.pair[side]

        for direction in (-1, 1):
            path = []
            prediction = float(seed)
            gaps = 0

            stop = N_BANDS if direction == 1 else -1

            for index in range(band, stop, direction):
                lo = bounds[index]
                hi = bounds[index + 1]
                y = (lo + hi - 1) / 2

                if len(path) >= 2:
                    x1, y1 = path[-1]
                    x0, y0 = path[-2]

                    slope = np.clip(
                        (x1 - x0) / (y1 - y0),
                        -4,
                        4,
                    )

                    prediction = x1 + slope * (y - y1)

                elif path:
                    prediction = path[-1][0]

                if gaps > 0 and guide is not None:
                    if path:
                        anchor_x, anchor_y = path[-1]
                    else:
                        anchor_x, anchor_y = float(seed), seed_y

                    prediction = (
                        anchor_x
                        + np.polyval(guide, y / (h - 1))
                        - np.polyval(guide, anchor_y / (h - 1))
                    )

                margin = int(w * 0.10)
                a = max(0, int(prediction - margin))
                b = min(w, int(prediction + margin + 1))

                if b - a < 7:
                    break

                crop = edges[lo:hi, a:b] > 0
                options = []
                histogram = crop.sum(axis=0)

                for local in find_peaks(histogram, 5):
                    xlo = max(0, local - 5)
                    xhi = min(b - a, local + 6)

                    ry, rx = np.nonzero(crop[:, xlo:xhi])
                    row_count = len(np.unique(ry))

                    if (
                        len(ry) < 3
                        or row_count < max(2, (hi - lo) * 0.20)
                    ):
                        continue

                    x = float(np.median(rx + xlo + a))
                    observed_y = float(np.median(ry + lo))

                    if (
                        gaps > 0
                        and guide is not None
                        and abs(x - prediction) > w * 0.05
                    ):
                        continue

                    score = (
                        row_count / (hi - lo)
                        - abs(x - prediction) / margin
                    )

                    options.append((score, x, observed_y))

                if not options:
                    gaps += 1

                    allowed_gaps = (
                        MAX_DASH_GAPS
                        if guide is not None
                        else 2
                    )

                    if gaps > allowed_gaps:
                        break

                    continue

                gaps = 0
                _, x, observed_y = max(options)
                path.append((x, observed_y))

            points.extend(path)

        points = np.unique(
            np.array(points).reshape(-1, 2),
            axis=0,
        )

        minimum_span = (bounds[-1] - bounds[0]) * 0.30

        if (
            len(points) < 5
            or np.ptp(points[:, 1]) < minimum_span
        ):
            return None

        t = points[:, 1] / (h - 1)
        keep = np.ones(len(points), dtype=bool)

        for _ in range(3):
            if keep.sum() < 5:
                return None

            coeff = np.polyfit(
                t[keep],
                points[keep, 0],
                2,
            )

            residual = np.abs(
                np.polyval(coeff, t) - points[:, 0]
            )

            median = np.median(residual[keep])
            mad = np.median(
                np.abs(residual[keep] - median)
            )

            keep = residual <= max(
                5,
                median + 3 * 1.4826 * mad,
            )

        if (
            keep.sum() < 5
            or np.ptp(points[keep, 1]) < minimum_span
        ):
            return None

        coeff = np.polyfit(
            t[keep],
            points[keep, 0],
            2,
        )

        rms = np.sqrt(np.mean(
            (
                np.polyval(coeff, t[keep])
                - points[keep, 0]
            ) ** 2
        ))

        if (
            not np.all(np.isfinite(coeff))
            or rms > w * 0.025
        ):
            return None

        if ROAD_MODE == "highway":
            samples = np.linspace(
                max(0.76, t[keep].min()),
                0.94,
                10,
            )

            slopes = np.polyval(
                np.polyder(coeff),
                samples,
            ) / (h - 1)

            if np.max(np.abs(slopes)) > 3:
                return None

            xs = np.polyval(
                coeff,
                [0.76, 0.86, 0.94],
            )

            if (
                xs.min() < -w * 0.15
                or xs.max() > w * 1.15
            ):
                return None

        return {
            "coeff": coeff,
            "top": float(points[keep, 1].min()),
            "bottom": float(points[keep, 1].max()),
            "score": float(keep.sum()) - rms / 4,
        }

    def update(self, edges, top, bottom, now):
        h, w = edges.shape

        if self.shape != edges.shape:
            self.reset()
            self.shape = edges.shape

        if (
            self.last_observed is not None
            and now - self.last_observed > HOLD_SECONDS
        ):
            self.reset()

        bounds = np.linspace(
            top,
            bottom,
            N_BANDS + 1,
        ).astype(int)

        seeds = []

        for band in (N_BANDS - 2, N_BANDS // 2, 2):
            lo = bounds[band]
            hi = bounds[band + 1]

            histogram = (
                edges[lo:hi] > 0
            ).sum(axis=0)

            seeds.extend(
                (band, x)
                for x in find_peaks(histogram)
            )

        if self.pair is not None:
            band = N_BANDS - 2

            t = (
                (bounds[band] + bounds[band + 1])
                / 2
                / (h - 1)
            )

            seeds.extend(
                (band, int(np.polyval(coeff, t)))
                for coeff in self.pair
            )

        tracks = []

        for band, seed in seeds:
            if not 0 <= seed < w:
                continue

            track = self.trace(
                edges,
                band,
                seed,
                bounds,
            )

            if track is None:
                continue

            duplicate = None

            for i, old in enumerate(tracks):
                lo = max(
                    track["top"],
                    old["top"],
                ) / (h - 1)

                hi = min(
                    track["bottom"],
                    old["bottom"],
                ) / (h - 1)

                if hi <= lo:
                    continue

                difference = np.max(np.abs(
                    np.polyval(
                        track["coeff"] - old["coeff"],
                        np.linspace(lo, hi, 8),
                    )
                ))

                if difference < 18:
                    duplicate = i
                    break

            if duplicate is None:
                tracks.append(track)

            elif track["score"] > tracks[duplicate]["score"]:
                tracks[duplicate] = track

        sample_y = (
            np.array([0.76, 0.86, 0.94])
            if ROAD_MODE == "highway"
            else np.array([0.65, 0.78, 0.90])
        )

        tracks.sort(
            key=lambda track: np.polyval(
                track["coeff"],
                sample_y[1],
            )
        )

        choices = []

        for i, left in enumerate(tracks):
            for right in tracks[i + 1:]:
                pair = np.array([
                    left["coeff"],
                    right["coeff"],
                ])

                widths = np.polyval(
                    pair[1] - pair[0],
                    sample_y,
                )

                centers = np.polyval(
                    pair.mean(axis=0),
                    sample_y,
                )

                if ROAD_MODE == "highway":
                    min_width = w * 0.03
                    max_width = w * 0.95
                else:
                    min_width = w * 0.10
                    max_width = w * 1.8

                if (
                    widths.min() < min_width
                    or widths.max() > max_width
                ):
                    continue

                if (
                    centers.min() < -w * 0.10
                    or centers.max() > w * 1.10
                ):
                    continue

                score = (
                    left["score"]
                    + right["score"]
                    - abs(centers[1] - w / 2) / 40
                )

                if self.pair is not None:
                    delta = max(
                        np.max(np.abs(
                            np.polyval(
                                coeff - previous,
                                sample_y,
                            )
                        ))
                        for coeff, previous in zip(
                            pair,
                            self.pair,
                        )
                    )

                    if delta > w * 0.18:
                        continue

                    score -= delta / 12

                choices.append((score, pair))

        inferred = None

        if choices:
            raw_pair = max(
                choices,
                key=lambda item: item[0],
            )[1]

            measured_width = raw_pair[1] - raw_pair[0]

            if self.width_model is None:
                self.width_model = measured_width
            else:
                self.width_model = (
                    0.8 * self.width_model
                    + 0.2 * measured_width
                )

            self.last_pair = now
            self.pair_count += 1
            status = "BOTH"

        else:
            raw_pair = None

            if (
                self.pair is not None
                and self.width_model is not None
                and self.pair_count >= 3
            ):
                matches = []

                for track in tracks:
                    observed_y = np.linspace(
                        track["top"] / (h - 1),
                        track["bottom"] / (h - 1),
                        8,
                    )

                    differences = [
                        np.max(np.abs(
                            np.polyval(
                                track["coeff"] - previous,
                                observed_y,
                            )
                        ))
                        for previous in self.pair
                    ]

                    side = int(np.argmin(differences))

                    if (
                        differences[side] < w * 0.14
                        and abs(
                            differences[0] - differences[1]
                        ) > w * 0.04
                    ):
                        matches.append((
                            track["score"] - differences[side] / 10,
                            side,
                            track["coeff"],
                        ))

                if matches:
                    _, side, coeff = max(
                        matches,
                        key=lambda item: item[0],
                    )

                    if side == 0:
                        raw_pair = np.array([
                            coeff,
                            coeff + self.width_model,
                        ])
                    else:
                        raw_pair = np.array([
                            coeff - self.width_model,
                            coeff,
                        ])

                    center_y = 0.85

                    center_x = float(np.polyval(
                        raw_pair.mean(axis=0),
                        center_y,
                    ))

                    widths = np.polyval(
                        raw_pair[1] - raw_pair[0],
                        sample_y,
                    )

                    if (
                        widths.min() < w * 0.03
                        or not -w <= center_x <= 2 * w
                    ):
                        raw_pair = None
                    else:
                        inferred = 1 - side
                        status = (
                            "LEFT_ONLY"
                            if side == 0
                            else "RIGHT_ONLY"
                        )

            if raw_pair is None:
                if (
                    self.pair is not None
                    and now - self.last_observed <= HOLD_SECONDS
                ):
                    return self.pair.copy(), "HELD", None

                self.reset()
                return None, "LOST", None

        if self.pair is None:
            self.pair = raw_pair
        else:
            self.pair = (
                0.6 * self.pair
                + 0.4 * raw_pair
            )

        self.last_observed = now
        return self.pair.copy(), status, inferred


def draw_curve(frame, coeff, ys, color, thickness):
    h, w = frame.shape[:2]

    xs = np.polyval(coeff, ys / (h - 1))
    inside = (xs >= 0) & (xs < w)

    parts = np.split(
        np.arange(len(ys)),
        np.flatnonzero(
            np.diff(inside.astype(int))
        ) + 1,
    )

    for indices in parts:
        if len(indices) >= 2 and inside[indices[0]]:
            points = (
                np.column_stack([
                    xs[indices],
                    ys[indices],
                ])
                .astype(np.int32)
                .reshape(-1, 1, 2)
            )

            cv2.polylines(
                frame,
                [points],
                False,
                color,
                thickness,
                cv2.LINE_AA,
            )


def draw_result(frame, pair, status, inferred, top, bottom):
    h, w = frame.shape[:2]
    reference = (w // 2, int(h * 0.94))

    cv2.line(
        frame,
        (w // 2, top),
        (w // 2, bottom),
        (100, 100, 100),
        1,
    )

    is_detected = status in (
        "BOTH",
        "LEFT_ONLY",
        "RIGHT_ONLY",
    )

    error = 0.0
    target_valid = False

    if pair is not None:
        draw_top = (
            max(top, int(h * 0.76))
            if ROAD_MODE == "highway"
            else top
        )

        ys = np.arange(draw_top, bottom)

        for side, coeff in enumerate(pair):
            if status == "HELD":
                color = (130, 130, 130)
            elif side == inferred:
                color = (0, 165, 255)
            else:
                color = (0, 255, 0)

            draw_curve(frame, coeff, ys, color, 2)

        center = pair.mean(axis=0)

        draw_curve(
            frame,
            center,
            ys,
            (0, 255, 255),
            1,
        )

        target_y = int(h * 0.85)

        target_x = float(np.polyval(
            center,
            target_y / (h - 1),
        ))

        if (
            np.isfinite(target_x)
            and -w <= target_x <= 2 * w
        ):
            target = (int(target_x), target_y)
            target_valid = True
            error = reference[0] - target_x

            visible, p1, p2 = cv2.clipLine(
                (0, 0, w, h),
                reference,
                target,
            )

            if visible:
                cv2.line(
                    frame,
                    p1,
                    p2,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

            if 0 <= target_x < w:
                cv2.circle(
                    frame,
                    target,
                    5,
                    (0, 255, 255),
                    -1,
                )
        else:
            is_detected = False

    cv2.circle(
        frame,
        reference,
        5,
        (255, 0, 0),
        -1,
    )

    status_color = (
        (0, 255, 0)
        if status == "BOTH"
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

    if target_valid:
        text = f"Error: {error:.1f} px"

        if not 0 <= target_x < w:
            text += " / target offscreen"
    else:
        text = "Error: unavailable"

    cv2.putText(
        frame,
        text,
        (15, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 255, 255),
        1,
    )

    return frame, error, is_detected