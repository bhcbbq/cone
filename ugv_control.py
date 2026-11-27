import serial
import json
import math


def init_ugv(port="/dev/ttyACM0", baudrate=115200):
    """젯슨과 UGV02 하체를 USB 시리얼로 연결합니다."""
    try:
        ser = serial.Serial(
            port,
            baudrate,
            timeout=0,
            write_timeout=0.1,
        )

        print("[알림] UGV02와 정상적으로 연결되었습니다!")
        return ser

    except Exception as e:
        print(f"[에러] UGV02 연결 실패: {e}")
        return None


def clear_feedback_buffer(ser):
    """한 번에 최대 2048바이트만 읽어 수신 버퍼를 비웁니다."""
    count = min(ser.in_waiting, 2048)

    if count:
        ser.read(count)


def read_odometry(ser, dt):
    """
    기존 거리 측정 함수 유지.
    현재 main_ver1에서는 호출하지 않습니다.
    실제 거리 제어에 사용하기 전 별도 수정이 필요합니다.
    """
    distance_added = 0.0
    latest_speed = 0.0
    has_data = False

    while ser and ser.in_waiting > 0:
        try:
            raw_data = (
                ser.readline()
                .decode("utf-8", errors="ignore")
                .strip()
            )

            if not raw_data:
                continue

            feedback = json.loads(raw_data)

            if not isinstance(feedback, dict):
                continue

            if feedback.get("T") == 1001:
                left_speed = feedback.get("L", 0.0)
                right_speed = feedback.get("R", 0.0)

                latest_speed = (
                    left_speed + right_speed
                ) / 2.0

                has_data = True

        except json.JSONDecodeError:
            pass

        except (serial.SerialException, OSError) as e:
            print(
                "[에러] 시리얼 읽기 실패 "
                f"(오도메트리): {e}"
            )
            break

        except Exception as e:
            print(
                "[경고] 오도메트리 데이터 처리 오류: "
                f"{e}"
            )
            continue

    if has_data:
        distance_added = latest_speed * dt

    return distance_added


def calculate_pid(
    current_error,
    previous_error,
    dt,
    filtered_derivative=0.0,
    is_new_frame=True,
    Kp=0.003,
    Kd=0.004,
    D_ALPHA=0.2,
):
    """
    차선 오차로 조향 각속도를 계산하는 프레임 동기화 PD 제어입니다.

    dt:
        새 카메라 프레임 사이의 실제 시간 간격(초).

    previous_error=None:
        시작 또는 재출발 첫 측정이므로 D항을 0으로 처리.

    filtered_derivative:
        직전까지 저역통과 필터(EMA)가 적용된 오차 변화율(px/s).

    is_new_frame:
        True일 때만 새 미분값을 계산해 필터를 갱신하고,
        False일 때는 직전 필터링된 미분값을 유지해 50 Hz 제어 출력을 부드럽게 유지.
    """
    values = (
        current_error,
        filtered_derivative,
        Kp,
        Kd,
        D_ALPHA,
    )

    if not all(math.isfinite(v) for v in values):
        raise ValueError(
            "유효하지 않은 오차값 또는 PD 계수"
        )

    if not (0.0 < D_ALPHA <= 1.0):
        raise ValueError(
            "D_ALPHA는 0 초과 1 이하의 값이어야 합니다"
        )

    # 프레임 동기화 + EMA 저역통과 필터 D항 계산
    MIN_FRAME_DT = 0.005
    MAX_FRAME_DT = 0.25

    if is_new_frame:
        if previous_error is None:
            filtered_derivative = 0.0
        else:
            if not math.isfinite(previous_error):
                raise ValueError(
                    "이전 오차값이 유효하지 않습니다"
                )
            if not math.isfinite(dt) or dt <= 0.0:
                raise ValueError(
                    "유효하지 않은 프레임 시간 간격(dt)"
                )

            if MIN_FRAME_DT <= dt <= MAX_FRAME_DT:
                raw_derivative = (
                    current_error - previous_error
                ) / dt
                filtered_derivative = (
                    (1.0 - D_ALPHA) * filtered_derivative
                    + D_ALPHA * raw_derivative
                )
            else:
                # 카메라 프레임 간격이 비정상적으로 길거나 짧으면 D항 초기화
                filtered_derivative = 0.0

    # 2단계 비례 제어:
    # - ±80 px 이내: Kp
    # - ±80 px 초과: 초과분에 KP_LARGE 적용
    ERROR_BOOST_THRESHOLD = 80.0
    KP_LARGE = 0.003

    abs_err = abs(current_error)
    if abs_err <= ERROR_BOOST_THRESHOLD:
        p_term = Kp * current_error
    else:
        sign = 1.0 if current_error > 0 else -1.0
        p_term = sign * (
            Kp * ERROR_BOOST_THRESHOLD
            + KP_LARGE * (abs_err - ERROR_BOOST_THRESHOLD)
        )

    d_term = Kd * filtered_derivative
    angular_z = p_term + d_term

    if not math.isfinite(angular_z):
        raise ValueError(
            "계산한 조향값이 유효하지 않습니다"
        )

    MAX_ANGULAR_Z = 0.5
    angular_z = max(
        -MAX_ANGULAR_Z,
        min(MAX_ANGULAR_Z, angular_z),
    )

    return angular_z, filtered_derivative


def send_driving_command(ser, base_speed, angular_z):
    """선속도 X와 조향 각속도 Z를 JSON으로 전송합니다."""
    if ser:
        if not all(
            math.isfinite(v)
            for v in (base_speed, angular_z)
        ):
            raise ValueError(
                "유효하지 않은 주행 명령"
            )

        command_dict = {
            "T": 13,
            "X": base_speed,
            "Z": round(angular_z, 3),
        }

        data = (
            json.dumps(
                command_dict,
                allow_nan=False,
            )
            + "\n"
        ).encode("utf-8")

        if ser.write(data) != len(data):
            raise IOError(
                "주행 명령이 일부만 전송되었습니다"
            )


def stop_ugv(ser):
    """정지 명령을 보내고 시리얼 포트를 닫습니다."""
    if ser:
        try:
            send_driving_command(
                ser,
                0.0,
                0.0,
            )

        except Exception as e:
            print(
                f"[에러] 정지 명령 전송 실패: {e}"
            )

        finally:
            try:
                ser.close()
            except Exception:
                pass