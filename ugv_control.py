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
    Kp=0.01,
    Kd=0.002,
):
    """
    차선 오차로 조향 각속도를 계산하는 PD 제어입니다.

    dt:
        새 영상 결과 사이의 실제 시간 간격(초).

    previous_error=None:
        시작 또는 재출발 첫 측정이므로 D항을 0으로 처리.
    """
    values = (
        current_error,
        dt,
        Kp,
        Kd,
    )

    if (
        not all(math.isfinite(v) for v in values)
        or dt <= 0
    ):
        raise ValueError(
            "유효하지 않은 오차값, 시간 간격 또는 PD 계수"
        )

    if previous_error is None:
        derivative = 0.0

    else:
        if not math.isfinite(previous_error):
            raise ValueError(
                "이전 오차값이 유효하지 않습니다"
            )

        derivative = (
            current_error - previous_error
        ) / dt

    angular_z = (
        Kp * current_error
        + Kd * derivative
    )

    if not math.isfinite(angular_z):
        raise ValueError(
            "계산한 조향값이 유효하지 않습니다"
        )

    # 기존 조향 방향과 출력 상한 유지
    MAX_ANGULAR_Z = 1.5

    return max(
        -MAX_ANGULAR_Z,
        min(MAX_ANGULAR_Z, angular_z),
    )


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