import time

import Jetson.GPIO as GPIO


OUTPUT_PIN = 29  # WAT LV1에 연결
INPUT_PIN = 18   # WAT LV2에 연결


GPIO.setmode(GPIO.BOARD)
GPIO.setup(OUTPUT_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(INPUT_PIN, GPIO.IN)

print("WAT 루프백 시험을 시작합니다. 종료하려면 Ctrl+C를 누르세요.")
print("정상이면 출력과 입력이 모두 0, 다음에는 모두 1로 표시됩니다.")

try:
    while True:
        GPIO.output(OUTPUT_PIN, GPIO.LOW)
        time.sleep(0.5)
        low_input = GPIO.input(INPUT_PIN)
        print(f"출력=0, 입력={low_input}")

        GPIO.output(OUTPUT_PIN, GPIO.HIGH)
        time.sleep(0.5)
        high_input = GPIO.input(INPUT_PIN)
        print(f"출력=1, 입력={high_input}")

except KeyboardInterrupt:
    print("\n시험을 종료합니다.")

finally:
    GPIO.output(OUTPUT_PIN, GPIO.LOW)
    GPIO.cleanup()