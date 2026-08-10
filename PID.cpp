// =========================
// L298N 핀 설정
// =========================

// 왼쪽 모터
const int ENA = 10;
const int IN1 = 9;
const int IN2 = 8;

// 오른쪽 모터
const int ENB = 7;
const int IN3 = 6;
const int IN4 = 5;


// ========================
// PID 설정
// ========================

float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.5;

float target = 0.0;

float error = 0;
float previousError = 0;
float integral = 0;

unsigned long previousTime = 0;


// =========================
// 기본 속도
// =========================

int baseSpeed = 110;


void setup() {

  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  previousTime = millis();
}


void loop() {

  if (Serial.available() > 0) {

    // Jetson에서 Offset 수신
    float offset = Serial.parseFloat();

    // =========================
    // PID 계산
    // =========================

    unsigned long currentTime = millis();

    float dt = (currentTime - previousTime) / 1000.0;

    if (dt <= 0) {
      dt = 0.001;
    }

    // 목표 = 0
    error = target - offset;

    // Integral
    integral += error * dt;

    // Integral windup 방지
    integral = constrain(integral, -100, 100);

    // Derivative
    float derivative = (error - previousError) / dt;

    // PID 출력
    float correction =
        Kp * error +
        Ki * integral +
        Kd * derivative;

    previousError = error;
    previousTime = currentTime;


    // =========================
    // 좌우 모터 속도 계산
    // =========================

    int leftSpeed  = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;


    // PWM 범위 제한
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);


    // =========================
    // 모터 구동
    // =========================

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    analogWrite(ENA, leftSpeed);
    analogWrite(ENB, rightSpeed);


    // =========================
    // 확인용 출력
    // =========================

    Serial.print("Offset: ");
    Serial.print(offset);

    Serial.print(" | PID: ");
    Serial.print(correction);

    Serial.print(" | L: ");
    Serial.print(leftSpeed);

    Serial.print(" | R: ");
    Serial.println(rightSpeed);
  }
}