// =========================
// L298N 핀 설정
// =========================
const int ENA = 10;
const int IN1 = 9;
const int IN2 = 8;
const int ENB = 5;
const int IN3 = 7;
const int IN4 = 6;

// =========================
// PID 설정
// =========================
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.5;

float target = 0.0;
float error = 0;
float previousError = 0;
float integral = 0;

unsigned long previousTime = 0;
unsigned long lastReceiveTime = 0; 

// =========================
// 기본 속도 및 제어 설정
// =========================
int baseSpeed = 120;
const float DEAD_ZONE = 5.0; // [개선] 이 픽셀 이내의 오차는 직진으로 간주

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  // [개선] micros()로 초기화
  previousTime = micros();
}

void loop() {
  if (Serial.available() > 0) {
    float offset = Serial.parseFloat();
    
    unsigned long currentTime = millis();
    
    // [개선 3] 정지 후 재출발 시 PID 적분값 초기화 (0.5초 이상 끊겼다 연결된 경우)
    if (currentTime - lastReceiveTime > 500) {
      integral = 0;
      previousError = target - offset;
      previousTime = micros(); // 시간도 초기화
    }
    lastReceiveTime = currentTime;

    // =========================
    // PID 계산
    // =========================
    unsigned long currentMicros = micros();
    
    // [개선 1] 마이크로초 단위로 더 정밀한 dt 계산
    float dt = (currentMicros - previousTime) / 1000000.0; 
    if (dt <= 0.0) {
      dt = 0.001; 
    }

    error = target - offset;

    // [개선 2] Deadzone 적용: 미세한 노이즈로 인한 떨림 방지
    if (abs(error) < DEAD_ZONE) {
      error = 0;
    }

    integral += error * dt;
    integral = constrain(integral, -100, 100);

    float derivative = (error - previousError) / dt;

    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

    previousError = error;
    previousTime = currentMicros;

    // =========================
    // 좌우 모터 속도 계산
    // =========================
    int leftSpeed  = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // ========================
    // 모터 구동
    // ========================
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

  // =========================
  // 안전 정지 방어 코드 
  // =========================
  if (millis() - lastReceiveTime > 500) {
    stopMotors();
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);
}