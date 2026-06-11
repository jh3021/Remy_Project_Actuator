#include <Arduino.h>
#include <SimpleFOC.h>

// 모터 극쌍수 (RI50 = 7)
BLDCMotor motor = BLDCMotor(7);

// A:16, B:5, C:17, EN:4
BLDCDriver3PWM driver = BLDCDriver3PWM(16, 5, 17, 4);

// 목표 속도를 저장할 변수 (초기값: 0 rad/s - 정지 상태로 시작)
float target_velocity = 0;

// 시리얼 커맨드 객체 생성
Commander command = Commander(Serial);

// 커맨드 콜백 함수 생성
// V가 입력되면 target_velocity 값을 변경
void doVelocity(char* cmd) { command.scalar(&target_velocity, cmd); }
// L이 입력되면 motor.voltage_limit 값을 변경
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void setup() {
  Serial.begin(115200);
  
  // 드라이버 설정
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);

  // 초기 전압 제한 설정
  motor.voltage_limit = 1.5; 
  motor.velocity_limit = 3;

  // 제어 모드: 속도 오픈 루프
  motor.controller = MotionControlType::velocity_openloop;
  
  motor.init();

  // 커맨드 문자에 콜백 함수 연결
  command.add('V', doVelocity, (char*)"Target Velocity (rad/s)");
  command.add('L', doLimit, (char*)"Voltage Limit (V)");

  Serial.println("--- Motor Ready ---");
  Serial.println("Commands:");
  Serial.println("  V[값] : 속도 변경 (예: V5, V-10)");
  Serial.println("  L[값] : 전압 제한 변경 (예: L4.5, L2)");
}

void loop() {
  // 설정된 목표 속도로 모터 구동
  motor.move(target_velocity);
  
  // 시리얼 모니터에서 들어오는 명령을 계속 확인하고 실행
  command.run();
}