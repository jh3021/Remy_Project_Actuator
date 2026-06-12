#include <Arduino.h>
#include <SimpleFOC.h>
#include <SPI.h>
#include "MT6835.h"

// --- 하드웨어 핀 및 통신 설정 ---
#define CS_PIN 13
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23
#define SPI_FREQ_MOT 1000000
#define ABZ_A_PIN_MOT 34
#define ABZ_B_PIN_MOT 35

// --- 객체 생성 ---
// 1. 모터 및 드라이버 (RI50 = 7극쌍)
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(16, 5, 17, 4);

// 2. MT6835 커스텀 센서 객체
MT6835 mt6835(CS_PIN, SPI_FREQ_MOT, ABZ_A_PIN_MOT, ABZ_B_PIN_MOT);

// --- SimpleFOC 연동용 콜백 함수 ---
// SimpleFOC가 이 함수를 통해 MT6835의 절대 각도(rad) 읽음
float readMT6835() {
  return mt6835.readAbsAng();
}

// 3. GenericSensor를 이용해 커스텀 함수를 SimpleFOC에 바인딩
GenericSensor sensor = GenericSensor(readMT6835, nullptr);

// --- 커맨더 설정 ---
float target_angle_arm = 0;
Commander command = Commander(Serial);
void doAngle(char* cmd) { 
  float target_deg = 0;
  command.scalar(&target_deg, cmd);
  
  target_angle_arm = target_deg * (PI / 180.0); 
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  SimpleFOCDebug::enable(&Serial);

  // SPI 버스 핀 지정 및 명시적 초기화
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  // MT6835 센서 자체 초기화 (내부 인터럽트 및 해상도 설정 포함)
  mt6835.initialize();
  
  // SimpleFOC 센서 객체 초기화 및 모터에 연결
  sensor.init();
  motor.linkSensor(&sensor);

  // 드라이버 설정
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);

  // 제어 모드 설정 (전압 기반 위치 제어)
  motor.controller = MotionControlType::angle;
  motor.torque_controller = TorqueControlType::voltage;

  // 제한 설정
  motor.voltage_limit = 1.5;
  motor.velocity_limit = 50.0;
  motor.voltage_sensor_align = 5.0; // 정렬용 전압

  // PID 튜닝
  motor.P_angle.P = 20.0;
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1.0;

  // FOC 초기화 및 영점 정렬
  motor.init();
  motor.initFOC();

  command.add('A', doAngle, (char*)"Target Angle (rad)");
  Serial.println("--- MT6835 Closed Loop Ready ---");
}

// 타이머를 위한 변수 추가 (loop 함수 밖에 두거나 static으로 선언)
unsigned long last_plot_time = 0;

void loop() {
  motor.loopFOC();
  motor.move(target_angle_arm * 23.0);
  command.run();

  if (millis() - last_plot_time > 50) {
    last_plot_time = millis();
    
    // 라디안 단위에 (180 / PI)를 곱해서 도(Degree) 단위로 변환
    float target_deg = target_angle_arm * (180.0 / PI);
    float actual_deg = (motor.shaft_angle / 23.0) * (180.0 / PI);

    // Teleplot으로 변환된 '도(Degree)' 값 전송
    Serial.print(">Target_Deg:");
    Serial.println(target_deg);
    
    Serial.print(">Actual_Deg:");
    Serial.println(actual_deg);
  }
}