#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <SimpleFOC.h>

// LED 설정
#define PIN        21  
#define NUMPIXELS  30  

Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
TaskHandle_t Task1;


// 모터 설정
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(16, 5, 17, 4);

// 부팅 시 초기값 15 rad/s
float target_velocity = 15.0; 
Commander command = Commander(Serial);

void doVelocity(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }


// LED 코어 0 태스크
void ledTask(void * parameter) {
  // 완전한 파란색 (R: 0, G: 0, B: 255)
  uint32_t myColor = strip.Color(0, 0, 255); 

  // 전체 LED 해당 색상으로
  strip.fill(myColor);
  strip.show();

  for(;;) { 
    // 색상이 고정되어 있으므로 통신 없이 대기
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
  }
}


// SETUP
void setup() {
  Serial.begin(115200);
  
  // --- LED 초기화 ---
  strip.begin();           
  // [밝기 설정] (최대 255)
  strip.setBrightness(150); 
  strip.show();            

  // --- 모터 초기화 ---
  driver.voltage_power_supply = 24;
  driver.init();
  motor.linkDriver(&driver);

  // 전압 제한
  motor.voltage_limit = 1.5; 
  
  // 속도 제한
  motor.velocity_limit = 20.0;

  motor.controller = MotionControlType::velocity_openloop;
  motor.init();

  command.add('V', doVelocity, (char*)"Target Velocity (rad/s)");
  command.add('L', doLimit, (char*)"Voltage Limit (V)");

  // --- 듀얼 코어 태스크 생성 (LED) ---
  xTaskCreatePinnedToCore(
    ledTask,   
    "Task1",   
    10000,     
    NULL,      
    1,         
    &Task1,    
    0          
  );

  Serial.println("--- Motor & Solid Bright Blue LED Ready ---");
  Serial.println("Running automatically at V=15");
  Serial.println("Commands:");
  Serial.println("  V[값] : 속도 변경 (예: V5, V-10)");
  Serial.println("  L[값] : 전압 제한 변경 (예: L4.5, L2)");
}


// LOOP (Core 1)
void loop() {
  motor.move(target_velocity);
  command.run();
}