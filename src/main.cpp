#include <Arduino.h>
#include <SPI.h>
#include "MT6835.h"

// 요청하신 핀 번호 설정 반영
#define CS_PIN 13
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

// 기타 설정
#define SPI_FREQ_MOT   1000000   // SPI 통신 속도 (1MHz)
#define ABZ_A_PIN_MOT  34        // 인코더 A상 핀 (기존 유지, 필요시 변경)
#define ABZ_B_PIN_MOT  35        // 인코더 B상 핀 (기존 유지, 필요시 변경)

// Magnetic Encoder Class (CS_PIN을 새로 정의한 이름으로 적용)
MT6835 mt6835(CS_PIN, SPI_FREQ_MOT, ABZ_A_PIN_MOT, ABZ_B_PIN_MOT);

void setup() {
    Serial.begin(115200); 
    while(!Serial) {} // Wait for Serial to initialize

    // ESP32의 SPI 버스를 설정한 핀으로 명시적 초기화 (SCK, MISO, MOSI, SS)
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

    // Encoder initialization
    mt6835.initialize();

    delay(1000);
    Serial.println("MT6835 Encoder Ready!");
}

void loop() { 
    // Absolute Angle
    float abs = mt6835.readAbsAng(); 

    // Incremental Angle
    float inc = mt6835.readIncAng(); 

    // 시리얼 플로터나 모니터에서 값을 보기 위해 출력
    Serial.print(">abs: ");  
    Serial.print(abs, 5);  
    Serial.print("\t>inc: ");    
    Serial.println(inc, 5); 

    // 시리얼 모니터가 너무 빠르게 지나가지 않도록 약간의 딜레이
    delay(10);
}