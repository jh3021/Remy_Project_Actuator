#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define PIN        21  
#define NUMPIXELS  30  

// 네오픽셀 설정
Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// 함수 선언 (VS Code/C++ 필수 규칙)
void rainbow(int wait);

void setup() {
  strip.begin();           // 네오픽셀 시작
  
  // [전력 보호] 밝기를 30으로 제한 (테스트용)
  strip.setBrightness(30); 
  
  strip.show();            // 일단 다 끈 상태로 시작
}

void loop() {
  // 무지개 효과 함수 실행 (속도: 10)
  rainbow(10); 
}

// 🌈 무지개 효과 함수
void rainbow(int wait) {
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    strip.rainbow(firstPixelHue);
    strip.show();            // 실제 LED에 출력
    delay(wait);
  }
}