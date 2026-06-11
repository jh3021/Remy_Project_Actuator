#pragma once
#include <Arduino.h> // ESP32에서 TWO_PI 등 아두이노 기본 상수를 사용하기 위해 추가
#include <SPI.h>

class MT6835 {
public:

    MT6835(uint8_t CS_PIN, uint32_t SPI_FREQ, uint8_t A_PIN, uint8_t B_PIN);
    
    void initialize();
    int32_t readRawAbsAng();
    int32_t readRawIncAng();
    float readAbsAng();
    float readIncAng();
    float readAngVelRaw();
    float readAngVelFilt(float ALPHA_FILT); // ALPHA_FILT -> 0.0 = stark geglättet. ALPHA_FILT -> 1.0 = kein Filter

private:
    //SPI Settings
        SPIClass* SPI; //Pointer on SPI interface
        uint8_t CS_PIN;
        uint32_t SPI_FREQ;
        SPISettings SPI_SETTINGS;

    // SPI Commands and Registers
        static constexpr uint8_t BURST_CMD = 0b1010; // burst command 4 bit
        static constexpr uint8_t READ_CMD  = 0b0011; // burst command 4 bit
        static constexpr uint8_t WRITE_CMD = 0b0110; // burst command 4 bit
        static constexpr uint8_t ANGLE_REG_MSB   = 0x003; // ANGLE[20:13]
        static constexpr uint8_t ABZ_RES_REG_MSB = 0x007; // ABZ_RES[13:6]
        static constexpr uint8_t ABZ_RES_REG_LSB = 0x008; // ANGLE[5:0]

    // Absolute Angle Parameters
        static constexpr int32_t COUNTS_PER_REV = (1 << 21); // 21 bit
        static constexpr float RAD_PER_LSB      = TWO_PI / COUNTS_PER_REV; // 21 Bit LSB

    // Incremental Angle Parameters
        uint8_t A_PIN;
        uint8_t B_PIN;
        long LAST_TICKS;
        float INCR_ANGLE;
        void setABZResolution(bool ABZ_ON, bool SWAP_DIR, uint16_t ABZ_REZ);
        uint16_t getABZResolution();
        static constexpr uint16_t ABZ_RES_VAL = 0x3FFF; // counts per revolution
        static constexpr int TICKS_PER_REV    = (ABZ_RES_VAL+1)*4;
        static constexpr float TICK_ANGLE     = TWO_PI / TICKS_PER_REV;
        float ALPHA_FILT;

    
    // --- ISR-geschützte Variablen ---
        volatile long position;
        volatile uint32_t lastCycle;
        volatile uint32_t lastDt;
        volatile int lastDir;
        volatile int lastA;
        volatile int lastB;
        volatile long pos;
        volatile int dir;
        volatile uint32_t dt;
        volatile uint32_t lastPulse;
        float ANG_VEL_RAW;
        float ANG_VEL_FILT;

    // neu 

        // History für Geschwindigkeit pro Instanz
        long oldPosRaw;
        uint32_t oldTimeRaw;
        
        // ISR Helfer
        void encoderISR(); 
        //neu
        static void encoderISRWrapper1();
        static void encoderISRWrapper2();

        // zwei statische Zeiger für zwei Encoder
        static MT6835* instance1;
        static MT6835* instance2;

        // static void encoderISRWrapper();  // static für attachInterrupt
        // static MT6835* instance; //alt
        
        // ESP32 호환 DWT-Counter
        static inline uint32_t getCycleCount() { return ESP.getCycleCount(); }

};