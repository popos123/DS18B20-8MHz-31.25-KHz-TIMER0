#include <Arduino.h>
#include <DS18B20.h>

unsigned long previousMillis = 0;
const long interval = 1000;

void setup() {
    // Fast PWM, prescaler 1 (31.25 kHz)
    TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0A1) | (1 << COM0B1); // TIMER0 (pins 5 & 6)
    TCCR0B = (1 << CS00);

    TCCR1A = (1 << WGM10) | (1 << COM1A1) | (1 << COM1B1); // TIMER1 (pins 9 & 10)
    TCCR1B = (1 << WGM12) | (1 << CS10);

    TCCR2A = (1 << WGM21) | (1 << WGM20) | (1 << COM2A1) | (1 << COM2B1); // TIMER2 (pins 3 & 11)
    TCCR2B = (1 << CS20);

    Serial.begin(200000);

    uint8_t rom[8];
    if (GetID(rom)) {
    Serial.print("ROM ID: ");
    for (int i = 0; i < 8; i++) {
        if (rom[i] < 0x10) Serial.print("0");
        Serial.print(rom[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    } else {
        Serial.println("Sensor not found.");
    }
}

void loop() {
    unsigned long currentMillis = millis() * (1.0 / 64.0);
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        startTemperatureConversion();
        Serial.print("Temp.: ");
        Serial.print(currentTemperature, 1);
        startTemperatureConversionCRC();
        Serial.print(" TempCRC.: ");
        Serial.println(currentTemperature, 1);
    }
}