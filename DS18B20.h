#ifndef DS18B20_H
#define DS18B20_H

// This library is for BLDC driver on atmega 8 / 328 @ 8MHz 3.3V or 5V
// The custom timings was made, due to all timers running on 31.25 KHz
// You can change PIN and calibration value below

#include <Arduino.h>

#define OW_PIN 2  // Arduino pin D2 (PD2) for Dallas DS18B20
#define calibration 0 // Temp. calibration

volatile float currentTemperature = 0;

// --------- 1-Wire: Low-level implementation ----------

void delay_us(uint16_t us) {
    __asm__ __volatile__ (
      "1: \n\t"
      "nop\n\t"       // 1 nop = 1 cykl
      "sbiw %0,1\n\t" // 2 cykle (decrement us)
      "brne 1b\n\t"   // 2 cykle jeśli skok, 1 jeśli nie
      : "=w" (us)
      : "0" (us)
    );
}

bool ow_reset() {
  pinMode(OW_PIN, OUTPUT);
  digitalWrite(OW_PIN, LOW);
  delay_us(460);
  pinMode(OW_PIN, INPUT);
  delay_us(60);
  bool presence = !digitalRead(OW_PIN);
  delay_us(400);
  return presence;
}

void ow_write_byte(uint8_t byte) { // Universal function, more flexible
    for (uint8_t i = 0; i < 8; i++) {
        noInterrupts();
        pinMode(OW_PIN, OUTPUT);
        digitalWrite(OW_PIN, LOW);
        if (byte & 0x01) {
            pinMode(OW_PIN, INPUT); // release line for '1'
            delay_us(30);
        } else {
            delay_us(35);
            pinMode(OW_PIN, INPUT); // release line after long '0'
        }
        byte >>= 1;
        interrupts();
    }
}

/* uncomment for signal analysis, more stable, more precise, for PulseView
void ow_write_byte(uint8_t byte) { // Fixed to D2 pin
    __asm__ __volatile__ (
      "ldi r18, 8            \n\t"  // licznik bitów
      "ow_write_loop:        \n\t"
      // Przygotowanie - ustaw pin jako output i LOW
      "sbi %[ddr], %[bit]    \n\t"  // DDRD |= (1 << PD2)
      "cbi %[port], %[bit]   \n\t"  // PORTD &= ~(1 << PD2)
  
      // Sprawdzenie najmłodszego bitu
      "mov r19, %[byte]      \n\t"
      "andi r19, 0x01        \n\t"
      "cpi r19, 0            \n\t"
      "breq write_zero       \n\t"
  
      // write '1' -------------------------------------
      "cbi %[ddr], %[bit]    \n\t"  // pin jako input (pull-up)
  
      // delay 30 us (przy 8 MHz to ~120 cykli)
      "ldi r20, 100          \n\t"
      "1: dec r20            \n\t"
      "brne 1b               \n\t"
      "rjmp write_continue   \n\t"
  
      // write '0' -------------------------------------
      "write_zero:           \n\t"
  
      // delay 35 us (przy 8 MHz to ~120 cykli)
      "ldi r20, 100          \n\t"
      "2: dec r20            \n\t"
      "brne 2b               \n\t"
  
      "cbi %[ddr], %[bit]    \n\t"  // pin jako input
  
      // przesunięcie i kolejna iteracja ---------------
      "write_continue:       \n\t"
      "lsr %[byte]           \n\t"  // byte >>= 1
      "dec r18               \n\t"
      "brne ow_write_loop    \n\t"
      :
      [byte] "+r" (byte)
      :
      [port] "I" (_SFR_IO_ADDR(PORTD)),
      [ddr]  "I" (_SFR_IO_ADDR(DDRD)),
      [bit]  "I" (2) // D2 = PD2
      :
      "r18", "r19", "r20"
    );
}
*/

uint8_t ow_read_byte() { // Universal function, more flexible
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
      noInterrupts();
      pinMode(OW_PIN, OUTPUT);
      digitalWrite(OW_PIN, LOW);
      delay_us(1);
      pinMode(OW_PIN, INPUT); // release line
      delay_us(1);
      if (digitalRead(OW_PIN)) {
        byte |= (1 << i);
      }
      interrupts();
      delay_us(35); // wait for rest of timeslot
    }
    return byte;
}

/* uncomment for signal analysis, more stable, more precise, for PulseView
uint8_t ow_read_byte() { // Fixed to D2 pin
    uint8_t byte;
    __asm__ __volatile__ (
      "clr %[byte]               \n\t"   // byte = 0
      "ldi r18, 0x01             \n\t"   // maska bitu (1 << i)

      "read_loop:                \n\t"
      "sbi %[ddr], %[bit]        \n\t"   // DDR |= (1 << PD2) -> OUTPUT
      "cbi %[port], %[bit]       \n\t"   // PORT &= ~(1 << PD2) -> LOW

      // delay_us(1)
      "ldi r20, 5                \n\t"
      "2: dec r20                \n\t"
      "brne 2b                   \n\t"

      "cbi %[ddr], %[bit]        \n\t"   // DDR &= ~(1 << PD2) -> INPUT

      // delay_us(1)
      "ldi r20, 1                \n\t"
      "2: dec r20                \n\t"
      "brne 2b                   \n\t"

      // Odczyt pinu
      "sbic %[pin], %[bit]       \n\t"   // if (PIN & (1<<PD2)) skip
      "rjmp no_set               \n\t"

      "or %[byte], r18           \n\t"   // byte |= maska

      "no_set:                   \n\t"

      // delay_us(35)
      "ldi r20, 100              \n\t"
      "3: dec r20                \n\t"
      "brne 3b                   \n\t"

      "lsl r18                   \n\t"   // maska <<= 1
      "brne read_loop            \n\t"

      "com %[byte]               \n\t"   // bitowa negacja byte = ~byte

      : [byte] "=r" (byte)
      :
        [port] "I" (_SFR_IO_ADDR(PORTD)),
        [ddr]  "I" (_SFR_IO_ADDR(DDRD)),
        [pin]  "I" (_SFR_IO_ADDR(PIND)),
        [bit]  "I" (2)                 // D2 = PD2
      :
        "r18", "r20"
    );
    return byte;
}
*/

void ow_set_resolution_11bit() {
    if (!ow_reset()) return;
    ow_write_byte(0xCC);
    ow_write_byte(0x4E);
    ow_write_byte(0x4B);
    ow_write_byte(0x46);
    ow_write_byte(0x5F); // 0x1F 9-bit, 0x3F 10-bit, 0x5F 11-bit, 0x7F 12-bit
    if (!ow_reset()) return;
    ow_write_byte(0xCC);
    ow_write_byte(0x48);
    delay_us(10000); // wait 10 ms
}

bool GetID(uint8_t* buffer) {
    if (!ow_reset()) return false;
    ow_write_byte(0x33); // READ ROM
    for (uint8_t i = 0; i < 8; i++) {
        buffer[i] = ow_read_byte();
    }
    return true;
}

void startTemperatureConversion() {
    static bool once = false;
    static float lastGoodTemperature = 0;
    static bool lastReadingWasJump = false;
    if (!once) {
        ow_set_resolution_11bit();
        once = true;
    }
    if (!ow_reset()) return;
    ow_write_byte(0xCC); // Skip ROM
    ow_write_byte(0x44); // Convert T
    if (!ow_reset()) return;
    ow_write_byte(0xCC); // Skip ROM
    ow_write_byte(0xBE); // Read scratchpad
    uint8_t tempL = ow_read_byte();
    uint8_t tempH = ow_read_byte();
    int16_t rawTemp = (tempH << 8) | tempL;
    float temp = rawTemp / 16.0; // rawTemp * 0.0625 = rawTemp / 16.0;
    // --- Zabezpieczenia ---
    bool valid = true;
    // 1. Błąd -0.0625°C (może oznaczać problem z odczytem)
    if (temp == -0.0625) {
        valid = false;
    }
    // 2. Sprawdź skok 2x
    if (valid && lastGoodTemperature > 0 && temp > lastGoodTemperature * 1.5) {
    if (lastReadingWasJump) {
        lastReadingWasJump = false; // drugi raz = akceptujemy
    } else {
        valid = false;
        lastReadingWasJump = true;
    }
    } else {
        lastReadingWasJump = false;
    }
    if (valid) {
        currentTemperature = temp + calibration;
        lastGoodTemperature = temp;
    } else {
        currentTemperature = lastGoodTemperature + calibration;
    }
}

// CRC-8 (Dallas/Maxim) - Polynomial: x^8 + x^5 + x^4 + 1 = 0x31 (reflected 0x8C)
uint8_t ow_crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    while (len--) {
        uint8_t inbyte = *data++;
        for (uint8_t i = 0; i < 8; i++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return crc;
}

void startTemperatureConversionCRC() {
    static bool once = false;
    static float lastGoodTemperature = 0;
    if (!once) {
        ow_set_resolution_11bit();
        once = true;
    }
    if (!ow_reset()) return;
    ow_write_byte(0xCC); // Skip ROM
    ow_write_byte(0x44); // Convert T
    if (!ow_reset()) return;
    ow_write_byte(0xCC); // Skip ROM
    ow_write_byte(0xBE); // Read scratchpad
    uint8_t scratchpad[9];
    for (uint8_t i = 0; i < 9; i++) {
        scratchpad[i] = ow_read_byte();
    }
    if (ow_crc8(scratchpad, 8) == scratchpad[8]) {
        int16_t rawTemp = (scratchpad[1] << 8) | scratchpad[0];
        float temp = rawTemp / 16.0;
        lastGoodTemperature = temp;
        currentTemperature = temp + calibration;
    } else {
        // CRC error: return last good value
        currentTemperature = lastGoodTemperature + calibration;
    }
}

#endif