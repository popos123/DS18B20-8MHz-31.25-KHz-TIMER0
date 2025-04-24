# DS18B20-8MHz-31.25-KHz-TIMER0
This is a repository for Dallas DS18B20, running on ATmega328 with 8MHz clock and 31.25 KHz TIMER0.<br>
I was created this library only for using this temperature sensor as addon to my BLDC driver for e-scooter.<br>
There is some limitations in ATmega8 / 328 chips, so it was necessery to create this library to drive 6 MOSFETS at 31.25 KHz and using all timers (timer0, 1 and 2).<br>
The TIMER0 was changed so function like delay(), millis(), micros() are broken, it needs to create a new one or divided by 64 its readings.<br>

So this library is working with:
- One 1-Wire DS18B20 on any pin,
- 8 MHz clock at 3.3V or 5V,
- TIMER0 or TIMER1 and TIMER2 running at 31.25 KHz

To run this library just set in Your ATmega / ATtiny fusebits to run at 8 MHz and set clock in compiler e.g. in platformio.ini: board_build.f_cpu = 8000000L
Example code is in main.cpp
