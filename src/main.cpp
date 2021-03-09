#include <Arduino.h>
#include "read_pwm.h"

void setup()
{
    Serial.begin(9600);
    setupPwmRead();
}

void loop()
{
    if (pwmHasNewData())
    {
        PWM_Reading reading = PWM_read(1);
        Serial.print(reading.raw);
        Serial.print(" ");
        Serial.println(RC_decode(1));
    }
}
