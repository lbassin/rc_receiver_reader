#include <Arduino.h>
#include <Servo.h>
#include "read_pwm.h"

Servo myservo;
// int pos = 0;

void setup()
{
    myservo.attach(10);
    Serial.begin(9600);
    setupPwmRead();
}

void loop()
{
    // move(116); // Middle
    // move(105); // Left
    // move(127); // Right

    if (pwmHasNewData())
    {
        PWM_Reading steeringRemote = PWM_read(1);
        PWM_Reading speedRemote = PWM_read(2);
        PWM_Reading throttleRemote = PWM_read(3);

        // long steeringValue = map(steeringRemote.normalized * 1000, -1000, 1000, 100, 132);
        // long throttleValue = map(throttleRemote.normalized * 1000, -1000, 1000, 10, 170);
        long speedValue = map(speedRemote.normalized * 1000, -1000, 1000, 0, 100);

        boolean isReverse = throttleRemote.normalized < 0;
        float throttleAbsolute = (abs(throttleRemote.normalized) / 100.0f) * speedValue;
        
        
        float test = (throttleAbsolute * (isReverse ? -1 : 1))*1000;

        float throttleValue = map(test, -1000, 1000, 10, 170);

        Serial.println(throttleValue);

        myservo.write(throttleValue);


        // steeringServo.write(steeringValue);
        // throttleServo.write(throttleValue);
    }
}
