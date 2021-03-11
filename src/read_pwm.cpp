#include <Arduino.h>
#include "read_pwm.h"

#define REGISTER_PORT_B 0
#define REGISTER_PORT_C 1
#define REGISTER_PORT_D 2

const int pwmInputPins[] = {2, 3, 4, 5, 6, 7};

int RC_min[6] = {1068, 1068, 1068, 1064, 1068, 1420};
int RC_mid[6] = {1488, 1488, 1488, 1492, 1492, 1488};
int RC_max[6] = {1920, 1920, 1908, 1912, 1916, 1594};

const int numberChannels = sizeof(pwmInputPins) / sizeof(int);

volatile unsigned long interuptionTime;
volatile boolean hasNewdataFlag;

volatile int pulseWidthMeasurements[numberChannels];
volatile boolean previousPinStates[numberChannels];
volatile unsigned long measurementTimes[numberChannels];
volatile boolean pinHasNewValue[numberChannels];
unsigned long pwmPeriod[numberChannels];
byte pwmPIN_reg[numberChannels];
byte pwmPIN_port[numberChannels];

const int size_RC_min = sizeof(RC_min) / sizeof(int);
const int size_RC_mid = sizeof(RC_mid) / sizeof(int);
const int size_RC_max = sizeof(RC_max) / sizeof(int);

void enablePinChangeDetection(byte pin)
{
    // code from http://playground.arduino.cc/Main/PinChangeInterrupt

    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
    PCIFR |= bit(digitalPinToPCICRbit(pin));
    PCICR |= bit(digitalPinToPCICRbit(pin));
}

void generatePinToRegisterMapping()
{
    for (int i = 0; i < numberChannels; i++)
    {
        pwmPIN_port[i] = REGISTER_PORT_C;

        if (pwmInputPins[i] >= 0 && pwmInputPins[i] <= 7)
        {
            pwmPIN_port[i] = REGISTER_PORT_D;
        }
        else if (pwmInputPins[i] >= 8 && pwmInputPins[i] <= 13)
        {
            pwmPIN_port[i] = REGISTER_PORT_B;
        }

        if (pwmInputPins[i] == 0 || pwmInputPins[i] == A0 || pwmInputPins[i] == 8)
            pwmPIN_reg[i] = 0b00000001;
        else if (pwmInputPins[i] == 1 || pwmInputPins[i] == A1 || pwmInputPins[i] == 9)
            pwmPIN_reg[i] = 0b00000010;
        else if (pwmInputPins[i] == 2 || pwmInputPins[i] == A2 || pwmInputPins[i] == 10)
            pwmPIN_reg[i] = 0b00000100;
        else if (pwmInputPins[i] == 3 || pwmInputPins[i] == A3 || pwmInputPins[i] == 11)
            pwmPIN_reg[i] = 0b00001000;
        else if (pwmInputPins[i] == 4 || pwmInputPins[i] == A4 || pwmInputPins[i] == 12)
            pwmPIN_reg[i] = 0b00010000;
        else if (pwmInputPins[i] == 5 || pwmInputPins[i] == A5 || pwmInputPins[i] == 13)
            pwmPIN_reg[i] = 0b00100000;
        else if (pwmInputPins[i] == 6)
            pwmPIN_reg[i] = 0b01000000;
        else if (pwmInputPins[i] == 7)
            pwmPIN_reg[i] = 0b10000000;
    }
}

void setupPwmRead()
{
    for (int i = 0; i < numberChannels; i++)
    {
        enablePinChangeDetection(pwmInputPins[i]);
    }

    generatePinToRegisterMapping();
}

ISR(PCINT0_vect)
{
    interuptionTime = micros();

    for (int i = 0; i < numberChannels; i++)
    {
        if (pwmPIN_port[i] != REGISTER_PORT_B)
        {
            continue;
        }

        if (previousPinStates[i] == 0 && PINB & pwmPIN_reg[i])
        {
            previousPinStates[i] = 1;
            pwmPeriod[i] = interuptionTime - measurementTimes[i];
            measurementTimes[i] = interuptionTime;
        }
        else if (previousPinStates[i] == 1 && !(PINB & pwmPIN_reg[i]))
        {
            previousPinStates[i] = 0;
            pulseWidthMeasurements[i] = interuptionTime - measurementTimes[i];
            pinHasNewValue[i] = HIGH;
            if (i + 1 == numberChannels)
                hasNewdataFlag = HIGH;
        }
    }
}

ISR(PCINT1_vect)
{
    interuptionTime = micros();

    for (int i = 0; i < numberChannels; i++)
    {
        if (pwmPIN_port[i] != REGISTER_PORT_C)
        {
            continue;
        }

        if (previousPinStates[i] == 0 && PINC & pwmPIN_reg[i])
        {
            previousPinStates[i] = 1;
            pwmPeriod[i] = interuptionTime - measurementTimes[i];
            measurementTimes[i] = interuptionTime;
        }
        else if (previousPinStates[i] == 1 && !(PINC & pwmPIN_reg[i]))
        {
            previousPinStates[i] = 0;
            pulseWidthMeasurements[i] = interuptionTime - measurementTimes[i];
            pinHasNewValue[i] = HIGH;
            if (i + 1 == numberChannels)
                hasNewdataFlag = HIGH;
        }
    }
}

ISR(PCINT2_vect)
{
    interuptionTime = micros();

    for (int i = 0; i < numberChannels; i++)
    {
        if (pwmPIN_port[i] != REGISTER_PORT_D)
        {
            continue;
        }

        if (previousPinStates[i] == 0 && PIND & pwmPIN_reg[i])
        {
            previousPinStates[i] = 1;
            pwmPeriod[i] = interuptionTime - measurementTimes[i];
            measurementTimes[i] = interuptionTime;
        }
        else if (previousPinStates[i] == 1 && !(PIND & pwmPIN_reg[i]))
        {
            previousPinStates[i] = 0;
            pulseWidthMeasurements[i] = interuptionTime - measurementTimes[i];
            pinHasNewValue[i] = HIGH;
            if (i + 1 == numberChannels)
                hasNewdataFlag = HIGH;
        }
    }
}

boolean pwmHasNewData()
{
    boolean hasNewData = hasNewdataFlag;
    hasNewdataFlag = LOW;

    return hasNewData;
}

float calibrate(float Rx, int Min, int Mid, int Max)
{
    float calibrated;

    if (Rx >= Mid)
    {
        calibrated = map(Rx, Mid, Max, 0, 1000);
    }
    else if (Rx == 0)
    {
        calibrated = 0; // neutral
    }
    else
    {
        calibrated = map(Rx, Min, Mid, -1000, 0);
    }

    if (calibrated > 1000)
    {
        calibrated = 1000;
    }
    else if (calibrated < -1000)
    {
        calibrated = -1000;
    }

    return calibrated * 0.001f;
    // return calibrated;
}

float RC_decode(int channel)
{
    if (channel < 1 || channel > numberChannels)
    {
        return 0.0f;
    }

    int portIndex = channel - 1;

    int Min;
    if (channel <= size_RC_min)
        Min = RC_min[portIndex];
    else
        Min = 1000;

    int Mid;
    if (channel <= size_RC_mid)
        Mid = RC_mid[portIndex];
    else
        Mid = 1500;

    int Max;
    if (channel <= size_RC_max)
        Max = RC_max[portIndex];
    else
        Max = 2000;

    return calibrate(pulseWidthMeasurements[portIndex], Min, Mid, Max);
}

PWM_Reading PWM_read(int channel)
{
    PWM_Reading reading;

    if (channel < 1 && channel > numberChannels)
    {
        return reading;
    }

    int indexPort = channel - 1;

    if ((reading.available = pinHasNewValue[indexPort]) == LOW)
    {
        return reading;
    }

    noInterrupts();
    reading.time = measurementTimes[indexPort];
    reading.raw = pulseWidthMeasurements[indexPort];
    reading.period = pwmPeriod[indexPort];
    reading.normalized = RC_decode(channel); // @TODO: Refactor
    interrupts();

    pinHasNewValue[indexPort] = LOW;

    return reading;
}

float pwmPeriodToFrequency(float period)
{
    return 1000000.0f / period;
}

float pwmPeriodToDuty(float raw, float period)
{
    return raw / period;
}