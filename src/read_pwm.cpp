#include <Arduino.h>
#include "read_pwm.h"

#define REGISTER_PORT_B 0
#define REGISTER_PORT_C 1
#define REGISTER_PORT_D 2

const int pwmInputPins[] = {2, 3, 4, 5, 6, 7, 8, 9};

int RC_min[8] = {1024, 1064, 1064, 1064, 1068, 1420, 1000, 1000};
int RC_mid[8] = {1444, 1492, 1492, 1492, 1492, 1488, 1500, 1500};
int RC_max[8] = {1872, 1916, 1908, 1912, 1916, 1594, 2000, 2000};

const int numberChannels = sizeof(pwmInputPins) / sizeof(int);

volatile int pulseWidthMeasurements[numberChannels];

volatile boolean previousPinStates[numberChannels];      // an array used to determine whether a pin has gone low-high or high-low
volatile unsigned long interuptionTime;                  // the time of the current pin change interrupt
volatile unsigned long measurementTimes[numberChannels]; // an array to store the start time of each PWM pulse

volatile boolean pinHasNewValue[numberChannels]; // flag whenever new data is available on each pin
volatile boolean hasNewdataFlag;                 // flag when all RC receiver channels have received a new pulse

unsigned long pwmPeriod[numberChannels]; // period, mirco sec, between two pulses on each pin

byte pwmPIN_reg[numberChannels];  // each of the input pins expressed as a position on it's associated port register
byte pwmPIN_port[numberChannels]; // identify which port each input pin belongs to (0 = PORTB, 1 = PORTC, 2 = PORTD)

const int size_RC_min = sizeof(RC_min) / sizeof(int); // measure the size of the calibration and failsafe arrays
const int size_RC_mid = sizeof(RC_mid) / sizeof(int);
const int size_RC_max = sizeof(RC_max) / sizeof(int);

void enablePinChangeDetection(byte pin)
{
    // code from http://playground.arduino.cc/Main/PinChangeInterrupt

    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); // enable pin
    PCIFR |= bit(digitalPinToPCICRbit(pin));                   // clear any outstanding interrupt
    PCICR |= bit(digitalPinToPCICRbit(pin));                   // enable interrupt for the group
}

void generatePinToRegisterMapping()
{
    for (int i = 0; i < numberChannels; i++)
    {
        // determine which port and therefore ISR (PCINT0_vect, PCINT1_vect or PCINT2_vect) each pwmInputPins belongs to.
        pwmPIN_port[i] = REGISTER_PORT_C; // pin belongs to PCINT1_vect (PORT C)
        if (pwmInputPins[i] >= 0 && pwmInputPins[i] <= 7)
            pwmPIN_port[i] = REGISTER_PORT_D; // pin belongs to PCINT2_vect (PORT D)
        else if (pwmInputPins[i] >= 8 && pwmInputPins[i] <= 13)
            pwmPIN_port[i] = REGISTER_PORT_B; // pin belongs to PCINT0_vect (PORT B)

        // covert the pin number (i.e. pin 11 or pin A0) to the pin position in the port register. There is most likely a better way of doing this using a macro...
        // (Reading the pin state directly from the port registers speeds up the code in the ISR)

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

// D8 - 13
ISR(PCINT0_vect)
{
    interuptionTime = micros();

    for (int i = 0; i < numberChannels; i++)
    {
        if (pwmPIN_port[i] != REGISTER_PORT_B)
        {
            continue;
        }

        if (previousPinStates[i] == 0 && PINB & pwmPIN_reg[i]) // From LOW to HIGH (Start of pulse)
        {
            previousPinStates[i] = 1;
            pwmPeriod[i] = interuptionTime - measurementTimes[i];
            measurementTimes[i] = interuptionTime;
        }
        else if (previousPinStates[i] == 1 && !(PINB & pwmPIN_reg[i])) // From HIGH to LOW (End of pulse)
        {
            previousPinStates[i] = 0;
            pulseWidthMeasurements[i] = interuptionTime - measurementTimes[i];
            pinHasNewValue[i] = HIGH;

            if (i + 1 == numberChannels)
            {
                hasNewdataFlag = HIGH; // Is it working?
            }
        }
    }
}

// A0 - 5
ISR(PCINT1_vect)
{
    interuptionTime = micros();

    for (int i = 0; i < numberChannels; i++)
    {
        if (pwmPIN_port[i] != REGISTER_PORT_C)
        {
            continue;
        }

        if (previousPinStates[i] == 0 && PINC & pwmPIN_reg[i]) // From LOW to HIGH (Start of pulse)
        {
            previousPinStates[i] = 1;
            pwmPeriod[i] = interuptionTime - measurementTimes[i];
            measurementTimes[i] = interuptionTime;
        }
        else if (previousPinStates[i] == 1 && !(PINC & pwmPIN_reg[i])) // From HIGH to LOW (End of pulse)
        {
            previousPinStates[i] = 0;
            pulseWidthMeasurements[i] = interuptionTime - measurementTimes[i];
            pinHasNewValue[i] = HIGH;

            if (i + 1 == numberChannels)
            {
                hasNewdataFlag = HIGH; // Is it working?
            }
        }
    }
}

// D0 - 7
ISR(PCINT2_vect)
{
    Serial.println("interupt");
    interuptionTime = micros();

    for (int i = 0; i < numberChannels; i++)
    {
        if (pwmPIN_port[i] == REGISTER_PORT_D)
        {
            continue;
        }

        if (previousPinStates[i] == 0 && PIND & pwmPIN_reg[i]) // From LOW to HIGH (Start of pulse)
        {
            previousPinStates[i] = 1;
            pwmPeriod[i] = interuptionTime - measurementTimes[i];
            measurementTimes[i] = interuptionTime;
        }
        else if (previousPinStates[i] == 1 && !(PIND & pwmPIN_reg[i])) // From HIGH to LOW (End of pulse)
        {
            previousPinStates[i] = 0;
            pulseWidthMeasurements[i] = interuptionTime - measurementTimes[i];
            pinHasNewValue[i] = HIGH;

            if (i + 1 == numberChannels)
            {
                hasNewdataFlag = HIGH; // Is it working?
            }
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
        calibrated = map(Rx, Mid, Max, 0, 1000); // map from 0% to 100% in one direction
    }
    else if (Rx == 0)
    {
        calibrated = 0; // neutral
    }
    else
    {
        calibrated = map(Rx, Min, Mid, -1000, 0); // map from 0% to -100% in the other direction
    }

    return calibrated * 0.001f;
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

void print_RCpwm()
{ // display the raw RC Channel PWM Inputs
    for (int i = 0; i < numberChannels; i++)
    {
        //Serial.print(" ch");Serial.print(i+1);
        Serial.print("  ");
        if (pulseWidthMeasurements[i] < 1000)
            Serial.print(" ");
        Serial.print(pulseWidthMeasurements[i]);
    }
    Serial.println("");
}