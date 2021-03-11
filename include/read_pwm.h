
struct PWM_Reading
{
    float raw = 0.0f;
    float period = 0.0f;
    unsigned long time = 0;
    float normalized = 0.0f;
    
    boolean available = false;
};

struct Calibration
{
    int min = 1000;
    int mid = 1500;
    int max = 2000;
};

struct Channel
{
    int pinNumber = 1;

    int pulseWidthMeasurement = 0;
    unsigned long measurementTime = 0;
    unsigned long pwmPeriod = 0;

    boolean previousPinState = false;
    boolean hasNewValue = false;

    byte pin_reg;
    byte pin_port;

    Calibration calibration;
};

void setupPwmRead();
boolean pwmHasNewData();
// float RC_decode(int channel);
PWM_Reading PWM_read(int channel);

void enablePinChangeDetection(byte pin);
void generatePinToRegisterMapping();

float calibrate(float Rx, int Min, int Mid, int Max);
float RC_decode(int CH);
