
struct PWM_Reading {
    float raw = 0.0f;
    float period = 0.0f;
    unsigned long time = 0;
    boolean available = false;
};

void setupPwmRead();
boolean pwmHasNewData();
float RC_decode(int channel);
PWM_Reading PWM_read(int channel);
void print_RCpwm();
