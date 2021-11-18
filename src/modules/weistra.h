#include <Arduino.h>


class Weistra {
public:
    Weistra(unsigned char);
    void begin();
    void update();
    void setSpeed(unsigned char);
    void stop(); 

private:
    unsigned int  intervalTime;
    unsigned char dutyCycle;
    unsigned char trackPin;

    uint32_t prevTime;
    uint8_t counter;

    volatile uint8_t *portx_p;
    bool power ;
};

