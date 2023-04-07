#ifndef Motors_h
#define Motors_h

#include <Arduino.h>

#define V3

#ifndef V3

#define pin_PWMA 6
#define pin_PWMB 5
#define pin_AIN1 8
#define pin_AIN2 7
#define pin_BIN1 9
#define pin_BIN2 10
#define pin_STBY 11

#endif
#ifdef V3

#define pin_PWMA 4
#define pin_PWMB 9  /* this was originally 9      became 11*/
#define pin_AIN1 6
#define pin_AIN2 5
#define pin_BIN1 7
#define pin_BIN2 8
#define pin_STBY 14

#endif

class Motors {

    public:

        void enableMotors();
        void disableMotors();
        void setLeftMotorSpeed(int speed);
        void setRightMotorSpeed(int speed);
        void setMotorsSpeed(int speed);
        void setLeftMotorDirection(bool forward);
        void setRightMotorDirection(bool forward);
        void setMotorsDirection(bool forward);

};

extern Motors* motors_instance;

#endif