#include "snail_motor.h"

void setup() {
   gpioSetMode(in1, PI_OUTPUT);
   gpioSetMode(in2, PI_OUTPUT);
   
   gpioSetMode(in3, PI_OUTPUT);
   gpioSetMode(in4, PI_OUTPUT);
   
   gpioSetMode(en1, PI_OUTPUT);
   gpioSetMode(en2, PI_OUTPUT);
   
   gpioPWM(en1, 175);
   gpioPWM(en2, 255);
}

void forward() {
   gpioWrite(in1, PI_LOW);
   gpioWrite(in2, PI_HIGH);
   
   gpioWrite(in3, PI_LOW);
   gpioWrite(in4, PI_HIGH);
}

void backward() {
   gpioWrite(in1, PI_HIGH);
   gpioWrite(in2, PI_LOW);
   
   gpioWrite(in3, PI_HIGH);
   gpioWrite(in4, PI_LOW);
}

void turnLeft() {
   gpioWrite(in1, PI_HIGH);
   gpioWrite(in2, PI_LOW);
   
   gpioWrite(in3, PI_LOW);
   gpioWrite(in4, PI_HIGH);
}

void turnRight() {
   gpioWrite(in1, PI_LOW);
   gpioWrite(in2, PI_HIGH);
   
   gpioWrite(in3, PI_HIGH);
   gpioWrite(in4, PI_LOW);
}

void slow() {
   gpioPWM(en1, 100);
   gpioPWM(en2, 100);
}

void fast() {
   gpioPWM(en1, 175);
   gpioPWM(en2, 255);
}

void halt() {
   gpioWrite(in1, PI_HIGH);
   gpioWrite(in2, PI_HIGH);
   
   gpioWrite(in3, PI_HIGH);
   gpioWrite(in4, PI_HIGH);
}

