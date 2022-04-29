#include <iostream>

#include <pigpio.h>

#include <cstdlib>

#define in1 25
#define in2 23

#define in3 24
#define in4 26

#define en1 19
#define en2 13


void setup();

void forward();

void backward();

void turnLeft();

void turnRight();

void slow();

void fast();

void halt();
