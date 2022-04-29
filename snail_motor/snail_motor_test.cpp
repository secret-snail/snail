#include "snail_motor.h"

int main(int argc, char *argv[])
{
   if (gpioInitialise() < 0) {
    return 1; 
  }
  
   setup();
   
   char in = 'x';
   
   
   while (1) {
      
      std::cout << "Input Commands:\n w: forward \n s: backward \n a: turn left \n d: turn right \n l: slow \n f: fast \n h: halt \n q: quit" << std::endl;
      std::cin >> in;
      
      if (in =='w') {
         forward();
      }
      else if (in =='s') {
         backward();
      }
      else if (in =='a') {
         turnLeft();
      }
      else if (in =='d') {
         turnRight();
      }
      else if (in =='l') {
         slow();
      }
      else if (in =='f') {
         fast();
      }
      else if (in =='h') {
         halt();
      }
      else if (in =='q') {
         halt();
         break;
      }
      
   
   }
   

   gpioTerminate(); 
}
