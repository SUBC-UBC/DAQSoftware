// written by Arthur Blouin, 2021-03-13

#include <Adafruit_NeoPixel.h>

#define PIN 6 // might need to be changed later
#define NUMPIXELS 7 // number of pixels for the BAD display system

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800); // set up for the pixels, tells the system the number of pixels, the pin they are connected to, the frequency, and the colour space

void setup() {
  // put your setup code here, to run once:
  int LED_instruction = 0; // instruction encoding received from WORSE
  int Overall_instruction = 0;
  int Centre_instruction = 0;
  pixels.begin()
  pixels.setPixelColor(0, pixels.Color(255, 255, 255)); // Pixel system on currently set to white let me know if youd rather something else
}

void loop() {
  // put your main code here, to run repeatedly:
  Overall_instruction = LED_instruction / 100; // Overall buoyancy instruction
  Centre_instruction = LED_instruction % 100; // Centre of buoyancy instruction
  
  switch(Overall_instruction % 10)
  {
   case 1:
         if (Overall_instruction / 10 == 2) 
              pixels.setPixelColor(1, pixels.Color(0, 200, 0));
         else if (Overall_instruction / 10 == 1)
              pixels.setPixelColor(1, pixels.Color(196, 98, 16));
         break;
   case 2:
         if (Overall_instruction / 10 == 2) 
         {
              pixels.setPixelColor(1, pixels.Color(0, 200, 0));
              pixels.setPixelColor(2, pixels.Color(0, 200, 0));
         }
         else if (Overall_instruction / 10 == 1)
         {
              pixels.setPixelColor(1, pixels.Color(196, 98, 16));
              pixels.setPixelColor(2, pixels.Color(196, 98, 16));
         }
         break;
   case 3: // I dont know if we need this one
         if (Overall_instruction / 10 == 2) 
         {
              pixels.setPixelColor(1, pixels.Color(0, 200, 0));
              pixels.setPixelColor(2, pixels.Color(0, 200, 0));
              pixels.setPixelColor(3, pixels.Color(0, 200, 0));
         }
         else if (Overall_instruction / 10 == 1)
         {
              pixels.setPixelColor(1, pixels.Color(196, 98, 16));
              pixels.setPixelColor(2, pixels.Color(196, 98, 16));
              pixels.setPixelColor(3, pixels.Color(196, 98, 16));
         }
         break;
   default:
         // no LEDs on 
         for int(i = 1; i < 4; i++){    
              pixels.setPixelColor(i, pixels.Color(0, 0, 0));
         }
         break;
  }

  
  switch(Centre_instruction % 10)
  { 
    case 1:
         if (Overall_instruction / 10 == 1) 
              pixels.setPixelColor(4, pixels.Color(200, 0, 0));
         else if (Overall_instruction / 10 == 0)
              pixels.setPixelColor(4, pixels.Color(255, 255, 0));
         break;
    case 2:
         if (Overall_instruction / 10 == 1) 
         {
              pixels.setPixelColor(4, pixels.Color(200, 0, 0));
              pixels.setPixelColor(5, pixels.Color(200, 0, 0));
         }
         else if (Overall_instruction / 10 == 0)
         {
              pixels.setPixelColor(4, pixels.Color(255, 255, 0));
              pixels.setPixelColor(5, pixels.Color(255, 255, 0));
         }
         break;
     case 3:
         if (Overall_instruction / 10 == 1) 
         {
              pixels.setPixelColor(4, pixels.Color(200, 0, 0));
              pixels.setPixelColor(5, pixels.Color(200, 0, 0));
              pixels.setPixelColor(6, pixels.Color(200, 0, 0));
         }
         else if (Overall_instruction / 10 == 0)
         {
              pixels.setPixelColor(4, pixels.Color(255, 255, 0));
              pixels.setPixelColor(5, pixels.Color(255, 255, 0));
              pixels.setPixelColor(5, pixels.Color(255, 255, 0));
         }
         break;     
    default:
         for int(i = 4; i < 6; i++){    
              pixels.setPixelColor(i, pixels.Color(0, 0, 0));
         }
         break;            
  }
}
