#include "Wire.h"
char buff [50];
volatile byte indx;

void setup() {
  Serial.begin(115200);
  Serial.println("Start...");
}

void loop() {
  
  if (Serial.available() > 0) {
    byte c = Serial.read();
    if (indx < sizeof buff) {
      buff [indx++] = c; // save data in the next index in the array buff
      if (c == '\r') { //check for the end of the word
        Serial.print(buff);
        indx= 0; //reset button to zero
      }
    }
  }
}