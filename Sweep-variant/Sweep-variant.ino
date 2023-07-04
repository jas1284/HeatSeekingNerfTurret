/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
unsigned long time_1 = 30000;


void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  pinMode(A5, INPUT);
  myservo.write(15);
  delay(1500);
  Serial.begin(9600);
}

void loop() {
  int value = analogRead(A5);
  //Serial.println(value);
  //Serial.println(millis() - time_1);
  if((value > 35) && (millis() - time_1 > 30000)){
    time_1 = millis();
    myservo.write(80);
    delay(750);
    myservo.write(15);
    delay(500);
  }
}
