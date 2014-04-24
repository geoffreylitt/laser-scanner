// Arduino code for laser scanning
// Geoffrey Litt

// This is a lightweight Arduino program which receives movement commands over
// serial and outputs appropriate signals to two servos to move a pan-tilt.

#include <Servo.h>

Servo ps; // phi servo
Servo ts; // theta servo

const int PHI_PIN = 9;
const int THETA_PIN = 10;

void output(int p, int t){}

void setup(){
  ts.attach(10);
  ps.attach(9);

  // initialize all the way down and turned clockwise
  ts.write(150);
  ps.write(180);

  Serial.begin(9600);
}

void loop(){
  int phi; // phi angle
  int theta; // theta angle
  char inChar;
  String serialInput = "";

  while(Serial.available() > 0){
    inChar = Serial.read();
    if(inChar == '\n'){
      phi = serialInput.substring(0, 3).toInt();
      theta = serialInput.substring(4, 7).toInt();
      ps.write(phi);
      ts.write(theta);
      serialInput = "";
      Serial.println("OK");
    }
    else{
      serialInput += inChar;
      delay(10); // this seems to be necessary to let the string have time to be reassigned in memory
    }
  }
}
