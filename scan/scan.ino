// Arduino code for laser scanning
// Geoffrey Litt


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

/*
void loop() 
{ 
  for(pos = 180; pos>=1; pos -= 5)     // goes from 180 degrees to 0 degrees 
  {                                
    h.write(pos);              // tell servo to go to position in variable 'pos' 
    //v.write(pos);
    Serial.println(pos);
    delay(1000);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 0; pos < 180; pos += 5)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    h.write(pos);              // tell servo to go to position in variable 'pos' 
    //v.write(pos);
    Serial.println(pos);
    delay(1000);                       // waits 15ms for the servo to reach the position 
  } 
}
*/
