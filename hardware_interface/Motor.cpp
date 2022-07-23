
#include "Arduino.h"
#include "Motor.h"


Motor::Motor(int plus, int minus, int en_a, int en_b) {
  pinMode(plus,OUTPUT);
  pinMode(minus,OUTPUT);
  pinMode(en_a,INPUT_PULLUP);
  pinMode(en_b,INPUT_PULLUP);
  Motor::plus = plus;
  Motor::minus = minus;
  Motor::en_a = en_a;
  Motor::en_b = en_b;
}

void Motor::rotate(int value) {
  if(value>=0){
    //Max Voltage with 11.1V battery with 12V required
    //(12/11.1)*255 ~=275
//    Serial.println("called");
//    Serial.println(plus);
    int out = map(value, 0, 100, 0, 275);
    analogWrite(plus,out);
    digitalWrite(minus,LOW);
  }else{
    //Max Voltage with 11.1V battery with 12V required
    //(12/11.1)*255 ~=275
    int out = map(value, 0, -100, 0, 275);
    analogWrite(plus,out);
    digitalWrite(minus,HIGH);
  }
}
