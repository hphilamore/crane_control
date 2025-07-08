/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

Servo myservo;  // create Servo object to control a servo

int button_pin = 2;  

int val;           // value to send to servo
int button_status; // status of push button

unsigned long t_start, t_end, period; // variables to calculate length of button press

void setup() {
  pinMode(button_pin, INPUT_PULLUP);  // internal pullup on button pin
  myservo.attach(9);                  // attaches the servo on pin 9 to the Servo object
  Serial.begin(9600);
}

void loop() {

  // get the stats of the push button
  button_status = digitalRead(button_pin);            // reads the value of the potentiometer (value between 0 and 1023)
   
  // if the button is not pressed, do nothing 
  while (button_status == 1){
    val = 94;  // stop servo
    Serial.println("stop");
    myservo.write(val); 
    button_status = digitalRead(button_pin);  
  }

  t_start = millis();

  // when the button is pressed and held, lower the crane 
  while (button_status == 0){
    val = 180;
    Serial.println("lower");
    myservo.write(val); 
    button_status = digitalRead(button_pin);  
  }

  // calculate length if time button pressed 
  t_end = millis();
  period = t_end - t_start;

  // raise the crabe for the same amount of time as it was lowered for
  val = 0;
  Serial.println("return");
  Serial.println(period);
  myservo.write(val); 
  delay(period);   

                          
}
