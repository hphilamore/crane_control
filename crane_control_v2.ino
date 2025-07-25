/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>
#include "Adafruit_VL53L0X.h"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Servo myservo;  // create Servo object to control a servo

int val;                    // value to send to servo
int button_pin_grab = 2; 
int button_pin_lift = 3; 
int button_pin_lower = 4; 
int button_status_grab;         // status of grab push buttons
int button_status_lift;
int button_status_lower;
int last_button_status_grab = 1; // previous status of grab push button


int encoder_count = 0;
float encoder_distance;
float last_encoder_distance;

unsigned long t_last_state_change = millis(); // last time the button pin was toggled
unsigned long debounce_delay = 50;                  // time to wait before confirming button press
unsigned long t_start, t_end, period; // variables to calculate length of button press

void setup() {
  pinMode(button_pin_grab, INPUT_PULLUP);  // internal pullup on button pin
  pinMode(button_pin_lift, INPUT_PULLUP);  // internal pullup on button pin
  pinMode(button_pin_lower, INPUT_PULLUP);  // internal pullup on button pin

  myservo.attach(9);                  // attaches the servo on pin 9 to the Servo object
  Serial.begin(115200);
  //Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }

  // check if laser ToF sensor is present 
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
 
}

void loop() {
  // read the state of the button
  button_status_grab = digitalRead(button_pin_grab); 
  button_status_lift = digitalRead(button_pin_lift); 
  button_status_lower = digitalRead(button_pin_lower); 
  

  // check if the grab button changed state and reset debounce timer
  if (button_status_grab != last_button_status_grab){
    t_last_state_change = millis();
  }

  // lift crane if button pressed
  while (button_status_lift == 0){
    Serial.println("lift");
    val = 0;
    myservo.write(val); 
    button_status_lift = digitalRead(button_pin_lift);

  }

  // lower crane if button pressed
  while (button_status_lower == 0){
    Serial.println("lower");
    val = 180;
    myservo.write(val); 
    button_status_lower = digitalRead(button_pin_lower);
    
  }

  // check if the time since the last state change indicates an actual grab button press
  if (millis() - t_last_state_change > debounce_delay){

    // check button is pressed
    if (button_status_grab == 0){

      t_start = millis(); // time of start of button press

      // when the button is pressed and held, lower the crane
      while (button_status_grab == 0){
        val = 180;
        //Serial.println("lower");
        myservo.write(val); 
        button_status_grab = digitalRead(button_pin_grab); 
        encoder("+"); // measure distance lowered using encoder
      }

      // calculate length of time the button was pressed 
      t_end = millis();
      period = t_end - t_start;

      // raise the crane the same distance as it was lowered
      val = 0;
      Serial.println("return");
      myservo.write(val); 
      Serial.println(period);
      // delay(period);
      while (encoder_count > 0){
        encoder("-");
      }

      // tiny delay to compensate for encoder inaccuracy
      //delay(200);

    }
  }

  //if the push button is not pressed, stop the servo 
  val = 94;  // stop servo
  myservo.write(val); 
  last_button_status_grab = button_status_grab; 

}  

void encoder(String count_direction){
  /*
  Measures the distance the crane is lifted/lowered
  */

  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data

    encoder_distance = measure.RangeMilliMeter;


    // if the encoder wheel changes state, increment counter by 1
    if ((10 < encoder_distance && encoder_distance < 30 && 
        30 <= last_encoder_distance && last_encoder_distance < 50) || 
        (10 < last_encoder_distance && last_encoder_distance < 30 && 
        30 <= encoder_distance && encoder_distance < 50)){

          Serial.println("***");

          if (count_direction == "+"){
            encoder_count += 1;
          }
          else if (count_direction == "-"){
            encoder_count -= 1;
          }

          // Serial.println("***");
      }

    // Serial.print("Distance (mm): "); 
    // Serial.println(encoder_distance);

    Serial.println(encoder_count);

    // update encoder measurement 
    last_encoder_distance = encoder_distance;

    
    } 
        
  else {
    Serial.println(" out of range ");
    }

  // delay(10);

}




    



//   // // get the stats of the push button
//   // button_status_grab = digitalRead(button_pin_grab);            // reads the value of the potentiometer (value between 0 and 1023)
   
//   // if the button is not pressed, do nothing 
//   while (button_status_grab == 1){
//     val = 94;  // stop servo
//     Serial.println("stop");
//     myservo.write(val); 
//     button_status_grab = digitalRead(button_pin_grab);  

//   }

//   t_start = millis();

//   // when the button is pressed and held, lower the crane 
//   while (button_status_grab == 0){
//     val = 100;
//     // Serial.println("lower");
//     myservo.write(val); 
//     button_status_grab = digitalRead(button_pin_grab);  

//     VL53L0X_RangingMeasurementData_t measure;
    
//     // Serial.print("Reading a measurement... ");
//     lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

//     if (measure.RangeStatus != 4) {  // phase failures have incorrect data
//       float distance = measure.RangeMilliMeter;
//       Serial.print("Distance (mm): "); //Serial.println(measure.RangeMilliMeter);
//       if (distance < 30){
//         Serial.print("*******");
//       }
//       Serial.println(distance);

//     } else {
//       Serial.println(" out of range ");
//     }
      
//     delay(10);
//   }

//   // calculate length if time button pressed 
//   t_end = millis();
//   period = t_end - t_start;

//   // raise the crabe for the same amount of time as it was lowered for
//   val = 0;
//   Serial.println("return");
//   Serial.println(period);
//   myservo.write(val); 
//   delay(period);   

                          
// }
