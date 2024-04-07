//LIBRARIES

#include <avr/io.h>
#include <Wire.h>
#include <Servo.h>
#include <SoftwareWire.h>

//***************************************************************************************************************************************//
//STEPPER PINS

#define step_pin    4   //pin that gives pulses for steps
#define dir_pin     5   //pin that controls the direction of rotation (+ or - for clockwise or counterclockwise depending on the driver)
#define enable_pin  6   //when pulled HIGH disables the driver
#define break_pin   13     //when pulled LOW engages the break of the stepper

uint8_t step_mode_pins[3] = {7, 8, 9};    //pins connected to {MS1, MS2, MS3}

//***************************************************************************************************************************************//
//STEPPER PARAMETERS

#define back_dir    0   //bit that moves the stepper towards the back ENDstop
#define forward_dir 1   //bit that moves the stepper away from the back ENDstop
//
#define enable_on   0   //bit that turns on the enable when written to enable_pin
#define enable_off  1   //bit that turns off the enable when written to enable_pin
//
#define break_on    0   //bit that engages the break when written to break_pin
#define break_off   1   //bit that disengages the break when written to break_pin
//
#define enable_break_delay    5   //the delay between the engagement and disengagement of the stepper 
//
unsigned long step_range = 1276000; //151171;   //length of the track that the stepper can travel IN STEPS!!!
/*volatile*/ unsigned long step_position;         //how many steps are we away from the ENDstop
#define scale_max  100             //upper limit of the scale used for positioning punctuationnds
#define scale_min  0               //lowwer limit of the scale used for positioning punctuationnds
/*volatile*/ uint8_t scale_position;   //current position of gantry on the scale from scale_min to scale_max

/*volatile*/ bool stepper_is_calibrated = 0;      //0 - not calibrated, 1 - calibrated

#define min_HIGH_pulse   1     //put the minimum delay needed for the HIGH pulse of step_pin

uint8_t allowed_cycle_number = 3;  //the allowable number of angle position correction cycles

#define default_step_delay 1    //default speed value

/*
bool step_modes[5][3]=
{
  {0, 0, 0},  //full steps
  {1, 0, 0},  //1/2 steps
  {0, 1, 0},  //1/4 steps
  {1, 1, 0},  //1/8 steps
  {1, 1, 1}  //1/16 steps
};
*/

volatile uint8_t wanted_position;
volatile uint8_t wanted_angle;
unsigned int step_delay;
unsigned long steps_needed;

//***************************************************************************************************************************************//
//ENDstop & FARstop PARAMETERS
//ENDstop - at MIN position
//FARstop - at MAX position

// - emergency stop pin is 3 (this is unchangable) - //
#define ENDstop_pin     2       //this is unchangable
#define FARstop_pin     10      //this is unchangable
volatile bool ENDstop_state;    //variable that has the state of the ENDstop (0 = pressed, 1 = not pressed)
volatile bool FARstop_state;    //variable that has the state of the FARstop (0 = pressed, 1 = not pressed)

//***************************************************************************************************************************************//
//ANGLE STEPPER PINS

#define angle_step_pin    7   //pin that gives pulses for steps
#define angle_dir_pin     8   //pin that controls the direction of rotation (+ or - for clockwise or counterclockwise depending on the driver)
#define angle_enable_pin  9   //when pulled LOW disables the driver

//***************************************************************************************************************************************//
//ANGLE STEPPER PARAMETERS

#define down_dir    0   //bit that moves the angle stepper towards the back ENDstop
#define up_dir      1   //bit that moves the angle stepper away from the back ENDstop
#define max_angle  100             //upper limit of the scale used for positioning punctuationnds
#define min_angle  0               //lowwer limit of the scale used for positioning punctuationnds
//
#define angle_enable_on   0   //bit that turns on the enable when written to angle_enable_pin
#define angle_enable_off  1   //bit that turns off the enable when written to angle_enable_pin
//
volatile unsigned int angle_step_range = 427;   //number of steps that the stepper IN STEPS!!!
volatile unsigned int angle_step_position;       //how many steps are we away from the ENDstop
volatile uint8_t angle_scale_position;   //current position of gantry on the scale from scale_min to scale_max
volatile bool angle_stepper_is_calibrated = 0;      //0 - not calibrated, 1 - calibrated

#define angle_min_HIGH_pulse   5000     //put the minimum delay needed for the HIGH pulse of step_pin

#define allowed_number_of_angle_cycles    3  //the allowable number of angle position correction cycles

#define angle_default_step_delay 5000    //default speed value
//

//***************************************************************************************************************************************//
//angle_ENDstop & angle_FARstop PARAMETERS
//angle_ENDstop - at MIN point of angle
//angle_FARstop - at MAX point of angle

#define angle_ENDstop_pin  12      //this can't be changed here, it needes to be changed in angle_ENDstop_setup()
#define angle_FARstop_pin  A3      //this can't be changed here, it needes to be changed in angle_FARstop_setup()

volatile bool angle_ENDstop_state;    //variable that has the state of the angle_ENDstop (0 = pressed, 1 = not pressed)
volatile bool angle_FARstop_state;    //variable that has the state of the angle_FARstop (0 = pressed, 1 = not pressed)

//***************************************************************************************************************************************//
//SERIAL VARIABLES

char* input_command;           //only needed if take_serial_request() is used

//***************************************************************************************************************************************//
//BUTTON TESTING
/*
#define button_forward    11    //button pin that moves the stepper forward, button connected to GND
#define button_back       10    //button pin that moves the stepper back, button connected to GND
#define speed_pot         A6    //measures analog voltage, pin is connected to the middle pin of potenciometer, GND and 5V to other 2 pins
*/
//***************************************************************************************************************************************//
//IR VARIABLES

#define IR_pin                    A7    //IR pin for analog reading
#define position_tolerance        30    //IR allowable tolerance
#define IR_recorrection_delay     1     //delay to not get wrong readings because of inertia and other factors during recorrection
#define IR_sampling_cycles        20    //number of samples that vill be taken for an IR reading
#define IR_sampling_delay         1     //delay between IR sampling readings
#define IR_throwaway_threshold    40    //threshold for throwing away the value
//
//#define IR_min_distance_from_sensor 50   //min distance from the IR sensor in [cm]

//#define IR_max_distance_from_sensor 45  //max distance from the IR sensor in [cm]
//
#define IR_number_of_samples      20    //number of samples for sampling
uint8_t IR_distances[IR_number_of_samples + 1] = {
0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };  //IR distances that will be measured
int IR_readings[IR_number_of_samples + 1] = {
169, 279, 378, 456, 523, 583, 630, 673, 708, 749, 770, 802, 822, 851, 869, 887, 905, 922, 937, 950, 959 };  //IR values of the IR_distances
float IR_step_weights[IR_number_of_samples] = {
0.47, 0.57, 0.68, 0.97, 0.90, 0.99, 1.39, 1.29, 1.78, 1.91, 1.98, 1.98, 3.03, 1.78, 3.96, 3.03, 3.68, 2.71, 3.68, 3.68  };
//
int IR_MIN_position;
int IR_MAX_position;

//***************************************************************************************************************************************//
//IR FOR BALL PROXIMITY

#define IR_ball_pin               A6    //IR pin for analog reading of the ball distance
#define IR_ball_threshold         600   //analog reading value at which the ball is in position
char ball_is_in_position;               // P - ball is in position, O - ball isn't in position

//***************************************************************************************************************************************//
//TRIGGER (SERVO) VARIABLES

Servo   trigger_servo;                            //declare the name of the trigger servo
#define trigger_servo_pin                  11     //must be a PWM pin
#define trigger_servo_release_position     30     //the servo position that releases the trigger
#define trigger_servo_hold_position        100    //the servo position that holds the trigger
#define trigger_servo_release_delay        300    //the delay needed before release and holding again
volatile uint8_t trigger_servo_position;          //variable that holds the position of the trigger servo

//***************************************************************************************************************************************//
//AS5600 PINS

#define AS5600_SDA            A0
#define AS5600_SCL            A1
//#define AS5600_ADC            A2                //not in use
#define AS5600_address        0x36
#define HIGH_byte_address     0x0C
#define LOW__byte_address     0x0D

//***************************************************************************************************************************************//
//AS5600 PARAMETERS

#define angle_tolerance       7    //the allowable tolerance of the gotten angle value and the sensor value
int AS5600_current_position;
int AS5600_MIN_position = 240;
int AS5600_MAX_position = 2294;
#define AS5600_recorrection_delay 1 //delay to not get wrong readings because of inertia and other factors during recorrection

//***************************************************************************************************************************************//
//I2C VARIABLES

#define slave_address 69        //address of the slave 1-128

String received_command;        //what the slave reads
String message_for_master;      //what the slave will seFnd
char* error_message = "NO ERROR";           //error message feedback for master

bool new_command = 0;           //0 - no new command, 1 - command recieved
volatile bool trigger_ready = 0;         //0 - not ready to trigger, 1 - ready to trigger
bool ready_for_new_command = 1; //0 - not ready for new punctuationnd, 1 - ready for new punctuationnd

//SOFTWARE I2C FOR THE ANGLE SENSOR
SoftwareWire AS5600_I2C(AS5600_SDA, AS5600_SCL);

//***************************************************************************************************************************************//
//GENERAL KICKER PARAMETERS

#define default_shoot_position  30    //default position for preparing to shoot

bool calibration_flag = 0;
bool trigger_flag = 0;
uint8_t kicker_position;
uint8_t kicker_angle;

//***************************************************************************************************************************************//
//INLINE FUNCTIONS

//inline void calibrate_stepper(unsigned int step_delay) __attribute__((always_inline));
//inline void trigger_servo_release() __attribute__((always_inline));
//inline void trigger_servo_hold() __attribute__((always_inline));
//inline void use_preset_values_for_stepper_calibration(unsigned int step_delay) __attribute__((always_inline));
//inline void go_to_FARstop(unsigned int step_delay) __attribute__((always_inline));
inline uint8_t current_scale_position_from_IR(int IR_current_position) __attribute__((always_inline));

//***************************************************************************************************************************************//

void(* resetFunc) (void) = 0; // create a standard reset function

#define stupid_pin A2


void setup(){
  Serial.begin(9600);
  /*
  while (Serial) {
    ;
  }
  */
  //slave_I2C_setup();
  trigger_servo_setup();
  IR_setup();
  AS5600_I2C_setup();
  stepper_setup();
  angle_stepper_setup();
  ENDstop_setup();
  FARstop_setup();
  angle_ENDstop_setup();
  angle_FARstop_setup();
  stupid_mode_setup();
}

void loop() {
  /*
  angle_move_steps(0, 100, angle_default_step_delay);
  delay(1000);
  angle_move_steps(1, 100, angle_default_step_delay);
  delay(1000);
  */
  //Serial.println(AS5600_get_current_position());
  //Serial.println(current_scale_position_from_IR(IR_get_current_position()));
  //delay(300);
  //take_serial_request();
  //take_I2C_request();
  //Serial.println(IR_get_current_position());
  //Serial.println(IR_get_current_position());
  //Serial.println(analogRead(IR_ball_pin));
  stupid_mode();
}

void stupid_mode(){
  bool A2_state;
  bool stay_in_loop = 1;
  while(stay_in_loop == 1){
    A2_state = analogRead(stupid_pin);
    Serial.println(A2_state);
    if(A2_state > 600){
      uint8_t number_of_failed_checks = 0;
      for(uint8_t i = 0; i<30; i++){
        if(analogRead(stupid_pin) > 600){
          number_of_failed_checks++;
          Serial.print("Number of failed check: ");
          Serial.println(number_of_failed_checks);
          delayMicroseconds(5);
          }
      }
    if(number_of_failed_checks >= 20){
      stay_in_loop = 0;
      Serial.println("AAAAAAAAAAAA");
      }
    }
  }
  received_command = "G";
  new_command = 1;
  take_I2C_request();
}

void stupid_mode_setup(){
  enable_on_break_off();
  digitalWrite(angle_enable_pin, angle_enable_on);
  digitalWrite(enable_pin, enable_on);
  pinMode(stupid_pin, INPUT_PULLUP);
  //digitalWrite(enable_pin, enable_on);
  digitalWrite(stupid_pin, HIGH);
  //digitalWrite(stupid_pin, HIGH);
  Serial.println("MAMAMA");
  received_command = "C";
  new_command = 1;
  take_I2C_request();
  Serial.println("GGGGGGG");
  received_command = "F";
  new_command = 1;
  take_I2C_request();
  enable_off_break_on();
  Serial.println("JOEEEEEE");
}

//***************************************************************************************************************************************//
//STEPPER FUNCTIONS

void stepper_setup(){
/*DESCRIPTION: Simple function that sets up the pins for the stepper driver.
 */
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  enable_and_break_setup();
}

void use_preset_values_for_stepper_calibration(unsigned int step_delay){
  //Serial.println("Going to endstop");
  go_to_ENDstop(step_delay);
  //Serial.println("ENDstop reached");
  step_position = 0;
  scale_position = 0;
  //stepper_is_calibrated = 1;
}

void calibrate_stepper(unsigned int step_delay){
/* DESCRIPTION: Moves the stepper to ENDstop then to the MAX position.
 *              Gets the sensor positions of the MIN and MAX state.
 */
    //Serial.println("JOEEEEEEEEE");
    go_to_FARstop(step_delay);
    //Serial.println("MAMA");
    delay(IR_recorrection_delay);
    //IR_MAX_position = IR_get_current_position();
    digitalWrite(dir_pin, back_dir);
    step_range = 0;
    //Serial.println("-going to ENDstop");
    while(ENDstop_state == 1){
      
      digitalWrite(step_pin, 1);
      delayMicroseconds(min_HIGH_pulse);
      digitalWrite(step_pin, 0);
      delayMicroseconds(step_delay - min_HIGH_pulse);
      step_range++;
    }
    //Serial.println("-ENDstop reached");
    trigger_servo_release();
    delay(IR_recorrection_delay);
    //IR_MIN_position = IR_get_current_position();
    step_position = 0;
    scale_position = scale_min;
    unsigned int steps_needed;
    uint8_t distance_step = (scale_max - scale_min) / IR_number_of_samples;
    //this variable controlls the step between the distance measurments (try to pick values that will give whole numbers)
    for(int i=0; i<=IR_number_of_samples; i++){                               //
      IR_distances[i] = scale_min + (distance_step * i);    //create the distance values
      steps_needed = map(IR_distances[i], scale_min, scale_max, 0, step_range) - step_position;
      move_steps(forward_dir, steps_needed, step_delay);
      delay(100);
      IR_readings[i] = IR_get_current_position();
    }
    //IR_steps_per_section();
    step_position = step_range;
    scale_position = scale_max;
    //stepper_is_calibrated = 1;
    IR_MIN_position = IR_readings[0];
    IR_MAX_position = IR_readings[IR_number_of_samples];
    /*//
    Serial.print("IR_MIN_position: ");
    Serial.println(IR_MIN_position);
    Serial.print("IR_MAX_position: ");
    Serial.println(IR_MAX_position);            // troubleshooting block
    Serial.print("STEP RANGE: ");
    Serial.println(step_range);
    Serial.print("STEPS POSITION: ");
    Serial.println(step_position);
    Serial.print("\n");
    for(int i; i<=IR_number_of_samples; i++){
      Serial.print("Scale value= ");
      
      Serial.print(IR_distances[i]);
      Serial.print(" || ");
      Serial.print(IR_readings[i]);
      Serial.println(" = IR readings");
    }
    //*/
}

void move_steps(bool spin_direction, unsigned long requested_steps, unsigned int step_delay){
/* DESCRIPTION: Moves a requested number of steps.
 * spin_direction - boolean 0 or 1
 * requested_steps - how many steps we want to make
 */
  bool x;
  if(spin_direction == forward_dir){
    x = 1;
  }else{
    x = 0;
  }
  digitalWrite(dir_pin, spin_direction);    //choosing the direction of movement
  for(unsigned long i=1; i <= requested_steps; i++){
     if(digitalRead(ENDstop_pin) == 0){
        if(spin_direction == back_dir){
          //Serial.println("Can't move back - at the ENDstop");
          //error_message = "DE";
          //stepper_is_calibrated = 1;
          step_position = 0;
          i = requested_steps + 1;
          }else{
            //Serial.println("FORWARD");
            digitalWrite(step_pin, 1);
            delayMicroseconds(min_HIGH_pulse);
            digitalWrite(step_pin, 0);
            delayMicroseconds(step_delay - min_HIGH_pulse);
            step_position++;
          }
      }else if(digitalRead(FARstop_pin) == 0){
        if(spin_direction == forward_dir){
          //Serial.println("Can't move forward - at the FARstop");
          //error_message = "DE";
          //stepper_is_calibrated = 1;
          step_position = step_range;
          scale_position = scale_max;
          i = requested_steps + 1;
          }else{
            //Serial.println("BACK");
            digitalWrite(step_pin, 1);
            delayMicroseconds(min_HIGH_pulse);
            digitalWrite(step_pin, 0);
            delayMicroseconds(step_delay - min_HIGH_pulse);
            step_position--;
          }
      }else{
      /*//
      if(spin_direction == back_dir){
        Serial.println("BACK");
      }else{
        Serial.println("FORWARD");
      }
      //*/
      digitalWrite(step_pin, 1);
      delayMicroseconds(min_HIGH_pulse);
      digitalWrite(step_pin, 0);
      delayMicroseconds(step_delay - min_HIGH_pulse);
      step_position = (int)x * 2 -1 + step_position;
      }
      //Serial.print("step_position : ");
      //Serial.println(step_position);
  }
  //Serial.println("steps moved");
}

void correct_achieved_position(int wanted_position, unsigned int step_delay, uint8_t allowed_cycle_number){
  //
  bool move_direction;
  bool move_for_correction = 0;
  uint8_t number_of_cycle = 1;
  int IR_wanted_sensor_position = IR_get_wanted_reading(wanted_position);
  int steps_for_correction;
  int IR_current_position;
  uint8_t scale_position;
  bool correction_check_needed = 1;           //variable that tells us if an angle correction check is needed (1 - it is needed, 0 - it isn't needed) 
  //
  while(correction_check_needed == 1){
    //
    correction_check_needed = 0;
    IR_current_position = IR_get_current_position();
    scale_position = current_scale_position_from_IR(IR_current_position);
    /*//
    Serial.print("Current scale position: ");
    Serial.println(scale_position);
    //*/
    if(IR_current_position < IR_wanted_sensor_position - position_tolerance){
      scale_position = current_scale_position_from_IR(IR_current_position);
      steps_for_correction = map(wanted_position - scale_position, scale_min, scale_max - scale_min, 0, step_range);
      /*//
      Serial.print("IR current position = ");
      Serial.print(IR_current_position);
      Serial.print(" | ");                            // troubleshooting block
      Serial.print(IR_wanted_sensor_position);
      Serial.println(" = IR wanted sensor position");
      //*/
      if(steps_for_correction == 0){
        //Serial.println("POSITION IN TOLERANCE");
        break;
      }
      /*//
      Serial.println("Correction needed!!!");
      Serial.print("Correction: number_of_cycle");
      Serial.print(number_of_cycle);
      Serial.print('/');                                // trubleshooting block
      Serial.println(allowed_cycle_number);
      Serial.print("Move: +");
      Serial.println(steps_for_correction);
      delay(5000);                                    // good for trubleshooting
      //*/
      move_direction = forward_dir;
      move_for_correction = 1;
      //move_steps(forward_dir, steps_for_correction, step_delay);
      correction_check_needed = 1;
      delay(IR_recorrection_delay);
    }else if(IR_current_position > IR_wanted_sensor_position + position_tolerance){
      scale_position = current_scale_position_from_IR(IR_current_position);
      steps_for_correction = map(scale_position - wanted_position, scale_min, scale_max - scale_min, 0, step_range);
      /*//
      Serial.print("IR current position = ");
      Serial.print(IR_current_position);
      Serial.print(" | ");                            // troubleshooting block
      Serial.print(IR_wanted_sensor_position);
      Serial.println(" = IR wanted sensor position");
      //*/
      if(steps_for_correction == 0){
        //Serial.println("POSITION IN TOLERANCE");
        break;
      }
      /*//
      Serial.println("Correction needed!!!");
      Serial.print("Correction: number_of_cycle");
      Serial.print(number_of_cycle);
      Serial.print('/');                                // trubleshooting block
      Serial.println(allowed_cycle_number);
      Serial.print("Move: -");
      Serial.println(steps_for_correction);
      delay(5000);                                    // good for trubleshooting
      //*/
      move_direction = back_dir;
      move_for_correction = 1;
      //move_steps(back_dir, steps_for_correction, step_delay);
      correction_check_needed = 1;
      delay(IR_recorrection_delay);
    }else{
      correction_check_needed = 0;
      IR_current_position = IR_get_current_position();
      scale_position = current_scale_position_from_IR(IR_current_position);
      step_position = map(scale_position, scale_min, scale_max-scale_min, 0, step_range);  //aproximate the new angle step position
      //Serial.println("POSITION IN TOLERANCE");
    }
    if(move_for_correction == 1){
      move_steps(move_direction, steps_for_correction, step_delay);
      move_for_correction = 0;
    }
    if(number_of_cycle + 1> allowed_cycle_number){
      if(correction_check_needed == 1){
        correction_check_needed = 0;        //go out of the loop
        //stepper_is_calibrated = 0;          //stepper needs calibration
        IR_current_position = IR_get_current_position();
        scale_position = current_scale_position_from_IR(IR_current_position);
        step_position = map(scale_position, scale_min, scale_max-scale_min, 0, step_range);  //aproximate the new angle step position
        //error_message = "PT";               //position tolerance problem
        //Serial.println("Couldn't correct angle");
      }else{
        correction_check_needed = 0;
        scale_position = current_scale_position_from_IR(IR_current_position);
        step_position = map(scale_position, scale_min, scale_max-scale_min, 0, step_range);  //aproximate the new angle step position
        //Serial.println("POSITION IN TOLERANCE");
      }
    }
    number_of_cycle++;
  }
  //number_of_cycle = 1;
}

void move_to_position(uint8_t wanted_position, int step_delay){
/*DESCRIPTION: Moves the gantry to a wanted position on a scale from scale_min to scale_max.
 */
  bool move_direction;
  unsigned long steps_needed = 0;
  if(wanted_position > scale_max){    //
    wanted_position = scale_max;      //
  }                                   //    limit wanted_position
  if(wanted_position < scale_min){    //    so it stays inside of
    wanted_position = scale_min;      //    the scale
  }                                   //
  if(wanted_position == scale_position){
    /*//
    Serial.print("Wanted position = ");
    Serial.print(wanted_position);
    Serial.print(" = ");                         // trubleshooting block
    Serial.print(scale_position);
    Serial.println(" = Scale position");
    Serial.println("Already at the position");
    Serial.print("\n");
    //*/
  }else{
    if(wanted_position > scale_position){
      /*//
      Serial.print("Wanted position = ");
      Serial.print(wanted_position);
      Serial.print(" > ");                         // trubleshooting block
      Serial.print(scale_position);
      Serial.println(" = Scale position");
      Serial.print("\n");
      //*/
      steps_needed = map(wanted_position, scale_min, scale_max, 0, step_range) - step_position;
      scale_position = wanted_position;
      move_direction = forward_dir;
      /*//
      Serial.print("Steps needed: +");              // trubleshooting block
      Serial.println(steps_needed);
      //*/
    }else if(wanted_position < scale_position){
      /*//
      Serial.print("Wanted position = ");
      Serial.print(wanted_position);
      Serial.print(" < ");                         // trubleshooting block
      Serial.print(scale_position);
      Serial.println(" = Scale position");
      Serial.print("\n");
      //*/
      steps_needed = step_position - map(wanted_position, scale_min, scale_max, 0, step_range);
      scale_position = wanted_position;
      move_direction = back_dir;
      /*//
      Serial.print("Steps needed: -");              // trubleshooting block
      Serial.println(steps_needed);
      //*/
    }
    //enable_on_break_off();
    move_steps(move_direction, steps_needed, step_delay);
    //correct_achieved_position(wanted_position, step_delay, allowed_cycle_number);
    //enable_off_break_on();
  }
}

/*
void set_step_mode(uint8_t step_mode){
//DESCRIPTION: Choses step mode by taking in a number from 0 to 4:
// * 0 - full steps
// * 1 - 1/2 steps
// * 2 - 1/4 steps
// * 3 - 1/8 steps
// * 4 - 1/16 steps
 
 for(int i=0; i<3; i++){
  digitalWrite(step_mode_pins[i], step_modes[step_mode][i]);
 }
}
*/

//***************************************************************************************************************************************//
//ANGLE STEPPER FUNCTIONS

void angle_stepper_setup(){
/*DESCRIPTION: Simple function that sets up the pins for the stepper driver.
 */
  pinMode(angle_step_pin, OUTPUT);
  pinMode(angle_dir_pin, OUTPUT);
  pinMode(angle_enable_pin, OUTPUT);
  digitalWrite(angle_enable_pin, angle_enable_on);
  //angle_step_position = 0;
  //angle_scale_position = 0;
}

void use_preset_values_for_angle_stepper_calibration(unsigned int step_delay){
  angle_go_to_ENDstop(step_delay);
  angle_step_position = 0;
  angle_scale_position = min_angle;
  //angle_stepper_is_calibrated = 1;
}

void calibrate_angle_stepper(unsigned int step_delay){
/* DESCRIPTION: Moves the stepper to angle_FARstop and then to angle_ENDstop.
 *              Gets the sensor positions of the MIN and MAX state.
 */
  int AS5600_current_position;
  angle_go_to_FARstop(step_delay);
  AS5600_current_position = AS5600_get_current_position();
  AS5600_MAX_position = AS5600_current_position;
  angle_step_range = 0;
  digitalWrite(angle_dir_pin, down_dir);
  //Serial.println("-going to angle_ENDstop");
  while(angle_ENDstop_state == 1){
    digitalWrite(angle_step_pin, 1);
    delayMicroseconds(angle_min_HIGH_pulse);
    digitalWrite(angle_step_pin, 0);
    delayMicroseconds(step_delay - angle_min_HIGH_pulse);
    angle_step_range++;
  }
  //Serial.println("-angle_ENDstop reached");
  angle_step_position = 0;
  angle_scale_position = min_angle;
  AS5600_current_position = AS5600_get_current_position();
  AS5600_MIN_position = AS5600_current_position;
  /*//
  Serial.print("AS5600_MIN_position: ");
  Serial.println(AS5600_MIN_position);
  Serial.print("AS5600_MAX_position: ");
  Serial.println(AS5600_MAX_position);
  Serial.print("ANGLE STEP RANGE: ");
  Serial.println(angle_step_range);
  Serial.print("ANGLE STEPS POSITION: ");
  Serial.println(angle_step_position);
  //*/
}

void angle_move_steps(bool spin_direction, int requested_steps, unsigned int step_delay){
/* DESCRIPTION: Moves angle_stepper a requested number of steps.
 * spin_direction - boolean 0 or 1
 * requested_steps - how many steps we want to make
 */
  bool x;
  if(spin_direction == up_dir){
    x = 1;
  }else{
    x = 0;
  }
  digitalWrite(angle_dir_pin, spin_direction);    //choosing the direction of movement
  for(unsigned int i=1; i <= requested_steps; i++){
      if(digitalRead(angle_ENDstop_pin) == 0){
        if(spin_direction == down_dir){
          //Serial.println("Can't move back - at the angle_ENDstop");
          error_message = "DE";
          //angle_stepper_is_calibrated = 1;
          angle_step_position = 0;
          //angle_scale_position = min_angle;
          i = requested_steps + 1;
          }else{
            //Serial.println("UP");
            digitalWrite(angle_step_pin, 1);
            delayMicroseconds(angle_min_HIGH_pulse);
            digitalWrite(angle_step_pin, 0);
            delayMicroseconds(step_delay - angle_min_HIGH_pulse);
            angle_step_position++;
          }
//      }else if(/*digitalRead(angle_FARstop_pin) == 1*/ 0 == 1){
//        if(spin_direction == up_dir){
//          Serial.println("Can't move forward - at the angle_FARstop");
//          //error_message = "DE";
//          //angle_stepper_is_calibrated = 1;
//          angle_step_position = angle_step_range;
//          angle_scale_position = max_angle;
//          i = requested_steps + 1;
//          }else{
//            Serial.println("DOWN");
//            digitalWrite(angle_step_pin, 1);
//            delayMicroseconds(angle_min_HIGH_pulse);
//            digitalWrite(angle_step_pin, 0);
//            delayMicroseconds(step_delay - angle_min_HIGH_pulse);
//            angle_step_position--;
//          }
      }else{
      digitalWrite(angle_step_pin, 1);
      delayMicroseconds(angle_min_HIGH_pulse);
      digitalWrite(angle_step_pin, 0);
      delayMicroseconds(step_delay - angle_min_HIGH_pulse);
      angle_step_position = (int)x * 2 -1 + angle_step_position;
      }
      //Serial.print("angle_step_position : ");
      //Serial.println(angle_step_position);
  }
  //ready_for_new_command = 1;
}

void move_to_angle(uint8_t wanted_angle, unsigned int step_delay){
/*DESCRIPTION: Moves the gantry to a wanted angle on a scale from min_angle to max_angle.
 */
  bool move_direction;
  unsigned int steps_needed=0;
  if(wanted_angle > max_angle){    //
    wanted_angle = max_angle;      //
  }                                //    limit wanted_position
  if(wanted_angle < min_angle){    //    so it stays inside of
    wanted_angle = min_angle;      //    the scale
  }                                //
  if(wanted_angle == angle_scale_position){
    /*//
    Serial.print("Wanted angle = ");
    Serial.print(wanted_angle);
    Serial.print(" = ");                         // trubleshooting block
    Serial.print(angle_scale_position);
    Serial.println(" = Angle scale position");
    Serial.println("Already at the angle");
    Serial.print("\n");
    //*/
  }else{
    if(wanted_angle > angle_scale_position){
      /*//
      Serial.print("Wanted angle = ");
      Serial.print(wanted_angle);
      Serial.print(" > ");                         // trubleshooting block
      Serial.print(angle_scale_position);
      Serial.println(" = Angle scale position");
      Serial.print("\n");
      //*/
      steps_needed = map(wanted_angle, min_angle, max_angle, 0, angle_step_range) - angle_step_position;
      angle_scale_position = wanted_angle;
      move_direction = up_dir;
      //move_direction = down_dir;
      /*//
      Serial.print("Steps needed: +");              // trubleshooting block
      Serial.println(steps_needed);
      //*/
    }else if(wanted_angle < angle_scale_position){
      /*//
      Serial.print("Wanted angle = ");
      Serial.print(wanted_angle);
      Serial.print(" < ");                         // trubleshooting block
      Serial.print(angle_scale_position);
      Serial.println(" = Angle scale position");
      Serial.print("\n");
      //*/
      steps_needed = angle_step_position - map(wanted_angle, min_angle, max_angle, 0, angle_step_range);
      angle_scale_position = wanted_angle;
      //angle_scale_position = map(AS5600_current_position, AS5600_MIN_position, AS5600_MAX_position, min_angle, max_angle);  //aproximate the new angle scale position
      move_direction = down_dir;
      //move_direction = up_dir;
      /*//
      Serial.print("Steps needed: -");              // trubleshooting block
      Serial.println(steps_needed);
      //*/
    }
    //enable_on_break_off();
    angle_move_steps(move_direction, steps_needed, step_delay);
    //correct_achieved_angle(wanted_angle, step_delay, allowed_cycle_number);
    //enable_off_break_on();
  }
}

void correct_achieved_angle(int wanted_angle, unsigned int step_delay, uint8_t allowed_number_of_cycles){
/*DESCRIPTION: Corrects the achived angle to get into tolerance.
 */
  uint8_t number_of_cycle = 1;
  int AS5600_wanted_sensor_position = map(wanted_angle, min_angle, max_angle, AS5600_MIN_position, AS5600_MAX_position);
  int AS5600_current_position;
  int angle_steps_for_correction;
  bool angle_correction_check_needed = 1;           //variable that tells us if an angle correction check is needed (1 - it is needed, 0 - it isn't needed) 
  //
  while(angle_correction_check_needed == 1){
    angle_correction_check_needed = 0;
    AS5600_current_position = AS5600_get_current_position();
    /*//
    Serial.print("AS5600 current position = ");
    Serial.print(AS5600_current_position);
    Serial.print(" | ");
    Serial.print(AS5600_wanted_sensor_position);
    Serial.println(" = AS5600 wanted position");
    //*/
    if(AS5600_current_position < AS5600_wanted_sensor_position - angle_tolerance){
      angle_steps_for_correction = map(AS5600_wanted_sensor_position - AS5600_current_position, 0, AS5600_MAX_position - AS5600_MIN_position, 0, angle_step_range);
      if(angle_steps_for_correction == 0){
        //Serial.println("ANGLE IN TOLERANCE");
        break;
      }
      /*//
      Serial.println("Correction needed!!!");
      Serial.print("Correction: number_of_cycle");
      Serial.print(number_of_cycle);
      Serial.print('/');                                // trubleshooting block
      Serial.println(allowed_number_of_cycles);
      Serial.print("Move: +");
      Serial.println(angle_steps_for_correction);
      //*/
      //delay(5000);                                    // good for trubleshooting
      angle_move_steps(up_dir, angle_steps_for_correction, step_delay);
      angle_correction_check_needed = 1;
      delay(AS5600_recorrection_delay);
    }else if(AS5600_current_position > AS5600_wanted_sensor_position + angle_tolerance){
      angle_steps_for_correction = map(AS5600_current_position - AS5600_wanted_sensor_position, 0, AS5600_MAX_position - AS5600_MIN_position, 0, angle_step_range);
      if(angle_steps_for_correction == 0){
        //Serial.println("ANGLE IN TOLERANCE");
        break;
      }
      /*//
      Serial.println("Correction needed!!!");
      Serial.print("Correction: number_of_cycle");
      Serial.print(number_of_cycle);
      Serial.print('/');                                // trubleshooting block
      Serial.println(allowed_number_of_cycles);
      Serial.print("Move: -");
      Serial.println(angle_steps_for_correction);
      //delay(5000);                                    // good for trubleshooting
      //*/
      angle_move_steps(down_dir, angle_steps_for_correction, step_delay);
      angle_correction_check_needed = 1;
      delay(AS5600_recorrection_delay);
    }else{
      angle_correction_check_needed = 0;
      angle_step_position = map(AS5600_current_position, AS5600_MIN_position, AS5600_MAX_position, 0, angle_step_range);  //aproximate the new angle step position
      //Serial.println("ANGLE IN TOLERANCE");
    }
    if(number_of_cycle + 1 > allowed_number_of_cycles){
      if(angle_correction_check_needed == 1){
        angle_correction_check_needed = 0;  //go out of the loop
        //angle_stepper_is_calibrated = 0;    //angle stepper needs calibration
        //error_message = "AT";               //angle tolerance problem
        angle_step_position = map(AS5600_current_position, AS5600_MIN_position, AS5600_MAX_position, 0, angle_step_range);  //aproximate the new angle step position
        angle_scale_position = map(AS5600_current_position, AS5600_MIN_position, AS5600_MAX_position, min_angle, max_angle);  //aproximate the new angle scale position
        //Serial.println("Couldn't correct angle");
      }else{
        angle_correction_check_needed = 0;
        //Serial.println("ANGLE IN TOLERANCE");
      }
    }
    number_of_cycle++;
  }
  number_of_cycle = 1;
}

//***************************************************************************************************************************************//
//ENABLE/BREAK FUNCTIONS

void enable_and_break_setup(){
  pinMode(enable_pin, OUTPUT);
  pinMode(break_pin, OUTPUT);
  enable_off_break_on();
}

void enable_on_break_off(){
  digitalWrite(enable_pin, enable_on);
  delay(enable_break_delay);
  digitalWrite(break_pin, break_off);
}

void enable_off_break_on(){
  digitalWrite(break_pin, break_on);
  delay(enable_break_delay);
  digitalWrite(enable_pin, enable_off);
}

//***************************************************************************************************************************************//
//TRIGGER SERVO punctuationNDS

void trigger_servo_setup(){
/* DESCRIPTION: Trigger servo setup.
 */
  pinMode(trigger_servo_pin, OUTPUT);
  trigger_servo.attach(trigger_servo_pin);
  //trigger_servo.write(trigger_servo_release_position);
}

void trigger_servo_release(){
  trigger_servo.write(trigger_servo_release_position);
  delay(trigger_servo_release_delay);
  /*
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }
  */
}

void trigger_servo_hold(){
  trigger_servo.write(trigger_servo_hold_position);
  delay(trigger_servo_release_delay);
}

//***************************************************************************************************************************************//
//AS5600 punctuationnds

void AS5600_I2C_setup(){
/* DESCRIPTION: Sets up the I2C communication of AS5600 sensor.
 */  
  AS5600_I2C.begin();
  //pinMode(AS5600_ADC, INPUT);
}

int AS5600_get_current_position(){
/* DESCRIPTION: Returns a value from 0 to 4096 depending on the magnet's position.
 */
  int AS5600_position;    //variable that will have the position
  byte highbyte;          //the data is 12-bit so we will have to
  byte lowbyte;           //split it up between two bytes
  //
  AS5600_I2C.beginTransmission(AS5600_address); //
  AS5600_I2C.write(LOW__byte_address);          //  request to read the low byte
  AS5600_I2C.endTransmission();                 //
  AS5600_I2C.requestFrom(AS5600_address, 1);    //
  //
  while(AS5600_I2C.available() != 0){   //wait until it becomes available
    lowbyte = AS5600_I2C.read();        //read the byte (only 4-bits, LSB first)
  }
  //
  AS5600_I2C.beginTransmission(AS5600_address); //
  AS5600_I2C.write(HIGH_byte_address);          // request to read the high byte
  AS5600_I2C.endTransmission();                 //
  AS5600_I2C.requestFrom(AS5600_address, 1);    //
  //
  while(AS5600_I2C.available() != 0){   //wait until it becomes available
    highbyte = AS5600_I2C.read();       //read the byte (4-bits)
  }
  AS5600_position =  (highbyte<<8) | lowbyte;   //shift the high byte and combine it with the lowbyte
  return AS5600_position;   //return position
}

//***************************************************************************************************************************************//
//GENERAL FUNCTIONS

void prepare_for_shooting(unsigned int step_delay){
/*DESCRIPTION: Prepares the kicker to shoot by going to scale_min lowering the servo
 *             and pulling back to default_shoot_position.
 */
  //trigger_servo_release();
  trigger_servo_hold();
  move_to_position(scale_min, default_step_delay);
  move_to_position(default_shoot_position, default_step_delay);
  move_to_angle(min_angle, default_step_delay);
  //digitalWrite(angle_enable_pin, angle_enable_off);
  trigger_ready = 1;
}

//***************************************************************************************************************************************//
//IR COMMANDS

void IR_setup(){
/* HARDWARE: IR sensor: blue wire - 3.3V , red wire - 3.3V , black wire - GND , yellow wire - IR_pin
 *           VREF connected to 3.3V through a series 15k resistor
 * DESCRIPTION: Sets up the IR sensor.
 */
 pinMode(IR_pin, INPUT);            //IR_pin needs to be an INPUT
 analogReference(EXTERNAL);         //take the reference voltage from the VREF pin
}

int IR_get_current_position(){
/* DESCRIPTION: Returns an average IR sensor value of IR_sampling_cycles number of readings.
 */
  int IR_values[IR_sampling_cycles];            //the value of the IR readings
  int IR_average;                               //the average of IR values
  unsigned int IR_sum = 0;                               //the sum of IR values
  uint8_t number_of_faulty_values = 0;          //number of faulty values
  //
  for(int i=0; i<IR_sampling_cycles; i++){
    IR_values[i] = analogRead(IR_pin);          //read the IR_pin analog values
    delay(IR_sampling_delay);                   //delay between readings
  }
  /*
  Serial.println("IR values:");                 //
  for(int i=0; i < IR_sampling_cycles; i++){    // troubleshooting block
    Serial.println(IR_values[i]);               //
  }                                             //
  */
  for(int i=0; i < IR_sampling_cycles; i++){    //
    IR_sum = IR_sum + IR_values[i];             //get the rough sum of values
  }                                             //
  //
  IR_average = IR_sum / IR_sampling_cycles;     //get the rough average
  /*
  Serial.print("IR rough average: ");
  Serial.println(IR_average);
  //                                            // troubleshooting block
  Serial.println("IR smooth values: ");
  */
  for(int i=0; i < IR_sampling_cycles; i++){    //throwaway the values that diverge over an acceptable threshoald
    if(IR_values[i] < (IR_average - IR_throwaway_threshold) || IR_values[i] > (IR_average + IR_throwaway_threshold)){
      IR_values[i] = 0;                         //if IR value is out of tolerance make it zero
      number_of_faulty_values++;                //count faulty values
    }
    //Serial.println(IR_values[i]);             // troubeshooting block
  }
  if(IR_sampling_cycles == number_of_faulty_values){
    number_of_faulty_values = 9;                //this is just so we don't divide by zero
  }
  /*
  Serial.print("Number of faulty values: ");    // troubleshooting block
  Serial.println(number_of_faulty_values);      //
  */
  IR_sum = 0;
  for(int i=0; i < IR_sampling_cycles; i++){    //
    IR_sum = IR_sum + IR_values[i];             //get the smooth sum
  }                                             //
  //
  IR_average = IR_sum / (IR_sampling_cycles - number_of_faulty_values);   //get the smooth average
  /*//
  Serial.print("IR smooth average: ");          //
  Serial.println(IR_average);                   // troubleshooting block
  Serial.print("\n");                           //
  //*/
  return IR_average;      //return the smooth average
}

int IR_get_wanted_reading(uint8_t wanted_position){
  /*DESCRIPTION: Function returns the IR_values that we want to have in the wanted_position.
   */
   uint8_t lower_position;
   uint8_t higher_position;
   int IR_wanted_value;
   for(int i=0; i<=IR_number_of_samples; i++){
    if(IR_distances[i] <= wanted_position){
      lower_position = IR_distances[i];
      if(i == IR_number_of_samples +1){
        //higher_position = lower_position;
        IR_wanted_value = IR_readings[i];
      }else{
        higher_position = IR_distances[i+1];
        IR_wanted_value = map(wanted_position, lower_position, higher_position, IR_readings[i], IR_readings[i+1]);
      }
    }
   }
   return IR_wanted_value;
}

void IR_steps_per_section(){
  /*DESCRITION: Calcultes the steps per value for each section of IR_readings.
   */
   for(int i=0; i<IR_number_of_samples; i++){
    IR_step_weights[i] = (float)((step_range *((float)(((scale_max - scale_min)/IR_number_of_samples))/(scale_max - scale_min)))/((float)(IR_readings[i+1] - IR_readings[i])));
    //(float)((scale_max - scale_min)/IR_number_of_samples)/(float)(IR_readings[i+1] - IR_readings[i])*step_range;
   }
   Serial.println("IR step weigths: ");
   for(int i=0; i<IR_number_of_samples; i++){
    Serial.print("IR step weigth for ");
    Serial.print(i*((scale_max - scale_min)/IR_number_of_samples));
    Serial.print('-');
    Serial.print((i+1)*((scale_max - scale_min)/IR_number_of_samples));
    Serial.print(": ");
    Serial.println(IR_step_weights[i]);
   }
}

uint8_t current_scale_position_from_IR(int IR_current_position){
 /*DESCRIPTION: Returns the real position.
  */
  uint8_t section_position=0;
  uint8_t lower_position=0;
  uint8_t real_position;
  if(IR_current_position > IR_readings[IR_number_of_samples]){
    IR_current_position = IR_readings[IR_number_of_samples];
    //error_message = "OB";   // -out of bounds
  }else if(IR_current_position < IR_readings[0]){
    IR_current_position = IR_readings[0];
    //error_message = "OB";   // -out of bounds
  }
  for(int i=0; i<IR_number_of_samples; i++){
    //Serial.print("i =");
    //Serial.println(i);
    if((IR_current_position >= IR_readings[i]) && (IR_current_position <= IR_readings[i+1])){
      //if((IR_current_position >= IR_readings[i])
      section_position = map(IR_current_position - IR_readings[i], 0, IR_readings[i+1] - IR_readings[i], 0, (scale_max - scale_min)/IR_number_of_samples);
      lower_position = IR_distances[i];
      real_position = lower_position + section_position;
      /*//
      Serial.print("IR_current_position= ");            //
      Serial.println(IR_current_position);              //
      Serial.print("Section of IR readings= ");         //
      Serial.print(IR_readings[i]);                     //
      Serial.print(" - ");                              //
      Serial.println(IR_readings[i+1]);                 //  troubleshooting block
      Serial.print("Section position= ");               //
      Serial.println(section_position);                 //
      Serial.print("Real position= ");                  //
      Serial.println(real_position);                    //
      Serial.print("\n");                               //
      //*/
    }
  }
  return real_position; 
}

/*
int IR_steps_for_correction(int IR_wanted_sensor_position, int IR_current_position, uint8_t wanted_position){
  /*DESCRIPTION: Function takes in IR_wanted_sensor_position & IR_current_position,
   *             returns the steps needed for correction.
   //
   float steps_for_correction;
   if(IR_wanted_sensor_position > IR_current_position){
    for(int i=0; i<IR_number_of_samples; i++){
      if((IR_readings[i] > IR_current_position) && (IR_readings[i] !> IR_wanted_sensor_position)){
        steps_for_correction = 
      }else if(){
        
        break;
      }
    }
   }
}
*/

//***************************************************************************************************************************************//
//SERIAL COMMANDS

void take_serial_request(){
/*DESCRIPTION: This function is used only for trubleshooting. It allowes us to give comands through Serial. example punctuationnds:
 * move_steps(1,200,1000)   - test punctuationnd (one rotation, full steps
 * move_steps(forward_dir,200,3000)
 * go_to_ENDstop(3000)
 * move_to_position(15,3000)
 */
 //switch(received_command[0])
 if(digitalRead(3) == 0){
  uint8_t number_of_failed_checks = 0;
  for(uint8_t i = 0; i<30; i++){
    if(digitalRead(3) == 0){
      number_of_failed_checks++;
      delayMicroseconds(10);
    }
  }
  if(number_of_failed_checks >= 20){
    digitalWrite(enable_pin, enable_on);
  delay(enable_break_delay);
  digitalWrite(break_pin, break_off);
  //Serial.println("Estop pressed");
  digitalWrite(dir_pin, back_dir);
  while(digitalRead(2) == 1){
    digitalWrite(step_pin, 1);
    delayMicroseconds(min_HIGH_pulse);
    digitalWrite(step_pin, 0);
    delayMicroseconds(default_step_delay - min_HIGH_pulse);
    //Serial.println("BACK");
  }
  //cli();
  digitalWrite(break_pin, break_on);
  delay(enable_break_delay);
  digitalWrite(enable_pin, enable_off);
  /*
  while(digitalRead(3) == 0){
    //Serial.println("BRICKED");
  }*/
  bool is_in_break_loop = 1;
  while(is_in_break_loop == 1){
    number_of_failed_checks = 0;
    }
    if(number_of_failed_checks >= 20){
      is_in_break_loop = 0;
    }else{
      is_in_break_loop = 1;
    }
  }
  // affter the Estop stops being pressed
    for(uint8_t i = 0; i<30; i++){
      if(digitalRead(3) == 1){
        number_of_failed_checks++;
        delayMicroseconds(10);
      }
  // use C command
  step_delay = default_step_delay;
  enable_on_break_off();
  trigger_servo_hold();
  use_preset_values_for_stepper_calibration(step_delay);
  prepare_for_shooting(step_delay);
  enable_off_break_on();
  stepper_is_calibrated = 1;
  // use F command
  step_delay = angle_default_step_delay;
  use_preset_values_for_angle_stepper_calibration(step_delay);
  angle_stepper_is_calibrated = 1;
  ready_for_new_command = 1;
  //
  step_position = 0;
  scale_position = scale_min;
  new_command = 0;
  ready_for_new_command = 1;
  calibration_flag = 1;
  trigger_flag = 1;
  kicker_position = default_shoot_position;
  kicker_angle = max_angle;
  //Serial.println("CALIBRATION_DONE");
  }
 }else{
  if(Serial.available() != 0){   //wait to recieve something from Serial monitor
    String serial_information = Serial.readString();    //save the recieved string
    //Serial.println(serial_information);
    serial_information.trim();                          //removes exces blank spaces, \r and \n
    uint8_t str_len = serial_information.length() + 1;
    char recieved_serial[str_len]; 
    serial_information.toCharArray(recieved_serial, str_len);
    uint8_t start_bracket = (serial_information.indexOf('('));
    uint8_t punctuation;
    uint8_t end_bracket;
    uint8_t wanted_position;
    uint8_t wanted_angle;
    unsigned int step_delay;
    //recieved_serial = serial_information;
    //
    //Serial.println(recieved_serial[0]);
    switch(serial_information[0]){
      case 'B':
        if(stepper_is_calibrated == 0){
          //Serial.println("STEPPER IS NOT CALLIBRATED!!!");
          //error_message = "CS";
          ready_for_new_command = 1;
          Serial.println("DONE");
        }else{
          step_delay = default_step_delay;
          start_bracket = (serial_information.indexOf('('));
          //punctuation = (serial_information.indexOf(','));
          end_bracket = (serial_information.indexOf(')'));
          wanted_position = serial_information.substring(start_bracket + 1, end_bracket).toInt();
          //step_delay = recieved_serial.substring(punctuation + 1, end_bracket).toInt();
          //Serial.print("move_to_position: ");
          //Serial.println(wanted_position);
          enable_on_break_off();
          move_to_position(wanted_position, step_delay);
          //correct_achieved_position(wanted_position, step_delay, allowed_cycle_number);
          enable_off_break_on();
          ready_for_new_command = 1;
          kicker_position = wanted_position;
          Serial.println("DONE");
        }
        break;
      case 'C':
        //Serial.println("Calibrate stepper with preset values.");
        step_delay = default_step_delay;
        //start_bracket = (received_command.indexOf('('));
        //end_bracket = (received_command.indexOf(')'));
        //step_delay = received_command.substring(start_bracket + 1, end_bracket).toInt();
        enable_on_break_off();
        trigger_servo_hold();
        use_preset_values_for_stepper_calibration(step_delay);
        prepare_for_shooting(step_delay);
        enable_off_break_on();
        stepper_is_calibrated = 1;
        ready_for_new_command = 1;
        //sei();
        Serial.println("DONE");
        break;
      case 'F':
        digitalWrite(angle_enable_pin, angle_enable_on);
        step_delay = angle_default_step_delay;
        //Serial.println("Calibrate angle_stepper with prest values.");
        //start_bracket = (received_command.indexOf('('));
        //end_bracket = (received_command.indexOf(')'));
        //step_delay = received_command.substring(start_bracket + 1, end_bracket).toInt();
        use_preset_values_for_angle_stepper_calibration(step_delay);
        angle_stepper_is_calibrated = 1;
        ready_for_new_command = 1;
        break;
      case 'E':
        if(angle_stepper_is_calibrated == 0){
          //Serial.println("ANGLE STEPPER IS NOT CALLIBRATED!!!");
          //error_message = "CSM";
          Serial.println("DONE");
          ready_for_new_command = 1;
        }else{
          step_delay = angle_default_step_delay;
          start_bracket = (serial_information.indexOf('('));
          //punctuation = (serial_information.indexOf(','));
          end_bracket = (serial_information.indexOf(')'));
          wanted_angle = serial_information.substring(start_bracket + 1, end_bracket).toInt();
          //step_delay = serial_information.substring(punctuation + 1, end_bracket).toInt();
          /*//
          Serial.print("move_to_angle: ");
          Serial.println(wanted_angle);
          //*/
          digitalWrite(angle_enable_pin, angle_enable_on);
          move_to_angle(wanted_angle, step_delay);
          //correct_achieved_angle(wanted_angle, step_delay, allowed_cycle_number);
          angle_stepper_is_calibrated = 1;
          ready_for_new_command = 1;
          kicker_angle = wanted_angle;
          Serial.println("DONE");
        }
        break;
      case 'G':
        if(trigger_ready == 1){
          trigger_ready = 0;
          digitalWrite(angle_enable_pin, angle_enable_on);
          //Serial.println("SHOOT!!!");
          trigger_servo_release();
          delay(trigger_servo_release_delay );
          enable_on_break_off();
          prepare_for_shooting(step_delay);
          delay(2000);
          enable_off_break_on();
          //digitalWrite(angle_enable_pin, angle_enable_off);
          ready_for_new_command = 1;
          trigger_flag = 1;
          kicker_position = default_shoot_position;
          Serial.println("DONE");
        }else{
          //error_message = "NRA";
          //Serial.println("not ready to shoot!!!");
          ready_for_new_command = 1;
          Serial.println(":(");
        }
        break;
      case 'R':        
        Serial.print("Calibration: ");
        Serial.print(calibration_flag);
        Serial.print(", trigger: ");
        Serial.print(calibration_flag);
        Serial.print(", position ");
        Serial.print(kicker_position);
        Serial.print(", angle ");
        Serial.println(kicker_angle);
        ready_for_new_command = 1;
        break;
    }
  }
    new_command = 0;
 }
//    //
//    if(serial_information.indexOf("move_to_position")>-1){
//    //move_to_position(50, 5000)
//    if(stepper_is_calibrated == 0){
//      //Serial.println("STEPPER IS NOT CALLIBRATED!!!");
//    }else{
//      start_bracket = (serial_information.indexOf('('));                      //  these functions get
//      punctuation = (serial_information.indexOf(','));                      //  the indexes of wanted
//      end_bracket = (serial_information.indexOf(')'));                        //  string
//      wanted_position = serial_information.substring(start_bracket + 1, punctuation).toInt();
//      step_delay = serial_information.substring(punctuation + 1, end_bracket).toInt();
//      /*//
//      Serial.print("Start bracket: ");
//      Serial.println(start_bracket);
//      Serial.print("First punctuation: ");
//      Serial.println(punctuation_1);
//      Serial.print("End bracket: ");
//      Serial.println(end_bracket);                         // trubleshooting block
//      Serial.print("Wanted position: ");
//      Serial.println(wanted_position);
//      Serial.print("Speed of gantry: ");
//      Serial.println(speed_of_gantry);
//      //*/
//      enable_on_break_off();
//      move_to_position(wanted_position, step_delay);
//      correct_achieved_position(wanted_position, step_delay, allowed_cycle_number);
//      enable_off_break_on();
//      /*//
//      Serial.print("SCALE POSITION ");
//      Serial.print(scale_min);
//      Serial.print(" - ");
//      Serial.print(scale_max);
//      Serial.print(": ");
//      Serial.println(scale_position);
//      //
//      Serial.print("STEPS POSITION ");
//      Serial.print("0 - ");
//      Serial.print(step_range);
//      Serial.print(": ");
//      Serial.println(step_position);
//      Serial.println();
//      //*/
//      }
//  }else if(serial_information.indexOf("calibrate_stepper")>-1){
//    //calibrate_stepper(5000)
//    uint8_t start_bracket = (serial_information.indexOf('('));
//    uint8_t end_bracket = (serial_information.indexOf(')'));
//    unsigned int step_delay = serial_information.substring(start_bracket + 1, end_bracket).toInt();
//    //Serial.println("Calibrating stepper");
//    enable_on_break_off();
//    calibrate_stepper(step_delay);
//    enable_off_break_on();
//    stepper_is_calibrated = 1;
//    //Serial.println("Done calibrating.\n");
//  }else if(serial_information.indexOf("stepper_calibrate_from_preset")>-1){
//    //stepper_calibrate_from_preset(5000)
//    uint8_t start_bracket = (serial_information.indexOf('('));
//    uint8_t end_bracket = (serial_information.indexOf(')'));
//    unsigned int step_delay = serial_information.substring(start_bracket + 1, end_bracket).toInt();
//    Serial.println("Calibrating angle stepper");
//    use_preset_values_for_stepper_calibration(step_delay);
//    stepper_is_calibrated = 1;
//    Serial.println("Done calibrating.\n");
//  }else if(serial_information.indexOf("move_to_angle")>-1){
//    //move_to_angle(90, 5000)
//    if(angle_stepper_is_calibrated == 0){
//      Serial.println("STEPPER IS NOT CALLIBRATED!!!");
//    }else{
//      uint8_t start_bracket = (serial_information.indexOf('('));                      //  these functions get
//      uint8_t punctuation_1 = (serial_information.indexOf(','));                      //  the indexes of wanted
//      uint8_t end_bracket = (serial_information.indexOf(')'));                        //  string
//      uint8_t wanted_position = serial_information.substring(start_bracket + 1, punctuation_1).toInt();
//      unsigned int step_delay = serial_information.substring(punctuation_1 + 1, end_bracket).toInt();
//      /*Serial.print("Start bracket: ");
//      Serial.println(start_bracket);
//      Serial.print("First punctuation: ");
//      Serial.println(punctuation_1);
//      Serial.print("End bracket: ");
//      Serial.println(end_bracket);                         // trubleshooting block
//      Serial.print("Wanted position: ");
//      Serial.println(wanted_position);
//      Serial.print("Speed of gantry: ");
//      Serial.println(speed_of_gantry);*/
//      move_to_angle(wanted_position, step_delay);
//      correct_achieved_angle(wanted_position, step_delay, allowed_number_of_angle_cycles);
//      /*//
//      Serial.print("ANGLE SCALE POSITION ");
//      Serial.print(min_angle);
//      Serial.print(" - ");
//      Serial.print(max_angle);
//      Serial.print(": ");
//      Serial.println(angle_scale_position);
//      //
//      Serial.print("ANGLE STEPS POSITION ");
//      Serial.print("0 - ");
//      Serial.print(angle_step_range);
//      Serial.print(": ");
//      Serial.println(angle_step_position);
//      Serial.println();
//      //*/
//    }
//  }else if(serial_information.indexOf("calibrate_angle_stepper")>-1){
//    //calibrate_angle_stepper(5000)
//    uint8_t start_bracket = (serial_information.indexOf('('));
//    uint8_t end_bracket = (serial_information.indexOf(')'));
//    unsigned int step_delay = serial_information.substring(start_bracket + 1, end_bracket).toInt();
//    Serial.println("Calibrating angle stepper");
//    calibrate_angle_stepper(step_delay);
//    angle_stepper_is_calibrated = 1;
//    Serial.println("Done calibrating.\n");
//  }else if(serial_information.indexOf("stepper_angle_calibrate_from_preset")>-1){
//    //stepper_angle_calibrate_from_preset(5000)
//    uint8_t start_bracket = (serial_information.indexOf('('));
//    uint8_t end_bracket = (serial_information.indexOf(')'));
//    unsigned int step_delay = serial_information.substring(start_bracket + 1, end_bracket).toInt();
//    Serial.println("Calibrating angle stepper");
//    use_preset_values_for_angle_stepper_calibration(step_delay);
//    angle_stepper_is_calibrated = 1;
//    Serial.println("Done calibrating.\n");
//  }
}

//***************************************************************************************************************************************//
//I2C FUNCTIONS

void slave_I2C_setup(){
/*DESCRIPTION: Sets up the I2C connection (arduino as slave).
 */
  Wire.begin(slave_address);        //join i2c bus
  Wire.onReceive(receive_command);  //function to trigger on recieve
  Wire.onRequest(send_command);     //function to trigger on request
}

void receive_command(int how_many){
/*DESCRIPTION: Recieves punctuationnd.
 */
 //Serial.println("command request");
  if(ready_for_new_command == 1){
     received_command="";            //remove previous string
     char c;                         //buffer char
     while(Wire.available() != 0){   //while there are bytes to be read
       c = Wire.read();              //save char
       received_command = received_command + c;    //construct string from chars
     }
     new_command = 1;                //flag - new punctuationnd recieved
     ready_for_new_command = 0;      //flag - not ready for new punctuationnd
     //Serial.println(received_command);     //for trubleshooting
  }else{
    char c;
    while(Wire.available() != 0){   //while there are bytes to be read
       c = Wire.read();              //save char
       //received_command = received_command + c;    //construct string from chars
    }
    //Serial.println("Not ready for new command");     //for trubleshooting
  }
}

void send_command(){
/*DESCRIPTION: Sends status: 
 * 
 */
  //char availability_message;
  //char trigger_message;
  //char ball_is_in_position;
  char message_for_jeston[3];
  if(analogRead(IR_ball_pin) > IR_ball_threshold){
    //ball_is_in_position = 'P';      //ball in position
    message_for_jeston[2]='P';
  }else{
    //ball_is_in_position = 'O';      //ball out of position
    message_for_jeston[2]='O';
  }
  /*
  if(error_message != "NO ERROR"){
    //Serial.println("ERROR");
    Wire.write(error_message);
    error_message = "NO ERROR";
  }else{*/
    if(trigger_ready == 1){
      //trigger_message = 'R';        //trigger ready
      message_for_jeston[1]='R';
    }else{
      //trigger_message = 'N';        //trigger not ready
      message_for_jeston[1]='N';
    }
    if(ready_for_new_command == 1){
      //availability_message = 'A';   //available for new command
      message_for_jeston[0]='A';
    }else{
      //availability_message = 'B';   //busy for new punctuationnds
      message_for_jeston[0]='B';
    }
    /*
    Wire.write(availability_message); //send availability message
    Wire.write(trigger_message);      //send trigger message
    Wire.write(ball_is_in_position);
    */
    Wire.write(message_for_jeston);      //send trigger message
    //error_message = "NO ERROR";
  //}
}

void take_I2C_request(){
/*DESCRIPTION: Takes I2C requests.
 */
 //Serial.println(digitalRead(3));
 if(digitalRead(3) == 0){
  uint8_t number_of_failed_checks = 0;
  for(uint8_t i = 0; i<30; i++){
    if(digitalRead(3) == 0){
      number_of_failed_checks++;
      delayMicroseconds(10);
    }
  }
  if(number_of_failed_checks >= 20){
  digitalWrite(enable_pin, enable_on);
  delay(enable_break_delay);
  digitalWrite(break_pin, break_off);
  //Serial.println("Estop");
  digitalWrite(dir_pin, back_dir);
  while(digitalRead(2) == 1){
    digitalWrite(step_pin, 1);
    delayMicroseconds(min_HIGH_pulse);
    digitalWrite(step_pin, 0);
    delayMicroseconds(default_step_delay - min_HIGH_pulse);
    //Serial.println("BACK");
  }
  //cli();
  digitalWrite(break_pin, break_on);
  delay(enable_break_delay);
  digitalWrite(enable_pin, enable_off);
  while(digitalRead(3) == 0){
    //Serial.println("BRICKED");
  }
  step_position = 0;
  scale_position = scale_min;
  new_command = 0;
  ready_for_new_command = 1;
  }
 }else{
 //Serial.println("JOEEEEEEEEEE");
  if(new_command == 1){
    new_command = 0;
    switch(received_command[0]){
      //Serial.println("mamamamamam");
      int start_bracket;
      int end_bracket;
      int punctuation;
      int step_delay;
      int wanted_position;
      int wanted_angle;
      case 'A':
        //Serial.println("Calibrate stepper.");
        step_delay = default_step_delay;
        //start_bracket = (received_command.indexOf('('));
        //end_bracket = (received_command.indexOf(')'));
        //step_delay = received_command.substring(start_bracket + 1, end_bracket).toInt();
        enable_on_break_off();
        calibrate_stepper(step_delay);
        prepare_for_shooting(step_delay);
        enable_off_break_on();
        stepper_is_calibrated = 1;
        ready_for_new_command = 1;
        break;
      case 'B':
        if(stepper_is_calibrated == 0){
          //Serial.println("STEPPER IS NOT CALLIBRATED!!!");
          //error_message = "CS";
          ready_for_new_command = 1;
        }else{
          step_delay = default_step_delay;
          start_bracket = (received_command.indexOf('('));
          //punctuation = (received_command.indexOf(','));
          end_bracket = (received_command.indexOf(')'));
          wanted_position = received_command.substring(start_bracket + 1, end_bracket).toInt();
          //step_delay = received_command.substring(punctuation + 1, end_bracket).toInt();
          //Serial.print("move_to_position: ");
          //Serial.println(wanted_position);
          enable_on_break_off();
          move_to_position(wanted_position, step_delay);
          //correct_achieved_position(wanted_position, step_delay, allowed_cycle_number);
          enable_off_break_on();
          ready_for_new_command = 1;
        }
        break;
      case 'C':
        //Serial.println("Calibrate stepper with preset values.");
        step_delay = default_step_delay;
        //start_bracket = (received_command.indexOf('('));
        //end_bracket = (received_command.indexOf(')'));
        //step_delay = received_command.substring(start_bracket + 1, end_bracket).toInt();
        enable_on_break_off();
        trigger_servo_hold();
        use_preset_values_for_stepper_calibration(step_delay);
        prepare_for_shooting(step_delay);
        enable_off_break_on();
        stepper_is_calibrated = 1;
        ready_for_new_command = 1;
        //sei();
        break;
      case 'D':
        //Serial.println("Calibrate angle_stepper.");
        step_delay = angle_default_step_delay;
        //start_bracket = (received_command.indexOf('('));
        //end_bracket = (received_command.indexOf(')'));
        //step_delay = received_command.substring(start_bracket + 1, end_bracket).toInt();
        digitalWrite(angle_enable_pin, angle_enable_on);
        calibrate_angle_stepper(step_delay);
        move_to_angle(max_angle, step_delay);
        //digitalWrite(angle_enable_pin, angle_enable_off);
        angle_stepper_is_calibrated = 1;
        ready_for_new_command = 1;
        //digitalWrite(angle_enable_pin, angle_enable_off);
        break;
      case 'E':
        if(angle_stepper_is_calibrated == 0){
          //Serial.println("ANGLE STEPPER IS NOT CALLIBRATED!!!");
          //error_message = "CSM";
          ready_for_new_command = 1;
        }else{
          step_delay = angle_default_step_delay;
          start_bracket = (received_command.indexOf('('));
          //punctuation = (received_command.indexOf(','));
          end_bracket = (received_command.indexOf(')'));
          wanted_angle = received_command.substring(start_bracket + 1, end_bracket).toInt();
          //step_delay = received_command.substring(punctuation + 1, end_bracket).toInt();
          /*//
          Serial.print("move_to_angle: ");
          Serial.println(wanted_angle);
          //*/
          digitalWrite(angle_enable_pin, angle_enable_on);
          move_to_angle(wanted_angle, step_delay);
          //correct_achieved_angle(wanted_angle, step_delay, allowed_cycle_number);
          angle_stepper_is_calibrated = 1;
          ready_for_new_command = 1;
        }
        break;
      case 'F':
        digitalWrite(angle_enable_pin, angle_enable_on);
        step_delay = angle_default_step_delay;
        //Serial.println("Calibrate angle_stepper with prest values.");
        //start_bracket = (received_command.indexOf('('));
        //end_bracket = (received_command.indexOf(')'));
        //step_delay = received_command.substring(start_bracket + 1, end_bracket).toInt();
        use_preset_values_for_angle_stepper_calibration(step_delay);
        angle_stepper_is_calibrated = 1;
        ready_for_new_command = 1;
        break;
      case 'G':
        if(trigger_ready == 1){
          trigger_ready = 0;
          //Serial.println("SHOOT!!!");
          trigger_servo_release();
          delay(trigger_servo_release_delay );
          enable_on_break_off();
          prepare_for_shooting(step_delay);
          delay(2000);
          enable_off_break_on();
          ready_for_new_command = 1;
        }else{
          //error_message = "NRA";
          //Serial.println("not ready to shoot!!!");
          ready_for_new_command = 1;
        }
        break;
      case 'H':
        // this function goes to ENDstop(it needs to start in the max position) and counts the steps in step_range
        step_delay = default_step_delay;
        step_range = 0;
        enable_on_break_off();
        //Serial.println("-going to ENDstop");
        while(ENDstop_state == 1){
          digitalWrite(step_pin, 1);
          delayMicroseconds(min_HIGH_pulse);
          digitalWrite(step_pin, 0);
          delayMicroseconds(step_delay - min_HIGH_pulse);
          step_range++;
        }
        //Serial.println("-ENDstop reached");
        //Serial.print("Step range: ");
        //Serial.println(step_range);
        enable_off_break_on();
        ready_for_new_command = 1;
        break;
      case 'I':
        enable_on_break_off();
        // this function gives the IR_values for calibration, step_range needs to be preset
        while(ENDstop_state == 1){
          digitalWrite(step_pin, 1);
          delayMicroseconds(min_HIGH_pulse);
          digitalWrite(step_pin, 0);
          delayMicroseconds(step_delay - min_HIGH_pulse);
        }
        step_position = 0;
        scale_position = scale_min;
        unsigned int steps_needed;
        uint8_t distance_step = (scale_max - scale_min) / IR_number_of_samples;
        //this variable controlls the step between the distance measurments (try to pick values that will give whole numbers)
        for(int i=0; i<=IR_number_of_samples; i++){                               //
          IR_distances[i] = scale_min + (distance_step * i);    //create the distance values
          steps_needed = map(IR_distances[i], scale_min, scale_max, 0, step_range) - step_position;
          move_steps(forward_dir, steps_needed, step_delay);
          delay(100);
          IR_readings[i] = IR_get_current_position();
        }
        //IR_steps_per_section();
        step_position = step_range;
        scale_position = scale_max;
        //stepper_is_calibrated = 1;
        IR_MIN_position = IR_readings[0];
        IR_MAX_position = IR_readings[IR_number_of_samples];
        /*//
        Serial.print("IR_MIN_position: ");
        Serial.println(IR_MIN_position);
        Serial.print("IR_MAX_position: ");
        Serial.println(IR_MAX_position);            // troubleshooting block
        Serial.print("STEP RANGE: ");
        Serial.println(step_range);
        Serial.print("STEPS POSITION: ");
        Serial.println(step_position);
        Serial.print("\n");
        for(int i; i<=IR_number_of_samples; i++){
          Serial.print("Scale value= ");
          Serial.print(IR_distances[i]);
          Serial.print(" || ");
          Serial.print(IR_readings[i]);
          Serial.println(" = IR readings");
        }
        enable_off_break_on();
        ready_for_new_command = 1;
        //*/
      default:
        //Serial.println(received_command);
        //error_message = "UCA";
        ready_for_new_command = 1;
        Serial.println("DONE");
        break;
    }
    new_command = 0;
  }
 }
}

//***************************************************************************************************************************************//
//BUTTON TESTING FUNCTIONS
/*
void button_testing_setup(){
  pinMode(button_forward, INPUT_PULLUP);
  pinMode(button_back, INPUT_PULLUP);
  pinMode(speed_pot, INPUT);
}

void take_button_request(int step_delay){
  if(digitalRead(button_forward) == 0){
    move_steps(forward_dir, 1, step_delay);
  }else if(digitalRead(button_back) == 0){
    move_steps(back_dir, 1, step_delay);
  }
}

int pot_step_delay(){
  int step_delay = map(analogRead(speed_pot), 0, 1023, 1, 10) * 1000;
  return step_delay;
}
*/
//***************************************************************************************************************************************//
//ENDSTOP AND FARSTOP SETUP FUNCTIONS

void FARstop_setup(){
/*HARDWARE: On the pin D10 connect a 10k resisstor and a 104 capaccitor to GND. Connect an ENDstop (normally closed) to D2 and 5V.
 *DESCRIPTION: Sets up FARstop.
 */
  pinMode(FARstop_pin, INPUT);
  digitalWrite(FARstop_pin, LOW);
}

void ENDstop_setup(){
/*HARDWARE: On the pin D2 connect a 10k resisstor and a 104 capaccitor to GND. Connect an ENDstop (normally closed) to D2 and 5V.
 *DESCRIPTION: Sets up INT0 interrupt.
 */
 DDRB&=~(1<<2);   //Turn D2 into input
 PORTD&=~(1<<2);  //Write 0 on D2
 ENDstop_state = digitalRead(2);  //check if ENDstop is pressed at the start
 EICRA|=(1<<ISC00);   //Interrupt triggered by any logic change
 EIMSK|=(1<<INT0);    //Enable interrupt INT0
 //sei();   //General enable interrupts
}

void go_to_FARstop(unsigned int step_delay){
/*DESCRIPTION: Moves the stepper to the FARstop.
 */
  digitalWrite(dir_pin, forward_dir);
  while(digitalRead(FARstop_pin) == 1){
    digitalWrite(step_pin, 1);
    delayMicroseconds(min_HIGH_pulse);
    digitalWrite(step_pin, 0);
    delayMicroseconds(step_delay - min_HIGH_pulse);
  }
  //step_position = step_range;
  //scale_position = scale_max;
}

void go_to_ENDstop(unsigned int step_delay){
/*DESCRIPTION: Moves the stepper to the ENDstop.
 */
  digitalWrite(enable_pin, enable_on);
  digitalWrite(dir_pin, back_dir);
  while(digitalRead(ENDstop_pin) == 1){
    digitalWrite(step_pin, 1);
    delayMicroseconds(min_HIGH_pulse);
    digitalWrite(step_pin, 0);
    delayMicroseconds(step_delay - min_HIGH_pulse);
  }
  step_position = 0;
  scale_position = 0;
}

//***************************************************************************************************************************************//
//ANGLE ENDSTOP AND FARSTOP FUNCTIONS

void angle_FARstop_setup(){
/*HARDWARE: On the pin A3 connect a 10k resisstor GND. Connect a normally closed button connected to 5V.
 *DESCRIPTION: Sets up PCINT0 interrupt.
 */
  DDRC&=~(1<<3);
  PORTC&=~(1<<3);
  PCICR|=(1<<PCIE1);
  PCMSK1|=(1<<PCINT11);
  angle_FARstop_state = digitalRead(angle_FARstop_pin);
  digitalWrite(angle_FARstop_pin, 1);
}

void angle_ENDstop_setup(){
/*HARDWARE: On the pin D12 connect a 10k resisstor GND. Connect a normally closed button connected to 5V.
 *DESCRIPTION: Sets up PCINT1 interrupt.
 */
  DDRB&=~(1<<4);
  PORTB&=~(1<<4);
  PCICR|=(1<<PCIE0);
  PCMSK0|=(1<<PCINT4);
  angle_ENDstop_state = digitalRead(angle_ENDstop_pin);
}

void angle_go_to_FARstop(unsigned int step_delay){
/*HARDWARE: Normaly closed ENDstop connected to A3 and 5V, 10k resistor connected to D12 and GND.
 *DESCRIPTION: Moves the angle_stepper to the angle_ENDstop.
 */
  digitalWrite(angle_dir_pin, up_dir);
  angle_FARstop_state = digitalRead(angle_FARstop_pin);
  //Serial.println("-going to angle_FARstop");
  while(angle_FARstop_state == 1){
    digitalWrite(angle_step_pin, 1);
    delayMicroseconds(angle_min_HIGH_pulse);
    digitalWrite(angle_step_pin, 0);
    delayMicroseconds(step_delay - angle_min_HIGH_pulse);
  }
  //Serial.println("-angle_FARstop reached");
  //angle_step_position = angle_step_range;
  //angle_scale_position = max_angle;
}

void angle_go_to_ENDstop(unsigned int step_delay){
/*HARDWARE: Normaly closed ENDstop connected to D12 and 5V, 10k resistor connected to D12 and GND.
 *DESCRIPTION: Moves the angle_stepper to the angle_ENDstop.
 */
  digitalWrite(angle_dir_pin, down_dir);
  angle_ENDstop_state = digitalRead(angle_ENDstop_pin);
  //Serial.println("-going to angle_ENDstop");
  while(angle_ENDstop_state == 1){
    digitalWrite(angle_step_pin, 1);
    delayMicroseconds(angle_min_HIGH_pulse);
    digitalWrite(angle_step_pin, 0);
    delayMicroseconds(step_delay - angle_min_HIGH_pulse);
  }
  //Serial.println("-angle_ENDstop reached");
  //angle_step_position = 0;
  //angle_scale_position = min_angle;
}

//***************************************************************************************************************************************//
//EMERGENCY STOP SETUP

void interrupt_setup_emergency_stop(){
/*HARDWARE: On the pin D3 connect a 10k resisstor GND. Connect a normally closed button connected to 5V.
 *DESCRIPTION: Sets up INT1 interrupt.
 */
 DDRB&=~(1<<3);   //Turn D2 into input
 PORTD&=~(1<<3);  //Write 0 on D2
 //EICRA&=~((1<<ISC10)|(1<<ISC11));   //INT1 triggered at LOW state
 EICRA = 0;
 EIMSK|=(1<<INT1);    //Enable interrupt INT1
 sei();   //General enable interrupts
}

//***************************************************************************************************************************************//
//INTERRUPT SERVICE RUTINES

ISR(INT1_vect) {
/*DESCRIPTION: Goes to ENDstop and bricks the system 
 */
  digitalWrite(enable_pin, enable_on);
  delay(enable_break_delay);
  digitalWrite(break_pin, break_off);
  //Serial.println("Estop");
  digitalWrite(dir_pin, back_dir);
  while(digitalRead(2) == 1){
    digitalWrite(step_pin, 1);
    delayMicroseconds(min_HIGH_pulse);
    digitalWrite(step_pin, 0);
    delayMicroseconds(default_step_delay - min_HIGH_pulse);
    //Serial.println("BACK");
  }
  cli();
  digitalWrite(break_pin, break_on);
  delay(enable_break_delay);
  digitalWrite(enable_pin, enable_off);
  while(digitalRead(3) == 0){
    //Serial.println("BRICKED");
  }
  step_position = 0;
  scale_position = scale_min;
  resetFunc();
}

ISR(INT0_vect) {
/*DESCRIPTION: Writes the state of ENDstop to ENDstop_state, also handles the switching of interrupt modes.
 */
  ENDstop_state=digitalRead(2);   //save state of ENDstop
  if(ENDstop_state == 0){     //ENDstop is pressed
    EICRA|=(1<<ISC00)|(1<<ISC01);   //INT0 triggered at rissing edge
    delay(10);            //delay due to noise
    EIFR&=~(1<<INTF0);    //empty INT0 interrupt flag
    digitalWrite(A0, HIGH);
  }else{                      //ENDstop isn't pressed
    EICRA&=~((1<<ISC00)|(1<<ISC01)); //INT0 triggered at LOW level
    delay(10);            //delay due to noise
    EIFR&=~(1<<INTF0);    //empty INT0 interrupt flag
    digitalWrite(A0, LOW);
  }
}

ISR(PCINT0_vect) {
/*DESCRIPTION: Writes the state of angle_ENDstop to angle_ENDstop_state.
 */
 angle_ENDstop_state = digitalRead(angle_ENDstop_pin);
 //Serial.print("angle_ENDstop_state: ");
 //Serial.println(angle_ENDstop_state);
}

ISR(PCINT1_vect) {
/*DESCRIPTION: Writes the state of angle_FARstop to angle_FARstop_state.
 */
 angle_FARstop_state = digitalRead(angle_FARstop_pin);
 //Serial.print("angle_FARstop_state: ");
 //Serial.println(angle_FARstop_state);
}
