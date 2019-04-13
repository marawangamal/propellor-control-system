//---------------------LIBRARIES---------------------------// 
/*
  Download the ESC library here: https://github.com/RB-ENantel/RC_ESC/
*/

#include "ESC.h"

//---------------------ESC SETTINGS-----------------------//
#define SPEED_MIN (1000)                                  // Set the Minimum Speed Setting of the ESC
#define SPEED_MAX (2000)                                  // Set the Maximum Speed Setting of the ESC

ESC myESC (9, SPEED_MIN, SPEED_MAX, 500);                 // Initialize ESC (ESC PIN, Minimum Value, Maximum Value, Default Speed)
int oESC;                                                 // Variable for the speed sent to the ESC


//---------------------ENCODER PORTS----------------------//
#define outputA 2                                         // Define Encoder output A to Digital Pin 6 on Arduino Uno
#define outputB 3                                         // Define Encoder output B to Digital Pin 7 on Arduino Uno
#define NUMPULSES 1200                                    // Define the number of pulses on Encoder resolution (enter twice the amount of pulses on datasheet)

//---------------------ENCODER VARIABLES------------------//
float counter = 0;                                        // Counter variable for the number of pulses from the encoder
volatile byte reading = 0;
volatile byte aFlag = 0;
volatile byte bFlag = 0;
double encoderPos = 0;
double oldEncPos = 0;
double angle = 0;

//---------------------TIMER VARIABLES--------------------//
long t_now;
long t_last_print = 0;
long t_last_PID = 0;
int T_sample = 20;                                        // sample time in milliseconds (ms)
int T_print = 500;                                         // sample print to monitor time in milliseconds (ms)


//---------------------PID VARIABLES----------------------//
// Define variables for the output of the encoder(sensed_output), angle output of the encoder(sensed_output converted to angle), and error w.r.t the setpoint
//double sensed_output, error, sensed_angle;
double error = 0;
double errSum = 0;
double dErr = 0;
double lastErr = 0;

// Define variables for total error over time, previous error in the last sampling time interval, control signal, and limitations to the controller
int max_control = 1700;
int min_control = 1000;

long control_signal = 1000;
long nowSpeed = SPEED_MIN;
long lastSpeed = SPEED_MIN;


// ==================INSERT DESIRED SETPOINT ANGLE HERE================== //
double setpoint_angle = 30;

// ==================INSERT CONTROL GAINS HERE===========
double Kp = 0.8;                                          // proportional gain
double Ki = 0.003;                                        // integral gain in [ms^-1]
double Kd = 120;                                          // derivative gain in [ms]
//Original Values: 0.8, 0.003, 120
//Best so far for 30deg: 0.7, 0.007, 1000-1200
//Best so far for sinusoid:

void setup() {
  /*
    Setup function to initialize serial plotter/monitor, initialize origin of encoder,
    arm ESC driver, and to initialize propeller by ramping up and down
  */
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(9, OUTPUT);

  Serial.begin(9600);

  Serial.println("Wait");
  myESC.arm();                                              // Send the Arm value so the ESC will be ready to take commands
  delay(1000);                                              // Wait for a while
  Serial.println("Begin");
  rampUpDown();
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(outputA), PinA, RISING);
  attachInterrupt(digitalPinToInterrupt(outputB) ,PinB ,RISING);

  t_last_PID = millis();
}


void loop() {
  angle = (encoderPos/NUMPULSES)*720;
  //setpoint_angle = 15*sin(2*3.14*0.0001*t_now) + 30;
  detachInterrupt(digitalPinToInterrupt(outputA));
  detachInterrupt(digitalPinToInterrupt(outputB));
  PID_control();
  attachInterrupt(digitalPinToInterrupt(outputA), PinA, RISING);
  attachInterrupt(digitalPinToInterrupt(outputB), PinB, RISING);
  myESC.speed(control_signal);
  print_results();
}

void rampUpDown() {
  /*
    Function written to test functionality of brushless motor by accelerating and decelerating the rotation
    of the motor. Speed of the motor does not accelerate to rated speed.

  */

  for (oESC = SPEED_MIN; oESC <= 1150; oESC += 1) {        // iterate from minimum speed to a speed setting of 1150
    myESC.speed(oESC);                                     // write speed setting to motor
    delay(20);                                             // waits 10ms for the ESC to reach speed
  }
  delay(2000);                                             // wait a while

  for (oESC = 1150; oESC >= SPEED_MIN; oESC -= 1) {        // iterate from speed setting of 1150 to minimum speed
    myESC.speed(oESC);                                     // write speed setting to motor
    delay(20);                                             // waits 10ms for the ESC to reach speed
   }
  delay(2000);                                             // Wait for a while before going into control loop
}

void rampUpTo(double target) {
  /*  Function ramps the output to the target rpm starting from the current rpm and reading encoder along the way
  */
  for (oESC = lastSpeed; oESC <= target; oESC += 1) {     // iterate from minimum speed to a speed setting of 1150
    myESC.speed(oESC);                                    // write speed setting to motor
  }
  lastSpeed = target;
}

void PID_control(){
  t_now = millis();
  if(t_now - t_last_PID > T_sample){
    // Proportional Control
    error = setpoint_angle - abs(angle);

    // Integral control
    errSum += error*T_sample;

    // Derivative control
    dErr = (error - lastErr)/T_sample;

    // PID
    control_signal = Kp*error + Ki*errSum + Kd*dErr + 1000;

    // Limit control_signal within bounds
    if(control_signal > max_control){
      control_signal = max_control;
    }

    // Update values
    lastErr = error;
    t_last_PID = t_now;
  }
}

void PinA() {
  cli();
  reading = PIND & 0xC;
  if(reading == B00001100 && aFlag) {
    encoderPos --;
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00000100){
    bFlag = 1;
  }
  sei();
}

void PinB() {
  cli();
  reading = PIND & 0xC;
  if(reading == B00001100 && bFlag) {
    encoderPos ++;
    bFlag = 0;
    aFlag = 0;
  }
  else if (reading == B00001000){
    aFlag = 1;
  }
  sei();
}

void print_results() {
  /*
    STUDENTS TO ANSWER
    Function to print the sensed output/angle to the setpoint every 50 ms. Use Serial plotter on Arduino to graphically plot the
    sensed angle with respect to the setpoint angle.

    HINT: You might want to print sensed_output to verify that the encoder is reading correctly
    HINT: You might also want to print control_signal to ensure the PID is working properly before writing it to the motor

  */

  t_now = millis();

  if (t_now - t_last_print >= T_print){
    t_last_print = t_now;
    Serial.print(30);
    Serial.print(" ");
    Serial.print(30*1.05); // overshoot
    Serial.print(" ");
    Serial.print(30*1.02); // +ve settling parameter
    Serial.print(" ");
    Serial.print(30-0.6); // -ve settling parameter
    Serial.print(" ");
    Serial.println(abs(angle));
  }
}
