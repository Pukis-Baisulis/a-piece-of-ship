/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

//#define RADIO_MODE
#define STABILIZATION
#define DEBUG 
#define IGNORE_ERRORS
#define RADIO_MODE 

#include "Wire.h"
#include <MPU6050_light.h>
#include <VL53L0X.h>
#include <Servo.h>

#define BAUD_RATE 9600

#define SERVO_A_PIN 5
#define SERVO_B_PIN 6
#define SERVO_S_PIN 9
#define MOT_A_PIN 10
#define MOT_B_PIN 11

#define VLX_L_PIN A1
#define VLX_F_PIN A0
#define VLX_R_PIN A2

#define START_PIN 8
#define LED_A_PIN 12
#define LED_B_PIN 13
#define LED_C_PIN 7



MPU6050 mpu(Wire);

#define VLX_THRESHOLD_ERROR 8193
VL53L0X vlx_left;
bool vlxLeftOn = false; 
#define VLX_L_ADDRESS 0x37
#define VLX_L_AVG_VAL 600
#define VLX_L_MAX_RANGE 1200
#define VLX_L_MIN_RANGE 60
#define VLX_L_TIMING_BUDGET 20000
#define VLX_L_TIMEOUT 500

VL53L0X vlx_front;
bool vlxFrontOn = false; 
#define VLX_F_ADDRESS 0x38
#define VLX_F_AVG_VAL 600
#define VLX_F_MAX_RANGE 1200
#define VLX_F_MIN_RANGE 60
#define VLX_F_TIMING_BUDGET 20000
#define VLX_F_TIMEOUT 500

VL53L0X vlx_right;
bool vlxRightOn = false; 
#define VLX_R_ADDRESS 0x39
#define VLX_R_AVG_VAL 600
#define VLX_R_MAX_RANGE 1200
#define VLX_R_MIN_RANGE 60
#define VLX_R_TIMING_BUDGET 20000
#define VLX_R_TIMEOUT 500

Servo servoA;
#define LIM_A_MIN 85
#define LIM_A_MAX 95
#define SERVO_A_MIDPOINT 90
Servo servoB;
#define LIM_B_MIN 85
#define LIM_B_MAX 95
#define SERVO_B_MIDPOINT 90
Servo servoS;
#define LIM_S_MIN 85
#define LIM_S_MAX 95
#define SERVO_S_MIDPOINT 90

Servo motA;
Servo motB;
#define MOT_MIN_PULSE 1000
#define MOT_MAX_PULSE 2000
#define ENGINE_BOOST_TIME 50
#define ENGINE_SHUTDOWN_TIME 100
#define FRONT_FLIP_CORRECT 50 // amount to add to motor speed


#ifdef RADIO_MODE
  #include <ServoInput.h>
  #define RADIO_CH_1 2
  #define RADIO_CH_2 3
  ServoInputPin<RADIO_CH_1> steering(1000, 2000);
  ServoInputPin<RADIO_CH_2> throttle(1000, 2000);
#endif 


int maxLoopTime = 0; 
#define ABSOLUTE_LOOPTIME_LIMIT 10000 
unsigned long milk = 0; // whole loop timer 

const double kP_X = 1;  //   ^
const double kI_X = 0;  //   |
double Ix = 0;          //   |
const double kD_X = 0;  //   |
double lastErr_X = 0;

const double kP_Y = 1;  //
const double kI_Y = 0;  // <--->
double Iy = 0;          //
const double kD_Y = 0;  //
double lastErr_Y = 0;

double X_dat=0;
#define X_ROT_TARGET 0
#define X_ROT_MAX 5 
#define X_ROT_MIN -5 

double Y_dat=0;
int Y_ROT_TARGET = 0;
#define Y_ROT_MAX 5
#define Y_ROT_MIN -5 

double Z_dat=0;

int dist[3]={0,0,0};
int speed = 0; // -100
#define BLDC_0_POINT 90
int direction = 0; 

void init_all();
void sensors();
void stabilize();
void sail(int spd, int steer);
void signalCode(byte code);
void VLXdiagnostics();
void extremeSituations(int situation);

void setup() {
  init_all();
  #ifdef RADIO_MODE 
    while(1)
    {
      sensors();
      stabilize();
      sail(speed, direction);
    }
  #endif
}

void loop() {
  milk = micros();
  {
    sensors();
    stabilize();
  }
	int milk_1 = micros() - milk;
  if(milk_1 > maxLoopTime){
    maxLoopTime = milk_1;
    if(milk_1> ABSOLUTE_LOOPTIME_LIMIT)
      maxLoopTime = ABSOLUTE_LOOPTIME_LIMIT;
  }
  while(milk - micros() < maxLoopTime) {
    delayMicroseconds(1);
  }
}

void sail(int spd, int steer) {

}

void stabilize() {
  if(X_dat > X_ROT_MAX)
    extremeSituations(1);
  if(X_dat < X_ROT_MIN)
    extremeSituations(0);

  double err_X = X_ROT_TARGET - X_dat;
  Ix += err_X;
  double pid_X = (err_X * kP_X) + (Ix * kI_X) + ((err_X-lastErr_X) * kD_X);
  lastErr_X = err_X;

  double err_Y = Y_ROT_TARGET - Y_dat;
  Iy += err_Y;
  double pid_Y = (err_Y * kP_Y) + (Iy * kI_Y) + ((err_Y-lastErr_Y) * kD_Y);
  lastErr_Y = err_Y;

  servoA.write(SERVO_A_MIDPOINT + constrain(pid_X-pid_Y, LIM_A_MIN, LIM_A_MAX));
  servoB.write(SERVO_B_MIDPOINT + constrain(pid_X+pid_Y, LIM_B_MIN, LIM_B_MAX));
}

void sensors() {
  mpu.update();

  Y_dat = mpu.getAngleY();
  if(Y_dat > 180){
    Y_dat = Y_dat - 360;
  }
  X_dat = mpu.getAngleX();
  if(X_dat > 180){
    X_dat = X_dat - 360;
  }
  Z_dat = mpu.getAngleZ();

  #ifndef RADIO_MODE
    dist[0] = VLX_L_AVG_VAL; // sets average values ins tead of errored value to protect stability in case of failure
    dist[1] = VLX_F_AVG_VAL;
    dist[2] = VLX_R_AVG_VAL;
    if(vlxLeftOn)
      dist[0] = vlx_left.readRangeContinuousMillimeters();
    if(vlxFrontOn)
      dist[1] = vlx_front.readRangeContinuousMillimeters();
    if(vlxRightOn)
      dist[2] = vlx_right.readRangeContinuousMillimeters();

    //senssor monitoring
    if(dist[0] > VLX_THRESHOLD_ERROR){//error detetion
        #ifdef DEBUG
          signalCode(111);
        #endif
        vlxLeftOn = false;
    }
    if(dist[0] > VLX_L_MAX_RANGE)//sensor max value cap
      dist[0] = VLX_L_MAX_RANGE;
    if(dist[0] < VLX_L_MIN_RANGE)//sensor max value cap
      dist[0] = 0;

    if(dist[0] > VLX_THRESHOLD_ERROR){//error detetion
        #ifdef DEBUG
          signalCode(111);
        #endif
        vlxLeftOn = false;
    }
    if(dist[0] > VLX_L_MAX_RANGE)//sensor max value cap
      dist[0] = VLX_L_MAX_RANGE;
    if(dist[0] < VLX_L_MIN_RANGE)//sensor max value cap
      dist[0] = 0;

    if(dist[0] > VLX_THRESHOLD_ERROR){//error detetion
        #ifdef DEBUG
          signalCode(111);
        #endif
        vlxLeftOn = false;
    }
    if(dist[0] > VLX_L_MAX_RANGE)//sensor max value cap
      dist[0] = VLX_L_MAX_RANGE;
    if(dist[0] < VLX_L_MIN_RANGE)//sensor max value cap
      dist[0] = 0;
  #endif

  #ifdef RADIO_MODE
    direction = 90 - steering.getAngle(); 
    speed = 90 - throttle.getAngle(); 
  #endif
}

void signalCode(byte code) {
    // 0 OK
    // 1 VLX left OK
    // 11 VLX left no init 
    // 111 VLX left not working 
    // 2 VLX front OK
    // 22 VLX front no init 
    // 122 VLX front not working 
    // 3 VLX right OK 
    // 33 VLX right no init 
    // 133 VLX right not working 
    // 4 MPU OK
    // 44 MPU no init
    // 5 MPU calibration started
    // 6 MPU calibration ended
    // 7 RADIO_MODE waiting for input
    // 255 total error;
  #ifdef DEBUG
    Serial.println(code);
  #endif
  switch(code){
    // ------------------------------vlx----------------------------------
    case 1: 
      digitalWrite(LED_A_PIN, HIGH);
      delay(200);
      digitalWrite(LED_A_PIN, LOW);
      delay(200);
      digitalWrite(LED_A_PIN, HIGH);
      delay(200);
      digitalWrite(LED_A_PIN, LOW);
      delay(200);
      break;

    case 11: 
      digitalWrite(LED_A_PIN, HIGH);
      delay(500);
      digitalWrite(LED_A_PIN, LOW);
      delay(300);
      digitalWrite(LED_A_PIN, HIGH);
      delay(500);
      digitalWrite(LED_A_PIN, LOW);
      delay(300);
      break;

    case 111:
      digitalWrite(LED_A_PIN, HIGH);
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_C_PIN, LOW);
      #ifdef DEBUG 
        while(1){delay(1);}
      #endif
      break;

    case 2: // front
      digitalWrite(LED_B_PIN, HIGH);
      delay(200);
      digitalWrite(LED_B_PIN, LOW);
      delay(200);
      digitalWrite(LED_B_PIN, HIGH);
      delay(200);
      digitalWrite(LED_B_PIN, LOW);
      delay(200);
      break;

    case 22:// right
      digitalWrite(LED_B_PIN, HIGH);
      delay(500);
      digitalWrite(LED_B_PIN, LOW);
      delay(300);
      digitalWrite(LED_B_PIN, HIGH);
      delay(500);
      digitalWrite(LED_B_PIN, LOW);
      delay(300);
      break;

    case 122:
      digitalWrite(LED_A_PIN, LOW);
      digitalWrite(LED_B_PIN, HIGH);
      digitalWrite(LED_C_PIN, LOW);
      #ifdef DEBUG 
        while(1){delay(1);}
      #endif
      break;

    case 33:
      digitalWrite(LED_C_PIN, HIGH);
      delay(500);
      digitalWrite(LED_C_PIN, LOW);
      delay(300);
      digitalWrite(LED_C_PIN, HIGH);
      delay(500);
      digitalWrite(LED_C_PIN, LOW);
      delay(300);
      break;

    case 3:
      digitalWrite(LED_C_PIN, HIGH);
      delay(200);
      digitalWrite(LED_C_PIN, LOW);
      delay(200);
      digitalWrite(LED_C_PIN, HIGH);
      delay(200);
      digitalWrite(LED_C_PIN, LOW);
      delay(200);
      break;

    case 133:
      digitalWrite(LED_A_PIN, LOW);
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_C_PIN, HIGH);
      #ifdef DEBUG 
        while(1){delay(1);}
      #endif
      break;
    // -----------------------------MPU---------------------------
    case 4:
      digitalWrite(LED_A_PIN, HIGH);
      digitalWrite(LED_B_PIN, HIGH);
      digitalWrite(LED_C_PIN, HIGH);
      delay(200);
      digitalWrite(LED_A_PIN, LOW);
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_C_PIN, LOW);
      delay(200);
      digitalWrite(LED_A_PIN, HIGH);
      digitalWrite(LED_B_PIN, HIGH);
      digitalWrite(LED_C_PIN, HIGH);
      delay(200);
      digitalWrite(LED_A_PIN, LOW);
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_C_PIN, LOW);
      delay(200);
      break;

    case 44:
      digitalWrite(LED_A_PIN, HIGH);
      digitalWrite(LED_B_PIN, HIGH);
      digitalWrite(LED_C_PIN, HIGH);
      delay(500);
      digitalWrite(LED_A_PIN, LOW);
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_C_PIN, LOW);
      delay(300);
      digitalWrite(LED_A_PIN, HIGH);
      digitalWrite(LED_B_PIN, HIGH);
      digitalWrite(LED_C_PIN, HIGH);
      delay(500);
      digitalWrite(LED_A_PIN, LOW);
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_C_PIN, LOW);
      delay(300);
      break;
    
    case 5:
      digitalWrite(LED_A_PIN, HIGH);
      digitalWrite(LED_B_PIN, HIGH);
      digitalWrite(LED_C_PIN, HIGH);
      break;
    
    case 6:
      digitalWrite(LED_A_PIN, LOW);
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_C_PIN, LOW);
      break;

    // others 

    case 7:
      digitalWrite(LED_A_PIN, HIGH);
      digitalWrite(LED_B_PIN, HIGH);
      digitalWrite(LED_C_PIN, HIGH);
      delay(100);
      digitalWrite(LED_A_PIN, LOW);
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_C_PIN, LOW);
      delay(100);
      break; 

    case 0:
      digitalWrite(LED_A_PIN, HIGH);
      digitalWrite(LED_B_PIN, HIGH);
      digitalWrite(LED_C_PIN, HIGH);
      delay(100);
      digitalWrite(LED_A_PIN, LOW);
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_C_PIN, LOW);
      delay(100);
      digitalWrite(LED_A_PIN, HIGH);
      digitalWrite(LED_B_PIN, HIGH);
      digitalWrite(LED_C_PIN, HIGH);
      delay(100);
      digitalWrite(LED_A_PIN, LOW);
      digitalWrite(LED_B_PIN, LOW);
      digitalWrite(LED_C_PIN, LOW);
      delay(100);
      break;

    
    default: 

      break;
  }
}

void init_all() {
  //coms
  #ifdef DEBUG
    Serial.begin(BAUD_RATE);
  #endif
  Wire.begin();

  //pinmodes
  pinMode(VLX_F_PIN, OUTPUT);
  pinMode(VLX_L_PIN, OUTPUT);
  pinMode(VLX_R_PIN, OUTPUT);
  pinMode(LED_A_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  pinMode(START_PIN, INPUT);

  //vlx
  #ifndef RADIO_MODE
    digitalWrite(VLX_F_PIN, LOW);
    digitalWrite(VLX_L_PIN, LOW);
    digitalWrite(VLX_R_PIN, LOW);
    delay(10);

    digitalWrite(VLX_L_PIN, HIGH);//----------------LEFT---------------------
    delay(10);
    if(vlx_left.init()) {
      vlxLeftOn = true;
      signalCode(1);
    }
    else
      signalCode(11);
    vlx_left.setAddress(VLX_L_ADDRESS);
    vlx_left.setMeasurementTimingBudget(VLX_L_TIMING_BUDGET);
    vlx_left.setTimeout(VLX_L_TIMEOUT);
    vlx_left.startContinuous();

    digitalWrite(VLX_F_PIN, HIGH);//--------------FRONT--------------
    delay(10);
    if(vlx_front.init()) {
      vlxFrontOn = true;
      signalCode(2);
    }
    else
      signalCode(22);
    vlx_front.setAddress(VLX_F_ADDRESS);
    vlx_front.setMeasurementTimingBudget(VLX_F_TIMING_BUDGET);
    vlx_front.setTimeout(VLX_F_TIMEOUT);
    vlx_front.startContinuous();


    digitalWrite(VLX_R_PIN, HIGH);//------------------RIGHT--------------------------
    delay(10);
    if(vlx_right.init()) {
      vlxRightOn = true;
      signalCode(3);
    }
    else
      signalCode(33);
    vlx_right.setAddress(VLX_R_ADDRESS);
    vlx_right.setMeasurementTimingBudget(VLX_R_TIMING_BUDGET);
    vlx_right.setTimeout(VLX_R_TIMEOUT);
    vlx_right.startContinuous();


    VLXdiagnostics();
  #endif

  //radio input
  #ifdef RADIO_MODE
    while (!ServoInput.available()) {  // wait for all signals to be ready
    #ifdef DEBUG
		  Serial.println("Waiting for servo signals...");
      signalCode(7);
    #endif
	  }
  #endif

  //servos
  servoA.attach(SERVO_A_PIN);
  servoB.attach(SERVO_B_PIN);
  servoS.attach(SERVO_S_PIN);

  servoA.write(SEVO_A_MIDPOINT);
  servoB.write(SEVO_B_MIDPOINT);
  servoS.write(SEVO_S_MIDPOINT);

  //motors
  motA.attach(MOT_A_PIN, MOT_MIN_PULSE, MOT_MAX_PULSE);
  motB.attach(MOT_B_PIN, MOT_MIN_PULSE, MOT_MAX_PULSE);

  //need to figure out the calibration sequence

  //mpu
  if(mpu.begin())
    signalCode(4);
  else
    signalCode(44);
  delay(1000); 
  mpu.calcOffsets();

  //final
  signalCode(0);
}

void VLXdiagnostics() {
  int tmp = vlx_left.readRangeContinuousMillimeters();
  if(tmp > VLX_THRESHOLD_ERROR || tmp < 0)
    signalCode(111);

  tmp = vlx_front.readRangeContinuousMillimeters();
  if(tmp > VLX_THRESHOLD_ERROR || tmp < 0)
    signalCode(122);

  tmp = vlx_right.readRangeContinuousMillimeters();
  if(tmp > VLX_THRESHOLD_ERROR || tmp < 0)
    signalCode(133);
}

void extremeSituations(int situation) {
  switch(situation) {
    case 0: // too much front rotation
      sail(speed+FRONT_FLIP_CORRECT, direction);
      delay(ENGINE_BOOST_TIME);
      break;

    case 1: // too much back rotation
      sail(0, direction);
      delay(ENGINE_SHUTDOWN_TIME);
      break;

    default:
      break;
  }
}



