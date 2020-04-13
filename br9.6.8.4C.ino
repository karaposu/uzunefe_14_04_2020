/**********Bu Kod el emegi göz nurudur.********/
/*
 * 
 * */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <PID_v1.h>
#include <TimerOne.h>
MPU6050 mpu;

        
// uncomment "Bluetooth" if you want to send gyro values over bluetooth
//#define Bluetooth

// uncomment "robot" if you want to see the actual pid values 
#define ROBOT


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL





#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)


//---------------------------------------------------TB6612 VARIABLES  ⇓⇓⇓⇓⇓⇓⇓⇓⇓
int STBY = 9; //standby
//Motor A
int PWMA = 6; //Speed control
int AIN1 = 8; //Direction
int AIN2 = 7; //Direction

//Motor B
int PWMB = 11; //Speed control
int BIN1 = 10; //Direction
int BIN2 = 12; //Direction


//---------------------------------------------------MPU6050 VARIABLES  ⇓⇓⇓⇓⇓⇓⇓⇓⇓

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool blinkState = false;
float ax, ay, az;
float gx, gy, gz;


//---------------------------------------------BLUETOOTH VARIABLES  ⇓⇓⇓⇓⇓⇓⇓⇓⇓

float direction_speed_change=0;
//char fake_transmission_value[30]={S,S,S,S,S,S,S,S,S,S,S,B,B,S,S,S,S,S,S,S,S,S,S,S,7,S,S,S,S,S,S,S,S,S,S,S,S,S,S}
char fake_transmission_value[23]={83,83,83,7,83,83,83,83,83,83,83,83,60,60,83,83,83,83,7,83,83,83,83};
   int digit_1000,digit_100,digit_10,digit_1;
   int sign_of_number;
const int packet_size=5;
uint8_t sign_byte=0;
float fraction_packet[8];
uint8_t sign_packet[8];
float  packet2send[packet_size];
uint8_t  packet_p253_edited[packet_size];

float  example_packet_253[3]={-192.52 ,250.45 , 255 };
float  example_packet_254[3]={-10.45 ,-15.255 , 200 };

//---------------------------------------------PID VARIABLES   ⇓⇓⇓⇓⇓⇓⇓⇓⇓
double SPAdjust = 0, Setpoint = 0, Input, Output;


double consKp = 17, consKi = 10, consKd = 0.01; 
PID myPID(&Input, &Output,&Setpoint, consKp, consKi, consKd, DIRECT);


int pwm_cikis=0;;  
float tune =0;float tune2 =0;float tune3 =0;
float raw_pid=0;
uint8_t first_time=0;;
float iTerm = 0;
float ki, kp, kd;
float dt;
float angle_error_old=0;
float angle_difference=0;
int dly=1;
int wthf=0;
int inside_the_space_beyond_our_dimension=0;
int pos=10;
float aeo=0;
float aeod=0;
double angle_error;
int counter=0;
volatile int wtf=0;
int a=0;
//int ax, ay, az;
void parse_to_digits(int number);    
void dmpDataReady() {
    mpuInterrupt = true;
}

float      cog_offset=0 ;
float      angle_offset=0 ;
//---------------------------------------------------Modifying variables ⇓⇓⇓⇓⇓⇓⇓⇓⇓
 
float        aci_eksi_sinir=-0.5,        aci_arti_sinir=0.5;  //min negatitive and positive angle borders for balance robo9
float        aci_max_eksi_sinir=-40,   aci_max_arti_sinir=40; //max negatitive and positive angle borders for balance robot
int        deadband_pwm_n=-2,        deadband_pwm_p=2;  //to remove deadband in pwm signal and motor connection

  
float angle = 0;
float error_slow = 0;
float error_slower = 0;




void setup() {

   //  cog_offset=-0.5 ;    //offset for the chassis cog to be aligned with center line between motors. 
      angle_offset=-2.0 ;// OFFSET OF ANGLE MEASUREMENT. 

  
  angle_offset=angle_offset-cog_offset;
 
settings();
 
 Timer1.initialize(100000); // set timer  for every ... ms and calculate behaviour of chassis

 Timer1.attachInterrupt( timerIsr ); // attach the service routine here

   myPID.SetOutputLimits(-255 , 255);
  myPID.SetSampleTime(15);
 myPID.SetMode(AUTOMATIC);
 myPID.SetTunings(consKp, consKi, consKd); //Load the tunings




}

void loop() {
unsigned long start = micros();

if(first_time==0){delay(3000); first_time=1;} //wait and eliminate noisy data that occours in first 2 seconds

//fake_getvalue();
 incoming_data_interpretor( bluetooth_getvalue());
 //bluetooth_getvalue();

kp=tune; ki=tune2; kd=tune3;     // 16.28 7.48 1436.95


kp=13.5; ki=0; kd=1000;  // consKp = 1, consKi = 0.4, consKd = 0;//

dmp(); // calculates gx value via digital motion processor in mpu6050

  // angle = ypr[1] + 0.06;                   // 0.02 is center of gravity offset
 gx=ypr[1]* 180/M_PI;  // pitch
 gx=gx-angle_offset;




pid_control();



unsigned long end = micros();
unsigned long delta = end - start;
//Serial.println(delta);

  dt=delta/1000;

}




void timerIsr(){

//  if( ( (aci_eksi_sinir>gx ) && (gx>aci_max_eksi_sinir) ) || ( (aci_arti_sinir<gx ) && (gx<aci_max_arti_sinir ) )  )  {
//    
//  }
//  else{
//  pid_update();}

//realise();

//fake_getvalue();
//incoming_data_interpretor(bluetooth_getvalue());
}
