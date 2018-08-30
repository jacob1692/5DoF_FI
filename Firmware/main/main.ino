#include <STM32FreeRTOS.h>
#include <ros.h>
#include <QEC_1X_SPI.h>
#include <geometry_msgs/PoseStamped.h>
#include <wiring_analog.h>
#include <PinNames.h>
#include <stm32f3xx_hal_dac.h>

#include <PID_v1.h>


//! WORKSPACE AND FOOTPRINT FOR ****LEFT*** PLATFORM
#define X_LIMIT 0.351161450148F //! [m] 
#define Y_LIMIT 0.303287059069F //! [m]
#define P_LIMIT 90.0F //! [deg]

#define ENCODERSCALE1 X_LIMIT/15735.0F 
#define ENCODERSCALE2 Y_LIMIT/16986.0F
#define ENCODERSCALE3 P_LIMIT/4418.0F 

#define ENCODERSIGN1  -1
#define ENCODERSIGN2  -1
#define ENCODERSIGN3 -1

#define STACK_SIZE 512

#define COMM_FREQ 1000 //! [Hz] 1000Hz
#define CONTROL_FREQ 10000 //! [hz] 10kHz

#define CURRENT_K 42.43F //! K_i Faulhaber 3890024CR [mNm/A]
#define LINEAR_CONV_K 0.00915F //! Torque/Force
#define PITCH_CONV_R 40.0/9.15F //! Pulley Big / Pulley Small
#define CURRENT_MAX 15.0 //! A
//#define ESCON_PWM_K 10.0F/4096.0F //! Amp/PWM(12 bits)

#define MAX_TORQUE 250.0F //! [mNm]

#define PI 3.14159265359F

enum {
  homing, 
  impedance_mode
}stateM; 

  void TaskComm(void *pvParameters );
  void TaskStateMachine (void *pvParameters);

/**
*   Tasks handled by the microcontroller
    - Pose retrieval, using encoders and softpots
    - State Machine ( Homing, No Op, Impedance Control, Admittance Control, Force Rendering, etc.)
    - Force retrieval from ROS 
**/

//*Initialize Wrench*
double forceX=0.0F;
double forceY=0.0F;
double torqueP=0.0F;  

double positionX; 
double positionY; 
double positionP; 

int CS1 = D6; //! CS4 -> Lateral 
int CS2 = D9; //! CS5  -> Dorsi/Plantar Flexion
int CS3 = D10; //! CS6 -> Flexion/Extension of the Leg

int motorX_pin = D3; //! 
int motorY_pin = D4; //! D4 
int motorP_pin = A3; //! A3: DAC || D%

float springK_X = 100.0F; //! [N/m]
float damperK_X = 0.0F;
float springK_Y = 100.0F; //! [N/m]
float damperK_Y = 0.0F;
float springK_P = 150.0F; //! [mNm/rad]
float damperK_P = 0.0F;

//PID VARIABLES

//Define Variables we'll be connecting to
double SetpointX, SetpointY, SetpointP;



double Kp_X=100, Ki_X=0, Kd_X=3;
double Kp_Y=100, Ki_Y=0, Kd_Y=3;
double Kp_P=150, Ki_P=1, Kd_P=3;

//

PID PID_X(&positionX, &forceX, &SetpointX, Kp_X, Ki_X, Kd_X,P_ON_M, DIRECT); //! On measurement
PID PID_Y(&positionY, &forceY, &SetpointY, Kp_Y, Ki_Y, Kd_Y,P_ON_M, DIRECT);
PID PID_P(&positionP, &torqueP,&SetpointP, Kp_P, Ki_P, Kd_P,P_ON_M, DIRECT);

//Servo motorX_servo;

ros::NodeHandle nh;
geometry_msgs::PoseStamped pos_msg; //! 6DoF
ros::Publisher p("/FI_Pose/2", &pos_msg);


typedef enum 
{
  axisX=1, axisY=2, axisP=3
}axisID;


QEC_1X motorX(CS1);
QEC_1X motorY(CS3);
QEC_1X motorP(CS2);

void setup() {
 
  Serial.begin(115200);
  while (!Serial) {
    ; 
  }
/*
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }
*/
//! Set up of the tasks

  motorX.QEC_init( axisX , ENCODERSCALE1, ENCODERSIGN1);
  motorY.QEC_init( axisY , ENCODERSCALE2, ENCODERSIGN2);
  motorP.QEC_init( axisP , ENCODERSCALE3, ENCODERSIGN3);

  xTaskCreate(TaskStateMachine, (const portCHAR * ) "State Machine", STACK_SIZE , NULL , 1, NULL); 
  //xTaskCreate(TaskComm, (const portCHAR * ) "Communication", STACK_SIZE , NULL , 2, NULL);  //! Higher Priority the less frequent
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);

}

void loop() {
 }


void TaskStateMachine(void *pvParameters)
{
    (void) pvParameters;
    nh.initNode();
    nh.advertise(p);
	  analogWriteResolution(12); 
    
    //! Automatic initializations of PID's

    PID_X.SetOutputLimits(-25.0,25.0); //! N
    PID_Y.SetOutputLimits(-25.0,25.0); //! N
    PID_P.SetOutputLimits(-0.300,0.300); //! Nm
    PID_X.SetSampleTime(1000); //! [us]
    PID_Y.SetSampleTime(1000); //! [us]
    PID_P.SetSampleTime(1000); //! [us]

    PID_X.SetMode(AUTOMATIC);
    PID_Y.SetMode(AUTOMATIC);
    PID_P.SetMode(AUTOMATIC);   

  for(;;)
  {
    
    //!First step -> Get the pose of the platform.
    FI_getPose();
    FI_Force_Imp_Update();

   // FI_GOTO(X_LIMIT/2.0,Y_LIMIT/2.0,0.0);

    FI_SetForce(forceX,motorX_pin,1);
    FI_SetForce(forceY,motorY_pin,1);
    FI_SetTorque(torqueP,motorP_pin,1,PITCH_CONV_R); 

    FI_pubPose(); //! publish pose in ROS
    vTaskDelay ((float) (1000 / portTICK_PERIOD_MS ) / CONTROL_FREQ ); //! Control Rate
  }

}

/************************ Set of functions for Force Reflection *****************************
* FI_homing() retrieves the pose using the encoders and the IMU
* FI_impedance() generates a bias in the encoder counter that will set the zero dimension
*******************************************************************************************/ 

void FI_SetTorque(float torque, int pin, int sign, float reduction){

  double escon_current = (torque*1000/CURRENT_K)/reduction;
  escon_current = (escon_current > CURRENT_MAX ? CURRENT_MAX : (escon_current<-CURRENT_MAX ? -CURRENT_MAX : escon_current)); // Saturate the Current to the max of the escon
  int escon_current_PWM = sign * escon_current*(4096.0*0.8/10)+4096.0*0.5;
  analogWrite(pin,escon_current_PWM);
}

void FI_SetForce(float force, int pin, int sign){

  float escon_torque = force*LINEAR_CONV_K; //! The reduction is here.
  FI_SetTorque(escon_torque,pin,sign,1.0);
}


void FI_Force_Imp_Update(){
    forceX=(X_LIMIT/2.0-motorX.outDimension)*springK_X ; //! Remember Escon sign 
    forceY=(Y_LIMIT/2.0-motorY.outDimension)*springK_Y ; //! Remember Escon sign 
    torqueP=((P_LIMIT/2.0-motorP.outDimension )*PI/180.0F)*springK_P*0.001;    //! 0.001: Convert the stiffness from  mNm/rad to Nm/rad
}

void FI_GOTO(float x, float y, float p){
  SetpointX=x; SetpointY=y; SetpointP=p;
  PID_X.Compute();
  PID_Y.Compute();
  PID_P.Compute();
}
/************************ Set of functions for Pose Decoding *****************************
* FI_getPose() retrieves the pose using the encoders and the IMU
* FI_encBias() generates a bias in the encoder counter that will set the zero dimension
*******************************************************************************************/ 


void FI_getPose(){ //! Get the pose of the platform using the values of the encoders
   motorX.QEC_getPose(); //! Motor for motion in X
   motorY.QEC_getPose(); //! Motor for motion in Y
   motorP.QEC_getPose(); //! Motor for motion in Pitch
   positionX=motorX.outDimension; 
   positionY=motorY.outDimension; 
   positionP=motorP.outDimension;
}

void FI_encBias(){ //! Set an offset to the counter for setting a new zero //! Alternatively clear counters? 0X20 SPI
   motorX.QEC_home(); //! Motor for motion in X
   motorY.QEC_home(); //! Motor for motion in Y
   motorP.QEC_home(); //! Motor for motion in Pitch
}

/************************ Set of functions for Communication *****************************
* FI_pubPose() will broadcast a topic /FI_Pose with the pose of the platform
*******************************************************************************************/ 


void FI_pubPose(){
  pos_msg.header.frame_id="Left_Pedal";
  pos_msg.header.stamp=nh.now();
  pos_msg.pose.position.x = motorX.outDimension;
  pos_msg.pose.position.y = motorY.outDimension;
  pos_msg.pose.position.z = 0.0f;
  pos_msg.pose.orientation.x = motorP.outDimension;
  //pos_msg.pose.orientation.x = 0.0f;
  pos_msg.pose.orientation.y = 0.0f;
  pos_msg.pose.orientation.z = 0.0f;
  pos_msg.pose.orientation.w = 1.0f;
  p.publish(&pos_msg);
  nh.spinOnce();
  }


/************************ Set of functions for Debugging *****************************
* FI_print() will allow you to test the serial communication, by testing the broadcast of the pose of the platform.
*******************************************************************************************/ 

void FI_print() { //! For serial debugging
  Serial.print("PositionX = ");
  Serial.print(motorX.outDimension,4);
  Serial.print(", ");
  
  Serial.print("PositionY = ");
  Serial.print(motorY.outDimension,4);
  Serial.print(", ");

  Serial.print("PositionP = ");
  Serial.print(motorP.outDimension,4);
  Serial.print(", ");
  Serial.println();
  //!delay(100);
}
