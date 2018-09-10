#include <ros.h>
#include <QEC_1X_SPI.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <wiring_analog.h>
#include <PinNames.h>
#include <stm32f3xx_hal_dac.h>
#include <PID_v1.h>

//! TO DO: change orientation from euler to quaternion to deliver a logical ROS messsage!!

//************* Workspace **** Left Platform***********************/ 

#define FRAME "Left_Pedal"
#define INTERFACE "2"  //! 1:Right 2:Left
#define HOMING_FORCE_X -6.0
#define HOMING_FORCE_Y 5.0 
#define HOMING_OFFSET_X -X_LIMIT/2 //! Left
#define HOMING_OFFSET_Y Y_LIMIT/2 //! Left
#define HOMING_OFFSET_P -33.0F //! Right

//#define HOMING_FORCE -5.0 //! Left 

#define X_LIMIT 0.350F //! [m] 
#define Y_LIMIT 0.293F //! [m]
#define P_LIMIT 90.0F //! [deg]


#define ENCODERSIGN1  -1 //! LEFT
#define ENCODERSIGN2  1 //! LEFT
#define ENCODERSIGN3  -1 //! LEFT

/******************************************************************/

#define ENCODERSCALE1 X_LIMIT/15735.0F 
#define ENCODERSCALE2 Y_LIMIT/16986.0F
#define ENCODERSCALE3 P_LIMIT/4418.0F 

#define SOFTPOT_YAW_SCALE (90.0F/1090.0F)*(45.0/70.0)
#define SOFTPOT_YAW_BIAS 1251.0F - 301.0F

#define SOFTPOT_ROLL_SCALE 45.0F/522.0F
#define SOFTPOT_ROLL_BIAS 2174.0F - 92.0F + 8.0F

#define SOFTPOT_ROLL_SIGN -1
#define SOFTPOT_YAW_SIGN -1

#define STACK_SIZE 512

#define COMM_FREQ 1000 //! [Hz] 1000Hz
#define CONTROL_FREQ 10000 //! [hz] 10kHz

#define CURRENT_K 42.43F //! K_i Faulhaber 3890024CR [mNm/A]
//#define CURRENT_K 30.2F //! K_i Maxon Motor RE40mm 148867 [mNm/A]

#define BELT_PULLEY_R 0.00915F //! Torque/Force
#define PITCH_REDUCTION_R 40.0/9.15F //! Pulley Big [mm] / Pulley Belt [mm]
#define C_CURRENT_MAX 5 //! 6 for Right and 5 for Left [A] 

#define MAX_TORQUE 0.212F //! [Nm] Left
#define MAX_RPM 6000
//#define MAX_TORQUE 0.187F //! [Nm] Right
//#define MAX_RPM 12000


#define PI 3.14159265359F

#define VEL_LIMIT_X MAX_RPM*2*PI*BELT_PULLEY_R/60   //! 5.75 [m/s] 
#define VEL_LIMIT_Y VEL_LIMIT_X
#define ANG_VEL_LIMIT_P MAX_RPM*2*PI/60/PITCH_REDUCTION_R //! 143.78 [rad/s] 

#define POSE_PID_SAMPLE_R 1000 //! [us]
#define VELOCITY_PID_SAMPLE_R 100 //!  [us]

//  void TaskComm(void *pvParameters );
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

double s_forceX=0.0F;
double s_forceY=0.0F;
double s_torqueP=0.0F;

double positionX; 
double positionY; 
double positionX_offset=0.0;
double positionY_offset=0.0;
double positionP_offset=0.0;
double orientationP; 
double orientationY = 0.0;
double orientationR = 0.0;


double velocityX, velocityY, angVelocityP;

double positionX_old; 
double positionY_old; 
double orientationP_old = 0.0;
double orientationY_old = 0.0;
double orientationR_old = 0.0;


/*******************************************/

int CS1 = D6; //! CS4 -> Lateral 
int CS2 = D9; //! CS5  -> Dorsi/Plantar Flexion
int CS3 = D10; //! CS6 -> Flexion/Extension of the Leg

int oriRoll_pin = A0;
int oriYaw_pin = A1;

int limitSwitchX = D8; // VERIFY THIS NUMBER!!!
int limitSwitchY = D7;

int motorX_pin = D3; //! 
int motorY_pin = D4; //! D4 
int motorP_pin = A3; //! A3: DAC || D%


/**************Variables for Impedance Rendering****************************/

/*
float springK_X = 100.0F; //! [N/m]
float damperK_X = 0.0F;
float springK_Y = 100.0F; //! [N/m]
float damperK_Y = 0.0F;
float springK_P = 150.0F; //! [mNm/rad]
float damperK_P = 0.0F;*/

float springK_X = 0.0F; //! [N/m]
float damperK_X = 0.0F;
float springK_Y = 0.0F; //! [N/m]
float damperK_Y = 0.0F;
float springK_P = 0.0F; //! [mNm/rad]
float damperK_P = 0.0F;

/***************************************************************************/

/*************Variables for the state machine *******************************/

enum {
  homing, 
  centering,
  normal_op
}states; 

int stateM=homing;
int axes_calib[3] = {0,0,0}; //! TO DO: CHANGE THIS WHEN THE LIMIT SWITCH OF PITCH IS INCORPORATED, FOR THE MOMENT IT IS SIMULATED AS ALWAYS "HOMED"

/**************************************************************************/


//PID VARIABLES


double referencePosX, referencePosY, referenceOriP;



double kp_PosX=100, ki_PosX=0, kd_PosX=2;
double kp_PosY=100, ki_PosY=0, kd_PosY=2;
double kp_OriP=2000*PI/180.0F*0.001, ki_OriP=0*PI/180.0F*0.001, kd_OriP=0*PI/180.0F*0.001; //! Nm/rad

PID PID_posX(&positionX, &forceX, &referencePosX, kp_PosX, ki_PosX, kd_PosX, DIRECT); //! On measurement
PID PID_posY(&positionY, &forceY, &referencePosY, kp_PosY, ki_PosY, kd_PosY, DIRECT);
PID PID_oriP(&orientationP, &torqueP, &referenceOriP, kp_OriP, ki_OriP, kd_OriP, DIRECT);


ros::NodeHandle nh;
geometry_msgs::PoseStamped pose_msg; //! 6DoF
geometry_msgs::WrenchStamped wrench_msg;
geometry_msgs::TwistStamped twist_msg;

void FI_subWrench(const geometry_msgs::Wrench& s_wrench_msg){ 
    s_forceX = s_wrench_msg.force.x; 
    s_forceY = s_wrench_msg.force.y; 
    /* s_forceZ = s_wrench_msg.force.z;*/
    s_torqueP = s_wrench_msg.torque.x;
    /* s_torqueR = s_wrench_msg.torque.y;
    s_torqueY = s_wrench_msg.torque.z */
}

ros::Subscriber<geometry_msgs::Wrench> s_wrench("/FI_setWrench/2",FI_subWrench);
ros::Publisher p_pose("/FI_Pose/2", &pose_msg);
ros::Publisher p_wrench("/FI_Wrench/2", &wrench_msg);
ros::Publisher p_twist("/FI_Twist/2", &twist_msg);



typedef enum 
{
  axisX=1, axisY=2, axisP=3
}axisID;


QEC_1X encoderX(CS1);
QEC_1X encoderY(CS3);
QEC_1X encoderP(CS2);

void setup() {

  pinMode(limitSwitchX, INPUT);
  pinMode(limitSwitchY, INPUT);
  attachInterrupt(limitSwitchX, FI_posResetX, RISING);
  attachInterrupt(limitSwitchY, FI_posResetY, RISING);
  nh.initNode();
  nh.advertise(p_pose);
  nh.advertise(p_wrench);
  /*nh.advertise(p_twist);*/
  nh.subscribe(s_wrench);

      /*****ROS communication initialization*******/
  analogWriteResolution(12); //! Set resolution of the PWM's and DAC outputs for the ESCON Drivers.  
  analogReadResolution(12);
  //! Automatic initializations of PID's

  PID_posX.SetOutputLimits(-25.0,25.0); //! N
  PID_posY.SetOutputLimits(-25.0,25.0); //! N
  PID_oriP.SetOutputLimits(-0.927,0.927); //! Nm
  PID_posX.SetSampleTime(POSE_PID_SAMPLE_R); //! [us]
  PID_posY.SetSampleTime(POSE_PID_SAMPLE_R); //! [us]
  PID_oriP.SetSampleTime(POSE_PID_SAMPLE_R); //! [us]

  PID_posX.SetMode(AUTOMATIC);
  PID_posY.SetMode(AUTOMATIC);
  PID_oriP.SetMode(AUTOMATIC);   

  Serial.begin(230400);
  while (!Serial) {
    ; 
  }

//! Set up of the tasks

  encoderX.QEC_init( axisX , ENCODERSCALE1, ENCODERSIGN1);
  encoderY.QEC_init( axisY , ENCODERSCALE2, ENCODERSIGN2);
  encoderP.QEC_init( axisP , ENCODERSCALE3, ENCODERSIGN3);
}

void loop() {


    //!First step -> Get the pose of the platform.
    FI_getMotion();
    //FI_ImpUpdate();

    switch(stateM) {
      case homing: //! The robot tries to go outside the workspace until it meets with the limits switches
          forceY= HOMING_FORCE_Y; //![N]
          forceX= HOMING_FORCE_X; //! 
          torqueP = 0;  //! [Nm]
          FI_oriResetP(); //! resembling the limit switch
          if ( ( axes_calib[0]==1 ) && ( axes_calib[1]==1 ) && ( axes_calib[2]==1 )){
              static int idle = micros(); 
              forceY= 0, forceX= 0, torqueP = 0;  //! [Nm]              
              if (micros()-idle>1000){ //! After a second move to next state
                stateM=centering;
              }
              
            }
          break;
      case centering:
          referencePosX=0.0, referencePosY=0.0, referenceOriP=0.0;
          kp_PosX=70, kp_PosY=70, kp_OriP=1000*PI/180.0F*0.001;
          kd_PosX=0.1, kd_PosY=0.1, kd_OriP=0*PI/180.0F*0.001;
          FI_mPoseControl();
          if  (((positionX - referencePosX) < 0.010) && ((positionY - referencePosY) < 0.010) && ((orientationP - referenceOriP ) < 3) ){
            stateM=normal_op;
          }
          break;
      case normal_op: 
          FI_ImpUpdate();
          break;
        }

    FI_setWrenchs(); //! Apply forces and torques
    FI_pubMotion(); //! publish pose and twist in ROS*/
    FI_pubWrench();
    nh.spinOnce();

 }


/************************ Set of functions for Force Reflection *****************************

*******************************************************************************************/ 

void FI_setTorque(float torque, int pin, int sign, float reduction){

  double escon_current = (torque*1000/CURRENT_K)/reduction;
  escon_current = (escon_current > C_CURRENT_MAX ? C_CURRENT_MAX : (escon_current<-C_CURRENT_MAX ? -C_CURRENT_MAX : escon_current)); // Saturate the Current to the max of the escon
  int escon_current_PWM = sign * escon_current*(4096.0*0.8/(2*C_CURRENT_MAX))+4096.0*0.5;
  analogWrite(pin,escon_current_PWM);
}

void FI_setForce(float force, int pin, int sign){

  float escon_torque = force*BELT_PULLEY_R; //! The reduction is here.
  FI_setTorque(escon_torque,pin,sign,1.0);
}

void FI_setWrenchs(){
    FI_setForce(forceX,motorX_pin,1);
    FI_setForce(forceY,motorY_pin,1);
    FI_setTorque(torqueP,motorP_pin,1,PITCH_REDUCTION_R); 
}



void FI_mPoseControl(){
  PID_posX.SetTunings(kp_PosX,ki_PosX,kd_PosX);
  PID_posY.SetTunings(kp_PosY,ki_PosY,kd_PosY);
  PID_oriP.SetTunings(kp_OriP,ki_OriP,kd_OriP);
  PID_posX.Compute();
  PID_posY.Compute();
  PID_oriP.Compute();
}

void FI_sPoseControl(PID PID_axis, double kp, double ki, double kd){

  PID_axis.SetTunings(kp,ki,kd);
  PID_axis.Compute();
}


/**********************************Impedance Rendering******************************/

void FI_ImpUpdate(){
    forceX=(0-positionX)*springK_X + s_forceX; //! Remember Escon sign 
    forceY=(0-positionY)*springK_Y + s_forceY; //! Remember Escon sign 
    torqueP=((0-orientationP)*PI/180.0F)*springK_P*0.001 + s_torqueP;    //! 0.001: Convert the stiffness from  mNm/rad to Nm/rad
}

/***********************************************************************************/

void FI_getPose(){ //! Get the pose of the platform using the values of the encoders
   encoderX.QEC_getPose(); //! Motor for motion in X
   encoderY.QEC_getPose(); //! Motor for motion in Y
   encoderP.QEC_getPose(); //! Motor for motion in Pitch
   positionX=encoderX.outDimension + positionX_offset; 
   positionY=encoderY.outDimension + positionY_offset; 
   orientationP=encoderP.outDimension + positionP_offset; //! Rad/s
   orientationR=0.9*orientationR_old + 0.1 * SOFTPOT_ROLL_SIGN * SOFTPOT_ROLL_SCALE*(analogRead(oriRoll_pin) - SOFTPOT_ROLL_BIAS);
   orientationY=0.9*orientationY_old + 0.1 * SOFTPOT_YAW_SIGN * SOFTPOT_YAW_SCALE*(analogRead(oriYaw_pin) - SOFTPOT_YAW_BIAS);
   orientationR_old=orientationR;
   orientationY_old=orientationY;

}


void FI_getTwist(){
    velocityX= (positionX-positionX_old)/(POSE_PID_SAMPLE_R);
    velocityY= (positionY-positionY_old)/(POSE_PID_SAMPLE_R);
    angVelocityP = (orientationP-orientationP_old)/(POSE_PID_SAMPLE_R);
    positionX_old=positionX; 
    positionY_old=positionY; 
    orientationP_old=orientationP;
} 


void FI_getMotion(){
    FI_getPose();
    FI_getTwist();
}

void FI_encBias(QEC_1X &encoder){ //! Set an offset to the counter for setting a new zero //! Alternatively clear counters? 0X20 SPI
   encoder.QEC_offset(); 
}


void FI_posResetX(){  //! FOR THE MOMENT THERE ARE ONLY 2 LIMIT SWITCHES ( FOR X AND Y )
  if (axes_calib[0]==0){
    FI_encBias(encoderX);
    positionX_offset=HOMING_OFFSET_X;
    axes_calib[0]=1;
  }
}

void FI_posResetY(){ 
   if (axes_calib[1]==0){
    FI_encBias(encoderY);
    positionY_offset=HOMING_OFFSET_Y;
    axes_calib[1]=1;
  }
}

void FI_oriResetP(){ 
   if (axes_calib[2]==0){
    FI_encBias(encoderP);
    positionP_offset=HOMING_OFFSET_P;
    axes_calib[2]=1;
  }
}

/************************ Set of functions for Communication *****************************
* FI_pubPose() will broadcast a topic /FI_pose with the pose of the platform
*******************************************************************************************/ 


void FI_pubMotion(){
  pose_msg.header.frame_id=FRAME;
  pose_msg.header.stamp=nh.now();
  pose_msg.pose.position.x = positionX;
  pose_msg.pose.position.y = positionY;
  pose_msg.pose.position.z = 0.0f;
  pose_msg.pose.orientation.x = orientationP;
  pose_msg.pose.orientation.y = orientationR;
  pose_msg.pose.orientation.z = orientationY;
  pose_msg.pose.orientation.w = 0.0f;

  /*twist_msg.header.frame_id=FRAME;
  twist_msg.header.stamp=nh.now();
  twist_msg.twist.linear.x = velocityX;
  twist_msg.twist.linear.y = velocityY;
  twist_msg.twist.linear.z = 0.0f;
  twist_msg.twist.angular.x = angVelocityP;
  twist_msg.twist.angular.y = 0.0f;
  twist_msg.twist.angular.z = 0.0f;*/

  p_pose.publish(&pose_msg);
  /*p_twist.publish(&twist_msg);*/
  }


void FI_pubWrench(){
  wrench_msg.header.frame_id=FRAME;
  wrench_msg.header.stamp=nh.now();
  wrench_msg.wrench.force.x = forceX;
  wrench_msg.wrench.force.y = forceY;
  wrench_msg.wrench.force.z = 0.0;
  wrench_msg.wrench.torque.x = torqueP;
  wrench_msg.wrench.torque.y = 0.0f;
  wrench_msg.wrench.torque.z = 0.0f;
  p_wrench.publish(&wrench_msg);
  }


/************************ Set of functions for Debugging *****************************
* FI_print() will allow you to test the serial communication, by testing the broadcast of the pose of the platform.
*******************************************************************************************/ 

void FI_print() { //! For serial debugging
  Serial.print("positionX = ");
  Serial.print(encoderX.outDimension,4);
  Serial.print(", ");
  
  Serial.print("PositionY = ");
  Serial.print(encoderY.outDimension,4);
  Serial.print(", ");

  Serial.print("orientationP = ");
  Serial.print(encoderP.outDimension,4);
  Serial.print(", ");
  Serial.println();
  //!delay(100);
}
