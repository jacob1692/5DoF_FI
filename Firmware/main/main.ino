#include <STM32FreeRTOS.h>
#include <ros.h>
#include <QEC_1X_SPI.h>
#include <geometry_msgs/PoseStamped.h>
#include <wiring_analog.h>
#include <PinNames.h>
#include "stm32f3xx_hal_dac.h"


#define ENCODERSCALE1 0.350F/7360.0F/2.244126428F/0.9495171863F 
#define ENCODERSCALE2 0.293F/7560.0F/2.170616483F
#define ENCODERSCALE3 90.F/2140.0F/2.0644889F 

#define ENCODERSIGN1  1
#define ENCODERSIGN2  1
#define ENCODERSIGN3 -1

#define STACK_SIZE 512

#define COMM_FREQ 150 //! [Hz] 1000Hz
#define CONTROL_FREQ 10000 //! [hz] 10kHz

SemaphoreHandle_t xSerialSemaphore; //!Semaphore for Port Serial

  void TaskComm(void *pvParameters );
  void TaskStateMachine (void *pvParameters);
  void TaskForceRead(void *pvParameters);

/**
*   Tasks handled by the microcontroller
    - Pose retrieval, using encoders and IMU
    - State Machine ( Homing, No Op, Impedance Control, Admittance Control, Force Rendering, etc.)
    - Force retrieval from ROS 
**/

int CS1 = D6; //! CS4 -> Lateral 
int CS2 = D9; //! CS5  -> Dorsi/Plantar Flexion
int CS3 = D10; //! CS6 -> Flexion/Extension of the Leg

/*PinName motorX_pin = PIN_A4; //! Actually A4
PinName motorY_pin = PA_7; //! Actually A5
*/

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

  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

//! Set up of the tasks

  Serial.begin(115200);
  motorX.QEC_init( axisX , ENCODERSCALE1, ENCODERSIGN1);
  motorY.QEC_init( axisY , ENCODERSCALE2, ENCODERSIGN2);
  motorP.QEC_init( axisP , ENCODERSCALE3, ENCODERSIGN3);

  xTaskCreate(TaskStateMachine, (const portCHAR * ) "State Machine", STACK_SIZE , NULL , 1, NULL); 
  xTaskCreate(TaskComm, (const portCHAR * ) "Communication", STACK_SIZE , NULL , 2, NULL);  //! Higher Priority the less frequent
  vTaskStartScheduler();
  Serial.println("Insufficient RAM");
  while(1);

}

void loop() {
 }


void TaskComm(void *pvParameters)
{

  (void) pvParameters;
      nh.initNode();
      nh.advertise(p); 

  for (;;)
  {

    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 2 ) == pdTRUE ) //! Take the Serial for Itself //! 5 for 2
    {
      FI_pubPose(); //! Publish Pose in ROS
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    
     vTaskDelay( (float) (1000 / portTICK_PERIOD_MS) / COMM_FREQ  ); //! Communication RATE in MS
  }


}

void TaskStateMachine(void *pvParameters)
{
    (void) pvParameters;
   // analogOutputInit();
   // analogWriteResolution(8);
   // motorX_servo.attach(motorX_pin, 900 , 2100); 

  for(;;)
  {
    
    //!First step -> Get the pose of the platform.
    FI_getPose();
    /*analogWrite(motorX_pin,150);
    //analogWrite(motorY_pin,150);
    dac_write_value(motorX_pin, 2000, 1x);
    //dac_write_value(motorY_pin, 10, 1);
    // motorX_servo.write(2100);*/
    vTaskDelay ((float) (1000 / portTICK_PERIOD_MS ) / CONTROL_FREQ ); //! Control Rate
  }

}


/************************ Set of functions for Pose Decoding *****************************
* FI_getPose() retrieves the pose using the encoders and the IMU
* FI_encBias() generates a bias in the encoder counter that will set the zero dimension
*******************************************************************************************/ 


void FI_getPose(){ //! Get the pose of the platform using the values of the encoders
   motorX.QEC_getPose(); //! Motor for motion in X
   motorY.QEC_getPose(); //! Motor for motion in Y
   motorP.QEC_getPose(); //! Motor for motion in Pitch
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
