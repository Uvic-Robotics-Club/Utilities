// Including libries
#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Quaternion.h>
#include <Encoder.h>
#include <PID_v1.h>

// Defining Constants
#define Encoder_A 2
#define Encoder_B 3
#define PWM_A 5
#define PWM_B 6
#define Addr1 9
#define Addr2 10
#define Addr3 11

// Defining Variables
double Input;
double Setpoint, Output;
double Kp = 1, Ki = 1, Kd = 0;
int Output1, Output2;
int DisplayTime = 500;
int MotorNumber = 0;
String inputString = "";
auto mode = AUTOMATIC;
long oldPosition  = 0;
long newPosition = 0;
unsigned long thisTime = 0;
unsigned long lastTime = 0;
unsigned long printTime = 0;

// Creating Objects
ros::NodeHandle  nh;
Encoder myEnc(Encoder_A, Encoder_B);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void SetpointCB(const std_msgs::Int32& value) {
  Setpoint = (double) value.data;
}



geometry_msgs::Quaternion output_array;


ros::Subscriber<std_msgs::Int32> sub1("Motor1/Setpoint", SetpointCB );
ros::Publisher Output_Pub1("Motor1/Output", &output_array);


void setup() {
  // Define all of the Pin I/O
  pinMode (PWM_A, OUTPUT);
  pinMode (PWM_B, OUTPUT);
  // no need to setup encoder a and b, they will be setup in the Encoder class
  pinMode (Addr1, INPUT);
  pinMode (Addr2, INPUT);
  pinMode (Addr3, INPUT);
  // calcualte the motor number from which pins are high
  MotorNumber = digitalRead(Addr1) * 1 + digitalRead(Addr2) * 2 + digitalRead(Addr3) * 4 + 1;
  // make sure the output is limited to what the PWM is setup for
  myPID.SetOutputLimits(-255, 255);
  // turn on the pid
  myPID.SetMode(mode);

  // Initialize the Ros Node
  nh.initNode();
  nh.advertise(Output_Pub1);
  nh.subscribe(sub1);




}

void loop() {
  newPosition = myEnc.read();
  Input = (oldPosition - newPosition); // This will generate the difference in steps over the delay, this will be the velocity
  myPID.Compute();

  Output1 = Output > 0 ? (0) : (abs(Output) < 50 ? 0 : (int) - Output);
  Output2 = Output > 0 ? (abs(Output) < 50 ? 0 : (int) Output) : (0);

  analogWrite(PWM_A, Output1);
  analogWrite(PWM_B, Output2);

  oldPosition = newPosition;

  output_array.x = (int)MotorNumber;
  output_array.y = (int)Input;
  output_array.z = (int)Output;
  output_array.w = (int)Setpoint;


  Output_Pub1.publish( &output_array );
  
  nh.spinOnce();
  delay(50);

}
