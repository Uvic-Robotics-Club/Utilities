#include <Encoder.h>
#include <PID_v1.h>

#define Encoder_A 2
#define Encoder_B 3
#define PWM_A 5
#define PWM_B 6
#define Addr1 9
#define Addr2 10
#define Addr3 11

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

Encoder myEnc(Encoder_A, Encoder_B);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  pinMode (PWM_A, OUTPUT);
  pinMode (PWM_B, OUTPUT);
  // no need to setup encoder a and b, they will be setup in the Encoder class
  pinMode (Addr1, INPUT);
  pinMode (Addr2, INPUT);
  pinMode (Addr3, INPUT);

  // calcualte the motor number from which pins are high
  MotorNumber = digitalRead(Addr1)*1+digitalRead(Addr2)*2+digitalRead(Addr3)*4 + 1;
  // make sure the output is limited to what the PWM is setup for
  myPID.SetOutputLimits(-255, 255);
  // turn on the pid
  myPID.SetMode(mode);

  // start the serial and tell the comp what motor this is
  Serial.begin(9600);
  Serial.println("Motor:" + String(MotorNumber));
}


void loop() {
  
  thisTime = millis();
  newPosition = myEnc.read();
  Input = (oldPosition - newPosition); // This will generate the difference in steps over the delay, this will be the velocity
  myPID.Compute();
  
  //Output1 = Output>0 ? 0 : (int)-Output;
  //Output2 = Output>0 ? (int)Output : 0;
  
  if (Output > 0) {
    Output1 = 0;
    Output2 = (int) Output;
  }
  else {
    Output1 = (int) - Output;
    Output2 = 0;
  }
  if (abs(Output)<50){
    Output1 = 0;
    Output2 = 0;
  }


  analogWrite(PWM_A, Output1);
  analogWrite(PWM_B, Output2);
  if (thisTime - printTime > DisplayTime) {
    Serial.print(Input);
    Serial.print(",");
    Serial.print(Output);
    Serial.print(",");
    Serial.println(Setpoint);
    printTime = thisTime;
  }
  oldPosition = newPosition;
  lastTime = thisTime;
  delay(50);
}

void serialEvent() {
  char first = (char)Serial.read();
  wholeThing();
  if (first == 's') {
    Setpoint = inputString.toInt();
  }
  if (first == 'd') {
    Kd = inputString.toInt();
    myPID.SetTunings(Kp, Ki, Kd);
  }
  if (first == 'i') {
    Ki = inputString.toInt();
    myPID.SetTunings(Kp, Ki, Kd);
  }
  if (first == 'p') {
    Kp = inputString.toInt();
    myPID.SetTunings(Kp, Ki, Kd);
  }
  if (first == 'k') {
    mode = (mode == AUTOMATIC) ? MANUAL : AUTOMATIC;
    myPID.SetMode(mode);
    Output1 = 0;
    Output2 = 0;
  }
  if (first == 'r') {
    DisplayTime = inputString.toInt();
  }


}

void wholeThing() {
  inputString = "";
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
  }
}

