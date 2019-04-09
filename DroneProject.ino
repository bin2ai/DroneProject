
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <PID_v1.h>
#include<Servo.h>

MPU6050 mpu6050(Wire); // for the IMU sensor

// heading variables
float aroll, apitch;
float groll, gpitch, gyaw;
float roll, pitch, yaw;

// timer variables
float elapsedTime, time, timePrev;

// pid constants
double Kp_roll = 1, Ki_roll = 0.007, Kd_roll = 1;
double Kp_pitch = 2, Ki_pitch = 0.007, Kd_pitch = 1;
double Kp_yaw = 1, Ki_yaw = .007, Kd_yaw = 1;

// pitch PID
  double pitchSetpoint = 0, pitchInput, pitchOutput;
  PID pitchPID(&pitchInput, &pitchOutput, &pitchSetpoint, Kp_pitch, Ki_pitch, Kd_pitch, DIRECT);

// roll PID
  double rollSetpoint = 0, rollInput, rollOutput;
  PID rollPID(&rollInput, &rollOutput, &rollSetpoint, Kp_roll, Ki_roll, Kd_roll, DIRECT);

// yaw PID
  double yawSetpoint = 0, yawInput, yawOutput;
  PID yawPID(&yawInput, &yawOutput, &yawSetpoint, Kp_yaw, Ki_yaw, Kd_yaw, DIRECT);

// servo variables
  float servoPitch, servoRoll, servoYaw;
  Servo rudderServo, elevatorServo;
  //Servo deploymentServo;
  
// wing deployment
  int deploymentPin = 8;
  int wait2Deploy_ms = 15000;

// variables for flight phase
  boolean deployedWings = false;
  boolean glide = false;
  boolean launching = false;
  boolean autopilot = false;

// Set these pin numbers to any of the non-PWM digital pins (Note: 4 & 7 are valid)
int orPinNum = 2; // Digital pin number for the override switch (from tx/rx)
int elPinNum = 3; // Digital pin number for the elevator command (from tx/rx)

// Global override flag
bool manualOverride = false;

// Thresholds for what is considered "on" for the override switch
// TODO: Set this based on the pulse width when the switch is active vs inactive
static int orPwmThresh = 1500; // Units: microseconds

// Initialization of global variables needed for manual PWM capture (i.e. the failsafe feature)
volatile int orPwmValue = 0;
volatile int elPwmValue = 0;
volatile int orStartTime = 0;
volatile int elStartTime = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

// Initialize the interrupt to look for a rising edge on the override pin
  attachInterrupt(digitalPinToInterrupt(orPinNum), orRising, RISING);

  pinMode(deploymentPin, OUTPUT);
  //deploymentServo.attach(8);
  //deploymentServo.write(90);
  rudderServo.attach(9);
  elevatorServo.attach(10);

  pitchInput = pitch;
  rollInput = roll;
  yawInput = yaw;

  pitchSetpoint = 0;
  rollSetpoint = 0;
  yawSetpoint = 0;

  pitchPID.SetOutputLimits(-90,90);
  rollPID.SetOutputLimits(-90,90);
  yawPID.SetOutputLimits(-90,90);

  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
  yawPID.SetMode(AUTOMATIC);

  // the deploymentPin needs to be set to an unused digital pin
  //pinMode(deploymentPin, OUTPUT); 
  //setting up deployment enable pin. when ready to deploy the wings call the function 'activateDeployment(deploymentPin);'
  
  elevatorServo.write(45);
  delay(300);
  elevatorServo.write(120);
  delay(300);
  elevatorServo.write(45);
  delay(300);
  elevatorServo.write(90);
  rudderServo.write(90);
  delay(300);
  
  time = millis(); //Start counting time in milliseconds
}

void loop() {
  timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read
    elapsedTime = (time - timePrev) / 1000;
 
  mpu6050.update();

  aroll = mpu6050.getAccAngleY();
  apitch = mpu6050.getAccAngleX();
  groll = mpu6050.getGyroAngleY();
  gpitch = mpu6050.getGyroAngleX();  
  gyaw = mpu6050.getGyroAngleZ();
  
  groll += ((gpitch*.01745)*sin(gyaw*.01745));
  gpitch -= ((groll*.01745)*sin(gyaw*.01745));
  
  groll = groll*.9 + aroll*.1;
  gpitch = gpitch*.9 + apitch*.1;
  
  roll = roll*.8 + groll*.2;
  pitch = pitch*.8 + gpitch*.2;

  pitchInput = pitch;
  rollInput = roll;
  yaw = gyaw;
  yawInput = yaw;

  pitchPID.SetTunings(Kp_pitch, Ki_pitch, Kd_pitch);
  rollPID.SetTunings(Kp_roll, Ki_roll, Kd_roll);
  yawPID.SetTunings(Kp_yaw, Ki_yaw, Kd_yaw);

  pitchPID.Compute();
  rollPID.Compute();
  yawPID.Compute();
    
  servoPitch = map(pitchOutput, 90, -90, 0, 180);
  servoRoll = map(rollOutput, 90, -90, 180, 0);
  servoYaw = map(yawOutput, 90, -90, 0, 180);

 if(glide == false && deployedWings == false){
  launching = true;
 }

 if(gpitch <= 0 && deployedWings == false && time > wait2Deploy_ms){
  activateWingDeployment();
  launching = false;
  deployedWings = true;
  glide = true;
  }

  if(deployedWings == true && glide == true){
    autopilot = true;
  }
  
  if (manualOverride) {
    // This is where you decode the tx/rx PWM value for the elevator command (elPwmValue)
    // and convert it to the appropriate elevator signal. Also set the rudder and aileron
    // cmds to 0 (Note: it might not need any conversion if the pulse width for the Tx/Rx
    // and your servos aligns thus you can simply pass it straight through the way you
    // would normally) TODO
    elevatorServo.write(elPwmValue);
    rudderServo.write(90);
    Serial.print("manual overide");
    
  }

  else{
    
  if(launching == true){
    rudderServo.write(90);
    elevatorServo.write(90);
    Serial.print("launching");
  }
  

  if(autopilot == true){
      elevatorServo.write(servoPitch);
      if(roll>20 || roll<-20){
        
        rudderServo.write(servoRoll);
      }
      else{
        rudderServo.write(servoYaw);
      }
  }
  
  }
    Serial.println();
    Serial.print("gyaw:\t");
    Serial.print(gyaw);
    Serial.print("\t");
 
    Serial.print("roll:\t");
    Serial.print(roll);
    Serial.print("\t");

    Serial.print("pitch:\t");
    Serial.print(pitch);
    Serial.print("\t");
    
    Serial.print("pitch PID:\t");
    Serial.print(pitchOutput);
    Serial.print("\t");

    Serial.print("servo pitch position\t");
    Serial.print(servoPitch);
    Serial.print("\t");

    Serial.print("servo roll position:\t");
    Serial.print(servoRoll);
    
    Serial.println();


}


void activateWingDeployment(){
  Serial.println("Activating Deployment...the line should be burning");
  //deploymentServo.write(0);
  digitalWrite(deploymentPin, HIGH);
  delay(100);
  digitalWrite(deploymentPin, LOW);
  Serial.println("Deployment Finished.");

}

// This function runs when the rising edge of the override pin occurs
void orRising() {
  // Grab current time when signal went high
  orStartTime = micros(); // Units: microseconds

  // Start the interrupt that fires when the falling edge (signal low) is detected
  attachInterrupt(digitalPinToInterrupt(orPinNum), orFalling, FALLING);
}

// This function runs when the falling edge of the override pin occurs
void orFalling() {
  // Grab difference between current time (when falling edge occured) and  when
  // the rising edge occured - the delta is the pulse width
  orPwmValue = micros() - orStartTime; // Units: microseconds

  // Check if the threshold was met (i.e. the override switch was flipped on the Tx)
  if (orPwmValue > orPwmThresh) {
    // If so, set manual override flag true and start elevator cmd interrupt
    if (!manualOverride){
      manualOverride = true;
      attachInterrupt(digitalPinToInterrupt(elPinNum), elRising, RISING);
    }
  }
  else {
    
    // If not, set manual override flag false and detach elevator cmd interrupt
    manualOverride = false;
    detachInterrupt(digitalPinToInterrupt(elPinNum));
  }
  
  // Restart the interrupt that fires when the rising edge (signal high) is detected
  attachInterrupt(digitalPinToInterrupt(orPinNum), orRising, RISING);
}

// This function runs when the rising edge of the elevator PWM command occurs
void elRising() {
  // Grab current time when signal went high
  elStartTime = micros(); // Units: microseconds

  // Start the interrupt that fires when the falling edge (signal low) is detected
  attachInterrupt(digitalPinToInterrupt(elPinNum), elFalling, FALLING);
}

// This function runs when the falling edge of the elevator PWM command occurs
void elFalling() {
  // Grab difference between current time (when falling edge occured) and  when
  // the rising edge occured - the delta is the pulse width
  elPwmValue = micros() - elStartTime; // Units: microseconds

  // Start the interrupt that fires when the falling edge (signal low) is detected
  attachInterrupt(digitalPinToInterrupt(elPinNum), elRising, RISING);
}
