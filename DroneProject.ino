/* Daniel Gaviria
 *  2/21/2019
 *  Intel curienano board
 *  pitch heading with complimentary filter, PID control and servo control
 */

#include "CurieIMU.h"                                   // pulls libraries to communicate with the board
#include <Servo.h>                                      // pulls libraries to communicate with servo


  // heading variables
  float gx, gy, gz, ax, ay, az;                         // scaled values for gyroscope and acceleromter
  float groll, gpitch, gyaw;                            // value for gyroscopic roll pitch and yaw
  float atotal, aroll, apitch;                          // value for acceleromter total magnitude, roll, pitch, and yaw 
  float roll, pitch;                                    // filtered roll and pitch output
  float pitch_angle = 0;                                // angle that we want the pitch to maintain.
  
  // PID 
  float PID, error, previous_error;                     // Sum value of the PID controller, error between desired angle and actual angle, and records the error for the next loop run
  float pid_p=0;                                        // proportional value of the pid
  float pid_i=0;                                        // integrative value of the pid
  float pid_d=0;                                        // derivative value of the pid
  // PID constants should be optimized for the glider aspect
  double kp=3;                                          // k constant for the porportional part of the pid
  double ki=0;                                          // k constant for the integrative part of the pid
  double kd=2;                                          // k variable for the derrivative part of the pid
 
  // timer variables
  float elapsedtime, time, time_prev;                   // variables used to keep time of the program. Will be used to help intergrate gyro data
  unsigned long time_now = 0;                           // variable to signify the starting time for the loop run

  // Servo variable and constants
  Servo elevator;                                      // creating a new servo called elevator. Guess what it will control. Thats right....the elevator 

  // some extra stuff for the led that will let me know if i should be happy or crying
  boolean gyro_set;
  const int led = 13;
  boolean blinkstate = false;
  
  int calibrateOffsets = 1;                             // int to determine whether calibration takes place or not
  
  
void setup() {
  Serial.begin(9600);                                   // begins serial communications
  while (!Serial);                                      // waits for the serial port to open before starting the program

  elevator.attach(9);                                   // attached my elevator servo to pin 9. Hope they dont get too attatched though
  
  Serial.println("Initializing device");
  CurieIMU.begin();                        

  
  CurieIMU.setGyroRange(250);                           // sets alecceromter range to 250 degrees/second
  CurieIMU.setGyroRate(25);                             // sets the rate at witch gyro data is read in HZ (used to intergrate and acuire attitude)
  CurieIMU.setAccelerometerRange(2);                    // sets acceleromter range to 2G

  if (calibrateOffsets == 1) {                          // calibrateoffset is equal to 1, so its true, therfore begin calibrating data
    Serial.println("Internal sensor offsets BEFORE calibration...");
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));       // reads the gyro vslue for roll
    Serial.print("\t");
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));       // reads the gyro value for pitch
    Serial.print("\t"); 
    Serial.print(CurieIMU.getGyroOffset(Z_AXIS));       // reads the gyro value for yaw
    Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));      // reads the acceleromete vslue for x axis
    Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));      // reads the acceleromete vslue for y axis
    Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));      // reads the acceleromter value for z axis
    Serial.println();
    
  Serial.println("About to calibrate. Make sure your board is stable and upright");
    delay(2000);

    
   Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateGyroOffset();                // initiates the calibration software built in to the board
    Serial.println(" Done"); 

   Serial.print("Starting Acceleration calibration and enabling offset compensation...");
    CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);        // initiates the calibration software for x axis accelermoter built in to the board
    CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);        // initiates the calibration software for y axis accelermoter built in to the board
    CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);        // initiates the calibration software for z axis accelermoter built in to the board
    Serial.println(" Done"); 
    delay(1000);

   // prints out the offset value that is used to calibrate
   Serial.println("Internal sensor offsets AFTER calibration...");                    
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));   
    Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
    Serial.print("\t");
    Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));                      
    Serial.print("\t");
    Serial.print(CurieIMU.getGyroOffset(X_AXIS));
    Serial.print("\t");      
    Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
    Serial.print("\t");
    Serial.print(CurieIMU.getGyroOffset(Z_AXIS));                                        
    Serial.println();
     delay(1000);
  }
   time = millis();                                 // begins counting in milliseconds
}
void loop() {

  time_prev = time;                                 // previos time stored before actual time read
  time = millis();                                  // actual time read
  elapsedtime = (time - time_prev)/1000;            // since in milliseconds, this value is divided by 1000 to get seconds

  CurieIMU.readGyroScaled(gx, gy, gz);              // reads gyro measurement and is scaled to the configured range
  CurieIMU.readAccelerometerScaled(ax, ay, az);     // reads accelremoter measurement and is scaled to configured range

  // intergrates gyro rate to get angle heading. 1/Hz = s
  groll+=(gx*elapsedtime);                                      
  gpitch+=(gy*elapsedtime);
  gyaw+=(gz*elapsedtime);

  atotal = sqrt((ax*ax)+(ay*ay)+(az*az));           // calcultaes the accelerometer total vector
  aroll=asin((float) ay/atotal)*57.296;             // 57.296 = 180/3.142 converting to radians
  apitch=asin((float) ax/atotal)*-57.296;

  // low band pass filter that takes 99.96 percent of the gyro data and adds .04 percent of the acceleromter data
  if(gyro_set){                                     
    gpitch = gpitch*.98 + apitch*.02;
    groll = groll*.98 + aroll*.02;
  }
  // during initial setup, the gyro data will initially match the accelerometer data since the gyro data wont read 0 initially
  else{                                             
    gpitch = apitch;            
    groll = aroll;
    gyro_set = true;
  }
  // high band pass filter, takes 90 percent of the previous reading and adds 10 percent of the current reading
  roll = roll*.9 + groll*.1;                       
  pitch = pitch*.9 + gpitch*.1;

  // PID stuff!!!

  error = pitch - pitch_angle;                     // calculates the error between the measured angle and desired angle

  pid_p = kp*error;                                // proprtional value is just constant time error

  // integrative value just adjusts for small error in angle read out like between -2 and 2 degrees. Should only be needed for small angles
  if(-3 <error< 3){
    pid_i = pid_i+(ki*error);
  }

  // derrivative value takes in account previous error and adjust current error
  pid_d = kd*((error - previous_error)/elapsedtime);

  // Sum values equals the PID value to adjust the servos with
  PID = pid_p + pid_i + pid_d;

  //bunch of print statements to let me know values being read for gyro roll, gyro pitch, gyro yaw, accel roll,accel pitch and filtered roll and pitch 
  Serial.print("g_roll:\t");
  Serial.print(groll);
  Serial.print("\t");
  
  Serial.print("g_pitch:\t");
  Serial.print(gpitch);
  Serial.print("\t");
  
  Serial.print("g_yaw:\t");
  Serial.print(gyaw);
  Serial.print("\t");
  
  Serial.print("a_roll:\t");
  Serial.print(aroll);
  Serial.print("\t");
  
  Serial.print("a_pitch:\t");
  Serial.print(apitch);
  Serial.print("\t");

  Serial.print("f_roll:\t");
  Serial.print(roll);
  Serial.print("\t");

  Serial.print("f_pitch:\t");
  Serial.print(pitch);
  Serial.print("\t");

  Serial.print("PID value:\t");
  Serial.print(PID);

  // Time to tell this servo what to do
  PID = map(PID, -245, 245, 0, 180);                  // took PID values which range from -245 to 245 and mapped it from -90 to 90. im really sleepy
  elevator.write(PID);                                 // told that servo whats up. it should move accordingly to bring it back to 0 pitch. More work should be done on this though
  
  // will let me know the program is running by turning the LED on
  blinkstate = !blinkstate;
  digitalWrite(led, blinkstate);
  Serial.print("\n");
  //delay(1000);
  previous_error = error;                         // records the error to be used fo the next loop run
                    
  }
