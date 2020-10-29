/* Delta Space Systems
      Version 2.6
    October, 25th 2020 */

/*System State: 
 * 0 = Go/No Go before launch
 * 1 = PID Controlled Ascent
 * 2 = Main Engine Cutoff (MECO)
 * 3 = Apogee Detected
 * 4 = Chute Descent
 * 5 = Abort Detected
 * 6 = Battery voltage too low
 */

//Libraries
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include "BMI088.h"
#include <BMP280_DEV.h>                           


/* accel object */
Bmi088Accel accel(Wire,0x18);
/* gyro object */
Bmi088Gyro gyro(Wire,0x68);

BMP280_DEV bmp280;  

float temperature, pressure, altitude, altitudefinal, altitude2;            
   
// PID Controller Output
double PIDX, PIDY;

// Error of the flight computer angle
double errorX, errorY, previous_errorX, previous_errorY;

// PWM that gets sent to the servos
double pwmX, pwmY;

// Local Integrated Gyros in Degrees
double GyroAngleX, GyroAngleY, GyroAngleZ;

// Local Integrated Gyros in radians
double RADGyroX, RADGyroY, RADGyroZ, PreviousGyroX, PreviousGyroY, PreviousGyroZ; 

// Difference between beginning of the beginning and end of the loop
double DifferenceGyroX, DifferenceGyroY, DifferenceGyroZ;

// Matrix Variables
double matrix1, matrix2, matrix3, matrix4, matrix5, matrix6, matrix7, matrix8, matrix9;

// Local Angular Velocity(Raw Gyros)
double GyroRawX, GyroRawY, GyroRawZ;

// Final Matrix Multplication
double OreX, OreY, OreZ;

// Setting Orientation Vector
double OrientationX = 0;
double OrientationY = 0;
double OrientationZ = 1;

// Global Orientation in radians
double AxRAD, AyRAD;

// Global Orientation in degrees
double Ax, Ay;

//Upright Angle of the Flight Computer
int desired_angleX = 0;
int desired_angleY = 0;

// If the TVC mount is moving the wrong way and causing a positive feedback loop then change this to 1
int servodirection = -1;

//Offsets for tuning 
int servoY_offset = servodirection * 120;
int servoX_offset = servodirection * 120;

//Position of servos through the startup function
int servoXstart = servodirection * servoY_offset;
int servoYstart = servodirection * servoX_offset;

//The amount the servo moves by in the startup function
int servo_start_offset = 8;

//Ratio between servo gear and tvc mount
float servoX_gear_ratio = 4;
float servoY_gear_ratio = 4;

// Defining Digital Pins
int ledred = 0;    
int ledblu = 4;    
int ledgrn = 5; 
int statusled = 9;
int errorled = 1;
int pyro1 = 34;
int pyro2 = 35;
int pyro3 = 33;
int voltagedivider = 0;
int buzzer = 15;
int teensyled = 13;

// Defining the servo pins as integers
Servo servoX;
Servo servoY;

// Defining Time Variables
double dt, currentTime, currentTime_2, previousTime;

//SD CARD Chip Select Pin
const int chipSelect = BUILTIN_SDCARD;

//"P" Constants
float pidX_p = 0;
float pidY_p = 0;

//"I" Constants
float pidY_i = 0;
float pidX_i = 0;

//"D" Constants
float pidX_d = 0;
float pidY_d = 0;


//PID Gains
double kp = 0.12;
double ki = 0.0;
double kd = 0.02;

// Setting the system state to 0
int state = 0;

//Timer settings for dataLogging in Hz
unsigned long previousLog = 0;        
const long logInterval = 100;  

// Time in millis after launch when burnout can be triggered  
const long burnoutInterval = 750;

// Delay after burnout when the flight computer will deploy the chutes
const long burnoutTimeInterval = 1500;    

unsigned long liftoffTime, flightTime, burnoutTime_2, burnoutTime;

// Voltage divider variables
float voltageDividerIN;
float voltageDividerOUT;
float voltageDividermap;
float voltageDividerMultplier = 5.86;

// The degrees that triggers the abort function
int abortoffset = 45;

// Launch Site Altitude in Meters(ASL)
int launchsite_alt = 0;

// Servo frequency
int servoFrequency = 333;

void setup(){
  // Starting serial communication with your computer
  Serial.begin(9600);

  // Starting communication with the I2C bus
  Wire.begin();

  // Attaching the servos to the desired pins
  servoX.attach(2);
  servoY.attach(3);

  // Starting communication with the onboard BMP280
  bmp280.begin();              
  bmp280.startNormalConversion();  

  // Setting the servo frequency
  analogWriteFrequency(2, servoFrequency);
  analogWriteFrequency(3, servoFrequency);
  
  // Setting all of the digital pins to output
  pinMode(ledblu, OUTPUT);
  pinMode(ledgrn, OUTPUT);
  pinMode(ledred, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(pyro1, OUTPUT);
  pinMode(pyro2, OUTPUT);
  pinMode(pyro3, OUTPUT);
  pinMode(errorled, OUTPUT);
  pinMode(statusled, OUTPUT);
  pinMode(teensyled, OUTPUT);
 
  startup();
  sdstart();
  launchpoll();
  
}

void loop() {
  //Defining Time Variables      
  currentTime = millis();    
  currentTime_2 = millis();         
  dt = (currentTime - previousTime) / 1000; 

  // Get measurements from the BMP280 
  if (bmp280.getMeasurements(temperature, pressure, altitude)) {
  
  //Timer to wait a certain time after launch before burnout is activated
  if (state == 0) {
    liftoffTime = currentTime;
  }
  flightTime = currentTime - liftoffTime;

  //Timer that waits a certain time after burnout to deploy the chutes
  if (state == 0 || state == 1) {
    burnoutTime_2 = currentTime_2;
  }
  burnoutTime = currentTime_2 - burnoutTime_2;
  
  // Gets offsets of altitude at launch so you dont have to manually set your launch site altitude
  if (state == 0) {
    altitude2 = altitude;
  }
  altitudefinal = altitude - altitude2;
 
  launchdetect();
  sdwrite();
  burnout();
  abortsystem();
  voltage();
  
  // If the system voltage is less than 7.6
    if (voltageDividerOUT <= 7.6 && state == 0) {
      state = 6;
      digitalWrite(teensyled, HIGH);
      tone(buzzer, 1200);
      delay(400);
      digitalWrite(teensyled, LOW);
      noTone(buzzer);
      delay(400);
    
  }
 
  // Setting the previous time to the current time
  previousTime = currentTime;  
}
}
void rotationmatrices () {
  //Change Variable so its easier to refrence later on
  GyroRawX = (gyro.getGyroY_rads());
  GyroRawY = (gyro.getGyroZ_rads());
  GyroRawZ = (gyro.getGyroX_rads());

  //Integrate over time to get Local Orientation
  GyroAngleX += GyroRawX * dt;
  GyroAngleY += GyroRawY * dt;
  GyroAngleZ += GyroRawZ * dt;

  PreviousGyroX = RADGyroX;
  PreviousGyroY = RADGyroY;
  PreviousGyroZ = RADGyroZ;
  
  RADGyroX = GyroAngleX;
  RADGyroY = GyroAngleY;
  RADGyroZ = GyroAngleZ;
  
  // Finding the difference between the beginning of beginning and the end of rotationmatrices
  DifferenceGyroX = (RADGyroX - PreviousGyroX);
  DifferenceGyroY = (RADGyroY - PreviousGyroY);
  DifferenceGyroZ = (RADGyroZ - PreviousGyroZ);

  OreX = OrientationX;
  OreY = OrientationY;
  OreZ = OrientationZ;
  
 //X Matrices
  matrix1 = (cos(DifferenceGyroZ) * cos(DifferenceGyroY));
  matrix2 = (((sin(DifferenceGyroZ) * -1) * cos(DifferenceGyroX) + (cos(DifferenceGyroZ)) * sin(DifferenceGyroY) * sin(DifferenceGyroX)));
  matrix3 = ((sin(DifferenceGyroZ) * sin(DifferenceGyroX) + (cos(DifferenceGyroZ)) * sin(DifferenceGyroY) * cos(DifferenceGyroX)));
  
 //Y Matrices
  matrix4 = sin(DifferenceGyroZ) * cos(DifferenceGyroY);
  matrix5 = ((cos(DifferenceGyroZ) * cos(DifferenceGyroX) + (sin(DifferenceGyroZ)) * sin(DifferenceGyroY) * sin(DifferenceGyroX)));
  matrix6 = (((cos(DifferenceGyroZ) * -1) * sin(DifferenceGyroX) + (sin(DifferenceGyroZ)) * sin(DifferenceGyroY) * cos(DifferenceGyroX)));

 //Z Matrices
  matrix7 = (sin(DifferenceGyroY)) * -1;
  matrix8 = cos(DifferenceGyroY) * sin(DifferenceGyroX);
  matrix9 = cos(DifferenceGyroY) * cos(DifferenceGyroX);

  OrientationX = ((OreX * matrix1)) + ((OreY * matrix2)) + ((OreZ * matrix3));
  OrientationY = ((OreX * matrix4)) + ((OreY * matrix5)) + ((OreZ * matrix6));
  OrientationZ = ((OreX * matrix7)) + ((OreY * matrix8)) + ((OreZ * matrix9));

  // Convert to euler angles
  AxRAD = asin(OrientationX);
  AyRAD = asin(OrientationY);

  //Convert from radians to degrees
  Ax = AxRAD * (-180 / PI);
  Ay = AyRAD * (180 / PI);

  pidcompute();
}

void pidcompute () {
  previous_errorX = errorX;
  previous_errorY = errorY; 

  errorX = Ax - desired_angleX;
  errorY = Ay - desired_angleY;

  //Defining "P" 
  pidX_p = kp * errorX;
  pidY_p = kp * errorY;

  //Defining "D"
  pidX_d = kd * ((errorX - previous_errorX) / dt);
  pidY_d = kd * ((errorY - previous_errorY) / dt);

  //Defining "I"
  pidX_i = ki * (pidX_i + errorX * dt);
  pidY_i = ki * (pidY_i + errorY * dt);

  //Adding it all up
  PIDX = pidX_p + pidX_i + pidX_d;
  PIDY = pidY_p + pidY_i + pidY_d;
 
  pwmY = servodirection * ((PIDY * servoY_gear_ratio) + servoX_offset);
  pwmX = servodirection * ((PIDX * servoX_gear_ratio) + servoY_offset); 

  //Servo outputs
  servoX.write(pwmX);
  servoY.write(pwmY);
}

void startup () {
  digitalWrite(ledblu, HIGH);
  digitalWrite(ledred, HIGH);
  digitalWrite(ledgrn, HIGH);
  tone(buzzer, 1050);
  delay(800);
  noTone(buzzer);
  digitalWrite(ledgrn, LOW);
  tone(buzzer, 1150);
  delay(400);
  noTone(buzzer);
  digitalWrite(ledred, LOW);
  digitalWrite(ledblu, HIGH);
  servoX.write(servoXstart);
  servoY.write(servoYstart);
  delay(500);
  digitalWrite(ledblu, LOW);
  digitalWrite(ledgrn, HIGH);
  servoX.write(servoXstart + servo_start_offset);
  delay(400);
  digitalWrite(ledgrn, LOW);
  digitalWrite(ledred, HIGH);
  servoX.write(servoXstart - servo_start_offset);
  delay(200);
  servoX.write(servoXstart);
  delay(200);
  digitalWrite(ledred, LOW);
  digitalWrite(ledblu, HIGH);
  tone(buzzer, 1100);
  servoY.write(servoYstart + servo_start_offset);
  delay(400);
  tone(buzzer, 1150);
  servoY.write(servoYstart - servo_start_offset);
  delay(200);
  tone(buzzer, 1200);
  servoY.write(servoYstart);
  delay(400);
  noTone(buzzer);
}
 
void launchdetect () {  
  // Read from the accelerometers
  accel.readSensor();
  digitalWrite(statusled, HIGH);
  if (state == 0 && accel.getAccelX_mss() > 13) {
    state++;
  }
  if (state == 1) {
    // Read from the gyroscopes
    gyro.readSensor();
    digitalWrite(ledred, LOW);
    digitalWrite(ledgrn, LOW);
    digitalWrite(ledblu, HIGH);
    rotationmatrices();
 }
}

void sdstart () { 
if (!SD.begin(chipSelect)) {
    // Checking to see if the Teensy can communicate with the SD card
    Serial.println("SD card not present, please insert the card into the Teensy.");
    digitalWrite(errorled, HIGH);
    // Don't do anything else
    return;
  }
  Serial.println("SD Card Initialized.");
}

void sdwrite () {
  String datastring = "";

  datastring += "Pitch,"; 
  datastring += String(Ax);
  datastring += ",";

  datastring += "Yaw,";
  datastring += String(Ay);
  datastring += ",";
 
  datastring += "System_State,";
  datastring += String(state);
  datastring += ",";
 
  datastring += "Z_Axis_Accel,";
  datastring += String(accel.getAccelZ_mss());
  datastring += ",";

  datastring += "Servo_X_POS,";
  datastring += String(servoX.read());
  datastring += ",";
 
  datastring += "Servo_Y_POS,";
  datastring += String(servoY.read());
  datastring += ",";

  datastring += "Voltage,";
  datastring += String(voltageDividerOUT);
  datastring += ",";

  datastring += "IMU Temp,";
  datastring += String(accel.getTemperature_C());
  datastring += ",";

  datastring += "Barometer Temp,";
  datastring += String(temperature);
  datastring += ",";
  
  datastring += "Altitude,";
  datastring += String(altitudefinal);
  datastring += ",";
  
  datastring += "Pressure,";
  datastring += String(pressure);
  datastring += ",";
  
  
  File omegaFile = SD.open("log001.txt", FILE_WRITE);
  
  if (omegaFile) {
    if(currentTime - previousLog > logInterval){
      previousLog = currentTime;
      omegaFile.println(datastring);
      omegaFile.close();
  }    
 }
}

void burnout () { 
  if ((state == 1) && accel.getAccelX_mss() <= 2 && (flightTime > burnoutInterval)) {
    //Burnout Detected; changing the system state to state 2
    state++;
    digitalWrite(teensyled, LOW);
    digitalWrite(ledred, LOW);
    digitalWrite(ledblu, LOW);
    digitalWrite(ledgrn, HIGH);
    Serial.println("Burnout Detected");
  }

  if((state == 2) && burnoutTime > burnoutTimeInterval) {
    //Apogee Detected; changing the system state to state 3
    state++;
    digitalWrite(ledgrn, LOW);
    Serial.println("Apogee Detected");
    tone(buzzer, 1200, 200);
  }

  if (state == 3 && (altitudefinal - launchsite_alt) <= -1) {
    // Chute deployment
    state++;
    digitalWrite(pyro1, HIGH);
    digitalWrite(ledred, HIGH);
  }
}

void launchpoll () {
 delay(1000);


 if (state == 0) {
  int status;
  //Checking to see if the Teensy can communicate with the BMI088 accelerometer
  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    digitalWrite(errorled, HIGH);
    while (1) {}
    }

  //Checking to see if the Teensy can communicate with the BMI088 gyroscopes
  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error"); 
    digitalWrite(errorled, HIGH);
    while (1) {}
  }
  Serial.println("I2C Devices Found!");
  delay(250);
 
  // Calculating the gyro offsets
  float totalAccelVec = sqrt(sq(accel.getAccelZ_mss()) + sq(accel.getAccelY_mss()) + sq(accel.getAccelX_mss()));
  Ax = -asin(accel.getAccelZ_mss() / totalAccelVec);
  Ay = asin(accel.getAccelY_mss() / totalAccelVec);

  Serial.println("Gyroscopes have been calibrated.");
  delay(500);
  digitalWrite(ledgrn, HIGH);
  digitalWrite(ledred, HIGH);
  digitalWrite(ledblu, LOW);
  
  }
}
void abortsystem () {
  if ((state == 1) && (Ax > abortoffset || Ax < -abortoffset) || (Ay > abortoffset || Ay < -abortoffset) && (flightTime > burnoutInterval)) {
    Serial.println("Abort Detected.");
    digitalWrite(ledblu, HIGH);
    digitalWrite(ledgrn, LOW);
    digitalWrite(ledred, HIGH);
    digitalWrite(teensyled, LOW);

    // Changing the system state to 5
    state = 5;

    // Firing the pyrotechnic channel
    digitalWrite(pyro1, HIGH);
    tone(buzzer, 1200, 400);
  }
}

void voltage () {
  // Reading the voltage from the analog pin
  voltageDividerIN = analogRead(voltagedivider);
  
  // Map the voltage to 3.3V 
  voltageDividermap = map(voltageDividerIN, 0, 2048, 0, 3.3);
  
  // Multiply the output from the mapping function to get the true voltage
  voltageDividerOUT = voltageDividermap * voltageDividerMultplier;
}
