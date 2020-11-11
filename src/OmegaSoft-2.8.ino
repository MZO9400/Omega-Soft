/* Delta Space Systems
      Version 2.8
    November, 10th 2020 */

/*System State:
   0 = Go/No Go before launch
   1 = PID Controlled Ascent
   2 = Main Engine Cutoff (MECO)
   3 = Apogee Detected
   4 = Chute Descent
   5 = Abort Detected
   6 = Battery voltage too low
*/

//Libraries
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include "BMI088.h"
#include <BMP280_DEV.h>


// Accelerometer Register
Bmi088Accel accel(Wire, 0x18);
// Gyroscope Register
Bmi088Gyro gyro(Wire, 0x68);

BMP280_DEV bmp280;

// BMP280 Variables
struct barometer {
float temperature, pressure, altitude, altitudefinal, altitude2;
};
barometer bmp;

// Altitude at which the chutes will deploy
float altsetpoint = 120;

// PID Controller Output
double PIDX, PIDY;

// Error of the flight computer angle
double errorX, errorY, previous_errorX, previous_errorY;

// PWM that gets sent to the servos
double pwmX, pwmY;

struct localOri {
// Local Integrated Gyros in Degrees
double Ax, Ay, Az;

// Local Integrated Gyros in radians
double RADGyroX, RADGyroY, RADGyroZ; 

// Previous Local Integrated Gyros
double PreviousGyroX, PreviousGyroY, PreviousGyroZ;

// Difference between beginning of the beginning and end of the loop
double DifferenceGyroX, DifferenceGyroY, DifferenceGyroZ;
};
localOri local;

// Matrix Variables
double matrix1, matrix2, matrix3, matrix4, matrix5, matrix6, matrix7, matrix8, matrix9;

// Local Angular Velocity(Raw Gyros)
double GyroRawX, GyroRawY, GyroRawZ;

// Final Matrix Multplication
double OreX, OreY, OreZ;

// Setting Orientation Vector
struct oriVector {
double OrientationX = 0;
double OrientationY = 0;
double OrientationZ = 1;
};
oriVector vector;


struct globalOri {
// Global Orientation in radians
double AxRAD, AyRAD;

// Global Orientation in degrees
double Ax, Ay;
};
globalOri global;

//Upright Angle of the Flight Computer
int desired_angleX = 0;
int desired_angleY = 0;

// If the TVC mount is moving the wrong way and causing a positive feedback loop then change this to 1
int servodirection = -1;

//Offsets for tuning
int servoY_offset = servodirection * 117.75;
int servoX_offset = servodirection * 131;

//Position of servos through the startup function
int servoXstart = servodirection * servoY_offset;
int servoYstart = servodirection * servoX_offset;

//The amount the servo moves by in the startup function
int servo_start_offset = 8;

//Ratio between servo gear and tvc mount
float servoX_gear_ratio = 4.5;
float servoY_gear_ratio = 3.5;

int liftoffThresh = 13;

// Defining Digital Pins
struct digitalPins {
int statusled = 9;
int errorled = 1;
int pyro1 = 34;
int pyro2 = 35;
int pyro3 = 33;
int voltagedivider = 0;
int buzzer = 15;
int teensyled = 13;
};
digitalPins digital;

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
double kp = 0.09;
double ki = 0.1;
double kd = 0.0275;

//Timer settings for dataLogging in Hz
unsigned long previousLog = 0;
const long logInterval = 150;

// Time in millis after launch when burnout can be triggered
const long burnoutInterval = 750;

// Delay after burnout when the flight computer will deploy the chutes
const long burnoutTimeInterval = 1000;

// Event Timer Variables
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

// LED Struct
struct RGB {
  int r;
  int g;
  int b;
};

class RGB_LED {
  private:
    // Create the three variables for each pin on an RGB LED
    int red;
    int green;
    int blue;

  public:

    // Constructor for the class
    RGB_LED(int newRedPin, int newGreenPin, int newBluePin) {
      // Set the pins in private to the new pins
      red = newRedPin;
      blue = newBluePin;
      green = newGreenPin;

      // Set the pin mode of each pin to output
      pinMode(red, OUTPUT);
      pinMode(green, OUTPUT);
      pinMode(blue, OUTPUT);
    }

    void Color(int newRed, int newGreen, int newBlue) {
      analogWrite(red, newRed);
      analogWrite(green, newGreen);
      analogWrite(blue, newBlue);
    }
    
    void Color(RGB rgb) {
      analogWrite(red, rgb.r);
      analogWrite(green, rgb.g);
      analogWrite(blue, rgb.b);
    }

    void off() {
      // Turn each led pin off
      analogWrite(red, 0);
      analogWrite(green, 0);
      analogWrite(blue, 0);
    }
};

RGB_LED LED(0, 5, 4);

// All possible RGB LED Colors
RGB yellow = {255, 255, 0};
RGB red = {255, 0, 0};
RGB green = {0, 255, 0};
RGB blue = {0, 0, 255};
RGB purple = {255, 0, 255};
RGB white = {255, 255, 255};

// Kalman variables

struct kalman {
// Change the value of altVariance to make the data smoother or respond faster
float altVariance = 1.12184278324081E-07;  
float varProcess = 1e-8;
float PC = 0.0;
float K = 0.0;
float UP = 1.0;
float altEst = 0.0;

// Change the value of accVariance to make the data smoother or respond faster
float accVariance = 1.12184278324081E-07; 
float varProcess2 = 1e-8;
float PC2 = 0.0;
float K2 = 0.0;
float UP2 = 1.0;
float accEst = 0.0;

// Change the value of voltVariance to make the data smoother or respond faster
float voltVariance = 1.12184278324081E-05; 
float varProcess3 = 1e-8;
float PC3 = 0.0;
float K3 = 0.0;
float UP3 = 1.0;
float voltageEst = 0.0;
};
kalman kal; 


enum FlightState {
  PAD_IDLE = 0,
  POWERED_FLIGHT = 1,
  MECO = 2,
  APOGEE = 3,
  CHUTE_DEPLOYMENT = 4,
  ABORT = 5,
  VOLTAGE_WARNING = 6
};
FlightState flightState = PAD_IDLE;

void setup() {
  // Starting serial communication with your computer
  Serial.begin(9600);

  // Starting communication with the I2C bus
  Wire.begin();

  // Attaching the servos to the desired pins
  servoX.attach(2);
  servoY.attach(3);

  // Writing servos to their start posistions
  servoX.write(servoXstart);
  servoY.write(servoYstart);

  // Starting communication with the onboard BMP280
  bmp280.begin();
  bmp280.startNormalConversion();

  // Setting the servo frequency
  analogWriteFrequency(2, servoFrequency);
  analogWriteFrequency(3, servoFrequency);
  
  // Setting all of the digital pins to output
  pinMode(digital.buzzer, OUTPUT);
  pinMode(digital.pyro1, OUTPUT);
  pinMode(digital.pyro2, OUTPUT);
  pinMode(digital.pyro3, OUTPUT);
  pinMode(digital.errorled, OUTPUT);
  pinMode(digital.statusled, OUTPUT);
  pinMode(digital.teensyled, OUTPUT);

  startup();
  sdstart();
  sdSettings();
  launchpoll();
}

void loop() {
  //Defining Time Variables
  currentTime = millis();
  currentTime_2 = millis();
  dt = (currentTime - previousTime) / 1000;

  // Get measurements from the BMP280
  if (bmp280.getMeasurements(bmp.temperature, bmp.pressure, bmp.altitude)) {
    inflightTimer();
    altitudeOffset();
    launchdetect();
    sdwrite();
    burnout();
    abortsystem();
    voltage();
    altKalman(kal.altEst);
    accZKalman(kal.accEst);

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
  local.Ax += GyroRawX * dt;
  local.Ay += GyroRawY * dt;
  local.Az += GyroRawZ * dt;

  local.PreviousGyroX = local.RADGyroX;
  local.PreviousGyroY = local.RADGyroY;
  local.PreviousGyroZ = local.RADGyroZ;

  local.RADGyroX = local.Ax;
  local.RADGyroY = local.Ay;
  local.RADGyroZ = local.Az;

  // Finding the difference between the beginning of beginning and the end of rotationmatrices
  local.DifferenceGyroX = (local.RADGyroX - local.PreviousGyroX);
  local.DifferenceGyroY = (local.RADGyroY - local.PreviousGyroY);
  local.DifferenceGyroZ = (local.RADGyroZ - local.PreviousGyroZ);

  OreX = vector.OrientationX;
  OreY = vector.OrientationY;
  OreZ = vector.OrientationZ;

  //X Matrices
  matrix1 = (cos(local.DifferenceGyroZ) * cos(local.DifferenceGyroY));
  matrix2 = (((sin(local.DifferenceGyroZ) * -1) * cos(local.DifferenceGyroX) + (cos(local.DifferenceGyroZ)) * sin(local.DifferenceGyroY) * sin(local.DifferenceGyroX)));
  matrix3 = ((sin(local.DifferenceGyroZ) * sin(local.DifferenceGyroX) + (cos(local.DifferenceGyroZ)) * sin(local.DifferenceGyroY) * cos(local.DifferenceGyroX)));

  //Y Matrices
  matrix4 = sin(local.DifferenceGyroZ) * cos(local.DifferenceGyroY);
  matrix5 = ((cos(local.DifferenceGyroZ) * cos(local.DifferenceGyroX) + (sin(local.DifferenceGyroZ)) * sin(local.DifferenceGyroY) * sin(local.DifferenceGyroX)));
  matrix6 = (((cos(local.DifferenceGyroZ) * -1) * sin(local.DifferenceGyroX) + (sin(local.DifferenceGyroZ)) * sin(local.DifferenceGyroY) * cos(local.DifferenceGyroX)));

  //Z Matrices
  matrix7 = (sin(local.DifferenceGyroY)) * -1;
  matrix8 = cos(local.DifferenceGyroY) * sin(local.DifferenceGyroX);
  matrix9 = cos(local.DifferenceGyroY) * cos(local.DifferenceGyroX);

  vector.OrientationX = ((OreX * matrix1)) + ((OreY * matrix2)) + ((OreZ * matrix3));
  vector.OrientationY = ((OreX * matrix4)) + ((OreY * matrix5)) + ((OreZ * matrix6));
  vector.OrientationZ = ((OreX * matrix7)) + ((OreY * matrix8)) + ((OreZ * matrix9));

  // Convert to euler angles
  global.AxRAD = asin(vector.OrientationX);
  global.AyRAD = asin(vector.OrientationY);

  //Convert from radians to degrees
  global.Ax = global.AxRAD * (-180 / PI);
  global.Ay = global.AyRAD * (180 / PI);

  pidcompute();
}

void pidcompute () {
  previous_errorX = errorX;
  previous_errorY = errorY;

  errorX = global.Ax - desired_angleX;
  errorY = global.Ay - desired_angleY;

  // Defining "P"
  pidX_p = kp * errorX;
  pidY_p = kp * errorY;

  // Defining "D"
  pidX_d = kd * ((errorX - previous_errorX) / dt);
  pidY_d = kd * ((errorY - previous_errorY) / dt);

  // Defining "I"
  pidX_i = ki * (pidX_i + errorX * dt);
  pidY_i = ki * (pidY_i + errorY * dt);


  // Adding it all up
  PIDX = pidX_p + pidX_i + pidX_d;
  PIDY = pidY_p + pidY_i + pidY_d;

  pwmY = servodirection * ((PIDY * servoX_gear_ratio) + servoX_offset);
  pwmX = servodirection * ((PIDX * servoY_gear_ratio) + servoY_offset);
  
  //Servo outputs
  servoX.write(pwmX);
  servoY.write(pwmY);
}

void startup () {
  delay(500);
  LED.Color(white);
  tone(digital.buzzer, 1050);
  delay(800);
  noTone(digital.buzzer);
  LED.Color(purple);
  tone(digital.buzzer, 1100);
  delay(400);
  noTone(digital.buzzer);
  delay(500);

  servoX.write(servoXstart + servo_start_offset);
  delay(400);
  servoX.write(servoXstart - servo_start_offset);
  delay(200);
  servoX.write(servoXstart);
  delay(400);
  servoY.write(servoYstart + servo_start_offset);
  delay(400);
  servoY.write(servoYstart - servo_start_offset);
  delay(200);
  servoY.write(servoYstart);

  tone(digital.buzzer, 1100);
  LED.Color(blue);
  delay(150);
  tone(digital.buzzer, 1200);
  LED.Color(red);
  delay(150);
  tone(digital.buzzer, 1300);
  LED.Color(green);
  delay(200);
  noTone(digital.buzzer);
}

void launchdetect () {
  // Read from the accelerometers
  accel.readSensor();
  digitalWrite(digital.statusled, HIGH);
  if (flightState == PAD_IDLE && accel.getAccelX_mss() > liftoffThresh) {
    flightState = POWERED_FLIGHT;
  }
  if (flightState == POWERED_FLIGHT) {
    // Read from the gyroscopes
    gyro.readSensor();
    LED.Color(blue);
    rotationmatrices();
  }
}

void sdstart () {
  if (!SD.begin(chipSelect)) {
    // Checking to see if the Teensy can communicate with the SD card
    Serial.println("SD card not present, please insert the card into the Teensy.");
    digitalWrite(digital.errorled, HIGH);
    return;
  }
  Serial.println("SD Card Initialized.");
}

void sdSettings () {
  String settingstring = "";

  settingstring += "Abort Offsets: ";
  settingstring += (abortoffset);
  settingstring += ", ";

  settingstring += "Log Interval: ";
  settingstring += (logInterval);
  settingstring += ", ";

  settingstring += "Burnout Time Delay: ";
  settingstring += (burnoutTimeInterval);
  settingstring += ", ";

  settingstring += "Time after launch when burnout is activated: ";
  settingstring += (burnoutInterval);
  settingstring += ", ";

  settingstring += "Voltage Multiplier: ";
  settingstring += (voltageDividerMultplier);
  settingstring += ", ";

  settingstring += "Deployment Altitude: ";
  settingstring += (altsetpoint);
  settingstring += ", ";

  settingstring += "Kp: ";
  settingstring += (kp);
  settingstring += ", ";

  settingstring += "Ki: ";
  settingstring += (ki);
  settingstring += ", ";

  settingstring += "Kd: ";
  settingstring += (kd);

  File settingFile = SD.open("log002.txt", FILE_WRITE);

  if (settingFile) {
      settingFile.println(settingstring);
      settingFile.close();
  }
}

void sdwrite () {
  String datastring = "";

  datastring += "Pitch,";
  datastring += String(global.Ax);
  datastring += ",";

  datastring += "Yaw,";
  datastring += String(global.Ay);
  datastring += ",";

  datastring += "Roll,";
  datastring += String(local.Az);
  datastring += ",";

  datastring += "System_State,";
  datastring += String(flightState);
  datastring += ",";

  datastring += "Raw_Z_Axis_Accel,";
  datastring += String(accel.getAccelZ_mss());
  datastring += ",";

  datastring += "Filtered_Z_Axis_Accel,";
  datastring += String(kal.accEst);
  datastring += ",";

  datastring += "Servo_X_POS,";
  datastring += String(servoX.read());
  datastring += ",";

  datastring += "Servo_Y_POS,";
  datastring += String(servoY.read());
  datastring += ",";

  datastring += "Raw_Voltage,";
  datastring += String(voltageDividerOUT);
  datastring += ",";

  datastring += "Filtered_Voltage,";
  datastring += String(kal.voltageEst);
  datastring += ",";

  datastring += "IMU_Temp,";
  datastring += String(accel.getTemperature_C());
  datastring += ",";

  datastring += "Barometer_Temp,";
  datastring += String(bmp.temperature);
  datastring += ",";

  datastring += "Filtered_Altitude,";
  datastring += String(kal.altEst);
  datastring += ",";

  datastring += "Raw_Altitude,";
  datastring += String(bmp.altitudefinal);
  datastring += ",";

  datastring += "Pressure,";
  datastring += String(bmp.pressure);
  datastring += ",";


  File omegaFile = SD.open("log001.txt", FILE_WRITE);

  if (omegaFile) {
    if (currentTime - previousLog > logInterval) {
      previousLog = currentTime;
      omegaFile.println(datastring);
      omegaFile.close();
    }
  }
}

void burnout () {
  if ((flightState == POWERED_FLIGHT) && kal.accEst <= 2 && (flightTime > burnoutInterval)) {
    //Burnout Detected; changing the system state to state 2
    flightState = MECO;
    digitalWrite(digital.teensyled, LOW); 
    LED.Color(green);
    Serial.println("Burnout Detected");
  }

  if ((flightState == MECO) && burnoutTime > burnoutTimeInterval) {
    //Apogee Detected; changing the system state to state 3
    flightState = APOGEE;
    LED.off();
    Serial.println("Apogee Detected");
    tone(digital.buzzer, 1200, 200);
  }

  if ((flightState == APOGEE || flightState == CHUTE_DEPLOYMENT) && (kal.altEst - launchsite_alt) <= altsetpoint) {
    // Chute deployment; changing the system state to state 4
    flightState = CHUTE_DEPLOYMENT;
    digitalWrite(digital.pyro1, HIGH);
    digitalWrite(digital.pyro2, HIGH);
    digitalWrite(digital.pyro3, HIGH);
    LED.Color(red);
  }
}

void launchpoll () {
  delay(750);
  if (flightState == PAD_IDLE) {
    int status;
    //Checking to see if the Teensy can communicate with the BMI088 accelerometer
    status = accel.begin();
    if (status < 0) {
      Serial.println("Accel Initialization Error");
      digitalWrite(digital.errorled, HIGH);
      while (1) {}
    }

    //Checking to see if the Teensy can communicate with the BMI088 gyroscopes
    status = gyro.begin();
    if (status < 0) {
      Serial.println("Gyro Initialization Error");
      digitalWrite(digital.errorled, HIGH);
      while (1) {}
    }
    Serial.println("I2C Devices Found!");
    delay(250);

    calibrateGyroscopes(accel.getAccelY_mss(), accel.getAccelZ_mss(), accel.getAccelX_mss());
    Serial.println("Gyroscopes have been calibrated.");
    LED.Color(yellow);


  }
}
void abortsystem () {
  if ((flightState == POWERED_FLIGHT) && (global.Ax > abortoffset || global.Ax < -abortoffset) || (global.Ay > abortoffset || global.Ay < -abortoffset)) {
    Serial.println("Abort Detected.");
    LED.Color(purple);
    digitalWrite(digital.teensyled, LOW);

    // Changing the system state to 5
    flightState = ABORT;

    // Firing the pyrotechnic channel
    digitalWrite(digital.pyro1, HIGH);
    digitalWrite(digital.pyro2, HIGH);
    digitalWrite(digital.pyro3, HIGH);
    tone(digital.buzzer, 1200, 400);
  }
}

void voltage () {
  // Reading the voltage from the analog pin
  voltageDividerIN = analogRead(digital.voltagedivider);

  // Map the voltage to 3.3V
  voltageDividermap = map(voltageDividerIN, 0, 2048, 0, 3.3);

  // Multiply the output from the mapping function to get the true voltage
  voltageDividerOUT = voltageDividermap * voltageDividerMultplier;

  // Calls the function to filter the voltage data
  voltageKalman(kal.voltageEst);

}

void inflightTimer () {
  //Timer to wait a certain time after launch before burnout is activated
  if (flightState == PAD_IDLE) {
    liftoffTime = currentTime;
  }
  flightTime = currentTime - liftoffTime;

  //Timer that waits a certain time after burnout to deploy the chutes
  if (flightState == PAD_IDLE || flightState == POWERED_FLIGHT) {
    burnoutTime_2 = currentTime_2;
  }
  burnoutTime = currentTime_2 - burnoutTime_2;
}

void altitudeOffset () {
  // Gets offsets of altitude at launch so you dont have to manually set your launch site altitude
  if (flightState == PAD_IDLE) {
    bmp.altitude2 = bmp.altitude;
  }
  bmp.altitudefinal = bmp.altitude - bmp.altitude2;
}

void voltageWarning () {
  // If the system voltage is less than 7.6
  if (kal.voltageEst <= 7.4) {
    flightState = VOLTAGE_WARNING;
    digitalWrite(digital.teensyled, HIGH);
    tone(digital.buzzer, 1200);
    delay(400);
    digitalWrite(digital.teensyled, LOW);
    noTone(digital.buzzer);
    delay(400);

  }
}

void calibrateGyroscopes(float AcX, float AcY, float AcZ) { 
  // Calculating the gyro offsets
  accel.readSensor();
  float totalAccel = sqrt(sq(AcZ) + sq(AcX) + sq(AcY));
  global.Ax = -asin(AcZ / totalAccel);
  global.Ay = asin(AcX / totalAccel);
}

void altKalman (float Xp) {
  // Predict the next covariance
  kal.PC = kal.UP + kal.varProcess;

  // Compute the kalman gain
  kal.K = kal.PC / (kal.PC + kal.altVariance);

  // Update the covariance 
  kal.UP = (1 - kal.K) * kal.PC;

  // Re-define variables
  Xp = kal.altEst;
  float Zp = Xp;

  // Final altitude estimation
  kal.altEst = kal.K * (bmp.altitudefinal - Zp) + Xp;   
}

void accZKalman (double Xp2) {
  // Predict the next covariance
  kal.PC2 = kal.UP2 + kal.varProcess2;
  
  // Compute the kalman gain
  kal.K2 = kal.PC2 / (kal.PC2 + kal.accVariance);   

  // Update the covariance 
  kal.UP2 = (1 - kal.K2) * kal.PC2;

  // Re-define variables
  Xp2 = kal.accEst;
  float Zp2 = Xp2;

  // Final acceleration estimation
  kal.accEst = kal.K2 * (accel.getAccelX_mss() - Zp2) + Xp2;  
}

void voltageKalman (float Xp3) {
  // Predict the next covariance
  kal.PC3 = kal.UP3 + kal.varProcess3;

  // Compute the kalman gain
  kal.K3 = kal.PC3 / (kal.PC3 + kal.voltVariance); 

  // Update the covariance   
  kal.UP3 = (1 - kal.K3) * kal.PC3;

  // Re-define variables
  Xp3 = kal.voltageEst;
  float Zp3 = Xp3;

  // Final voltage estimation
  kal.voltageEst = kal.K3 * (voltageDividerOUT - Zp3) + Xp3;  
}

