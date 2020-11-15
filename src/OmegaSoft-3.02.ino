/* Delta Space Systems
      Version 3.02
    November, 14th 2020 */

/*System State:
   0 = Configuration
   1 = Gyroscope Calibration
   2 = Go/No Go before launch
   3 = PID Controlled Ascent
   4 = Main Engine Cutoff (MECO)
   5 = Apogee Detected
   6 = Chute Descent
   7 = Abort Detected
   8 = Battery voltage too low
   9 = Disco Mode
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

// Orientation Variables
struct localOri {
  // Local Integrated Gyros in Degrees
  double Ax, Ay, Az;

  // Local Integrated Gyros in radians
  double RADGyroX, RADGyroY, RADGyroZ; 

  // Previous Local Integrated Gyros
  double PreviousGyroX, PreviousGyroY, PreviousGyroZ;

  // Difference between beginning of the beginning and end of the loop
  double DifferenceGyroX, DifferenceGyroY, DifferenceGyroZ;

  // Local Angular Velocity(Raw Gyros)
  double GyroRawX, GyroRawY, GyroRawZ;
};
localOri local;

struct globalOri {
  // Global Orientation in radians
  double AxRAD, AyRAD;

  // Global Orientation in degrees
  double Ax, Ay;
};
globalOri global;

// Matrix Variables
double matrix[9];

// Final Matrix Multplication
double Ore[3];

// Setting Orientation Vector
struct oriVector {
  double OrientationX = 0;
  double OrientationY = 0;
  double OrientationZ = 1;
};
oriVector vector;


struct PIDVari {
  // PID Controller Output
  double X, Y;

  // Error of the flight computer angle
  double errorX, errorY, previous_errorX, previous_errorY;

  // PWM that gets sent to the servos
  double pwmX, pwmY;

  protected:
    // Upright Angle of the Flight Computer
    int16_t desiredAngleX;
    int16_t desiredAngleY;

    //"P" Constants
    float X_p, Y_p;

    //"I" Constants
    float Y_i, X_i;

    //"D" Constants
    float X_d, Y_d;
};
PIDVari pidV;

class PID : public PIDVari {
  public: 
    int16_t desiredAngleX = 0;
    int16_t desiredAngleY = 0;

    //"P" Constants
    float X_p = 0;
    float Y_p = 0;

    //"I" Constants
    float Y_i = 0;
    float X_i = 0;

    //"D" Constants
    float X_d = 0;
    float Y_d = 0;

    // PID Array
    const double Gain[3] = {0.09, 0.1, 0.0275};
};
PID pid;


// BMP280 Variables
struct barometer {
  float temperature, pressure, altitude, altitudefinal, altitude2;

  // Altitude at which the chutes will deploy
  const int16_t altsetpoint = 120;

  // Launch Site Altitude in Meters(ASL)
  int launchsite_alt = 0;
};
barometer bmp;


class tvc {
  public:
    // If the TVC mount is moving the wrong way and causing a positive feedback loop then change this to 1
    const int tvcDirection = -1;

    //Offsets for tuning
    const float YOffset = tvcDirection * 117.75;
    const float XOffset = tvcDirection * 131;

    //Position of servos through the startup function
    const float Xstart = tvcDirection * YOffset;
    const float Ystart = tvcDirection * XOffset;

    //The amount the servo moves by in the startup function
    const byte startOffset = 10;

    //Ratio between servo gear and tvc mount
    float XgearRatio = 4.5;
    float YgearRatio = 3.5;

    // Servo frequency - Set Frequency to 0 for standard servos and 1 for blue bird servos
    const uint16_t Frequency[2] = {50, 333};
} servo;


// Defining the servo pins as integers
Servo servoX;
Servo servoY;


class Time {
  public:
    // Defining Time Variables
    double dtseconds, dtmillis, currentTime, currentTime_2, previousTime;

    //Timer settings for dataLogging in Hz
    unsigned long previousLog = 0;
    const long logInterval = 50;

    // Time in millis after launch when burnout can be triggered
    const long burnoutInterval = 750;

    // Delay after burnout when the flight computer will deploy the chutes
    const long burnoutTimeInterval = 1000;

    // Event Timer Variables
    uint64_t liftoffTime, flightTime;
    uint32_t burnoutTime[2];
};
Time time;


// Defining Digital Pins
struct digitalPins {
  const int statusled = 9;
  const int errorled = 1;
  const int pyro[3] = {34, 35, 33};
  const int voltagedivider = 0;
  const int buzzer = 15;
  const int teensyled = 13;
};
digitalPins digital;


//SD CARD Chip Select Pin
const int chipSelect = BUILTIN_SDCARD;


struct volt {
// Voltage divider variables
float DividerIN;
float DividerOUT;
float Dividermap;
const float DividerMultplier = 5.86;
};
volt V;


// The degrees that triggers the abort function
const int abortoffset = 45;

// Liftoff acceleration threshold
uint32_t liftoffThresh = 13;

bool DiscoMode = false;
bool StaticFireMode = false;


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

 // Change the value of tempVariance to make the data smoother or respond faster
  float tempVariance = 1.12184278324081E-05;
  float varProcess4 = 1e-8;
  float PC4 = 0.0;
  float K4 = 0.0;
  float UP4 = 1.0;
  float bmpTempEst = 0.0;
  float bmiTempEst = 0.0;
};
kalman kal; 


enum FlightState {
  CONFIGURATION = 0, 
  CALIBRATION = 1, 
  PAD_IDLE = 2,
  POWERED_FLIGHT = 3,
  MECO = 4,
  APOGEE = 5,
  CHUTE_DEPLOYMENT = 6,
  ABORT = 7,
  VOLTAGE_WARNING = 8,
  DISCO = 9
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
  servoX.write(servo.Xstart);
  servoY.write(servo.Ystart);

  // Starting communication with the onboard BMP280
  bmp280.begin();
  bmp280.startNormalConversion();

  // Setting the servo frequency
  analogWriteFrequency(2, servo.Frequency[1]);
  analogWriteFrequency(3, servo.Frequency[1]);
  
  // Setting all of the digital pins to output
  pinMode(digital.buzzer, OUTPUT);
  pinMode(digital.errorled, OUTPUT);
  pinMode(digital.statusled, OUTPUT);
  pinMode(digital.teensyled, OUTPUT);

  for (int i; i < 4; i++) {
    pinMode(digital.pyro[i], OUTPUT);
  }

  flightState = CONFIGURATION;
  startupSequence();
  sdstart();
  sdSettings();
  launchpoll();
  flightState = PAD_IDLE;
}

void loop() {
  //Defining Time Variables
  time.currentTime = millis();
  time.currentTime_2 = millis();
  time.dtmillis = (time.currentTime - time.previousTime);
  time.dtseconds = (time.currentTime - time.previousTime) / 1000;

  // Get measurements from the BMP280
  if (bmp280.getMeasurements(bmp.temperature, bmp.pressure, bmp.altitude)) {
    discoMode();
    inflightTimer();
    altitudeOffset();
    launchdetect();
    sensordata();
    sdwrite();
    if (StaticFireMode == false) {
    burnout();
    abortsystem();
    
  } if (StaticFireMode ==  true) {
      liftoffThresh = 0;
    }
    // Setting the previous time to the current time
    time.previousTime = time.currentTime;
  }
}

void sensordata () {
  if (flightState == PAD_IDLE || flightState == POWERED_FLIGHT || flightState == MECO || flightState == APOGEE || flightState == CHUTE_DEPLOYMENT) {
    switch (flightState) {
      case PAD_IDLE:
        accZKalman(kal.accEst);
        tempKalman(kal.bmpTempEst, kal.bmiTempEst);
        voltage();

      case POWERED_FLIGHT:
        altKalman(kal.altEst);
    }
  }
}
void rotationmatrices () {
  //Change Variable so its easier to refrence later on
  local.GyroRawX = (gyro.getGyroY_rads());
  local.GyroRawY = (gyro.getGyroZ_rads());
  local.GyroRawZ = (gyro.getGyroX_rads());

  //Integrate over time to get Local Orientation
  local.Ax += local.GyroRawX * time.dtseconds;
  local.Ay += local.GyroRawY * time.dtseconds;
  local.Az += local.GyroRawZ * time.dtseconds;

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

  Ore[0] = vector.OrientationX;
  Ore[1] = vector.OrientationY;
  Ore[2] = vector.OrientationZ;

  //X Matrices
  matrix[0] = (cos(local.DifferenceGyroZ) * cos(local.DifferenceGyroY));
  matrix[1] = (((sin(local.DifferenceGyroZ) * -1) * cos(local.DifferenceGyroX) + (cos(local.DifferenceGyroZ)) * sin(local.DifferenceGyroY) * sin(local.DifferenceGyroX)));
  matrix[2] = ((sin(local.DifferenceGyroZ) * sin(local.DifferenceGyroX) + (cos(local.DifferenceGyroZ)) * sin(local.DifferenceGyroY) * cos(local.DifferenceGyroX)));

  //Y Matrices
  matrix[3] = sin(local.DifferenceGyroZ) * cos(local.DifferenceGyroY);
  matrix[4] = ((cos(local.DifferenceGyroZ) * cos(local.DifferenceGyroX) + (sin(local.DifferenceGyroZ)) * sin(local.DifferenceGyroY) * sin(local.DifferenceGyroX)));
  matrix[5] = (((cos(local.DifferenceGyroZ) * -1) * sin(local.DifferenceGyroX) + (sin(local.DifferenceGyroZ)) * sin(local.DifferenceGyroY) * cos(local.DifferenceGyroX)));

  //Z Matrices
  matrix[6] = (sin(local.DifferenceGyroY)) * -1;
  matrix[7] = cos(local.DifferenceGyroY) * sin(local.DifferenceGyroX);
  matrix[8] = cos(local.DifferenceGyroY) * cos(local.DifferenceGyroX);

  vector.OrientationX = ((Ore[0] * matrix[0])) + ((Ore[1] * matrix[1])) + ((Ore[2] * matrix[2]));
  vector.OrientationY = ((Ore[0] * matrix[3])) + ((Ore[1] * matrix[4])) + ((Ore[2] * matrix[5]));
  vector.OrientationZ = ((Ore[0] * matrix[6])) + ((Ore[1] * matrix[7])) + ((Ore[2] * matrix[8]));

  // Convert to euler angles
  global.AxRAD = asin(vector.OrientationX);
  global.AyRAD = asin(vector.OrientationY);

  //Convert from radians to degrees
  global.Ax = global.AxRAD * (-180 / PI);
  global.Ay = global.AyRAD * (180 / PI);

  pidcompute();
}

void pidcompute () {
  pid.previous_errorX = pid.errorX;
  pid.previous_errorY = pid.errorY;

  pid.errorX = global.Ax - pid.desiredAngleX;
  pid.errorY = global.Ay - pid.desiredAngleY;

  // Defining "P"
  pid.X_p = pid.Gain[0] * pid.errorX;
  pid.Y_p = pid.Gain[0] * pid.errorY;

  // Defining "I"
  pid.X_i = pid.Gain[1] * (pid.X_i + pid.errorX * time.dtseconds);
  pid.Y_i = pid.Gain[1] * (pid.Y_i + pid.errorY * time.dtseconds);

  // Defining "D"
  pid.X_d = pid.Gain[2] * ((pid.errorX - pid.previous_errorX) / time.dtseconds);
  pid.Y_d = pid.Gain[2] * ((pid.errorY - pid.previous_errorY) / time.dtseconds);  

  // Adding it all up
  pid.X = pid.X_p + pid.X_i + pid.X_d;
  pid.Y = pid.Y_p + pid.Y_i + pid.Y_d;

  pid.pwmY = servo.tvcDirection * ((pid.Y * servo.XgearRatio) + servo.XOffset);
  pid.pwmX = servo.tvcDirection * ((pid.X * servo.YgearRatio) + servo.YOffset);
  
  //Servo outputs
  servoX.write(pid.pwmX);
  servoY.write(pid.pwmY);
}

void startupSequence () {
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
  servoSequence();
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
  if ((flightState == PAD_IDLE) && accel.getAccelX_mss() > liftoffThresh) {
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
  settingstring += (time.logInterval);
  settingstring += ", ";

  settingstring += "Burnout Time Delay: ";
  settingstring += (time.burnoutTimeInterval);
  settingstring += ", ";

  settingstring += "Time after launch when burnout is activated: ";
  settingstring += (time.burnoutInterval);
  settingstring += ", ";

  settingstring += "Voltage Multiplier: ";
  settingstring += (V.DividerMultplier);
  settingstring += ", ";

  settingstring += "Deployment Altitude: ";
  settingstring += (bmp.altsetpoint);
  settingstring += ", ";

  settingstring += "Kp: ";
  settingstring += (pid.Gain[0]);
  settingstring += ", ";

  settingstring += "Ki: ";
  settingstring += (pid.Gain[1]);
  settingstring += ", ";

  settingstring += "Kd: ";
  settingstring += (pid.Gain[2]);
  settingstring += ", ";

  settingstring += "Altitude Variance: ";
  settingstring += (kal.altVariance);
  settingstring += ", ";

  settingstring += "Accel Variance: ";
  settingstring += (kal.accVariance);
  settingstring += ", ";

  settingstring += "Temp Variance: ";
  settingstring += (kal.tempVariance);
  settingstring += ", ";

  settingstring += "Voltage Variance: ";
  settingstring += (kal.voltVariance);
  settingstring += ", ";

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
  datastring += String(V.DividerOUT);
  datastring += ",";

  datastring += "Filtered_Voltage,";
  datastring += String(kal.voltageEst);
  datastring += ",";

  datastring += "Raw_IMU_Temp,";
  datastring += String(accel.getTemperature_C());
  datastring += ",";

  datastring += "Filtered_IMU_Temp,";
  datastring += String(kal.bmiTempEst);
  datastring += ",";

  datastring += "Raw_Barometer_Temp,";
  datastring += String(bmp.temperature);
  datastring += ",";

  datastring += "Filtered_IMU_Temp,";
  datastring += String(kal.bmpTempEst);
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
    if (time.currentTime - time.previousLog > time.logInterval) {
      time.previousLog = time.currentTime;
      omegaFile.println(datastring);
      omegaFile.close();
    }
  }
}

void burnout () {
  if ((flightState == POWERED_FLIGHT) && kal.accEst <= 2 && (time.flightTime > time.burnoutInterval)) {
    //Burnout Detected; changing the system state to state 2
    flightState = MECO;
    digitalWrite(digital.teensyled, LOW); 
    LED.Color(green);
    Serial.println("Burnout Detected");
  }

  if ((flightState == MECO) && time.burnoutTime[0] > time.burnoutTimeInterval) {
    //Apogee Detected; changing the system state to state 3
    flightState = APOGEE;
    LED.off();
    Serial.println("Apogee Detected");
    tone(digital.buzzer, 1200, 200);
  }

  if ((flightState == APOGEE || flightState == CHUTE_DEPLOYMENT) && (kal.altEst - bmp.launchsite_alt) <= bmp.altsetpoint) {
    // Chute deployment; changing the system state to state 4
    flightState = CHUTE_DEPLOYMENT;
    digitalWrite(digital.pyro[0], HIGH);
    digitalWrite(digital.pyro[1], HIGH);
    digitalWrite(digital.pyro[2], HIGH);
    LED.Color(red);
  }
}

void launchpoll () {
  delay(750);
  int status;
  switch (flightState) {
    case CONFIGURATION:
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

    flightState = CALIBRATION;
    case CALIBRATION:
      calibrateGyroscopes(accel.getAccelY_mss(), accel.getAccelZ_mss(), accel.getAccelX_mss());
      Serial.println("Gyroscopes have been calibrated.");
      LED.Color(yellow);  
  }
}

void abortsystem () {
  if ((flightState == POWERED_FLIGHT) && (global.Ax > abortoffset || global.Ax < -abortoffset) || (global.Ay > abortoffset || global.Ay < -abortoffset)) {
    // Changing the system state to 5
    flightState = ABORT;

    // Firing the pyrotechnic channel
    digitalWrite(digital.pyro[0], HIGH);
    digitalWrite(digital.pyro[1], HIGH);
    digitalWrite(digital.pyro[2], HIGH);

    Serial.println("Abort Detected.");
    LED.Color(purple);
    digitalWrite(digital.teensyled, LOW);

    tone(digital.buzzer, 1200, 400);
  }
}

void voltage () {
  // Reading the voltage from the analog pin
  V.DividerIN = analogRead(digital.voltagedivider);

  // Map the voltage to 3.3V
  V.Dividermap = map(V.DividerIN, 0, 2048, 0, 3.3);

  // Multiply the output from the mapping function to get the true voltage
  V.DividerOUT = V.Dividermap * V.DividerMultplier;

  // Calls the function to filter the voltage data
  voltageKalman(kal.voltageEst);

}

void inflightTimer () {
  //Timer to wait a certain time after launch before burnout is activated
  if (flightState == PAD_IDLE) {
    time.liftoffTime = time.currentTime;
  }
  time.flightTime = time.currentTime - time.liftoffTime;

  //Timer that waits a certain time after burnout to deploy the chutes
  if (flightState == PAD_IDLE || flightState == POWERED_FLIGHT) {
    time.burnoutTime[1] = time.currentTime_2;
  }
  time.burnoutTime[0] = time.currentTime_2 - time.burnoutTime[1];
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

void altKalman (double Xp) {
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

void voltageKalman (double Xp3) {
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
  kal.voltageEst = kal.K3 * (V.DividerOUT - Zp3) + Xp3;  

  voltageWarning();
}

void tempKalman (double Xp4, double Xp5) {
    // Predict the next covariance
  kal.PC4 = kal.UP4 + kal.varProcess4;

  // Compute the kalman gain
  kal.K4 = kal.PC4 / (kal.PC4 + kal.tempVariance);

  // Update the covariance
  kal.UP4 = (1 - kal.K4) * kal.PC4;

  // Re-define variables
  Xp4 = kal.bmiTempEst;
  Xp5 = kal.bmiTempEst;
  float Zp4 = Xp4;
  float Zp5 = Xp5;

  // Final voltage estimation
  kal.bmpTempEst = kal.K4 * (V.DividerOUT - Zp4) + Xp4;
  kal.bmiTempEst = kal.K4 * (V.DividerOUT - Zp5) + Xp5;
}

void discoMode () {
  if (DiscoMode == true) {
    flightState = DISCO;
    LED.Color(white);
    tone(digital.buzzer, 1100);
    delay(300);
    LED.Color(purple);
    noTone(digital.buzzer);
    delay(300);

    LED.Color(yellow);
    tone(digital.buzzer, 1100);
    delay(300);
    LED.Color(blue);
    noTone(digital.buzzer);
    delay(300);

    LED.Color(red);
    tone(digital.buzzer, 1100);
    delay(300);
    LED.Color(green);
    noTone(digital.buzzer);
    delay(300);
  }
}

void servoSequence () {
  servoX.write(servo.Xstart + servo.startOffset);
  delay(400);
  servoX.write(servo.Xstart - servo.startOffset);
  delay(200);
  servoX.write(servo.Xstart);
  delay(400);
  servoY.write(servo.Ystart + servo.startOffset);
  delay(400);
  servoY.write(servo.Ystart - servo.startOffset);
  delay(200);
  servoY.write(servo.Ystart);
}