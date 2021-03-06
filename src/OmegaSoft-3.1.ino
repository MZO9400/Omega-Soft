/* Delta Space Systems
   Version 3.1 BETA 5
  December, 16th 2020 */

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

/* ------------------------------- Libraries ------------------------------ */
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

/* -------------------------------------------------------------------------- */
/*                           Rocket Characteristics                           */
/* -------------------------------------------------------------------------- */

struct rocketChar {
  String Name = "Frontier";
  double motorThrust = Mass * accel.getAccelX_mss();
  float avgThrust = 20;

  private:
    // Rocket mass in g
    float Mass = 700; 
};
rocketChar rocket;

/* -------------------------------------------------------------------------- */
/*                            Orientation Variables                           */
/* -------------------------------------------------------------------------- */

struct localOri {
  // Orientation Axis'
  double Qw[4];

  // Local Angular Velocity(Radians)
  double GyroRawX, GyroRawY, GyroRawZ;
};
localOri local;

struct globalOri {
  // Global Orientation in radians
  double AxRAD, AyRAD, AzRAD;

  // Main Integration
  double Quat_ori[4] = {1, 0, 0, 0};

  //Euler Integration
  double Quat_dot[4];

  // Global Orientation in degrees
  double Ax, Ay, Az;

  double Ax2, Ay2, Az2;
};
globalOri global;

struct oriVector {
  double oriquat[4];
};
oriVector vector;

struct gyroCalibration {
  double Ax, Ay, Az;
};
gyroCalibration gyroCal;

double newIntegrator1;
double newIntegrator2;
double newIntegrator3;
double newIntegrator4;


/* -------------------------------------------------------------------------- */
/*                                PID Variables                               */
/* -------------------------------------------------------------------------- */

struct PIDVari {
  // PID Controller Output
  double X, Y;

  // Error of the flight computer angle
  double errorX, errorY, previous_errorX, previous_errorY;

  // PWM that gets sent to the servos
  double pwmX, pwmY;

  // Upright Angle of the Flight Computer
  int16_t desiredAngleX;
  int16_t desiredAngleY;
  
  protected:
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
    double Gain[2] = {0.10, 0.03};
    double Integrator[2];
};
PID pid;

/* -------------------------------------------------------------------------- */
/*                              BMP280 Variables                              */
/* -------------------------------------------------------------------------- */

struct barometer {
  float temperature, pressure, altitude, altitudefinal, altitude2;

  // Altitude at which the chutes will deploy
  const int16_t altsetpoint = 75;

  // Launch Site Altitude in Meters(ASL)
  int launchsite_alt = 0;
};
barometer bmp2;

/* -------------------------------------------------------------------------- */
/*                            Servo & TVC Variables                           */
/* -------------------------------------------------------------------------- */

class tvc {
  public:
    // If the TVC mount is moving the wrong way and causing a positive feedback loop then change this to 1
    const int tvcDirection = 1;

    void setOffsetY(int Yoff) {
    YOff = Yoff;
    }

    void setOffsetX(int Xoff) {
    XOff = Xoff;
    }
    float servoOffX = 119;
    float servoOffY = 110.5;

    //Position of servos through the startup function
    const float Xstart = tvcDirection * (tvcDirection * servoOffY);
    const float Ystart = tvcDirection * (tvcDirection * servoOffX);

    //The amount the servo moves by in the startup function
    const byte startOffset = 5;

    //Ratio between servo gear and tvc mount
    float XgearRatio = 3.5; 
    float YgearRatio = 1.7;

    // Servo frequency - Set Frequency to 0 for standard servos and 1 for blue bird servos
    const uint16_t Frequency[2] = {50, 333};

    protected:
      float XOff;
      float YOff;
}; 
tvc servo;

class servoOffsets : public tvc {
  public:
    int getOffsetX () {
      const float XOffset = tvcDirection * XOff; // 131
      return XOffset;
    }
    int getOffsetY () {
      const float YOffset = tvcDirection * YOff; // 117.75
      return YOffset;
  }
};
servoOffsets servoOffset;

// Defining the servo pins as integers
Servo servoX;
Servo servoY;

/* -------------------------------------------------------------------------- */
/*                               Time Variables                               */
/* -------------------------------------------------------------------------- */

class Time {
  public:
    // Defining Time Variables
    double dtseconds, dtmillis, currentTime, currentTime_2, previousTime;

    //Timer settings for dataLogging in Hz
    unsigned long previousLog = 0;
    const long logInterval = 10;

    //Timer settings for the integrator in Hz
    unsigned long previousInt = 0;
    const long Interval = 20;

    // Time in millis after launch when burnout can be triggered
    const long burnoutInterval = 1000;

    // Delay after burnout when the flight computer will deploy the chutes
    const long burnoutTimeInterval = 1000;

    // Event Timer Variables
    uint32_t liftoffTime, flightTime;
    uint32_t burnoutTime[2];
};
Time time;

// Defining Digital Pins
struct digitalPins {
  const int statusled = 9;
  const int errorled = 1;
  const int voltagedivider = 0;
  const int buzzer = 15;
  const int teensyled = 13;
  const int pyro[3] = {34, 35, 33};
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
const int abortoffset = 60;

// Liftoff acceleration threshold
uint32_t liftoffThresh = 13;

// Vertical Velocity variable
double verticalVel;

struct Event {
  const bool DiscoMode = false;
  const bool StaticFireMode = false;
};
Event event;

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

/* -------------------------------------------------------------------------- */
/*                           Kalman Filter Variables                          */
/* -------------------------------------------------------------------------- */

struct kalman {
  // Change the value of accVariance to make the data smoother or respond faster
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
  float voltVariance = 1.12184278324081E-07; 
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

/* -------------------------------------------------------------------------- */
/*                                Flight States                               */
/* -------------------------------------------------------------------------- */

enum FlightState {
  CONFIGURATION = 0, 
  PAD_IDLE = 1,
  POWERED_FLIGHT = 2,
  MECO = 3,
  APOGEE = 4,
  CHUTE_DEPLOYMENT = 5,
  ABORT = 6,
  VOLTAGE_WARNING = 7,
  DISCO = 8
};
FlightState flightState = PAD_IDLE;

enum integralScaler {
  START = 1,
  END = 2
};
integralScaler Integralscaler = START;


void setup() {
  // Starting serial communication with your computer
  Serial.begin(9600);

  // Starting communication with the I2C bus
  Wire.begin();

  // Starting communication with the onboard BMP280
  bmp280.begin();
  bmp280.startNormalConversion();
  
  // Setting all of the digital pins to output
  pinMode(digital.buzzer, OUTPUT);
  pinMode(digital.errorled, OUTPUT);
  pinMode(digital.statusled, OUTPUT);
  pinMode(digital.teensyled, OUTPUT);

  servoX.attach(2);
  servoY.attach(3);

  // Setting the servo frequency
  analogWriteFrequency(2, servo.Frequency[1]);
  analogWriteFrequency(3, servo.Frequency[1]);

  // Setting the servo offset
  servoOffset.setOffsetX(servo.servoOffX);
  servoOffset.setOffsetY(servo.servoOffY);

  // Writing servos to their start posistions
  servoX.write(servo.Xstart);
  servoY.write(servo.Ystart);

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

  local.GyroRawX = gyro.getGyroX_rads();
  local.GyroRawY = gyro.getGyroY_rads();
  local.GyroRawZ = gyro.getGyroZ_rads();



  // Get measurements from the BMP280
  if (bmp280.getMeasurements(bmp2.temperature, bmp2.pressure, bmp2.altitude)) {
    inflightTimer();
    altitudeOffset();
    launchdetect();
    sensordata();
    sdwrite();
    integralScale();

    if (flightState == PAD_IDLE || flightState == POWERED_FLIGHT) {
      gyroOffset();

  } if (event.StaticFireMode == false) {
    burnout();
    abortsystem();
   
  } if (event.StaticFireMode ==  true) {
    liftoffThresh = 0;
  
  } if (event.DiscoMode ==  true) {
    discoMode();
  }

    // Setting the previous time to the current time
    time.previousTime = time.currentTime;
  }
}

void sensordata () {
  switch (flightState) {
    case POWERED_FLIGHT:
      verticalVel += accel.getAccelZ_mss() * time.dtseconds; 
  }   
  voltage(); 
  accZKalman(kal.accEst);
  altKalman(kal.altEst);  
  tempKalman(kal.bmpTempEst, kal.bmiTempEst);  
}

void quaternion () {
  
  // Setting the orientation axis'
  local.Qw[0] = 0; 
  local.Qw[1] = gyroCal.Ay; //Ay
  local.Qw[2] = gyroCal.Az; //Az
  local.Qw[3] = gyroCal.Ax; //Ax

  // Euler Integration
  global.Quat_dot[0] = (-0.5 * global.Quat_ori[1] * local.Qw[1] - 0.5 * global.Quat_ori[2] * local.Qw[2] - 0.5 * global.Quat_ori[3] * local.Qw[3]);
  global.Quat_dot[1] = (0.5 * global.Quat_ori[0] * local.Qw[1] + 0.5 * global.Quat_ori[2] * local.Qw[3] - 0.5 * global.Quat_ori[3] * local.Qw[2]);
  global.Quat_dot[2] = (0.5 * global.Quat_ori[0] * local.Qw[2] - 0.5 * global.Quat_ori[1] * local.Qw[3] + 0.5 * global.Quat_ori[3] * local.Qw[1]);
  global.Quat_dot[3] = (0.5 * global.Quat_ori[0] * local.Qw[3] + 0.5 * global.Quat_ori[1] * local.Qw[2] - 0.5 * global.Quat_ori[2] * local.Qw[1]);

  // Main Integration
  global.Quat_ori[0] = global.Quat_ori[0] + global.Quat_dot[0] * time.dtseconds;
  global.Quat_ori[1] = global.Quat_ori[1] + global.Quat_dot[1] * time.dtseconds;
  global.Quat_ori[2] = global.Quat_ori[2] + global.Quat_dot[2] * time.dtseconds;
  global.Quat_ori[3] = global.Quat_ori[3] + global.Quat_dot[3] * time.dtseconds;
  
  double quatNorm = sqrt(global.Quat_ori[0] * global.Quat_ori[0] + global.Quat_ori[1] * global.Quat_ori[1] + global.Quat_ori[2] * global.Quat_ori[2] + global.Quat_ori[3] * global.Quat_ori[3]);
  
  // Normalizing the orientation quaternion
  global.Quat_ori[0] = global.Quat_ori[0] / quatNorm;
  global.Quat_ori[1] = global.Quat_ori[1] / quatNorm;
  global.Quat_ori[2] = global.Quat_ori[2] / quatNorm;
  global.Quat_ori[3] = global.Quat_ori[3] / quatNorm;

  // Converting from quaternion to euler angles through a rotation matrix
  global.AxRAD = atan((2 * (global.Quat_ori[0] * global.Quat_ori[1] + global.Quat_ori[2] * global.Quat_ori[3])) / (1 - 2 * (sq(global.Quat_ori[1]) + sq(global.Quat_ori[2]))));
  global.AyRAD = atan(2 * (global.Quat_ori[0] * global.Quat_ori[2] - global.Quat_ori[3] * global.Quat_ori[1]));
  global.AzRAD = atan((2 * (global.Quat_ori[0] * global.Quat_ori[3] + global.Quat_ori[1] * global.Quat_ori[2])) / ( 1 - 2 * (sq(global.Quat_ori[2]) + sq(global.Quat_ori[3]))));

  global.Ax = global.AxRAD * (180 / PI);
  global.Ay = ((global.AyRAD * (180 / PI)) * 2);
  global.Az = global.AzRAD * (180 / PI); 

  pidcompute();
}

void pidcompute () {
  pid.previous_errorX = pid.errorX;
  pid.previous_errorY = pid.errorY;

  pid.errorX = global.Ay - pid.desiredAngleX;
  pid.errorY = global.Ax - pid.desiredAngleY;

  if (global.Ax >= 0.001) {
    pid.Integrator[1] = newIntegrator1;
  }
  if (global.Ax <= -0.001) {
    pid.Integrator[1] = newIntegrator2;
  }

  if (global.Ay >= 0.001) {
    pid.Integrator[2] = newIntegrator3;
  }
  if (global.Ay <= -0.001) {
    pid.Integrator[2] = newIntegrator4;
  }

  // Defining "P"
  pid.X_p = pid.Gain[0] * pid.errorX;
  pid.Y_p = pid.Gain[0] * pid.errorY;

  // Defining "I"
  pid.X_i = pid.Integrator[1] * (pid.X_i + pid.errorX * time.dtseconds);
  pid.Y_i = pid.Integrator[2] * (pid.Y_i + pid.errorY * time.dtseconds);

  // Defining "D"
  pid.X_d = pid.Gain[1] * ((pid.errorX - pid.previous_errorX) / time.dtseconds);
  pid.Y_d = pid.Gain[1] * ((pid.errorY - pid.previous_errorY) / time.dtseconds);  

  // Adding it all up
  pid.X = pid.X_p + pid.X_i + pid.X_d;
  pid.Y = pid.Y_p + pid.Y_i + pid.Y_d;

  pid.pwmY = servo.tvcDirection * ((pid.Y * servo.XgearRatio) + servoOffset.getOffsetX());
  pid.pwmX = servo.tvcDirection * ((pid.X * servo.YgearRatio) + servoOffset.getOffsetY());
  
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
    quaternion();
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
  settingstring += (bmp2.altsetpoint);
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
  datastring += String(global.Az);
  datastring += ",";

  datastring += "SystemState,";
  datastring += String(flightState);
  datastring += ",";

  datastring += "RawZAxisAccel,";
  datastring += String(accel.getAccelZ_mss());
  datastring += ",";

  datastring += "FilteredZAxisAccel,";
  datastring += String(kal.accEst);
  datastring += ",";

  datastring += "ServoXPOS,";
  datastring += String((servoX.read() - servo.servoOffY) / servo.YgearRatio);
  datastring += ",";

  datastring += "ServoYPOS,";
  datastring += String((servoY.read() - servo.servoOffX) / servo.XgearRatio);
  datastring += ",";

  datastring += "RawVoltage,";
  datastring += String(V.DividerOUT);
  datastring += ",";

  datastring += "FilteredVoltage,";
  datastring += String(kal.voltageEst);
  datastring += ",";

  datastring += "RawIMUTemp,";
  datastring += String(accel.getTemperature_C());
  datastring += ",";

  datastring += "FilteredIMUTemp,";
  datastring += String(kal.bmiTempEst);
  datastring += ",";

  datastring += "RawBarometerTemp,";
  datastring += String(bmp2.temperature);
  datastring += ",";

  datastring += "FilteredIMUTemp,";
  datastring += String(kal.bmpTempEst);
  datastring += ",";

  datastring += "FilteredAltitude,";
  datastring += String(kal.altEst);
  datastring += ",";

  datastring += "RawAltitude,";
  datastring += String(bmp2.altitudefinal);
  datastring += ",";

  datastring += "Pressure,";
  datastring += String(bmp2.pressure);
  datastring += ",";

  datastring += "VerticalVelocity,";
  datastring += String(verticalVel);
  datastring += ",";

  datastring += "MotorThrust,";
  datastring += String(rocket.motorThrust);
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
  switch (flightState) {
    case MECO:
      apogee();
  }
}

void apogee () {
  if ((flightState == MECO) && time.burnoutTime[0] > time.burnoutTimeInterval) {
    //Apogee Detected; changing the system state to state 3
    flightState = APOGEE;
    LED.off();
    Serial.println("Apogee Detected");
    tone(digital.buzzer, 1200, 200);
  }
  switch (flightState) {
    case APOGEE:
      chuteDeployment();
  }
}

void chuteDeployment () {
  if ((flightState == APOGEE || flightState == CHUTE_DEPLOYMENT) && (kal.altEst - bmp2.launchsite_alt) <= bmp2.altsetpoint) {
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
    LED.Color(yellow);  
    Serial.print("Go");
    Serial.println(rocket.Name);
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
    bmp2.altitude2 = bmp2.altitude;
  }
  bmp2.altitudefinal = bmp2.altitude - bmp2.altitude2;
}

void gyroOffset () {
  if (flightState == PAD_IDLE) {
    local.GyroRawX = global.Ax2;
    local.GyroRawY = global.Ay2;
    local.GyroRawZ = global.Az2;
  }
  gyroCal.Ax = local.GyroRawX - global.Ax2;
  gyroCal.Ay = local.GyroRawY - global.Ay2;
  gyroCal.Az = local.GyroRawZ - global.Az2;
}

void voltageWarning () {
  // If the system voltage is less than 7.6
  if (V.DividerOUT <= 7.2) {
    flightState = VOLTAGE_WARNING;
    digitalWrite(digital.teensyled, HIGH);
    tone(digital.buzzer, 1200);
    delay(400);
    digitalWrite(digital.teensyled, LOW);
    noTone(digital.buzzer);
    delay(400);
  }
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
  kal.altEst = kal.K * (bmp2.altitudefinal - Zp) + Xp;   
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
  //voltageWarning();
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
  kal.bmpTempEst = kal.K4 * (bmp2.temperature - Zp4) + Xp4;
  kal.bmiTempEst = kal.K4 * (accel.getTemperature_C() - Zp5) + Xp5;
}

void discoMode () {
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

void integralScale () {
  if (flightState == POWERED_FLIGHT) {
    if (Integralscaler = START) {
      if ((global.Ax >= 20 || global.Ax <= -20) || (global.Ay >= 20 || global.Ay <= -20)) {
        Integralscaler = END;
      }

    if (Integralscaler = END) {
      if (global.Ax >= 0.01) {
          newIntegrator1 = map(global.Ax, 0, 50, 0.1, 0.4);

      } else if (global.Ax <= -0.01){
        newIntegrator2 = map(global.Ax, -50, 0, 0.4, 0.1);
      }

      if (global.Ay >= 0.01) {
        newIntegrator3 = map(global.Ay, 0, 50, 0.1, 0.4);

      } else if (global.Ay <= -0.01){
        newIntegrator4 = map(global.Ay, -50, 0, 0.4, 0.1);
        }
      }
    } 
  }
}



