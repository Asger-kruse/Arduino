

/************************************************************************

  Test of Pmod NAV (Based on Jim Lindblom's program)

*************************************************************************

  Description: Pmod_NAV
  All data (accelerometer, gyroscope, magnetometer) are displayed
  In the serial monitor

  Material
  1. Arduino Uno
  2. Pmod NAV (dowload library
  https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library )
  Licence Beerware

  Wiring
  Module<--------------------> Arduino
  J1 pin 6 (3V3)     to        3V3
  J1 pin 5 (GND)     to        GND
  J1 pin 4 (SCK)     to        A5 (SCL)
  J1 pin 2 (SDI)     to        A4 (SDA)

************************************************************************/

// The earth's magnetic field varies according to its location.
// Add or subtract a constant to get the right value
// of the magnetic field using the following site
// http://www.ngdc.noaa.gov/geomag-web/#declination

#define DECLINATION -5.37 // declination (in degrees) in Cluj-Napoca (Romania).

/************************************************************************/

#define PRINT_CALCULATED  //print calculated values
//#define PRINT_RAW       //print raw data


// Call of libraries
#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include <math.h>
#include <Servo.h>
#include <SD.h>
#include <SimpleTimer.h>

// defining module addresses
#define LSM9DS1_M 0x1E  //magnetometer
#define LSM9DS1_AG 0x6B //accelerometer and gyroscope

const float StartRoll = 0;
const float startPitch = 0;

const int buzzerPIN = 3; 
const int servoPIN = 5;
const int buttonPIN = 2;

int Press = 0;
int readingNr = 0;
int ONEplay = 0;

float gForce; 
float Pitch; 
float Roll;
float Heading;

bool buttonPressed = false;
bool RUNNING = true;
bool firstRun = true;
bool landing = false;

Servo Lock;
unsigned long myTime;

LSM9DS1 imu; // Creation of the object
File myFile;

void setup(void)
{
  Lock.attach(servoPIN);
  Lock.write(50); 
  Serial.begin(115200); // initialization of serial communication
  Wire.begin();     //initialization of the I2C communication
  imu.settings.device.commInterface = IMU_MODE_I2C; // initialization of the module
  imu.settings.device.mAddress = LSM9DS1_M;        //setting up addresses
  imu.settings.device.agAddress = LSM9DS1_AG;

  pinMode(buzzerPIN, OUTPUT);
  pinMode(servoPIN, OUTPUT);
  pinMode(buttonPIN, INPUT);
  
  
  if (!imu.begin()) //display error message if that's the case
  {
    Serial.println("Communication problem.");
    while (1);
  }
  if(firstRun)
  {
    firstRun = false;
    waitButton();
  }
  else 
  {
    loop();
  }
}

void waitButton()
{
   tone(buzzerPIN, 1000);
   delay(1000);
   noTone(buzzerPIN);

   buttonPressed = false;
   Press = 0;
   
   while(buttonPressed != true){
    
    Press = digitalRead(buttonPIN);

    if(Press == HIGH)
    {
      Serial.println("PROGRAM INITIATED");
      buttonPressed = true;
      delay(200);
    }
  }
}

  
void loop()
{
  //measure
  if ( imu.gyroAvailable() )
  {
    imu.readGyro(); //measure with the gyroscope 
  }
  
  if ( imu.accelAvailable() )
  {
    imu.readAccel(); //measure with the accelerometer
  }
  if ( imu.magAvailable() )
  {
    imu.readMag(); //measure with the magnetometer
  }

  //display data
  calc(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz, true);

  Serial.println("");

  if (gForce < 0.2 & landing != true)
  {
    Lock.write(95);
  } 
  else if(sqrt(pow(Pitch, 2)) >  60 & landing != true)
  {
    Lock.write(95);
    landing = true;
  }
  else if(sqrt(pow(Roll, 2)) > 60 & landing != true)
  {
    Lock.write(95);
    landing = true;
  }
  
  if(sqrt(pow(Pitch, 2)) >  60 & landing == true)
  {
    Press = digitalRead(buttonPIN);
    tone(buzzerPIN, 2000);
    
  }else if(sqrt(pow(Roll, 2)) > 60 & landing == true)
  {
    Press = digitalRead(buttonPIN);
    tone(buzzerPIN, 2000);
  }else {noTone(buzzerPIN);}

  if(Press == HIGH)
  {
    landing = false;
  }
  
   delay(100);
}

void calc(float ax, float ay, float az, float mx, float my, float mz, bool print)
{
  

float gX = imu.calcGyro(imu.gx);
float gY = imu.calcGyro(imu.gy);
float gZ = imu.calcGyro(imu.gz);

//Getting accelarometer data
float aX = imu.calcAccel(imu.ax);
float aY = imu.calcAccel(imu.ay);
float aZ = imu.calcAccel(imu.az);
gForce =  sqrt(pow(gX, 2) + pow(gY, 2) + pow(gZ, 2));


//Getting  magnetic data
float mX = imu.calcMag(imu.mx);
float mY = imu.calcMag(imu.my);
float mZ = imu.calcMag(imu.mz);

  float roll = atan2(ay, az); //calculate roll
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));  //calculate pitch
  float heading;  //variable for hading

  //calculate heading
  if (my == 0) {
    heading = (mx < 0) ? PI : 0;
  }
  else {
    heading = atan2(mx, my);
  }

  //correct heading according to declination
  heading -= DECLINATION * PI / 180;
  if (heading > PI) {
    heading -= (2 * PI);
  }
  else if (heading < -PI) {
    heading += (2 * PI);
  }
  else if (heading < 0) {
    heading += 2 * PI;
  }

  //convert values in degree
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;

  Pitch = pitch;
  Roll = roll;
  Heading = heading;

  //print data to serial port 
  if(print)
  {
    Serial.println("Gyro: " + String(gX)+", " + String(gY)+", " + String(gZ));
    Serial.println("Accel: " + String(aX)+", " + String(aY)+", " + String(aZ)+ " Total G-Force: " + String(gForce));
    Serial.println("magn: " + String(mX)+", " + String(mY)+", " + String(mZ));
    Serial.println("Pitch, Roll, Heading: " + String(Pitch)+", " + String(Roll)+", " + String(Heading));
    Serial.println(" ");
    Serial.println("-----------------------------------------------------------------");
    Serial.println(" ");
  }

  readingNr++;

  if(readingNr == 1){write_to_file("#,Tid,gX,gY,gZ,aX,aY,aZ,G,mX,mY,mZ,Pitch,Roll,Heading");}
  
  String test = String(readingNr)+","+String(myTime)
                +","+String(gX)+","+String(gY)+","+String(gZ)
                +","+String(aX)+","+String(aY)+","+String(aZ)+","+String(gForce)
                +","+String(mX)+","+String(mY)+","+String(mZ)
                +","+String(pitch)+","+String(roll)+","+String(heading)+";";

Serial.println(test);
write_to_file(test);
delay(250); //Normal 1000ms
}

void PrepLanding()
{

  
  Serial.println("LANDING INITIATED");
  
  while(RUNNING)
  {
  
    if(sqrt(pow(Pitch, 2)) >  60)
    {
      tone(buzzerPIN, 3000);
    }
    else if(sqrt(pow(Roll, 2)) > 60)
    {
      tone(buzzerPIN, 3000);
    }
    else 
    {
      noTone(buzzerPIN);
    }
    
    Press = digitalRead(buttonPIN);
    
    if(Press == HIGH)
    {
      Serial.println("PROGRAM TERMINATED");
      STANDBY();
    }
  }
}

void write_to_file(String data) 
{
  if (!SD.begin(10)) { // Initialize the sd card 
    Serial.println("SD Failed to initialize");
    while (1);
  }

  myFile = SD.open("data.txt", FILE_WRITE); // Open the file 
  
  if (!myFile) {
    // if the file didn't open, print an error:
    Serial.println("error opening data.txt");
    while (1);
  }

  myFile.println(data);
  myFile.close();
  //Serial.println("Done.");
}

void STANDBY()
{
  while (true)
  {
    Serial.println("");
  }
  
}
