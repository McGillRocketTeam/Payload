
#include "quaternionFilters.h"
#include "MPU9250.h"
#include <Wire.h>
#include "IntersemaBaro.h"
#include <SD.h>
#include <SPI.h>

#define AHRS false         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

Intersema::BaroPressure_MS5607B baro(true);    //The file named IntersemaBaro.h must be in the same folder as this code file



int solenoid = 27;            // set to the digital pin that the solenoid valve is connected to
int vacuumPump1 = 25;        // set to the digital pins that pumps are connected to
int vacuumPump2 = 26;

int heightVariable = 0;
int pumpVariable = 0;
                              //SCL is pin 19 and SDL is pin 18

unsigned long time;

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

const int chipSelect = BUILTIN_SDCARD;                   //initialization for SD card
  File datalog = SD.open("datalog.txt", FILE_WRITE);    //there must be a file named datalog.txt on the SD card


void setup() {
  // put your setup code here, to run once:
  
Serial.begin(9600);

 Wire.begin();
                                                         // TWBR = 12;  // 400 kbit/sec I2C speed
 
baro.init();
pinMode(solenoid,OUTPUT);
pinMode(vacuumPump1,OUTPUT);
pinMode(vacuumPump2,OUTPUT);
if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
                                                             // if SD card is not present, won't do anything more:
    return;
  }
  Serial.println("card initialized.");




                                                       // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  datalog.print(F("MPU9250 I AM 0x"));
  datalog.print(c, HEX);
  datalog.print(F(" I should be 0x"));
  datalog.println(0x71, HEX);

 if (c == 0x71)                                         // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));
    datalog.println(F("MPU9250 is online..."));

                                                       // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

    datalog.print(F("x-axis self test: acceleration trim within : "));
    datalog.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
    datalog.print(F("y-axis self test: acceleration trim within : "));
    datalog.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
    datalog.print(F("z-axis self test: acceleration trim within : "));
    datalog.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
    datalog.print(F("x-axis self test: gyration trim within : "));
    datalog.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    datalog.print(F("y-axis self test: gyration trim within : "));
    datalog.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    datalog.print(F("z-axis self test: gyration trim within : "));
    datalog.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

                                                      // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
                                                    // Initialize device for active mode read of acclerometer, gyroscope, and
                                                    // temperature
    Serial.println("MPU9250 initialized for active data mode....");
    datalog.println("MPU9250 initialized for active data mode....");

                                                    // Read the WHO_AM_I register of the magnetometer, this is a good test of
                                                   // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    datalog.print("AK8963 ");
    datalog.print("I AM 0x");
    datalog.print(d, HEX);
    datalog.print(" I should be 0x");
    datalog.println(0x48, HEX);


    if (d != 0x48)
    {
                                                     // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      datalog.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

                                                      // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
                                                     // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    datalog.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);

      datalog.print("X-Axis factory sensitivity adjustment value ");
      datalog.println(myIMU.factoryMagCalibration[0], 2);
      datalog.print("Y-Axis factory sensitivity adjustment value ");
      datalog.println(myIMU.factoryMagCalibration[1], 2);
      datalog.print("Z-Axis factory sensitivity adjustment value ");
      datalog.println(myIMU.factoryMagCalibration[2], 2);
    }

                                                  // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

                                                  // The next call delays for 4 seconds, and then records about 15 seconds of
                                                       // data to calculate bias and scale.
    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    datalog.println("AK8963 mag biases (mG)");
    datalog.println(myIMU.magBias[0]);
    datalog.println(myIMU.magBias[1]);
    datalog.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);

    datalog.println("AK8963 mag scale (mG)");
    datalog.println(myIMU.magScale[0]);
    datalog.println(myIMU.magScale[1]);
    datalog.println(myIMU.magScale[2]);
//    delay(2000); // Add delay to see results before serial spew of data

    if(SerialDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);

      datalog.println("Magnetometer:");
      datalog.print("X-Axis sensitivity adjustment value ");
      datalog.println(myIMU.factoryMagCalibration[0], 2);
      datalog.print("Y-Axis sensitivity adjustment value ");
      datalog.println(myIMU.factoryMagCalibration[1], 2);
      datalog.print("Z-Axis sensitivity adjustment value ");
      datalog.println(myIMU.factoryMagCalibration[2], 2);
    }

  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    datalog.print("Could not connect to MPU9250: 0x");
    datalog.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    
    datalog.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
  

}

void loop() {

  time = millis();
  int alt = baro.getHeightCentiMeters();
  int feet = (float)(alt) / 30.48;
  int meters = (float)(alt) / 100;

  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
   myIMU.updateTime();
   MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);


  
  Serial.print(time);           //recording data in serial monitor
 
  Serial.print(", Feet: ");
  Serial.print(feet);
  Serial.print(", Meters: ");
  Serial.print(meters);

  datalog.print(time);        //recording data in SD card
 
  datalog.print(", Feet: ");
  datalog.print(feet);
  datalog.print(" Meters: ");
  datalog.print(meters);

  if (!AHRS)
    {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print(" X-acceleration: "); Serial.print(1000 * myIMU.ax);
        Serial.print(" mg ");
        Serial.print(" Y-acceleration: "); Serial.print(1000 * myIMU.ay);
        Serial.print(" mg ");
        Serial.print(" Z-acceleration: "); Serial.print(1000 * myIMU.az);
        Serial.print(" mg ");

        datalog.print(" X-acceleration: "); datalog.print(1000 * myIMU.ax);
        datalog.print(" mg ");
        datalog.print(" Y-acceleration: "); datalog.print(1000 * myIMU.ay);
        datalog.print(" mg ");
        datalog.print(" Z-acceleration: "); datalog.print(1000 * myIMU.az);
        datalog.print(" mg ");

        // Print gyro values in degree/sec
        Serial.print(" X-gyro rate: "); Serial.print(myIMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print(" Y-gyro rate: "); Serial.print(myIMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print(" Z-gyro rate: "); Serial.print(myIMU.gz, 3);
        Serial.print(" degrees/sec");

        datalog.print(" X-gyro rate: "); datalog.print(myIMU.gx, 3);
        datalog.print(" degrees/sec ");
        datalog.print(" Y-gyro rate: "); datalog.print(myIMU.gy, 3);
        datalog.print(" degrees/sec ");
        datalog.print(" Z-gyro rate: "); datalog.print(myIMU.gz, 3);
        datalog.print(" degrees/sec");

        // Print mag values in degree/sec
        Serial.print(" X-mag field: "); Serial.print(myIMU.mx);
        Serial.print(" mG ");
        Serial.print(" Y-mag field: "); Serial.print(myIMU.my);
        Serial.print(" mG ");
        Serial.print(" Z-mag field: "); Serial.print(myIMU.mz);
        Serial.print(" mG");

        datalog.print(" X-mag field: "); datalog.print(myIMU.mx);
        datalog.print(" mG ");
        datalog.print(" Y-mag field: "); datalog.print(myIMU.my);
        datalog.print(" mG ");
        datalog.print(" Z-mag field: "); datalog.print(myIMU.mz);
        datalog.print(" mG");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature: ");  Serial.print(myIMU.temperature, 1);
        Serial.println(" degrees C");

        datalog.print("Temperature: ");  datalog.print(myIMU.temperature, 1);
        datalog.println(" degrees C");
      }

      myIMU.count = millis();
     
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

   
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
        Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
        Serial.println(" mg");

        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
        Serial.println(" deg/s");

        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
        Serial.print(" my = "); Serial.print((int)myIMU.my);
        Serial.print(" mz = "); Serial.print((int)myIMU.mz);
        Serial.println(" mG");

        Serial.print("q0 = ");  Serial.print(*getQ());
        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
        Serial.print(" qz = "); Serial.println(*(getQ() + 3));

        datalog.print("ax = ");  datalog.print((int)1000 * myIMU.ax);
        datalog.print(" ay = "); datalog.print((int)1000 * myIMU.ay);
        datalog.print(" az = "); datalog.print((int)1000 * myIMU.az);
        datalog.println(" mg");

        datalog.print("gx = ");  datalog.print(myIMU.gx, 2);
        datalog.print(" gy = "); datalog.print(myIMU.gy, 2);
        datalog.print(" gz = "); datalog.print(myIMU.gz, 2);
        datalog.println(" deg/s");

        datalog.print("mx = ");  datalog.print((int)myIMU.mx);
        datalog.print(" my = "); datalog.print((int)myIMU.my);
        datalog.print(" mz = "); datalog.print((int)myIMU.mz);
        datalog.println(" mG");

        datalog.print("q0 = ");  datalog.print(*getQ());
        datalog.print(" qx = "); datalog.print(*(getQ() + 1));
        datalog.print(" qy = "); datalog.print(*(getQ() + 2));
        datalog.print(" qz = "); datalog.println(*(getQ() + 3));
      }


       myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw  -= 8.5;
      myIMU.roll *= RAD_TO_DEG;
    if(SerialDebug)
      {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.println(myIMU.roll, 2);

        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
        Serial.println(" Hz");

        datalog.print("Yaw, Pitch, Roll: ");
        datalog.print(myIMU.yaw, 2);
        datalog.print(", ");
        datalog.print(myIMU.pitch, 2);
        datalog.print(", ");
        datalog.println(myIMU.roll, 2);

        datalog.print("rate = ");
        datalog.print((float)myIMU.sumCount / myIMU.sum, 2);
        datalog.println(" Hz");
      }

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    }
                             
  if (pumpVariable = 0){
    if (heightVariable = 0){
      if (feet >= 30000){
        heightVariable = 1;
      }
    }
    if (heightVariable = 1){
      if ((1000 * myIMU.ay) < -500){
        pumpVariable = 1;
      }
    }
   
    
    //datalog.close();
}
  if (pumpVariable = 1) {
    digitalWrite(solenoid, HIGH);         //opens solenoid valve
    digitalWrite(vacuumPump1, HIGH);     //turns on pumps
    digitalWrite(vacuumPump2, HIGH);
    if(feet <= 10000){      //triggers with low altitude
      digitalWrite(vacuumPump1, LOW);        //turns off pumps
      digitalWrite(vacuumPump1, LOW);
      digitalWrite(solenoid, LOW);       //closes solenoid valve
      //while(1){  }                //triggers infinite loop, stopping code in current state
    }
   
}
  
}
}
