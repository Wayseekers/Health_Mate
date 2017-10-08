/************************************************************
Based on Jim Lindblom @ SparkFun Electronics
original creation date: November 23, 2016
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library
Development environment specifics:
Arduino IDE 1.6.12
SparkFun 9DoF Razor IMU M0
Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
*************************************************************/

/*========================= Notice ====================================
* Project name: WayFit Prototype
* description: Measuring Exercise Position with SparkFun 9DoF Razor IMU M0
* Version: 0.5
* Developer(Co.Op): BongO Moon, JunHee Lee, HyunGang Nah
* Github Address : http://github.com/wayseekers/health_mate
* first date: 2017.08.15
* last date: 2017.10.08
* Development environment specifics:
    Arduino IDE 1.6.12
    SparkFun 9DoF Razor IMU M0
* Supported Platforms:
  - ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
  - HM-10 Bluetooth 4.0 BLE
=====================================================================*/

// Changelog:
//      2017-10-01 add bluetooth on WayFit prototype (ver0.5)
//      2017-09-25 change brand name from healthmate to WayFit
//      2017-09-18 prepare to add Blutooth
//      2017-09-17 Change IMU sensor from MPU6050 to MPU9250 (ver 0.4)
//      2017-09-13 Complete second prototype for sensing biceps-carl exercise(ver0.3)
//      2017-09-12 Try to calculate two Quaternion size
//      2017-09-11 Exclude calibration code written by luis
//      2017-08-29 Add Calibration code written by Luis
//      2017-08-28 Exclude EMG Sensor and prototype-code written by DaeJangJangE(ver0.2)
//      2017-08-26 Refer to prototype-code written by DaeJangJangE
//      2017-08-15 Include library for MPU6050 class using DMP(ver0.1)

/* ==============================CONFIGURATION===================================
*  MCU - SparkFun 9DoF Razor IMU M0
*  GND
*  VCC = 3.3V
*  RX = TXD
*  TX = RXD
* ==============================================================================*/
#include "SparkFunMPU9250-DMP.h"

#define M0Serial SerialUSB
#define BTSerial Serial1

MPU9250_DMP imu;

String str="";

int exercise_state = 0;
int check_data;         // only check stabilization data
float w, x, y, z;       // Quaternion Reference Point Value
float exer_w, exer_x, exer_y, exer_z;   // Qauternion Exercise Point Value
double q_product;       // Quaternion Vector Product Value


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  M0Serial.begin(115200);
  BTSerial.begin(9600);
  
  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS) {
    while (1) {
      M0Serial.println("Unable to communicate with MPU-9250");
      M0Serial.println("Check connections, and try again.");
      M0Serial.println();
      delay(5000);
    }
  }

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
    DMP_FEATURE_GYRO_CAL, // Use gyro calibration
    10); // Set DMP FIFO rate to 10 Hz
       // DMP_FEATURE_LP_QUAT can also be used. It uses the 
       // accelerometer in low-power mode to estimate quat's.
       // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
       
  //Stabilize IMU Data(case 100) after turn on the device    
  while(check_data != 100){
   initValueStabilization();
  }
  
  M0Serial.println("bluetooth Connection Wait......");  
  while(true){
    if(BTSerial.available()){
      BTSerial.read();
      break;
    }
  }
  
  M0Serial.println("WayFit Start!!"); 
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
/*
void loop() {
  while(BTSerial.available()) {                    //블루투스 수신내용이 있을 경우 수신
    char c = (char)BTSerial.read();
    str+=c;
    delay(5);
  }
  
  if(!str.equals(""))                              //수신된 정보가 있을 경우
  {
    M0Serial.println("input value: "+str);
    //start exercise
    while (str.equals("start")){
      exercise_mode();
      M0Serial.println("start exercise.....");
    }
    //finish exercise or stop exercise
    if(str.equals("end")){
      exercise_state = 0;
      M0Serial.println("wait to select exercise.....");     
    }
    str="";
  }
}
*/

void loop() {
  // Check for new data in the FIFO
  if (imu.fifoAvailable()) {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {
      if(exercise_state == 0){
        w = imu.calcQuat(imu.qw);
        x = imu.calcQuat(imu.qx);
        y = imu.calcQuat(imu.qy);
        z = imu.calcQuat(imu.qz);
        M0Serial.println("Q: " + String(w, 4) + ", " + String(x, 4) + ", " + String(y, 4) + ", " + String(z, 4));  
      
        while(BTSerial.available()) {                  
          char c = (char)BTSerial.read();
          str+=c;                          
          delay(5);
        }
        if(!str.equals("")){
          M0Serial.println("input value: "+str);     
          if(str.equals("start")){
            w = imu.calcQuat(imu.qw);
            x = imu.calcQuat(imu.qx);
            y = imu.calcQuat(imu.qy);
            z = imu.calcQuat(imu.qz);
            exercise_state = 1;                                
          }
          str = "";                            
        }
      }
      else if(exercise_state == 1){
        exer_w = imu.calcQuat(imu.qw);
        exer_x = imu.calcQuat(imu.qx);
        exer_y = imu.calcQuat(imu.qy);
        exer_z = imu.calcQuat(imu.qz);
        q_product = w*exer_w + x*exer_x + y*exer_y + z*exer_z;

        BTSerial.print(q_product);
        BTSerial.print("#");
        M0Serial.print(q_product);
        M0Serial.println("#");
        
        while(BTSerial.available()) {                  
          char c = (char)BTSerial.read();
          str+=c;                          
          delay(5);
        }
        if(!str.equals("")){
          M0Serial.println("input value: "+str);     
          if(str.equals("end")){
            M0Serial.println("wait to select exercise");            
            exercise_state = 1;                                
          }
          str = "";                            
        }
      }
    }
  }
}

//Stabilize IMU Data(case 100) after turn on the device
void initValueStabilization(){
  // Check for new data in the FIFO
  if (imu.fifoAvailable()) {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {
      // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
      // are all updated.
      // Quaternion values are, by default, stored in Q30 long
      // format. calcQuat turns them into a float between -1 and 1
      w = imu.calcQuat(imu.qw);
      x = imu.calcQuat(imu.qx);
      y = imu.calcQuat(imu.qy);
      z = imu.calcQuat(imu.qz);
      M0Serial.println("Q: " + String(w, 4) + ", " + String(x, 4) + ", " + String(y, 4) + ", " + String(z, 4));
      check_data++;
    }
  }
}

/*
void exercise_mode() {
  // Check for new data in the FIFO
  if (imu.fifoAvailable()) {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if (imu.dmpUpdateFifo() == INV_SUCCESS) {
      switch (exercise_state) {
      case 0: w = imu.calcQuat(imu.qw);
        x = imu.calcQuat(imu.qx);
        y = imu.calcQuat(imu.qy);
        z = imu.calcQuat(imu.qz);
        exercise_state++;
        break;

      case 1: exer_w = imu.calcQuat(imu.qw);
        exer_x = imu.calcQuat(imu.qx);
        exer_y = imu.calcQuat(imu.qy);
        exer_z = imu.calcQuat(imu.qz);
        q_product = w*exer_w + x*exer_x + y*exer_y + z*exer_z;

        BTSerial.print(q_product);
        BTSerial.print("#");
        //M0Serial.println(q_product);
        break;
        
      default:
        break;
      }
    }
  }
}
*/
