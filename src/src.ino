// Based on calibration by Luis Rodenas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>

/* ==========================LICENCE===================================
I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
=======================================================================
*/


/*========================= Notice ====================================
 * Project name: Heath Mate
 * description: Measuring Exercise Position with MPU6050, exclude yaw data
 * Version: 0.2
 * Developer: BongO Moon
 * Github Address : 
 * first date: 2017.08.15
 * last date: 2017.08.27
 =====================================================================*/

 
// Changelog:
//    2017-08-29 Add Calibration code by Luis
//      2017-08-28 Exclude EMG Sensor and prototype-code by DaeJangJangE(ver0.2)
//      2017-08-26 Refer to prototype-code by DaeJangJangE
//      2017-08-15 Include library for MPU6050 class using DMP(ver0.1)


#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//#include <Thread.h>                //for Thread
#include <SoftwareSerial.h>       //for Blutooth
#include <Adafruit_NeoPixel.h>   //for operating color 3 LED


/* ==============================CONFIGURATION===================================
 *  GND
 *  VCC = 5V
 *  INT = D2 //interrupt pin for DMP(Digital Motion Processor) && this sketch depends on the MPU-6050's INT pin being connected to the Arduino's external interrupt #0 pin
 *  BTTX = D3
 *  BTRX = D4
 *  SDA = A4
 *  SCL = A5
 * ==============================================================================*/
#define BTTX        3
#define BTRX        4
#define PIXELPIN   10
#define NUMPIXELS   1

//#define CHECK 30

//Thread sensor = Thread(); //initialize Thread for using Sensor
SoftwareSerial mySerial(BTTX, BTRX); //initialize for using Blutooth
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXELPIN, NEO_GRB+NEO_KHZ800);  //initalize for using LED

/*
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
// Accel address = 0x3B ~ 040 / Gyro address = 0x43 ~ 0x48
// MPU6050 MPU(0x69); // <-- use for AD0 high
*/
MPU6050 MPU;
//const int MPU=0x68;                                 //MPU 6050's I2C address by DaeJangJangE

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[128]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int16_t ax, ay, az,gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state=0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

//대장장이
/*
int16_t AcX=0,AcY=0,AcZ=0,Tmp=0,GyX=0,GyY=0,GyZ=0;  //받아오는 자이로센서의 데이터(사용은 AcZ하나만)

int16_t sumZ=0,refZ=0;                              //자이로센서의 초기화보정을 위한 레퍼런스 데이터 수집
int electro=0;                                      //근전도센서의 데이터를 저장할 변수

int maxGyro=0;                                      //최대,최소 데이터를 저장해두기 위한 변수
int minGyro=32000;
int maxElect=0;
int minElect=32000;

int mappedData=0;                                   //수집된 데이터를 기준으로 현재의 근전도가 최대,최소 근전도에 대한 비중을 매핑한 비율 근전도 데이터

int cnt = 0;                                        //수집한 데이터의 개수체크를 위한 카운터

String myString="";
*/

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. FastMode
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  Serial.begin(9600);
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  MPU.initialize();     //initializing sensing
  pinMode(INTERRUPT_PIN, INPUT);    //INTERRUT_PIN = D2
  
  // verify connection
  Serial.println(F("Testing device connections..."));   //If you use F() you can move constant strings to the program memory instead of the ram.
  Serial.println(MPU.testConnection() ? F("Motion Sensor connection successful") : F("Motion Sensor connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = MPU.dmpInitialize();

  // reset offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    MPU.setDMPEnabled(true);    //Enable DMP

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);    //When Interrupt occurs, it has executed.
    mpuIntStatus = MPU.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = MPU.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
  //ready to use Blutooth, LED, MPU
  mySerial.begin(9600);
  pixels.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true); 

  //listen connection of Blutooth 
  Serial.println("bluetooth Connection Wait......");
  ledColor(150,0,0);
  while(true){
    if(mySerial.available()){
      mySerial.read();
      ledColor(0,150,0);
      break;
    }
  }

  for(int i=0;i<10;i++){
    Wire.beginTransmission(MPU);   
    Wire.write(0x3B);               
    Wire.endTransmission(false);
    Wire.requestFrom(MPU,14,true); 
    readData();                                       //최초 데이터 측정
    AcZ=filter(AcZ);                                  //측정한 데이터를 보기 편한 데이터로 변환
    sumZ += AcZ/10;
    Serial.print(" | AcZ = "); Serial.println(AcZ);
    Serial.print(" | sumZ = "); Serial.println(sumZ);
    delay(200);
  }
  refZ = sumZ;
  Serial.println("INIT SUSS");                        //최초데이터수집완료
/*
  sensor.onRun(sensorThread);                          //스레드를 사용하기위한 초기화
  sensor.setInterval(100);
*/
}

s
// ================================================================
// ===                      PRECATION OF SETUP                  ===
// ================================================================
/* set full scale value of accel/gyro range
 * execute after 'mpu.initialize()'

 accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_GYRO_SCALE_VALUE); <- GYRO_SCALE_VALUE = 250 or 500 or 1000 or 2000
 accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_ACELL_SCALE_VALUE); <- ACELL_SCALE_VALUE = 2 or 4 or 8 or 16
 */
//This fucntion is that reduce the noise of the sensor reading
//mpu.setDLPFMode(MPU6050_DLPF_BW_256); <- 5 or 10 or 20 or 42 or 98 or 188 or 256




// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
 
void loop(){

  
  /* 주문사항: 어플에서 운동을 선택 -> 운동 id 값 전송(ex. biceps_curl)
        -> 아두이노에서 운동 id값을 받아서 그에 따른 운동 함수 호출
  */ 


  while(mySerial.available()){            //If there is Blutooth receipt, it receive
    char myChar = (char)mySerial.read();
    myString+=myChar;
    delay(5);
  }
  if(!myString.equals(""))                              //수신된 정보가 있을 경우
  {
    Serial.println("input value: "+myString);
    if(myString.equals("reset")){                       //리셋이라는 데이터를 수신했을경우 데이터수집을 다시 할 수 있도록 하는 명령
      maxGyro=0;
      minGyro=32000;
      maxElect=0;
      minElect=32000;
      cnt = 0 ;
    }
    if(myString.equals("c")){                           //블루투스 연결에 성공할 경우 LED의 색 변경
      ledColor(0,150,0);
    }
    if(myString.equals("d")){                           //블루투스 연결이 해제될 경우 LED의 색 변경
      ledColor(150,0,0);
    }
    myString="";
  }  
}


// ================================================================
// ===                 Accel/Gyro Calibration                   ===
// ================================================================
void mpu_meansensors(){
  long i=0, buff_ax=0, buff_ay=0, buff_az=0, buff_gx=0, buff_gy=0, buff_gz=0;

  while(i < (buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if(i > 100 && i <= (buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if(i == (buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void mpu_calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  
  while(1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    mpu_meansensors();
    Serial.println("...");

    if(abs(mean_ax)<=acel_deadzone)
    ready++;
    else
    ax_offset=ax_offset-mean_ax/acel_deadzone;

    if(abs(mean_ay)<=acel_deadzone)
    ready++;
    else
    ay_offset=ay_offset-mean_ay/acel_deadzone;

    if(abs(16384-mean_az)<=acel_deadzone)
    ready++;
    else
    az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if(abs(mean_gx)<=giro_deadzone)
    ready++;
    else
    gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if(abs(mean_gy)<=giro_deadzone)
    ready++;
    else
    gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if(abs(mean_gz)<=giro_deadzone)
    ready++;
    else
    gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6)
    break;
  }
}


// ================================================================
// ===                        Exercise List                     ===
// ================================================================

//------------------------- biceps exercise -----------------------//
void biceps_curl_exercise(){
  if (state==0){
    Serial.println("\nReading sensors for first time...");
    mpu_meansensors();
    state++;
    delay(1000);
  }

  if (state==1) {
    Serial.println("\nCalculating offsets for seconds time...");
    mpu_calibration();
    state++;
    delay(1000);
  }

  if (state==2) {
    mpu_meansensors();
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax); 
    Serial.print("\t");
    Serial.print(mean_ay); 
    Serial.print("\t");
    Serial.print(mean_az); 
    Serial.print("\t");
    Serial.print(mean_gx); 
    Serial.print("\t");
    Serial.print(mean_gy); 
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset); 
    Serial.print("\t");
    Serial.print(ay_offset); 
    Serial.print("\t");
    Serial.print(az_offset); 
    Serial.print("\t");
    Serial.print(gx_offset); 
    Serial.print("\t");
    Serial.print(gy_offset); 
    Serial.print("\t");
    Serial.println(gz_offset); 
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
    Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
    while (1); // ????check????
  }

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    MPU.setDMPEnabled(true);    //Enable DMP

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);    //When Interrupt occurs, it has executed.
    mpuIntStatus = MPU.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = MPU.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
  if (!dmpReady) return;    // if programming failed, don't try to do anything
  //if(sensor.shouldRun()) sensor.run();                 //start Thread(MPU6050)
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    MPU.getFIFOBytes(fifoBuffer, packetSize);
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      MPU.dmpGetQuaternion(&q, fifoBuffer);
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
    #endif

    #ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      MPU.dmpGetQuaternion(&q, fifoBuffer);
      MPU.dmpGetEuler(euler, &q);
      Serial.print("euler\t");
      Serial.print(euler[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(euler[2] * 180/M_PI);
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      MPU.dmpGetQuaternion(&q, fifoBuffer);
      MPU.dmpGetGravity(&gravity, &q);
      MPU.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL
     // display real acceleration, adjusted to remove gravity
     MPU.dmpGetQuaternion(&q, fifoBuffer);
     MPU.dmpGetAccel(&aa, fifoBuffer);
     MPU.dmpGetGravity(&gravity, &q);
     MPU.dmpGetLinearAccel(&aaReal, &aa, &gravity);
     Serial.print("areal\t");
     Serial.print(aaReal.x);
     Serial.print("\t");
     Serial.print(aaReal.y);
     Serial.print("\t");
     Serial.println(aaReal.z);
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      MPU.dmpGetQuaternion(&q, fifoBuffer);
      MPU.dmpGetAccel(&aa, fifoBuffer);
      MPU.dmpGetGravity(&gravity, &q);
      MPU.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      MPU.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      Serial.print("aworld\t");
      Serial.print(aaWorld.x);
      Serial.print("\t");
      Serial.print(aaWorld.y);
      Serial.print("\t");
      Serial.println(aaWorld.z);
    #endif
    
    #ifdef OUTPUT_TEAPOT
      // display quaternion values in InvenSense Teapot demo format:
      teapotPacket[2] = fifoBuffer[0];
      teapotPacket[3] = fifoBuffer[1];
      teapotPacket[4] = fifoBuffer[4];
      teapotPacket[5] = fifoBuffer[5];
      teapotPacket[6] = fifoBuffer[8];
      teapotPacket[7] = fifoBuffer[9];
      teapotPacket[8] = fifoBuffer[12];
      teapotPacket[9] = fifoBuffer[13];
      Serial.write(teapotPacket, 14);
      teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
    #endif
  }
  
  ---------------------------revising----------------------------
}

// ================================================================
// ===                        SENSOR_THREAD                     ===
// ================================================================
/*
void sensorThread(){

}
*/

/*
void sensorThread(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);
  readData();
  AcZ=filter(AcZ);
  electro=analogRead(A0);
  
  //시리얼 모니터에 출력 (가속XYZ 이후 자이로XYZ)
  //Serial.print("AcX = "); Serial.print(AcX);
  //Serial.print(" | AcY = "); Serial.print(AcY);

  
  if(-300<(refZ-AcZ)&&(refZ-AcZ)<300){                  //움직임이 매우 작거나 없을 경우에 스마트폰으로 -1 이라는 데이터를 전송
    Serial.println("extra low data");
    mySerial.print(-1);
    mySerial.print("#");
  }
  else if(-5000<(refZ-AcZ)&&(refZ-AcZ)<5000){           //적절한 움직임을 감지하였을때 실행
    Serial.print(" | AcZ = "); Serial.println(AcZ);refZ = AcZ;
    Serial.print("electromyogram = "); Serial.println(electro);
    if(cnt<CHECK){                                      //데이터 수집이 덜 된 경우에 자이로, 근전도센서의 최대, 최소값을 수집
      if(AcZ>maxGyro)maxGyro=AcZ;
      if(AcZ<minGyro)minGyro=AcZ;
      if(electro>maxElect)maxElect=electro;
      if(electro<minElect)minElect=electro;
      cnt++;
    }
    else if(cnt==CHECK){                                //데이터 수집을 완료하면 자이로, 근전도센서의 최대, 최소값을 전송
      String str = "";
      str+="ref,";
      str+=maxGyro;
      str+=",";
      str+=minGyro;
      str+=",";
      str+=maxElect;
      str+=",";
      str+=minElect;
      str+="#";
      mySerial.print(str);
      str = "";
      Serial.println("ref set finish");
      Serial.print(maxGyro);
      Serial.print(" | ");
      Serial.print(minGyro);
      Serial.print(" | ");
      Serial.print(maxElect);
      Serial.print(" | ");
      Serial.print(minElect);
      Serial.println();
      delay(3000);
      cnt++;
    }
    else{                                             //적절한 수준의 움직임이 된 상황이기때문에 매핑된 근전도센서값을 전송
      mapping(maxElect,minElect);
      mySerial.print(mappedData);
      mySerial.print("#");
    }
  }
  else {                                            //움직임이 너무 크거나 엉뚱한 움직임일 때 스마트폰으로 9999라는 데이터를 전송
    Serial.println("extra high data");
    mySerial.print(9999);
    mySerial.print("#");
  }
  //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  
  //Serial.print(" | GyX = "); Serial.print(GyX);
  //Serial.print(" | GyY = "); Serial.print(GyY);
  //Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(300);
}

int16_t filter(int16_t n){
  n -= 13000;
  if(n < 0) n *= -1;
  return n;
}

void readData(){
  //데이터 한 바이트 씩 읽어서 반환
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void mapping(int maxData, int minData){
  mappedData = map(electro,minData,maxData,0,100);
}

*/
void ledColor(int r, int g, int b){  
  for(int i=0;i<NUMPIXELS;i++){
    pixels.setPixelColor(i,pixels.Color(r,g,b));
  }
  pixels.show();
}

