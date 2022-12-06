#include <Wire.h>

volatile float Ax, Ay, Az;
volatile float Axcal = -756;
volatile float Aycal = -3247-55;
volatile float Azcal = 90;

volatile float Gx, Gy, Gz;
volatile float Gxcal = 0, Gycal = 0, Gzcal = 0;
float dt = 0.004;

volatile bool GAsync = false;
volatile bool calibrated = false;
int getridof;

volatile float roll = 0, pitch = 0;
volatile float accvector = 0, accroll = 0, accpitch = 0;

volatile float P = 0.9996; //gyro
volatile float p = 0;   //acc filter in its own

unsigned long timer = 0;
unsigned long timeresolution = 4000;

int x, y; //float for more precision

void setup() 
{
  Serial.begin(1000000);
  while (!Serial);
  Wire.begin();
  
  setupsensor();
  calibrate();
  timer = micros() + timeresolution;
}

void loop() {
//  TI1_ON
  while(true)
  {
    readvalues();
    calculate();
    
    Serial.print(-300);
    Serial.print(", ");
    Serial.print(300);
    Serial.print(", ");
    Serial.print(x);
    Serial.print(", ");
    Serial.println(y);
    
    while(timer > micros());
    timer += timeresolution;
  }
}

void setupsensor()
{
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void calibrate()
{
  for (int i = 0; i < 2000 ; i++)
  {
    readvalues();
    Gxcal += Gx;
    Gycal += Gy;
    Gzcal += Gz;
  }
  Gxcal /= 2000.;
  Gycal /= 2000.;
  Gzcal /= 2000.;
  calibrated = true;
}

void readvalues()
{
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  Ax = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the acc_x variable
  Ay = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the acc_y variable
  Az = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the acc_z variable
  getridof = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  Gx = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the gyro_x variable
  Gy = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the gyro_y variable
  Gz = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the gyro_z variable

  Ax += Axcal;
  Ay += Aycal;
  Az += Azcal;

  //due to sensor settings and converting to rad/sec
  Gx *= PI/11790.;
  Gy *= PI/11790.;
  Gz *= PI/11790.;
  
  if(calibrated)
  {
    Gx -= Gxcal;
    Gy -= Gycal;
    Gz -= Gzcal;
  }
}

void calculate()
{
  accvector = sqrt(pow(Ax,2)+pow(Ay,2)+pow(Az,2));
  if(GAsync)
  {
    accroll = p*accroll + (1-p)*(asin(Ay/accvector));
    accpitch = p*accpitch + (1-p)*(asin(Az/accvector));
    roll = P * (roll - Gz * dt) + (1-P) * accroll + pitch * sin(Gx * dt);
    pitch = P * (pitch + Gy * dt) + (1-P) * accpitch - roll * sin(Gx * dt);
  }
  else
  {
    roll = asin(Ay/accvector);
    pitch = asin(Az/accvector);
    GAsync = true;
  }
  //convert to degrees*10
  x = roll * 1800./PI;
  y = pitch * 1800./PI;
}
