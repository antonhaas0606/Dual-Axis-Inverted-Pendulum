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

volatile float thetax = 0;
volatile float accx = 0;

volatile float P = 0.9996;//0.995;
volatile float p = 0.00;//0.995;

unsigned long timer = 0;
unsigned long timeresolution = 4000;

int xangle, yangle; //limiting the precision of theta for more stable output

float delete_this_u_idiot = 0.8*0.0174533;

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
//  Serial.print(4098);
//  Serial.print(",");
//  Serial.print(-4098);
//  Serial.print(",");
//  Serial.println(Ax);
  
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
  if(GAsync)
  {
    accx = p*accx + (1-p)*atan(Ay/Ax);
    thetax = P * (thetax - Gz * dt) + (1-P) * accx;//+ thetax * sin(Gz * dt)
  }
  else
  {
    thetax = atan(Ay/Ax);
    GAsync = true;
  }
  //convert to degrees *10
  xangle = thetax * 1800./PI;
}


////////////////////////////////////    end of mpu  /////////////////////////
#define pinOfPin(P)\
  (((P)>=0&&(P)<8)?&PIND:(((P)>7&&(P)<14)?&PINB:&PINC))
#define pinIndex(P)((uint8_t)(P>13?P-14:P&7))
#define pinMask(P)((uint8_t)(1<<pinIndex(P)))
#define isLow(P)((*(pinOfPin(P))& pinMask(P))==0)

#define ENABLE_PIN 8

#define TI1_ON             TIMSK1 |= (1 << OCIE1A);
#define TI1_OFF            TIMSK1 &= ~(1 << OCIE1A);

#define A_STEP_HIGH        PORTD |= B00000100;
#define A_STEP_LOW         PORTD &= ~B00000100;

#define B_STEP_HIGH        PORTD |= B00001000;
#define B_STEP_LOW         PORTD &= ~B00001000;

#define minspeed 15//15 prevents the motor from jerking //when PIDp = 0 then c = minspeed
#define maxspeed 13
#define frequencyreduction 10

int counter = 0;

#define Kpc 0.21//21
#define Kic 0.0001//0001
#define Kdc 0.11//.11

volatile float x = 0;
float Sc = 0;
float Ec = 0;
float oldEc = 0;
float Pcterm;
float Icterm;
float Dcterm;
float PIDc;


#define Kpp 9000
#define Kip 600//600
#define Kdp 300

volatile float theta = 0;
float Sp = 0;
float Ep = 0;
float oldEp = 0;
float Ppterm;
float Ipterm;
float Dpterm;
float PIDp;

volatile bool Direction = true;
volatile float c;                                //inter step delays
volatile bool failed = false;

struct stepper
{
  int dirpin;
};

struct stepper motor[2];

void balance ()
{
  readvalues();
  calculate();

  theta = -thetax;// + delete_this_u_idiot;
  
  PIDcart ();
  PIDpendulum ();
  dir();
  
//  //OPTIONAL
//  if(fabs(PIDp) < 1)  //adding deadband
//    PIDp = 0;

  c = 2560.*PI/(fabs(PIDp) + minspeed);  //anton ur a dick 4 being secretive

  while(timer > micros());
  timer += timeresolution;
}

void PIDcart ()
{
  Ec = Sc - (x/4200.)*2*0.0075*PI; //converting x to meters
  Pcterm = Kpc * Ec;
  Icterm += Kic * Ec;
  Dcterm = Kdc * (Ec - oldEc) / dt;
  PIDc = Pcterm + Icterm + Dcterm;
  oldEc = Ec;
}
void PIDpendulum ()
{
  Sp = 0 - PIDc;
  Ep = Sp - theta;
  Ppterm = Kpp * Ep;
  Ipterm += Kip * Ep;
//  Dpterm = Kdp * (Ep - oldEp) / dt;
//  oldEp = Ep;
  
  if(fabs(Ep - oldEp) > 0.000025 && counter == frequencyreduction) //adding low pass frequency filter, 0.000025 works great
  {
    Dpterm = Kdp * (Ep - oldEp) / (dt * frequencyreduction);
    oldEp = Ep;
    counter = 0;
  }
  else
    counter++;
  
  PIDp = Ppterm + Ipterm + Dpterm;
}
void dir ()
{
  if(0. < PIDp && !Direction)//dirchange
  {
     Direction = true;
     digitalWrite(motor[0].dirpin, LOW);
     digitalWrite(motor[1].dirpin, LOW);
  }
  else if(0. > PIDp && Direction)
  {
     Direction = false;
     digitalWrite(motor[0].dirpin, HIGH);
     digitalWrite(motor[1].dirpin, HIGH);
  }
}
void setup() 
{
  //for mpu
  Wire.begin();
  setupsensor();
  calibrate();
  
  motor[0].dirpin = 5;
  motor[1].dirpin = 6;

  pinMode(motor[0].dirpin, OUTPUT);
  pinMode(motor[1].dirpin, OUTPUT);

  
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH);

  //setting up interrupt system
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 0;
  TCCR1B |= (1 << WGM12);
  
  // Set CS12, CS11 and CS10 bits for 64 presca.er
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  interrupts();
  
//  Serial.begin(1000000);
//  while (!Serial);

  if(thetax > 0)//initial dir
  {
     Direction = true;
     digitalWrite(motor[0].dirpin, LOW);
     digitalWrite(motor[1].dirpin, LOW);
  }
  else 
  {
     Direction = false;
     digitalWrite(motor[0].dirpin, HIGH);
     digitalWrite(motor[1].dirpin, HIGH);
  }
  
  digitalWrite(ENABLE_PIN, LOW);

  delay(4000);
  
  timer = micros() + timeresolution;
}
void STEP ()
{
  if(x < 7200 && x > -7200 && !failed) //still balancing
  {

    if(Direction)
      x++;
    else
      x--;
      
    A_STEP_HIGH
    A_STEP_LOW
  
    B_STEP_HIGH
    B_STEP_LOW
  }
  else
  {
    failed = true;
    digitalWrite(ENABLE_PIN, HIGH);
  }
}

ISR(TIMER1_COMPA_vect)
{
  OCR1A = 65500;
  if(PIDp != 0.)
    STEP();
  if(c < maxspeed)
      c = maxspeed;
  OCR1A = c;
}

void loop()
{
  TI1_ON
  while(true)//(!failed)
  {
    balance();
//    Serial.print(-300);
//    Serial.print(", ");
//    Serial.print(300);
//    Serial.print(", ");
//    Serial.println(PIDp);
  }
  TI1_OFF
}
