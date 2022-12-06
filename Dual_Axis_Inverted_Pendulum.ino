///////////////////////////// IMU ///////////////////////////

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

unsigned long timer = 0;  //unsigned
unsigned long timeresolution = 4000;

float delete_this_u_idiot = 0.8*0.0174533 * 0.6;

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
  if(GAsync)
  {
    accroll = p*accroll + (1-p)*(atan(Ay/Ax));
    accpitch = p*accpitch + (1-p)*(atan(Az/Ax));
    roll = P * (roll - Gz * dt) + (1-P) * accroll;// + pitch * sin(Gx * dt);
    pitch = P * (pitch + Gy * dt) + (1-P) * accpitch;// - roll * sin(Gx * dt);

  }
  else
  {
    accvector = sqrt(pow(Ax,2)+pow(Ay,2)+pow(Az,2));
    roll = asin(Ay/accvector);
    pitch = asin(Az/accvector);
    GAsync = true;
  }
}
///////////////////////////////// stepper control /////////////////////////

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
#define maxspeed 1000//13*64//other is way faster!!!!                                    //1000 if shit goes sideways was 800
#define frequencyreduction 10

#define Kpc 0.21
#define Kic 0.0001//0001        not to mess with: p=0.21 i=0.0001 d=0.011
#define Kdc 0.011//.2

float Sc = 0;
float Ec = 0;
float Pcterm;
float PIDc;


#define Kpp 15000//40000//9000   not to mess with: p=15000 i=400 d=500
#define Kip 400//100
#define Kdp 500//300

float ROLL, PITCH;
float Sp = 0;
float Ep = 0;
float Ppterm;
float PIDp;

volatile bool failed = false;
volatile int OCR1A0 = 0;
volatile int motortostep;

struct stepper
{
  int dirpin;
  volatile bool dir;
  volatile long stepcount = 0;
  volatile long control = 0; //ticks since last step
  volatile long delta = 0; //ticks since last step
  volatile float speed0; //PID of motor if axes were aligned
  volatile float actualspeed;
  volatile float c;
  float oldEc = 0;
  float oldEp = 0;
  int counter = 0;
  float Dcterm = 0;
  float Dpterm = 0;
  float Icterm = 0;
  float Ipterm = 0;
};

struct stepper motor[2];

void balance ()
{
  //setting roll and pitch values as constants throughout the calculations
  readvalues();
  calculate();
  
  ROLL = roll;//+ delete_this_u_idiot); //get rid of -
  PITCH = -(pitch + delete_this_u_idiot);

//  Serial.print(-300);
//  Serial.print(", ");
//  Serial.print(300);
//  Serial.print(", ");
//  Serial.println(ROLL * 1800./PI);

  // if axis werent in H bridge motorspeeds
  
  PIDcart (sqrt(2) * (motor[0].stepcount + motor[1].stepcount), 0);
  PIDpendulum (ROLL, 0);
  
    //OPTIONAL
//  if(fabs(PIDp) < 8)  //adding deadband
//    PIDp = 0;

  motor[0].speed0 = PIDp;  

  PIDcart (sqrt(2) * (motor[1].stepcount - motor[0].stepcount), 1);
  PIDpendulum (PITCH, 1);

  //OPTIONAL
//  if(fabs(PIDp) < 8)  //adding deadband
//    PIDp = 0;
    
  motor[1].speed0 = PIDp;

  // H bridge compensation
  
  motor[0].actualspeed = motor[0].speed0 - motor[1].speed0;
  motor[1].actualspeed = motor[0].speed0 + motor[1].speed0;

  //direction of motors
  dir(0);
  dir(1);

  // multiply by 64 since i changed the prescaler from 64 to 1
  motor[0].c = 514718.54 / (fabs(motor[0].actualspeed) + minspeed); //64 * 2560.*PI = 514718.54
  motor[1].c = 514718.54 / (fabs(motor[1].actualspeed) + minspeed);

//  Serial.print(-10000);
//  Serial.print(", ");
//  Serial.print(10000);
//  Serial.print(", ");
//  Serial.print(motor[0].actualspeed);
//  Serial.print(", ");
//  Serial.println(motor[1].actualspeed);
  
  while(timer > micros());
  timer += timeresolution;
}

void PIDcart (float distance, int i)
{
  Ec = Sc - (distance/4200.)*2*0.0075*PI; //converting x to meters
  Pcterm = Kpc * Ec;
  motor[i].Icterm += Kic * Ec;
  motor[i].Dcterm = Kdc * (Ec - motor[i].oldEc) / dt;
  PIDc = Pcterm + motor[i].Icterm + motor[i].Dcterm;
  motor[i].oldEc = Ec;
}
void PIDpendulum (float angle, int i)
{
  Sp = 0 - PIDc;
  Ep = Sp - angle;
  Ppterm = Kpp * Ep;
  motor[i].Ipterm += Kip * Ep;
//  Dpterm = Kdp * (Ep - oldEp) / dt;
//  oldEp = Ep;
  if(fabs(Ep - motor[i].oldEp) > 0.000025 && motor[i].counter >= frequencyreduction) //adding low pass frequency filter, 0.000025 works great
  {
    motor[i].Dpterm = Kdp * (Ep - motor[i].oldEp) / (dt*frequencyreduction);
    motor[i].oldEp = Ep;
    motor[i].counter = 0;
  }
  else
    motor[i].counter++;
  
  PIDp = Ppterm + motor[i].Ipterm + motor[i].Dpterm;
}
void dir (int i)
{
  if(0. <= motor[i].actualspeed && !motor[i].dir)//dirchange
  {
     motor[i].dir = true;
     digitalWrite(motor[i].dirpin, HIGH);
  }
  else if(0. > motor[i].actualspeed && motor[i].dir)
  {
     motor[i].dir = false;
     digitalWrite(motor[i].dirpin, LOW);
  }
}
void STEP (int i)
{
  if(motor[i].actualspeed != 0 && motor[i].stepcount < 7200 && motor[i].stepcount > -7200) //still balancing
  {
    if(motor[i].dir)
      motor[i].stepcount++;
    else
      motor[i].stepcount--;

    if(i == 0)
    {
      A_STEP_HIGH
      A_STEP_LOW
    }
    else
    {
      B_STEP_HIGH
      B_STEP_LOW
    }
  }
  else if (motor[i].stepcount >= 7200 || motor[i].stepcount <= -7200)
  {
    failed = true;
    digitalWrite(ENABLE_PIN, HIGH);
  }
}


ISR(TIMER1_COMPA_vect)
{
  OCR1A = 15999999; //freeze it

  //step
  if(motortostep != 2)
  {
    STEP(motortostep);
  }
  else
  {
    STEP(0);
    STEP(1);
  }

  //set inner timer to 0 again
  TCNT1 = 0;
  
  motor[0].delta = motor[0].c - motor[0].control;
  motor[1].delta = motor[1].c - motor[1].control;
  
  if(motor[0].delta == motor[1].delta)
  {
    OCR1A0 = motor[0].delta;
    motor[0].control = 0;
    motor[1].control = 0;
    motortostep = 2;
  }
  else if(motor[0].delta > motor[1].delta)
  {
    OCR1A0 = motor[1].delta;
    motor[0].control += OCR1A0;
    motor[1].control = 0;
    motortostep = 1;
  }
  else
  {
    OCR1A0 = motor[0].delta;
    motor[0].control = 0;
    motor[1].control += OCR1A0;
    motortostep = 0;
  }

  if(OCR1A0 < maxspeed)
    OCR1A0 = maxspeed;

  //if program takes longer to run than delay needed
  if(OCR1A0 < TCNT1)
    OCR1A0 = TCNT1 * 1.1;
    
  OCR1A = OCR1A0;
}

///////////////////////////////// setup and loop /////////////////////////

void setup() 
{
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
  
  // Set CS12, CS11 and CS10 bits for 1 prescaler
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  
  interrupts();
  
//  Serial.begin(1000000);
//  while (!Serial);

  motor[0].dir = true;
  digitalWrite(motor[0].dirpin, HIGH);
  
  motor[1].dir = true;
  digitalWrite(motor[1].dirpin, HIGH);
  
  digitalWrite(ENABLE_PIN, LOW);

  delay(4000);
  
  timer = micros();
}


void loop() {
  TI1_ON
  while(true)
  {
    balance();
  }
}

//    Serial.print(-300);
//    Serial.print(", ");
//    Serial.print(motor[1].speed0);
//    Serial.print(", ");
//    Serial.println(64 * 2560.*PI/(fabs(motor[0].speed0) + minspeed));





//OCR1A = 50000; //freeze until this program runs
//  
//  STEP(0);
//  STEP(1);
//  
//  if(motor[0].c < maxspeed)
//      motor[0].c = maxspeed;
//  TCNT1 = 0;
//  OCR1A = motor[0].c;

// at the start of the program to make sure it has time to start
//int problem = 0;
//bool done = false;
//    if(!done)
//    {
//      if(problem == 0)
//      {
//        TI1_ON
//        done = true;
//      }
//      problem++;
//    }


// probably only for troubleshooting (in balance() after roll and pitch were established)
//  x0 = motor[0].stepcount;
//  y0 = motor[1].stepcount;
//  x = (x0 + y0)/2;
//  y = (x0 - y0)/2;

//from the end of isr
//  if(OCR1A0 < maxspeed)
//    OCR1A0 = maxspeed;
    
//  if(OCR1A0 > TCNT1)
//  {
//    OCR1A0 -= TCNT1;
//  }
//  else
//  {
//    OCR1A0 = 200;
//  }
