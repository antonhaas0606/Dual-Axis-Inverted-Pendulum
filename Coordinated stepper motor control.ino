
//anguleraccel/decel value in [rad/s^2] (fullstep) 
//speed value in [rad/s] (less than 80 which is about 760rpm)
#define ENABLE_PIN 8

#define TI1_ON             TIMSK1 |= (1 << OCIE1A);
#define TI1_OFF            TIMSK1 &= ~(1 << OCIE1A);

#define A_STEP_HIGH        PORTD |= B00000100;
#define A_STEP_LOW         PORTD &= ~B00000100;

#define B_STEP_HIGH        PORTD |= B00001000;
#define B_STEP_LOW         PORTD &= ~B00001000;

#define MAXSPEED 5000//80
#define MAXACCEL 30000//300
#define MAXDECEL 30000//300

volatile int x = 0;
volatile int y = 0;

struct stepper
{
  volatile unsigned long stepcount = 0;
  //volatile long pos = 0;
  int dirpin;
  volatile unsigned long control = 0;              //ticks since motor stepped
  volatile long steps;
  volatile unsigned long accelsteps;
  volatile unsigned long decelsteps;
  volatile unsigned long n;             //used for calculating deceleration
  volatile float c;                     //inter step delays
  volatile unsigned long cmin;
  volatile bool finished = false;
};

struct stepper motor[2];

void setup() {
  motor[0].dirpin = 5;
  motor[1].dirpin = 6;
  
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  
  pinMode(motor[0].dirpin, OUTPUT);
  pinMode(motor[1].dirpin, OUTPUT);
  
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);

  //setting up interrupt system
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 0;
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 64 prescaler
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  interrupts();

  //pinMode(photor1, INPUT);

  //Serial.begin(9600);
}

//calculate first delay(c0), total steps needed, steps needed for acceleration and deceleration
void prepare(int which, long steps, long maxspeed, long anguleraccel, long angulardecel)
{
  //resetting
  motor[which].stepcount = 0;
  motor[which].control = 0;
  
  //set direction
  digitalWrite(motor[which].dirpin, steps < 0 ? HIGH : LOW);
  motor[which].steps = abs(steps);

  //calculating minimum inter step delay(max speed)
  //motor[which].cmin = (((2*3.14)/200)/maxspeed)*62500;
  
  //(timer frequency/prescaler)*sqrt((2*(2*pi/spr))/anguleraccel)*0,676  calculating first delay
  motor[which].c = (169000*sqrt(0.0628/anguleraccel));
  
  //calculating steps needed for acceleration to reach max speed (MaxSpeedLimit)
  long MSL = (maxspeed*maxspeed) / (2L*(2L * 3.14L / 200L) * anguleraccel);

  //calculating steps before deceleration starts disregarding speedlimit (AccelerationLimit)
  long AL = (motor[which].steps * angulardecel) / (anguleraccel + angulardecel);

  //steps needed for acceleration and deceleration
  if (MSL < AL)
  {
    motor[which].accelsteps = MSL;
    motor[which].decelsteps = (maxspeed*maxspeed) / (2L*(2L * 3.14L / 200L) * angulardecel);
  }
  else
  {
    motor[which].accelsteps = AL;
    motor[which].decelsteps = motor[which].steps - motor[which].accelsteps;
  }

  //setting up n
  motor[which].n = motor[which].decelsteps;

  if(motor[which].steps == 0)
  {
    motor[which].c = 0;
    motor[which].accelsteps = 0;
    motor[which].decelsteps = 0;
  }
}

void STEP (volatile int which) //stepping and calculating next delay
{
  if(motor[which].stepcount < motor[which].steps)
  {
    motor[which].stepcount++;
    if (which == 0)
    {
      A_STEP_HIGH
      A_STEP_LOW
    }
    else
    {
      B_STEP_HIGH
      B_STEP_LOW
    }
    
    if (motor[which].stepcount != 0 && motor[which].stepcount <= motor[which].accelsteps)
    {
      //using Taylor series approximation
      motor[which].c = motor[which].c - (motor[which].c * 2L) / (4L * motor[which].stepcount + 1L);
    }
    else if(motor[which].stepcount != motor[which].steps && motor[which].stepcount >= motor[which].steps - motor[which].decelsteps)
    {
      //using Taylor series approximation(solved for c(n-1))
      motor[which].c = (motor[which].c * (4L * motor[which].n + 1L)) / (4L * motor[which].n + 1L - 2L);
      motor[which].n--;
    }
  }
  else
  {
    //turn timer off and finish movement
    motor[which].finished = true;
  }
}

volatile int motortostep = 2;
volatile long nextdelay;
volatile long A;
volatile long B;

ISR(TIMER1_COMPA_vect)
{
  OCR1A = 65500;
  
  if(motortostep != 2)
  {
    STEP(motortostep);
  }
  else
  {
    STEP(0);
    STEP(1);
  }

  TCNT1 = 0;

  A = motor[0].c - motor[0].control;
  B = motor[1].c - motor[1].control;
  
  if((!motor[0].finished && A < B) || motor[1].finished)
  {
    nextdelay = A;
    motor[1].control += A;
    motor[0].control = 0;
    motortostep = 0;
  }
  else if(A > B || motor[0].finished)
  {
    nextdelay = B;
    motor[0].control += B;
    motor[1].control = 0;
    motortostep = 1;
  }
  else
  {
    nextdelay = B;
    motor[0].control = 0;
    motor[1].control = 0;
    motortostep = 2;
  }
  if(nextdelay < 5)
    nextdelay = 5;

  TCNT1 = 0;
  OCR1A = nextdelay;
}

void execute ()
{
  motor[0].finished = false;                                 //movement not finished
  motor[1].finished = false;
  TI1_ON                                                     //timer interrupt on
  while(! motor[0].finished || ! motor[1].finished);         //waiting for movement to finish
  TI1_OFF
}

void MoveTo(int xpos, int ypos)  //moves to the given coordinates (follows a linear trajectory)
{
  float xdif = abs(xpos - x);
  float ydif = abs(ypos - y);
  if(xdif < ydif)
  {
    float scaler = xdif / ydif;
    int scaledspeed = (int)(MAXSPEED*scaler);
    int scaledaccel = (int)(MAXACCEL*scaler);
    int scaleddecel = (int)(MAXDECEL*scaler);
    prepare(0,((xpos-x)*200L),scaledspeed,scaledaccel,scaleddecel);
    prepare(1,((ypos-y)*200L),MAXSPEED,MAXACCEL,MAXDECEL);
  }
  else if(xdif > ydif)
  {
    float scaler = ydif / xdif;
    int scaledspeed = (int)(MAXSPEED*scaler);
    int scaledaccel = (int)(MAXACCEL*scaler);
    int scaleddecel = (int)(MAXDECEL*scaler);
    prepare(0,((xpos-x)*200L),MAXSPEED,MAXACCEL,MAXDECEL);
    prepare(1,((ypos-y)*200L),scaledspeed,scaledaccel,scaleddecel);

  }
  else
  {
    prepare(0,((xpos-x)*200L),MAXSPEED,MAXACCEL,MAXDECEL);
    prepare(1,((ypos-y)*200L),MAXSPEED,MAXACCEL,MAXDECEL);
  }
  execute();
  x = xpos;
  y= ypos;
}

void loop()
{
  
  MoveTo(3,30);
  MoveTo(-3,-30);
  MoveTo(0,0);
  while(true);
}

//int photor1 = A1;
//int val1;
//bool laser1;

//val1 = analogRead(photor1);
//  laser1 = 270 > val1 ? true : false;
//
//  if(laser1)
//  {
//    MoveTo(0,0);
//    //Serial.println("detected");
//  }
//  if(! laser1)
//  {
//    MoveTo(-1,-1);
//    //Serial.println("not detected");
//  }


//  MoveTo(16,0);
//  MoveTo(16,4);
//  MoveTo(0,4);
//  MoveTo(0,8);
//  MoveTo(16,8);
//  MoveTo(16,12);
//  MoveTo(0,12);
//  MoveTo(0,16);
//  MoveTo(16,16);
//  MoveTo(16,0);
//  MoveTo(12,0);
//  MoveTo(12,16);
//  MoveTo(8,16);
//  MoveTo(8,0);
//  MoveTo(4,0);
//  MoveTo(4,16);
//  MoveTo(0,16);
//  MoveTo(0,0);
