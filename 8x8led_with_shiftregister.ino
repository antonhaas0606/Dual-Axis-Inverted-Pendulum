int latchpin = 4;
int clockpin = 5;
int datapin = 3;

int rows[8] = {6, 7, 8, 9, 10, 11, 12, 13};

byte A[8] = {B00000000,B00111100,B01100110,B01100110,B01111110,B01100110,B01100110,B01100110};
byte N[8] = {B00000000,B00100010,B00110010,B00101010,B00100110,B00100010,B00000000,B00000000};
byte T[8] = {B00000000,B01111100,B00010000,B00010000,B00010000,B00010000,B00010000,B00000000};
byte O[8] = {B00000000,B00111100,B01000010,B01000010,B01000010,B01000010,B00111100,B00000000};
byte smiley[8] = {0x3C,0x42,0xA5,0x81,0xA5,0x99,0x42,0x3C};

void setup() {
  pinMode(latchpin, OUTPUT);
  pinMode(clockpin, OUTPUT);
  pinMode(datapin, OUTPUT);

  for(int i = 0; i < 8; i++)
  {
    pinMode(rows[i], OUTPUT);
  }
}

void dispreset ()
{
  for(int i = 0; i < 8; i++)
  {
    digitalWrite(rows[i], LOW);
  }
  digitalWrite(latchpin, HIGH);
}

void scroll (byte k[])
{
  
  byte x[8];
  for(int i = 0; i < 8; i++)      
  {
    x[i] = 0xff-k[i];
  }
  
  for(int j = 7; 0 < j; j--)      
  {
    for(int i = 0; i < 8; i++)
     {
       k[i] = x[i];
     }
    for(int g = 0; g < 8; g++)
    {
      for(int i = 0; i < j; i++)
      {
        k[g] = k[g]/2 + 128;
      }
    }

    for (int i = 0; i < 100; i++)
    {
      
      for(int l = 0; l < 8; l++)
      {
        shiftOut(datapin, clockpin, LSBFIRST, k[l]);
        dispreset();
        digitalWrite(rows[l], HIGH);
        digitalWrite(latchpin, LOW);
      }
    }
  }

  for(int i = 0; i < 8; i++)
  {
    k[i] = x[i];
  }
  for(int i = 0; i < 7; i++)
  {
    
    for (int o = 0; o < 100; o++)
    {
      for(int l = 0; l < 8; l++)
      {
        shiftOut(datapin, clockpin, LSBFIRST, k[l]);
        dispreset();
        digitalWrite(rows[l], HIGH);
        digitalWrite(latchpin, LOW);
      }
    }
    for(int g = 0; g < 8; g++)
    {
      k[g] = k[g]*2 + 1;
    }
    
  }
  for(int i = 0; i < 8; i++)
  {
    k[i] = 0xff - x[i];
  }
}


void loop() {
  scroll(A);
  scroll(N);
  scroll(T);
  scroll(O);
  scroll(N);
  scroll(smiley);

}
