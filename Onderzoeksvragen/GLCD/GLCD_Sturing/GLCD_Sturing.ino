#include <SPI.h>

#define PD0 3
#define PD1 2
#define PD3 1

// A0 -> PF0
// RESET -> PF1
// CS -> PF4
// BEEP -> PF5
// RO_1 -> PD0/SCL
// RO_2 -> PD1/SDA
// RO_Button -> PD3/TX

// Als 1 eerst laag is dan is het clockwise
// Functions voor wanneer er op de RO wordt gedrukt/gedraait
void RO_1();
void RO_2();
void RO_Button();

volatile boolean RO_1_Eerst, RO_2_Eerst, PixelsON = false;  // Variable om te bepalen welke uitgang als eerste laag werd en of alle pixels al aan zijn of niet
boolean Tester1, Tester2;   // Variable die bepalen wanneer er terug een interrupt mag zijn
long Teller = 0;    // Teller om te zien hoeveel keer je al hebt gedraait aan de RO


void setup() 
{
  attachInterrupt(digitalPinToInterrupt(PD0), RO_1, FALLING);        // Interrupt bij INT0 wanneer de pin van hoog naar laag gaat
  attachInterrupt(digitalPinToInterrupt(PD1), RO_2, FALLING);        // Interrupt bij INT1 wanneer de pin van hoog naar laag gaat
  attachInterrupt(digitalPinToInterrupt(PD3), RO_Button, FALLING);   // Interrupt bij INT3 wanneer de pin van hoog naar laag gaat

  Serial.begin(9600); 
  SPI.begin();
  
  PORTF &= 0b11111101;  // Reset laag
  delay(1);
  PORTF |= 0b00000010;  // Reset hoog
  delay(1);

  PORTF &= 0b11111110;  // A0 laag
  PORTF |= 0b00010000;  // CS hoog

  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
}

void loop() 
{
  if(RO_1_Eerst)
  {
    detachInterrupt(digitalPinToInterrupt(PD1));   // Stop interrupts aan RO_2
    Teller++;

    while((Tester1 == false) | (Tester2 == false))
    { 
      Tester1 = digitalRead(PD0);
      Tester2 = digitalRead(PD1);
    }

    RO_1_Eerst = false;
    RO_2_Eerst = false;  
    attachInterrupt(digitalPinToInterrupt(PD1), RO_2, FALLING);   // Start interrupts terug aan RO_2
  }
  
  if(RO_2_Eerst)
  {
    detachInterrupt(digitalPinToInterrupt(PD0));  // Stop interrupts aan RO_1
    Teller--; 

    while((Tester1 == false) | (Tester2 == false))
    { 
      Tester1 = digitalRead(PD0);
      Tester2 = digitalRead(PD1);
    }

    RO_1_Eerst = false;
    RO_2_Eerst = false;
    attachInterrupt(digitalPinToInterrupt(PD0), RO_1, FALLING);   // Start interrupts terug aan RO_1
  }

  Serial.println(Teller);
}

void RO_1()         // Interrupt functie wanneer 1 als eerste laag wordt
{
  RO_1_Eerst = true;
  RO_2_Eerst = false;
}

void RO_2()         // Interrupt functie wanneer 2 als eerste laag wordt
{
  RO_2_Eerst = true;
  RO_1_Eerst = false;
}

void RO_Button()    // Interrupt functie wanneer er op de knop wordt gedrukt
{
  PORTF |= 0b00100000;  // BEEP hoog
  delay(1);             // BEEP hoog houden voor 1ms
  PORTF &= 0b11011111;  // BEEP laag

  if(PixelsON == false)   // Zijn alle pixels uit
  {
    PORTF &= 0b11101111;        // Chip select laag
    SPI.transfer(0b10100101);   // Zet alle pixels aan
    PORTF |= 0b00010000;        // Chip select hoog
    PixelsON = true;
  }

  else    // Zijn alle pixels aan
  {
    PORTF &= 0b11101111;        // Chip select laag
    SPI.transfer(0b10100100);   // Zet alle pixels uit
    PORTF |= 0b00010000;        // Chip select hoog
    PixelsON = false;
  }
}
