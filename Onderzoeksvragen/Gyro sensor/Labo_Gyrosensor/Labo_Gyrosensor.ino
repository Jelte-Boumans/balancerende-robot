#include <Wire.h>                                                 // I2C lib

boolean Hold;                                                     // Variable om de metingen te laten stoppen en in hold mode gaan
float GyX, GyY, GyZ;                                              // Variable om de rauwe accelerometer data in op te slaan
float AcX, AcY, AcZ;                                              // Variable om de rauwe gyro sensor data in op te slaan
float Temp;                                                       // Variable om de rauwe temperatuur sensor data in op te slaan
int GyX_Int, GyY_Int, GyZ_Int;                                    // Variable om de omgerekende afgeronde accelerometer data in op de slaan
int AcX_Int, AcY_Int, AcZ_Int;                                    // Variable om de omgerekende afgeronde gyro sensor data in op de slaan
int Temp_Int;                                                     // Variable om de omgerekende afgeronde temperatuur sensor data in op de slaan
float GyX_Offset, GyY_Offset, GyZ_Offset;                         // Variable om de offset van de accelerometer data in op de slaan
float AcX_Offset, AcY_Offset, AcZ_Offset;                         // Variable om de offset van de gyro sensor data in op de slaan
int Plotter_Max, Plotter_Min;                                     // 

void Read_Data();                                                 // Function waar de 14 bytes worden ingelezen van de MPU6050

void setup() 
{
//  while(PINF & 0b10000000)                                        // Zolang PF7 hoog blijft doe dan niets
//  {
//    Serial.println("Haal rode schakelaar over om te starten.");   // Print instructies op de monitor
//  }

  Plotter_Max = 100;
  Plotter_Min = -100;
  
  Serial.begin(9600);                                             // Baut rate van de seriele monitor is 9600
  Wire.begin();                                                   // Start de I2C connectie
  
  // Start de MPU6050
  Wire.beginTransmission(0x68);                                   // Start de communicatie met de MPU-6050 (write)
  Wire.write(0x6B);                                               // Selecteer de index
  Wire.write(0);                                                  // Stuur data naar geselcteerde index
  Wire.endTransmission();                                         // Eindig de transmissie 
  
  // Zet de accelerometer zijn Full Scale Range op 8g (FS_SEL = 2) 
  Wire.beginTransmission(0x68);                                   // Start de communicatie met de MPU-6050 (write)
  Wire.write(0x1C);                                               // Selecteer de index
  Wire.write(0x10);                                               // Stuur data naar geselcteerde index
  Wire.endTransmission();                                         // Eindig de transmissie 
  
  // Zet de gyrosensor zijn Full Scale Range op 500 (FS_SEL = 1) 
  Wire.beginTransmission(0x68);                                   // Start de communicatie met de MPU-6050 (write)
  Wire.write(0x1B);                                               // Selecteer de index
  Wire.write(0x08);                                               // Stuur data naar geselcteerde index
  Wire.endTransmission();                                         // Eindig de transmissie

  // Berekend offset
  for (int Repeat = 0; Repeat < 2000; Repeat ++)                  // Lees 2000 keer de 14 bytes in in
  {               
    Read_Data();                                                  // Lees de 14 bytes in
    
    GyX_Offset += GyX;                                            // Tel de gemete waarde van GyX elke keer bij elkaar op / integreer de data
    GyY_Offset += GyY;                                            // Tel de gemete waarde van GyY elke keer bij elkaar op / integreer de data
    GyZ_Offset += GyZ;                                            // Tel de gemete waarde van GyZ elke keer bij elkaar op / integreer de data

    AcX_Offset += AcX;                                            // Tel de gemete waarde van AcX elke keer bij elkaar op / integreer de data
    AcY_Offset += AcY;                                            // Tel de gemete waarde van AcY elke keer bij elkaar op / integreer de data
    AcZ_Offset += AcZ;                                            // Tel de gemete waarde van AcZ elke keer bij elkaar op / integreer de data

    Serial.println("BEREKENEN, MPU NIET BEWEGEN!");               // Print instructies op monitor
  }
  
  GyX_Offset /= 2000;                                             // Deel de totale GyX_Offset waarde door 2000 voor het gemiddelde te krijgen
  GyY_Offset /= 2000;                                             // Deel de totale GyY_Offset waarde door 2000 voor het gemiddelde te krijgen
  GyZ_Offset /= 2000;                                             // Deel de totale GyZ_Offset waarde door 2000 voor het gemiddelde te krijgen
  
  AcX_Offset /= 2000;                                             // Deel de totale AcX_Offset waarde door 2000 voor het gemiddelde te krijgen
  AcY_Offset /= 2000;                                             // Deel de totale AcY_Offset waarde door 2000 voor het gemiddelde te krijgen
  AcZ_Offset /= 2000;                                             // Deel de totale AcZ_Offset waarde door 2000 voor het gemiddelde te krijgen
}


void Read_Data()
{
  Wire.beginTransmission(0x68);                                   // Start de communicatie met de MPU-6050 (write)
  Wire.write(0x3B);                                               // Selecteer de index (0x3B is waar de eerste van de 14 bytes in staat)
  Wire.endTransmission();                                         // Eindig de transmissie 
  Wire.requestFrom(0x68, 14);                                     // Vraag de 14 opvolgende bytes van de MPU6050, 14 want 6 van de gyro, 6 van de accelero en 1 voor temp (read) (van 0x3B tot 0x48)
  
  while(Wire.available() < 14); {}                                // Wachten tot alle 14 bytes beschikbaar zijn

  // Inlezen
  AcX = -(Wire.read() << 8 | Wire.read());                        // Eerst worden de 8 LSB ingelezen en daarna de 8 MSB, eerst worden de LSBs toegevoegt en dan de MSBs die 8 plaatsen naar links zijn geschift
  AcY = -(Wire.read() << 8 | Wire.read());                        // Eerst worden de 8 LSB ingelezen en daarna de 8 MSB, eerst worden de LSBs toegevoegt en dan de MSBs die 8 plaatsen naar links zijn geschift
  AcZ = -(Wire.read() << 8 | Wire.read());                        // Eerst worden de 8 LSB ingelezen en daarna de 8 MSB, eerst worden de LSBs toegevoegt en dan de MSBs die 8 plaatsen naar links zijn geschift
  
  Temp = Wire.read() << 8 | Wire.read();                          // Eerst worden de 8 LSB ingelezen en daarna de 8 MSB, eerst worden de LSBs toegevoegt en dan de MSBs die 8 plaatsen naar links zijn geschift
  
  GyY = -(Wire.read() << 8 | Wire.read());                        // Eerst worden de 8 LSB ingelezen en daarna de 8 MSB, eerst worden de LSBs toegevoegt en dan de MSBs die 8 plaatsen naar links zijn geschift
  GyX = Wire.read() << 8 | Wire.read();                           // Eerst worden de 8 LSB ingelezen en daarna de 8 MSB, eerst worden de LSBs toegevoegt en dan de MSBs die 8 plaatsen naar links zijn geschift
  GyZ = Wire.read() << 8 | Wire.read();                           // Eerst worden de 8 LSB ingelezen en daarna de 8 MSB, eerst worden de LSBs toegevoegt en dan de MSBs die 8 plaatsen naar links zijn geschift 
}



void loop() 
{
  Read_Data();                                                    // Lees de 14 bytes in
  
  // Offset aftrekken
  GyX -= GyX_Offset;                                              // Trek de offset af van de gemete waarde voor een meer accurate gemete waarde
  GyY -= GyY_Offset;                                              // Trek de offset af van de gemete waarde voor een meer accurate gemete waarde
  GyZ -= GyZ_Offset;                                              // Trek de offset af van de gemete waarde voor een meer accurate gemete waarde

  AcX -= AcX_Offset;                                              // Trek de offset af van de gemete waarde voor een meer accurate gemete waarde
  AcY -= AcY_Offset;                                              // Trek de offset af van de gemete waarde voor een meer accurate gemete waarde
  AcZ -= AcZ_Offset;                                              // Trek de offset af van de gemete waarde voor een meer accurate gemete waarde

  // Berekeningen
  GyX /= 65.5;                                                    // Dit zet de output om naar °/s want als de sensor draait aan 1°/S dan zal de output waarde 65.5 zijn (datasheet p12)
  GyY /= 65.5;                                                    // Dit zet de output om naar °/s want als de sensor draait aan 1°/S dan zal de output waarde 65.5 zijn (datasheet p12)
  GyZ /= 65.5;                                                    // Dit zet de output om naar °/s want als de sensor draait aan 1°/S dan zal de output waarde 65.5 zijn (datasheet p12)
  

  AcX /= 45.51111;                                                // Dit zet de output om naar ° want als de sensor 90°zou zijn gekanteld dan zal de output waarde 4096 zijn(4096/90 = 45.51) (datasheet p13)
  AcY /= 45.51111;                                                // Dit zet de output om naar ° want als de sensor 90°zou zijn gekanteld dan zal de output waarde 4096 zijn(4096/90 = 45.51) (datasheet p13)
  AcZ /= 45.51111;                                                // Dit zet de output om naar ° want als de sensor 90°zou zijn gekanteld dan zal de output waarde 4096 zijn(4096/90 = 45.51) (datasheet p13)
  
  Temp /= 340;                                                    // Dit zet de output om naar °C
  Temp += 36.53;                                                  // Dit zet de output om naar °C

  // Afronden
  AcX_Int  = (int)AcX;                                            // Zet AcX om naar een int zodat het wordt afgerond tot een eenheid zonder kommagetallen
  AcY_Int = (int)AcY;                                             // Zet AcY om naar een int zodat het wordt afgerond tot een eenheid zonder kommagetallen
  AcZ_Int = (int)AcZ;                                             // Zet AcZ om naar een int zodat het wordt afgerond tot een eenheid zonder kommagetallen
  Temp_Int = (int)Temp;                                           // Zet Temp om naar een int zodat het wordt afgerond tot een eenheid zonder kommagetallen
  GyX_Int = (int)GyX;                                             // Zet GyX om naar een int zodat het wordt afgerond tot een eenheid zonder kommagetallen
  GyY_Int = (int)GyY;                                             // Zet GyY om naar een int zodat het wordt afgerond tot een eenheid zonder kommagetallen
  GyZ_Int = (int)GyZ;                                             // Zet GyZ om naar een int zodat het wordt afgerond tot een eenheid zonder kommagetallen

  // Data printen
  Serial.print(Plotter_Max);
  Serial.print(",");
  Serial.print(Plotter_Min);
  Serial.print(",");
//  Serial.print("AcX = ");
  Serial.print(AcX_Int);
  Serial.print(",");
//  Serial.print("AcY = ");
 Serial.print(AcY_Int);
 Serial.print(",");
//  Serial.print("AcZ = ");
  Serial.println(AcZ_Int);
 // Serial.println("    ");

//  Serial.print("Temp = "); 
//  Serial.print(Temp_Int);
//  Serial.print("     ");

//  Serial.print("GyX = ");
//  Serial.print(GyX_Int);
//  Serial.print("  ");
//  Serial.print("GyY = ");
 // Serial.print(GyY_Int);
 // Serial.print("  ");
//  Serial.print("GyZ = ");
//  Serial.println(GyZ_Int);

  if(PINF & 0b00000001)                                           // Als je drukt op de drukknop
  {
    delay(250);                                                   // Delay voor niet direct in nieuwe loop te springen
    Hold = HIGH;                                                  // Maak de Hold varible hoog
    Serial.println("Hold");                                       // Print hold op de seriele monitor
    
    while(Hold == 1)                                              // Zolang Hold hoog is
    {
      if(PINF & 0b00000001)                                       // En als je dan terug op de drukknop drukt
      {
        Hold = LOW;                                               // Maak dan Hold laag
      }
    }
    
    delay(250);                                                   // Delay voor niet direct in nieuwe loop te springen
  }
} 
