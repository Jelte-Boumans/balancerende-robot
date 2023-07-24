unsigned char Ucos = 0; // Variable die de spanning van het cosinus signaal weergeeft

#define ST_Input 6  // ADC6
#define ST_Output 20  

void setup()
{
  DDRF &= 0b10111111; // Maak PF6 laag (input)  
  DDRF |= 0b10100000; // Maak PF7 en PF5 hoog (output)
  Serial.begin(9600);

  unsigned char Schmitt_Trigger(unsigned char LowH, unsigned char HighH); // LowH = lage hysteresis en HighH = hoge hysteresis
  unsigned int READ_ADC_INT_CHANNEL( unsigned char channel );   // Prototype functie voor ADC naar een INT
}

void loop()
{
  Ucos = map(READ_ADC_INT_CHANNEL(ST_Input), 0, 1023, 0, 100);  // Met een waarde van 0 tot 100 kan ik gemakkelijk de hysteresis berekenen met procent
  
  Schmitt_Trigger(45,55); // Zet de hysteresis op 45% en 55% van 5V

  if(digitalRead(21))
  {
     PORTF |= 0b00100000;  // Maak alleen PF5 hoog
     _delay_ms(5);
     PORTF &= 0b11011111;  // Maak alleen PF5 laag
     _delay_ms(5);
  }
  
  else
  {
     PORTF |= 0b00100000;  // Maak alleen PF5 hoog
     _delay_ms(1);
     PORTF &= 0b11011111;  // Maak alleen PF5 laag
     _delay_ms(1);   
  }

  Serial.print(Ucos);
  Serial.print(" ");
  Serial.println(digitalRead(18));
}

unsigned char Schmitt_Trigger(unsigned char LowH, unsigned char HighH)
{
  if(Ucos <= LowH)  // 45% van 5V is 2.25 en dat is de lage hysteresis die ik wil
  {
    PORTF &= 0b01111111;   // Als de spanning van de RE gelijk ligt of onder 2.25V ligt dan word de output van de ST laag
  }

  if(Ucos >= HighH)  // 55% van 5V is 2.75 en dat is de lage hysteresis die ik wil
  {
    PORTF |= 0b10000000;  // Als de spanning van de RE gelijk ligt of boven 2.75V ligt dan word de output van de ST hoog 
  }
}

unsigned int READ_ADC_INT_CHANNEL( unsigned char channel )
{
  DDRF &= ~(1<<channel); // config selected channel as input
  
  // Right adjust + choice of channel
  ADMUX = 0b01000000 +  channel;
  
  // Activate ADC - Stop conversion - prescaler 128 (for ADC 50K<..<200k) (16Mhz / 128 = 125000)
  ADCSRA = 0b10000111;
  
  // start conversion and wait for completion
  ADCSRA |= (1<<ADSC);
  while (ADCSRA & (1<<ADSC));
  
  // return A/D conversion result
  unsigned char ADCLBuff = ADCL; // first read ADCL!!
  return ( ((unsigned int) ADCH ) << 8 ) + ADCLBuff;
}
