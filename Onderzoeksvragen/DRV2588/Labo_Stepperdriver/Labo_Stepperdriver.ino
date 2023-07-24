unsigned int ADC0;       // Variable waar de ADC waarde in wordt opgeslagen

void setup() 
{
  DDRF |= 0b00110000;    // Maak PF5 en PF4 outputs en verander de andere pinnen niet (door het masker)
    
  Serial.begin(9600);    // Bautrate van de seriele monitor zetten op 9600
  
  unsigned int READ_ADC_INT_CHANNEL( unsigned char channel );   // Prototype functie voor ADC naar een INT
}

void loop() 
{
  ADC0 = READ_ADC_INT_CHANNEL(0);     // Lees de potmeter waarde op PF0 in en slaag het op in de variable ADC0
  ADC0 = map(ADC0, 0, 1023, 1, 1005);  // De minimum waar de van de ADC meting is ipv 0 nu 1 en de maximum waarde is 1005 ipv 1023 
  
  Serial.println(ADC0);               // De potmeter waarde en de stand van de schakelaar op de seriele monitor weergeven
  Serial.print("\t");
  Serial.println(digitalRead(22));  
  
  if(ADC0 <= 1000)                     // Laat de motor alleen draaien als er al een klein beetje aan de potmeter is gedraait 
  {
    PORTF ^= 0b00100000;              // Maak pulsen waarvan je de frequentie van kan aanpassen met de potmeter
    delay(ADC0);                      // Delay wordt bepaald door de ADC meting
  }

  if(PINF & 0b00000010)               // Als PF1 hoog is/als de schakelaar is overgehaald
  {
    PORTF |= 0b00010000;              // Maak alleen PF4 hoog
  }
  
  else
  {
    PORTF &= 0b11101111;              // Maak alleen PF4 laag
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
