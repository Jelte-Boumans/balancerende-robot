// B-ROBOT  SELF BALANCE ARDUINO ROBOT WITH STEPPER MOTORS
// (Brainbox Arduino + Homemade shield v2 + STEPPER MOTOR drivers + MPU-6050)
// You need to install libraries I2Cdev and MPU6050 (from Github repository)
// Author: JJROBOTS.COM (Jose Julio & Juan Pedro)
// Updated: 20/05/2016
// Version: 2.0
// Remember to update the libraries

// The board needs at least 10-15 seconds with no motion (robot steady) at beginning to give good values...
// MPU6050 IMU using internal DMP processor. Connected via I2C bus
// Angle calculations and control part is running at 200Hz from DMP solution
// DMP is using the gyro_bias_no_motion correction method.

// The robot is OFF when the angle is high (robot is horizontal). When you start raising the robot it
// automatically switch ON and start a RAISE UP procedure.
// To switch OFF the robot you could manually put the robot down on the floor (horizontal)

// We use a standard PID controllers (Proportional, Integral derivative controller) for robot stability
// We have a PI controller for speed control and a PD controller for stability (robot angle)
// The output of the control (motors speed) is integrated so it´s really an acceleration not an speed.

// controls:
//    Throttle (0.0-1.0)
//    Steering (0.0-1.0)
//    toggle1: Enable PRO mode. On PRO mode steering and throttle are more aggressive

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include <I2Cdev.h> // I2Cdev lib from www.i2cdevlib.com
#include <avr/interrupt.h>
#include <JJ_MPU6050_DMP_6Axis.h> // Modified version of the MPU6050 library to work with DMP (see comments inside)
#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((( F_CPU / 16) + ( USART_BAUDRATE / 2) ) / (USART_BAUDRATE )) - 1)
// This version optimize the FIFO (only contains quaternion) and minimize code size

// NORMAL MODE PARAMETERS (MAXIMUM SETTINGS)
#define MAX_THROTTLE 175
#define MAX_STEERING 150
#define MAX_TARGET_ANGLE 12

// PRO MODE = MORE AGGRESSIVE (MAXIMUM SETTINGS)
#define MAX_THROTTLE_PRO 980 //680
#define MAX_STEERING_PRO 250
#define MAX_TARGET_ANGLE_PRO 40 //20

// Default control terms
#define KP 0.16 // 0.16
#define KD 68 // 63
#define KP_THROTTLE 0.07 // 0.07
#define KI_THROTTLE 0.04 // 0.04

// Control gains for raiseup (the raiseup movement requiere special control parameters)
#define KP_RAISEUP 0.16
//#define KD_RAISEUP 68
#define KP_THROTTLE_RAISEUP 0 // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define MAX_CONTROL_OUTPUT 500

// Servo definitions
#define SERVO_AUX_NEUTRO 1550 // Servo neutral position
#define SERVO_MIN_PULSEWIDTH 650
#define SERVO_MAX_PULSEWIDTH 2600

#define DEBUG 0 // 0 = No debug info (default)

#define ZERO_SPEED 65535
#define MAX_ACCEL 8 // Maximun motor acceleration (MAX RECOMMENDED VALUE: 8) (default:7)

#define MICROSTEPPING 16 // 8 or 16 for 1/8 or 1/16 driver microstepping (default:16)

#define I2C_SPEED 400000L // 400kHz I2C speed

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define ITERM_MAX_ERROR 25 // Iterm windup constants for PI control //40
#define ITERM_MAX 8000 // 5000

// Pinout of the GLCD
#define RE2 2 // Rotary encoder output 2
#define RE1 3 // Rotary encoder output 1
#define RE_S 18 // Rotary encoder switch
#define MISO 50 // SPI Master in Slave out pin
#define MOSI 51 // SPI Master out Slave in pin
#define SCK 52  // SPI Serial Clock pin
#define CS 54 // GLCD driver Chip Select pin
#define BEEP 55 // Buzzer pin
#define RESET 56 // GLCD driver reset pin
#define A0 57 // GLCD data control pin

#define S1 14
#define S2 15
#define S3 16

U8G2_ST7565_JLX12864_F_4W_HW_SPI u8g2(U8G2_R2, /* cs=*/ 54, /* a0=*/ 57, /* reset=*/ 56);


// MPU control/status vars
boolean dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (for us 18 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[18]; // FIFO storage buffer
Quaternion q;

long timer_old;
long timer_value;
int debug_counter;
float debugVariable;
float dt;

// class default I2C address is 0x68 for MPU6050
MPU6050 mpu;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;
int angle_adjusted_int;

// Default control values from constant definitions
float Kp = KP;
float Kd = KD;
float Kp_thr = KP_THROTTLE;
float Ki_thr = KI_THROTTLE;
float Kp_user = KP;
//float Kd_user = KD;
float Kp_thr_user = KP_THROTTLE;
float Ki_thr_user = KI_THROTTLE;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
float throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;

uint8_t mode;  // mode = 0 Normal mode, mode = 1 Pro mode (More agressive)

int16_t motor1;
int16_t motor2;

int16_t speed_M1, speed_M2;        // Actual speed of motors
int8_t  dir_M1, dir_M2;            // Actual direction of steppers motors
int16_t actual_robot_speed;        // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;
float estimated_speed_filtered;    // Estimated robot speed

float throttleHC = 0.5;
float steeringHC = 0.5;

boolean Calibration = 0;

boolean Scroll = 0;
unsigned char Pagina = 0;
unsigned char Snelheid = 50;
unsigned char SnelheidXas = 25;
unsigned char Gevoeligheid = 50;
unsigned char GevoeligheidXas = 25;

unsigned char IntroPagina = 1;

// DMP FUNCTIONS
// This function defines the weight of the accel on the sensor fusion
// default value is 0x80
// The official invensense name is inv_key_0_96 (??)
void dmpSetSensorFusionAccelGain(uint8_t gain)
{
  // INV_KEY_0_96
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(gain);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
}



// Quick calculation to obtein Phi angle from quaternion solution (from DMP internal quaternion solution)
float dmpGetPhi() {
  mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.resetFIFO();  // We always reset FIFO

  //return( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll
  //return Phi angle (robot orientation) from quaternion DMP output
  return (atan2(2 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * RAD2GRAD);
}



// PD controller implementation(Proportional, derivative). DT is in miliseconds
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, int Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  output = Kp * error + (Kd * (setPoint - setPointOld) - Kd * (input - PID_errorOld2)) / DT;
  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}

// PI controller implementation (Proportional, integral). DT is in miliseconds
float speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki)
{
  float error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  output = Kp * error + Ki * PID_errorSum * DT * 0.001; // DT is in miliseconds...
  return (output);
}



// 16 single cycle instructions = 1us at 16Mhz
void delay_1us()
{
  __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
}



// TIMER 1 : STEPPER MOTOR1 SPEED CONTROL
ISR(TIMER1_COMPA_vect)
{
  if (dir_M1 == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  digitalWrite(11, HIGH); // STEP MOTOR 1
  delay_1us();
  digitalWrite(11, LOW);
}
// TIMER 3 : STEPPER MOTOR2 SPEED CONTROL
ISR(TIMER3_COMPA_vect)
{
  if (dir_M2 == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  digitalWrite(7, HIGH); // STEP MOTOR 2
  delay_1us();
  digitalWrite(7, LOW);
}



/* interrupt routine for Bluetooth is called when the first byte is
received, We know that the second byte will follow imediately after the
first one so we read this second byte in the interrupt routine */
ISR (USART0_RX_vect) // interrupt routine that executes every time a new byte is received
{
 UCSR0B &= ~(1 << RXCIE0); // disable RX interrupt into the variable " ByteReceived " - the RXint flaq is automatically cleared now
 throttleHC = UDR0; // Fetch the received byte value
 throttleHC = throttleHC/255; // Constrain throttleHC between 0 - 0.5
 while ((UCSR0A & (1 << RXC0)) == 0) {}; // wait until byte 2 is received

 // Do nothing until data have been received and is ready to be read from UDR
 steeringHC = UDR0; // receive byte 2
 steeringHC = steeringHC/255; // Constrain steeringHC between 0 - 0.5
 UCSR0B |= (1 << RXCIE0); //Re-enable RX interrupt
}



// Set speed of Stepper Motor1
// tspeed could be positive or negative (reverse)
void setMotorSpeedM1(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M1 * 46; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M1 * 23; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M1 = 1;
    digitalWrite(12, LOW); // DIR Motor 1 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M1 = -1;
    digitalWrite(12, HIGH); // Dir Motor 1
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  OCR1A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT1 > OCR1A)
    TCNT1 = 0;
}

// Set speed of Stepper Motor2
// tspeed could be positive or negative (reverse)
void setMotorSpeedM2(int16_t tspeed)
{
  long timer_period;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;

#if MICROSTEPPING==16
  speed = speed_M2 * 46; // Adjust factor from control output speed to real motor speed in steps/second
#else
  speed = speed_M2 * 23; // 1/8 Microstepping
#endif

  if (speed == 0)
  {
    timer_period = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period = 2000000 / speed; // 2Mhz timer
    dir_M2 = 1;
    digitalWrite(8, HIGH);   // Dir Motor2 (Forward)
  }
  else
  {
    timer_period = 2000000 / -speed;
    dir_M2 = -1;       
    digitalWrite(8, LOW);  // DIR Motor 2
  }
  if (timer_period > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period = ZERO_SPEED;

  OCR3A = timer_period;
  // Check  if we need to reset the timer...
  if (TCNT3 > OCR3A)
    TCNT3 = 0;
}



void Beep(char keuze)
{
  if(keuze == 1)
  {
    digitalWrite(BEEP, HIGH); // Beep when robot is ready
    delay(50);
    digitalWrite(BEEP, LOW);
    delay(50);
    digitalWrite(BEEP, HIGH);
    delay(50);
    digitalWrite(BEEP, LOW);
    delay(50);
    digitalWrite(BEEP, HIGH);
    delay(50);
    digitalWrite(BEEP, LOW);
    delay(50);
  }

  if(keuze == 2)
  {
    digitalWrite(BEEP, HIGH);
    delay(50);
    digitalWrite(BEEP, LOW);
  }
}

// INITIALIZATION
void setup()
{
  // STEPPER PINS ON JJROBOTS BROBOT BRAIN BOARD
  pinMode(4, OUTPUT); // ENABLE MOTORS
  pinMode(7, OUTPUT); // STEP MOTOR 1
  pinMode(8, OUTPUT); // DIR MOTOR 1
  pinMode(11, OUTPUT); // STEP MOTOR 2
  pinMode(12, OUTPUT); // DIR MOTOR 2
  pinMode(S1, INPUT); // S1 (Black)
  pinMode(S2, INPUT); // S2 (Red)
  pinMode(S3, INPUT); // S3 (Black)
  pinMode(55, OUTPUT); // BEEP
  digitalWrite(4, HIGH);  // Disbale motors
  digitalWrite(55, LOW);  // Disbale buzzer

  //initialize RS232 comms on TX & RX pin to communicate with HC06 Bluetooth module
  UCSR0B = (1 << RXEN0 ) | (1 << TXEN0 ); // Enable uart1 for transmit and receive 
  UCSR0C = (1 << UCSZ10 ) | (1 << UCSZ11 );
  UBRR0H = (BAUD_PRESCALE >> 8);  // Set baud rate register
  UBRR0L = BAUD_PRESCALE; // Set baud rate register 
  UCSR0B |= (1 << RXCIE0);  // Enable interrupt when receive byte is complete
  sei();  // Enable global interrupts
  
  // Initialize I2C bus (MPU6050 is connected via I2C)
  Wire.begin();
  // I2C 400Khz fast mode
  TWSR = 0;
  TWBR = ((16000000L / I2C_SPEED) - 16) / 2;
  TWCR = 1 << TWEN;

  u8g2.begin();
  u8g2.setContrast(200);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(10,25,"Balancerende Robot");
  u8g2.drawStr(25,45,"Jelte Boumans");
  u8g2.sendBuffer();
  delay(3000);
  
  while(IntroPagina != 6)
  {
    if((digitalRead(S1)) && (IntroPagina != 1))
    {
      IntroPagina--; // Previous page
      Beep(2);
      delay(150);
    }
    
    if(digitalRead(S2))
    {
      IntroPagina = 6;  // Skip
      Beep(2); 
      delay(150);
    }
    
    if(digitalRead(S3)) 
    {
      IntroPagina++;  // Next page
      Beep(2);
      delay(150);
    }

    if(IntroPagina < 6)
    {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_unifont_t_symbols);
      u8g2.drawGlyph(1,10, 0x2190);
      u8g2.drawGlyph(119,10, 0x2192);
      u8g2.setFont(u8g2_font_profont11_tr);
      u8g2.drawStr(53,8,"SKIP");
    }
      
    switch(IntroPagina)
    {
      case 1:
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(5,22,"Laat eerst de robot op");
        u8g2.drawStr(7,32,"zijn rug liggen tot je");
        u8g2.drawStr(5,42,"de robot 3 keer achter");
        u8g2.drawStr(9,52,"elkaar hoort biepen.");
        u8g2.setFont(u8g2_font_profont11_tr);
        u8g2.drawStr(119,62,"1");
      break;

      case 2:
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(5,22,"Als dit langer dan 15s");
        u8g2.drawStr(7,32,"duurt, reset de robot.");
        u8g2.drawStr(16,42,"Na de biep zit je");
        u8g2.drawStr(13,52,"in de instellingen.");
        u8g2.setFont(u8g2_font_profont11_tr);
        u8g2.drawStr(119,62,"2");
      break;

      case 3:
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(16,22,"Daarna kan je de");
        u8g2.drawStr(12,32,"motoren aanzetten,");
        u8g2.drawStr(12,42,"en de robot rechop");
        u8g2.drawStr(11,52,"zetten om te rijden.");
        u8g2.setFont(u8g2_font_profont11_tr);
        u8g2.drawStr(119,62,"3");
      break;

      case 4:
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(7,22,"Je hebt ook de keuze");
        u8g2.drawStr(10,32,"om de robot met de");
        u8g2.drawStr(5,42,"app te besturen via de");
        u8g2.drawStr(12,53,"bluetooth module.");
        u8g2.setFont(u8g2_font_profont11_tr);
        u8g2.drawStr(119,62,"4");
      break;

      case 5:
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(15,22,"Tenslotte als je");
        u8g2.drawStr(15,32,"de robot terug op");
        u8g2.drawStr(18,42,"zijn rug legt, zie");
        u8g2.drawStr(13,52,"je de instellingen.");
        u8g2.setFont(u8g2_font_profont11_tr);
        u8g2.drawStr(119,62,"5");
      break;

      case 6:
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_ncenB08_tr);
        u8g2.drawStr(15,35,"Wacht even AUB...");
        u8g2.sendBuffer();
      break;
    }
    
    u8g2.sendBuffer();
  }

  #if DEBUG > 0
    delay(9000);
  #else
    delay(2000);
  #endif

  //mpu.initialize();
  // Manual MPU initialization... accel=2G, gyro=2000º/s, filter=20Hz BW, output=200Hz
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_10);  //10,20,42,98,188  // Default factor for BROBOT:10
  mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);

  delay(500);
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
  }
  else { // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
  }
  
  timer_old = millis();

  // STEPPER MOTORS INITIALIZATION
  // MOTOR1 => TIMER1
  TCCR1A = 0;                       // Timer1 CTC mode 4, OCxA,B outputs disconnected
  TCCR1B = (1 << WGM12) | (1 << CS11); // Prescaler=8, => 2Mhz
  OCR1A = ZERO_SPEED;               // Motor stopped
  dir_M1 = 0;
  TCNT1 = 0;

  // MOTOR2 => TIMER3
  TCCR3A = 0;                       // Timer3 CTC mode 4, OCxA,B outputs disconnected
  TCCR3B = (1 << WGM32) | (1 << CS31); // Prescaler=8, => 2Mhz
  OCR3A = ZERO_SPEED;   // Motor stopped
  dir_M2 = 0;
  TCNT3 = 0;

  //Adjust sensor fusion gain
  dmpSetSensorFusionAccelGain(0x20);

  delay(200);

  // Enable stepper drivers and TIMER interrupts
  digitalWrite(4, LOW);   // Enable stepper drivers
  // Enable TIMERs interrupts
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
  TIMSK3 |= (1 << OCIE1A); // Enable Timer1 interrupt

  mpu.resetFIFO();
  timer_old = millis();
  mode = 0;
}



// MAIN LOOP
void loop()
{  
  timer_value = millis();
  
  throttle = (throttleHC - 0.5) * max_throttle;  //eerste 0.5 moet throttle data van HC-06 zijn
  // We add some exponential on steering to smooth the center band
  steering = steeringHC - 0.5;                  //eerste 0.5 moet steering data van HC-06 zijn
  if (steering > 0)
  {
    steering = (steering * steering + 0.5 * steering) * max_steering;
  }
  else
  {
    steering = (-steering * steering + 0.5 * steering) * max_steering;
  }

  // New DMP Orientation solution?
  fifoCount = mpu.getFIFOCount();
  if (fifoCount >= 18)
  {
    if (fifoCount > 18) // If we have more than one packet we take the easy path: discard the buffer and wait for the next one
    {
      mpu.resetFIFO();
      return;
    }
    dt = (timer_value - timer_old);
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)
    angle_adjusted = dmpGetPhi();
    
    mpu.resetFIFO();  // We always reset FIFO

    if((angle_adjusted <= -89) && (angle_adjusted >= -92)) // If the gyro has finished calibrating and the program has been running for a while (this would often false trigger without the millis())
    {
      if((Calibration == false) && (millis() >= 13000))
      {
        Calibration = true;
        u8g2.clearBuffer();
        u8g2.sendBuffer();
        Beep(1);
      }
    }
    
    // We calculate the estimated robot speed:
    // Estimated_Speed = angular_velocity_of_stepper_motors(combined) - angular_velocity_of_robot(angle measured by IMU)
    actual_robot_speed_Old = actual_robot_speed;
    actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward

    int16_t angular_velocity = (angle_adjusted - angle_adjusted_Old) * 90.0; // 90 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = -actual_robot_speed_Old - angular_velocity;     // We use robot_speed(t-1) or (t-2) to compensate the delay
    estimated_speed_filtered = estimated_speed_filtered * 0.95 + (float)estimated_speed * 0.05;  // low pass filter on estimated speed

    // SPEED CONTROL: This is a PI controller.
    //    input:user throttle, variable: estimated robot speed, output: target robot angle to get the desired speed
    target_angle = speedPIControl(dt, estimated_speed_filtered, throttle, Kp_thr, Ki_thr);
    target_angle = constrain(target_angle, -max_target_angle, max_target_angle); // limited output

    // Stability control: This is a PD controller.
    //    input: robot target angle(from SPEED CONTROL), variable: robot angle, output: Motor speed
    //    We integrate the output (sumatory), so the output is really the motor acceleration, not motor speed.
    control_output += stabilityPDControl(dt, angle_adjusted, target_angle, Kp, Kd);
    control_output = constrain(control_output, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT); // Limit max output from control

    // The steering part from the user is injected directly on the output
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);
    motor2 = constrain(motor2, -MAX_CONTROL_OUTPUT, MAX_CONTROL_OUTPUT);

    // NOW we send the commands to the motors
    if ((angle_adjusted < 76) && (angle_adjusted > -76)) // Is robot ready (upright?)
    {
      // NORMAL MODE
      digitalWrite(4, LOW);  // Motors enable
      setMotorSpeedM1(motor1);
      setMotorSpeedM2(motor2);
      
      // Normal condition?
      if ((angle_adjusted < 45) && (angle_adjusted > -45))
      {
        Kp = Kp_user;            // Default user control gains
       // Kd = Kd_user;
        Kp_thr = Kp_thr_user;
        Ki_thr = Ki_thr_user;
      }
      else    // We are in the raise up procedure => we use special control parameters
      {
        Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
      //  Kd = KD_RAISEUP;
        Kp_thr = KP_THROTTLE_RAISEUP;
        Ki_thr = KI_THROTTLE_RAISEUP;
      }
    }
    else   // Robot not ready (flat), angle > 70º => ROBOT OFF
    {
      digitalWrite(4, HIGH);  // Disable motors
      setMotorSpeedM1(0);
      setMotorSpeedM2(0);
      
      PID_errorSum = 0;  // Reset PID I term
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
    //  Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      if(Calibration) Menu();
    }
    
  } // End of new IMU data
}



void Menu()
{
  if(Pagina == 0)
  {
    u8g2.clearBuffer();
    PaginaInstel();
    u8g2.setFont(u8g2_font_unifont_t_symbols);
    if(Scroll == 0) u8g2.drawGlyph(15,36, 0x2192);
    if(Scroll == 1) u8g2.drawGlyph(15,60, 0x2192);
    u8g2.sendBuffer();

    if((digitalRead(S1) == 1) || (digitalRead(S3) == 1)) 
    {
      Scroll = !Scroll;
      Beep(2);
      delay(300);
    }
    
    if((digitalRead(S2) == 1) && (Scroll == 0)) 
    {
      Pagina = 1;
      Scroll = 0;
      Beep(2);
      delay(300);
    }
    
    if((digitalRead(S2) == 1) && (Scroll == 1))
    {
      Pagina = 2;
      Scroll = 0;
      Beep(2);
      delay(300);
    }
  }


  if(Pagina == 1)
  {
    u8g2.clearBuffer();
    PaginaSnel();
    u8g2.setFont(u8g2_font_unifont_t_symbols);
    if(Scroll == 0) u8g2.drawGlyph(15,36, 0x2192);
    if(Scroll == 1) u8g2.drawGlyph(15,60, 0x2192);
    u8g2.setCursor(50,34);
    u8g2.print("%");
    u8g2.setFont(u8g2_font_profont12_tf);
    u8g2.setCursor(30,34);
    u8g2.print(Snelheid);
    u8g2.drawBox(64,25,SnelheidXas,8);
    u8g2.sendBuffer();

    if((digitalRead(S1) == 1) || (digitalRead(S3) == 1)) 
    {
      Scroll = !Scroll;
      Beep(2);
      delay(300);
    }
    
    if((digitalRead(S2) == 1) && (Scroll == 0)) 
    {
      if(Snelheid == 100) Snelheid = 0;
      Snelheid = Snelheid + 10;
      constrain(Snelheid, 0, 100);

      if(max_throttle == 225) max_throttle = 125;
      else max_throttle = max_throttle + 10;
      constrain(max_throttle, 125, 225);

      if(SnelheidXas == 50) SnelheidXas = 0;
      SnelheidXas = SnelheidXas + 5;
      constrain(SnelheidXas, 0, 50);

      Beep(2);
      
      delay(300);
    }
    
    if((digitalRead(S2) == 1) && (Scroll == 1))
    {
      Pagina = 0;
      Scroll = 0;
      Beep(2);
      delay(300);
    }
  }


  if(Pagina == 2)
  {
    u8g2.clearBuffer();
    PaginaGevoelig();
    u8g2.setFont(u8g2_font_unifont_t_symbols);
    if(Scroll == 0) u8g2.drawGlyph(15,36, 0x2192);
    if(Scroll == 1) u8g2.drawGlyph(15,60, 0x2192);
    u8g2.setCursor(50,34);
    u8g2.print("%");
    u8g2.setFont(u8g2_font_profont12_tf);
    u8g2.setCursor(30,34);
    u8g2.print(Gevoeligheid);
    u8g2.drawBox(64,25,GevoeligheidXas,8);
    u8g2.sendBuffer();

    if((digitalRead(S1) == 1) || (digitalRead(S3) == 1)) 
    {
      Scroll = !Scroll;
      Beep(2);
      delay(300);
    }
    
    if((digitalRead(S2) == 1) && (Scroll == 0)) 
    {
      if(Gevoeligheid == 100) Gevoeligheid = 0;
      Gevoeligheid = Gevoeligheid + 10;
      constrain(Gevoeligheid, 0, 100);

      if(Kd == 88) Kd = 48;
      else Kd = Kd + 4;
      constrain(Kd, 48, 88);

      if(GevoeligheidXas == 50) GevoeligheidXas = 0;
      GevoeligheidXas = GevoeligheidXas + 5;
      constrain(GevoeligheidXas, 0, 50);

      Beep(2);
      
      delay(300);
    }
    
    if((digitalRead(S2) == 1) && (Scroll == 1))
    {
      Pagina = 0;
      Scroll = 0;
      Beep(2);
      delay(300);
    }
  }
}

void PaginaInstel()
{
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(32,10,"Intellingen");
  u8g2.drawLine(32,13, 93,13);
  u8g2.drawFrame(5,20, 118,20);
  u8g2.drawFrame(5,44, 118,20);
  u8g2.drawStr(50,33,"Snelheid");
  u8g2.drawStr(40,58,"Gevoeligheid");
}

void PaginaSnel()
{
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(38,10,"Snelheid");
  u8g2.drawLine(38,13, 86,13);
  u8g2.drawFrame(5,20, 118,20);
  u8g2.drawFrame(5,44, 118,20);
  u8g2.drawStr(50,58,"Terug");
}

void PaginaGevoelig()
{
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(27,10,"Gevoeligheid");
  u8g2.drawLine(27,13, 100,13);
  u8g2.drawFrame(5,20, 118,20);
  u8g2.drawFrame(5,44, 118,20);
  u8g2.drawStr(50,58,"Terug");
}
