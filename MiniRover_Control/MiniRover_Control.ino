#include <SPI.h>
 
char buf [100];
volatile byte pos;
volatile boolean process_it;
//Model Rover Program

//Libraries
#include "Wire.h" // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#include "I2Cdev.h" //I2C development library
//#include "MPU6050_6Axis_MotionApps20.h" //Accellerometer Library

//Define Motor Power PWM Pins (Output)
#define PWM_LF 10 //PWM Pin for Left Front Motor
#define PWM_LB 9 //PWM Pin for Left Back Motor
#define PWM_RF 6 //PWM Pin for Right Front Motor
#define PWM_RB 5 //PWM Pin for Right Back Motor
//Define Motor Direction Pins (Output)
#define DIR_LF 8 //Direction Pin for Left Front Motor
#define DIR_LB 7 //Direction Pin for Left Back Motor
#define DIR_RF 4 //Direction Pin for Right Front Motor
#define DIR_RB 13 //Direction Pin for Right Back Motor
//Define Motor Current Measuring Pins - (Analog Input)
#define PWM_LF 5 //PWM Pin for Left Front Motor
#define PWM_LB 6 //PWM Pin for Left Back Motor
#define PWM_RF 9 //PWM Pin for Right Front Motor
#define PWM_RB 10 //PWM Pin for Right Back Motor
//Define Motor Encoder Pins (Digital Input)
#define ENC_LF_A //Encoder A Pin for Left Front Motor
#define ENC_LF_B //Encoder B Pin for Left Front Motor

#define LEFT 0
#define RIGHT 1
#define FORWARD 2
#define BACKWARD 3


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    //Wire.begin();
    // initialize serial communication
  Serial.begin(115200);
    
  initializeMotors();
    
  pinMode(MISO, OUTPUT);
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  
  // get ready for an interrupt 
  pos = 0;   // buffer empty
  process_it = false;
 
  // now turn on interrupts
  SPI.attachInterrupt();
    
    //Initialize Encoder Interrupt
    //attachInterrupt(0, incrementEncoder_LeftFront, CHANGE);  //encoderB is on pin 2, Interrupt pin triggers program updating tickcount
    //attachInterrupt(1, incrementEncoder_RightFront, CHANGE);  //encoderB is on pin 3, Interrupt pin triggers program updating tickcount
}

void loop() {
  //Turn Motors On
  
  //setDirection(RIGHT);
  //moveRobot(70, 3); 
  
  if (process_it) {
    buf [pos] = 0;  
    Serial.println (buf);
    pos = 0;
    process_it = false;
    
    if (strcmp(buf, "RIGHT\n") == 0){
      Serial.println("I'm turning right.");
      setDirection(RIGHT);
      moveRobot(50, 3); 
    }
    if (strcmp(buf, "LEFT\n") == 0){
      Serial.println("I'm turning left.");
      setDirection(LEFT);
      moveRobot(50, 3); 
    }
    if (strcmp(buf, "UP\n") == 0){
      Serial.println("I'm turning up.");
      setDirection(FORWARD);
      moveRobot(50, 3); 
    }
    if (strcmp(buf, "DOWN\n") == 0){
      Serial.println("I'm turning down.");
      setDirection(BACKWARD);
      moveRobot(50, 3); 
    }
    
    
    memset(buf, 0, sizeof(buf));
  }
  
}

//SPI METHODS

// SPI interrupt routine
ISR (SPI_STC_vect) {
  byte c = SPDR;  // grab byte from SPI Data Register
  
  // add to buffer if room
  if (pos < sizeof buf) {
    buf [pos++] = c;
    
    // example: newline means time to process buffer
    if (c == '\n') {
      process_it = true;
    }
      
    }  // end of room available
}  // end of interrupt routine SPI_STC_vect

// Motor Data

void initializeMotors(){
  //Set up Motor Power PWM Pins
    pinMode(PWM_LF, OUTPUT); //PWM Pin for Left Front Motor
    pinMode(PWM_LB, OUTPUT); //PWM Pin for Left Back Motor
    pinMode(PWM_RF, OUTPUT); //PWM Pin for Right Front Motor
    pinMode(PWM_RB, OUTPUT); //PWM Pin for Right Back Motor
    //Set up Motor Direction Pins
    pinMode(DIR_LF, OUTPUT); //Direction Pin for Left Front Motor
    pinMode(DIR_LB, OUTPUT); //Direction Pin for Left Back Motor
    pinMode(DIR_RF, OUTPUT); //Direction Pin for Right Front Motor
    pinMode(DIR_RB, OUTPUT); //Direction Pin for Right Back Motor
    
    //Initially Set Motors Off
    digitalWrite(PWM_LF,LOW);
    digitalWrite(PWM_LB,LOW);
    digitalWrite(PWM_RF,LOW);
    digitalWrite(PWM_RB,LOW);
    //Initially Set Motors Directions
    digitalWrite(DIR_LF,LOW);
    digitalWrite(DIR_LB,HIGH);
    digitalWrite(DIR_RF,LOW);
    digitalWrite(DIR_RB,HIGH);
}

void moveRobot(int power, int seconds){
  analogWrite(PWM_LF,power);
  analogWrite(PWM_LB,power);
  analogWrite(PWM_RF,power);
  analogWrite(PWM_RB,power);
  
  delay(seconds*1000);
  
  digitalWrite(PWM_LF,LOW);
  digitalWrite(PWM_LB,LOW);
  digitalWrite(PWM_RF,LOW);
  digitalWrite(PWM_RB,LOW);
  
} 

void setDirection(int dir){
  switch(dir){
    case LEFT:
      digitalWrite(DIR_LF, LOW);
      digitalWrite(DIR_LB, HIGH);
      digitalWrite(DIR_RF, HIGH);
      digitalWrite(DIR_RB, LOW);
    break;
    case RIGHT:
      digitalWrite(DIR_LF, HIGH);
      digitalWrite(DIR_LB, LOW);
      digitalWrite(DIR_RF, LOW);
      digitalWrite(DIR_RB, HIGH);
    break;
    case FORWARD:
      digitalWrite(DIR_LF, LOW);
      digitalWrite(DIR_LB, HIGH);
      digitalWrite(DIR_RF, LOW);
      digitalWrite(DIR_RB, HIGH);
    break;
    case BACKWARD:
      digitalWrite(DIR_LF, HIGH);
      digitalWrite(DIR_LB, LOW);
      digitalWrite(DIR_RF, HIGH);
      digitalWrite(DIR_RB, LOW);
    break;
  }

}
