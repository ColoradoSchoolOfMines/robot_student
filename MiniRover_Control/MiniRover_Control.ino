  //
  #include <SPI.h>
char buf [100];
volatile byte pos;
volatile boolean process_it;
  //Model Rover Program

  //Libraries
  #include "Wire.h" // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
  //#include "I2Cdev.h" //I2C development library
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
  #define ENC_LF_A 22 //Encoder A Pin for Left Front Motor
  #define ENC_RF_A 26 //Encoder A Pin for Right Front Motor
  #define ENC_LB_A 24 //Encoder A Pin for Left Back Motor
  #define ENC_RB_A 28 //Encoder A Pin for Right Back Motor

  //Define Directions
  #define LEFT 0
  #define RIGHT 1
  #define FORWARD 2
  #define BACKWARD 3

  //declare encoder counter variables, initially = 0, long int range: -2,147,483,648 to 2,147,483,647
long int count_LeftFront = 0;
long int count_RightFront = 0;
long int count_LeftBack = 0;
long int count_RightBack = 0;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  //Wire.begin();
  // initialize serial communication
  Serial.begin(115200);

  Serial.println("Setup");

  initializeMotors();
  initializeSPI();

}

void loop() {
  if (process_it) {    //CODE FOR RASPB_PI COMMUNICATION
    buf [pos] = 0;  
    //Serial.println (buf);
    pos = 0;
    process_it = false;

    memset(buf, 0, sizeof(buf));
  }

}

void getValues(char in[], int* angle_out, int* distance_out){
  char* angle = strtok(input, ", ");
  char* distance = strtok(NULL, ", ");

  sscanf(angle, "%d", angle_out);
  sscanf(distance, "%d", distance_out);

  delete angle;
  delete distance;
}

//---------------------
//  SPI METHODS
//---------------------
void initializeSPI(){
  pinMode(MISO, OUTPUT);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // get ready for an interrupt 
  pos = 0;   // buffer empty
  process_it = false;

  // now turn on interrupts
  SPI.attachInterrupt();
}

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

//---------------------
//  MOTOR METHODS
//---------------------
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

  //Initialize Encoder Interrupts for B signal
  attachInterrupt(0, incrementEncoder_LeftFront, CHANGE);  //encoderB is on pin 2, Interrupt pin triggers program updating tickcount
  attachInterrupt(2, incrementEncoder_RightFront, CHANGE);  //encoderB is on pin 21, Interrupt pin triggers program updating tickcount
  attachInterrupt(1, incrementEncoder_LeftBack, CHANGE);  //encoderB is on pin 3, Interrupt pin triggers program updating tickcount
  attachInterrupt(3, incrementEncoder_RightBack, CHANGE);  //encoderB is on pin 20, Interrupt pin triggers program updating tickcount
  //For A Signal
  pinMode( ENC_LF_A, INPUT);
  pinMode( ENC_RF_A, INPUT);
  pinMode( ENC_LB_A, INPUT);
  pinMode( ENC_RB_A, INPUT);
}

void moveRobot(int power, int ticks){
  count_LeftFront = 0;

  while (abs(count_LeftFront) < ticks){
    Serial.println(count_LeftFront);
    analogWrite(PWM_LF,power);
    analogWrite(PWM_LB,power);
    analogWrite(PWM_RF,power);
    analogWrite(PWM_RB,power);
  }

  digitalWrite(PWM_LF,LOW);
  digitalWrite(PWM_LB,LOW);
  digitalWrite(PWM_RF,LOW);
  digitalWrite(PWM_RB,LOW);
} 

void rotate(int angle){
  //Ranges from -90 to 90
  if (angle < 0){
    Serial.println("LEFT");
    setDirection(LEFT);
  } else if (angle > 0) {
    Serial.println("RIGHT");
    setDirection(RIGHT);
  } else {
    return;
  }

  moveRobot(50, 2*abs(angle));

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

void incrementEncoder_LeftFront(){
  if ( digitalRead(2)){
  //If Rising B Signal

    if (digitalRead(ENC_LF_A)) {
  //increment if A is high
      count_LeftFront++;
    } else  { 
  //decrement if A is low
      count_LeftFront--;
    }
  }
}
void incrementEncoder_RightFront(){
  if( digitalRead(21) ){//If Rising B Signal
  //increment if A is high
    if (digitalRead(ENC_RF_A)) count_RightFront++;
  //decrement if A is low
    else count_RightFront--;
  }
}
void incrementEncoder_LeftBack(){
  if( digitalRead(3) ){//If Rising B Signal
  //increment if A is high
    if (digitalRead(ENC_LB_A)) count_LeftBack++;
  //decrement if A is low
    else count_LeftBack--;
  }
}
void incrementEncoder_RightBack(){
  if( digitalRead(20) ){//If Rising B Signal
  //increment if A is high
    if (digitalRead(ENC_RB_A)) count_RightBack++;
  //decrement if A is low
    else count_RightBack--;
  }
}
