// #include <Pololu3pi.h>
// #include <PololuQTRSensors.h>
// #include <OrangutanMotors.h>
// #include <OrangutanAnalog.h>
// #include <OrangutanLEDs.h>
// #include <OrangutanLCD.h>
// #include <OrangutanPushbuttons.h>
// #include <OrangutanBuzzer.h>

#include <Pololu3piPlus32U4.h>
#include <PololuOLED.h>

#define NUM_SENSORS 5
#define PASS_DELAY 50
#define IS_BLACK(p) p>800
#define IS_WHITE(p) p<100

#define MIN_SPEED 0
#define MAX_SPEED 150
#define BASE_SPEED 70
#define CALIB_SPEED 50
#define TURN_RANGE 150
#define TURN_SPEED 150

using namespace Pololu3piPlus32U4;

// Pololu3pi robot;
// OrangutanPushbuttons buttons;
// OrangutanLCD lcd;
// OrangutanBuzzer buzzer;

PololuSH1106 lcd(1, 30, 0, 17, 13);

ButtonB buttonB;
Buzzer buzzer;
LineSensors lineSensors;
Motors motors;
Encoders encoders;

unsigned int sensors[NUM_SENSORS]; 
unsigned int sensorsT[NUM_SENSORS]; 
unsigned int counter;
char pathTurn[100];
int turnCount=0;

float KP= 0.05;
float KD= 0.01;
float KI= 1/10000;
int lastError=0;
int integral=0;
int M=BASE_SPEED;
int turn_index=0;

char turnMode='A';

int m1Speed;
int m2Speed;

const char fugue[] PROGMEM =
  "! O6 L12 E D O5 L7 f# g#"
  "O6 L12 c# O5 B L7 D E"
  "O5 L12 B A L7 c# E L4 A";

void setup() {
 
  buzzer.play(">g32>>c32");

  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print(F(" * Tom *"));
  lcd.gotoXY(0, 1);
  lcd.print(F("Press B"));

  // press switch
  while(!buttonB.getSingleDebouncedPress()); 

  // start calibration
  lcd.gotoXY(0, 1);
  lcd.print(F("         "));
  delay(1000);
  lineSensors.emittersOn();

  for (counter=0; counter <= 60; counter++)
  {
    if (counter >= 15 && counter < 45)
      motors.setSpeeds(CALIB_SPEED, -CALIB_SPEED);
    else
      motors.setSpeeds(-CALIB_SPEED, CALIB_SPEED);
    
    lineSensors.calibrate();
  }

  // End calibration
  motors.setSpeeds(0,0);
  lineSensors.readCalibrated(sensors);

  lcd.gotoXY(0, 1);
  lcd.print(F("          "));


  Serial1.begin(9600);
  Serial.begin(9600);

  delay(1000);
  playS();

  while(1){
    if(Serial1.available()){
      char received = (char)Serial1.read();
      Serial.print(received);
      if(received == 'A'){
        break;
      }
      pathTurn[turnCount++] = received;
    }
  }

  // pathTurn[0] ='L';
  // pathTurn[1] ='S';
  // pathTurn[2] ='R';
  // pathTurn[3] ='L';
  // turnCount = 4;

}

void loop() {

  lcd.gotoXY(0, 1);
  lcd.print("Coming..");

  int j=0;
  turn_index = 0;
  for(j=0;j<turnCount;j++){
    while(1){
      doPID();

      if (IS_BLACK(sensors[4]) || IS_BLACK(sensors[0]))  //right || cross
      {
        break;
      }
    }
    motors.setSpeeds(50, 50);
    delay(PASS_DELAY);
    motorStop();
    delay(50);

    if (IS_BLACK(sensors[4]) && IS_BLACK(sensors[3]) && IS_BLACK(sensors[2]) && IS_BLACK(sensors[1]) && IS_BLACK(sensors[0]))  //right || cross
    {
      motors.setSpeeds(50, 50);
      delay(PASS_DELAY+100);
      motorStop();

      while(!buttonB.getSingleDebouncedPress());
      return;
    }
    // turn
    playS();
    char turnTo = pathTurn[j];
    switch(turnTo){
      case 'R': turnRight();break;
      case 'L': turnLeft();break;
      case 'S': {
        motors.setSpeeds(50, 50);
        delay(PASS_DELAY);
      } break; 
    } 
    // motors.setSpeeds(50, 50);
    // delay(PASS_DELAY);
  }
}

void doPID(){
  int position = lineSensors.readLineBlack(sensors);
  int error = position - 2000;
  integral+=error;
  int motorSpeed = KP * error + KD * (error - lastError) + integral*KI;
  lastError = error;
  
   m1Speed = constrain(M + motorSpeed, MIN_SPEED, MAX_SPEED);
   m2Speed = constrain(M - motorSpeed, MIN_SPEED, MAX_SPEED);
   motors.setSpeeds(m1Speed, m2Speed);
}

void checkTurn(float c){
  int16_t countsRight = encoders.getCountsAndResetRight();
  int16_t countsLeft = encoders.getCountsAndResetLeft();
  float count = 0;
  float max_count = TURN_RANGE * c;
  do{
    countsRight = abs(encoders.getCountsRight());
    countsLeft = abs(encoders.getCountsLeft());
    count = (countsRight + countsLeft) * 0.5;
  }while(count < max_count);
}

void turnLeft(){
    motorStop();
    playS();
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    checkTurn(1);
    motorStop();
}

void turnRight(){
    motorStop();
    playS();
    motors.setSpeeds(+TURN_SPEED,-TURN_SPEED);
    checkTurn(1);
    motorStop();
}

void turnBack(){
    motorStop();
    playS();
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    checkTurn(2.8);
    motorStop();
}

void moveForward(int spl,int spr){
  motors.setSpeeds(spl, spr);  
}

void motorStop(){
 motors.setSpeeds(0, 0);  
}


void playS(){
  buzzer.playNote(NOTE_G(5), 30, 15);
}