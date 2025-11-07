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
#define MAX_SPEED 200
#define BASE_SPEED 100
#define CALIB_SPEED 50
#define TURN_RANGE 150
#define TURN_SPEED 150
#define PASS_SPEED 50

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
  lcd.print(F(" Jerry "));
  lcd.gotoXY(0, 1);
  lcd.print(F("Press B"));

  // press switch
  while(!buttonB.getSingleDebouncedPress()); 

  // start calibration
  lcd.gotoXY(0, 1);
  lcd.print(F("           "));
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
  lcd.print(F("Done !"));

  delay(1000);

  lcd.gotoXY(0, 1);
  lcd.print("Press B");

  Serial1.begin(9600);

  // press switch
  while(!buttonB.getSingleDebouncedPress()); 
  delay(1000);
  playS();

  lcd.gotoXY(0, 1);
  lcd.print("           ");

}

void loop() {
  while(1){
    doPID();
    if (IS_WHITE(sensors[4]) && IS_WHITE(sensors[3]) && IS_WHITE(sensors[2]) && IS_WHITE(sensors[1]) && IS_WHITE(sensors[0])){
      turnMode='B';
      break;
    }

    // cross or right (Right priority)
    if (IS_BLACK(sensors[4]))  
    {
      turnMode='R';
      break;
    }

    // left or straight
    if (IS_BLACK(sensors[0]))
    {
      turnMode='S';
      break;
    }

  }
  
  motors.setSpeeds(PASS_SPEED, PASS_SPEED);
  delay(PASS_DELAY);
  motorStop();
  
  switch(turnMode){
    case 'R':  {
      lineSensors.readLineBlack(sensors);
      if(IS_BLACK(sensors[4])){
        motors.setSpeeds(PASS_SPEED, PASS_SPEED);
        delay(PASS_DELAY + 350);
        motorStop();
        search_completed();
      }else{
        pathTurn[turnCount++] = 'R';
        turnRight();
      }
    };break;
    case 'L':  {
      turnLeft();
    }break;
    case 'B':  {
      pathTurn[turnCount++]='B';
      turnBack();
    };break;
    case 'E':  search_completed();break;
    case 'S':  {
      lineSensors.readLineBlack(sensors);
      if(IS_WHITE(sensors[3])){
        pathTurn[turnCount++]='L';
        turnLeft();
      }else{
        pathTurn[turnCount++]='S';  
      }
    }break;
  }
  simplify_path();
  motors.setSpeeds(PASS_SPEED, PASS_SPEED);
  delay(PASS_DELAY);
}

void search_completed(){
    motorStop();
    playS();
    delay(100);

    lcd.clear();
    lcd.print(F("Catch Me"));
    lcd.gotoXY(0, 1);
    lcd.print("If You Can");

    delay(1000);
    
    // Send data
    Serial1.print(pathTurn);
    Serial1.print('A');

    while(1){
      while(!buttonB.getSingleDebouncedPress()); 
      for(int i=0; i< turnCount; i++){
        Serial1.print(pathTurn[i]);
      }
      Serial1.print('A');
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

int getPos(){
 return lineSensors.readLineBlack(sensors);
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


void turn(){
    playS();
    switch(pathTurn[turn_index++]){
      case 'R': turnRight();break;
      case 'L': turnLeft();break;
      case 'S': {
        motors.setSpeeds(PASS_SPEED, PASS_SPEED);
        delay(PASS_DELAY);
      } break; 
    }  
}

void simplify_path()
{
  if (turnCount < 3 || pathTurn[turnCount-2] != 'B')
    return;

  int total_angle = 0;
  int i;
  for (i = 1; i <= 3; i++)
  {
    switch (pathTurn[turnCount - i])
    {
    case 'R':
      total_angle += 90;
      break;
    case 'L':
      total_angle += 270;
      break;
    case 'B':
      total_angle += 180;
      break;
    }
  }

  total_angle = total_angle % 360;

  switch (total_angle)
  {
  case 0:
    pathTurn[turnCount - 3] = 'S';
    break;
  case 90:
    pathTurn[turnCount - 3] = 'R';
    break;
  case 180:
    pathTurn[turnCount - 3] = 'B';
    break;
  case 270:
    pathTurn[turnCount - 3] = 'L';
    break;
  }

  turnCount -= 2;
  
}

void playS(){
  buzzer.playNote(NOTE_G(5), 30, 15);
}