#include <ezButton.h>
#include <LinkedList.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>

// colour sensor setup
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

// motor analogue pins
#define MOT_A1_PIN 5
#define MOT_A2_PIN 6
#define MOT_B1_PIN 3
#define MOT_B2_PIN 11

#define in1 7
#define in2 8

Servo servo;
int servoPos = 45;
int servoColour;

ezButton LIMIT_SWITCH_END(2);
int switchState;

// ultrasonic sensor pins
const int echo = 12;
const int trig = 13;

int colour; // colour integer
int state; // reading from bluetooth sensor

// IR digital pins
int left = A3;
int centre = A2;
int right = A1;

// integers used to control speed of individual motors
int leftMotorSpeed;
int leftReverseSpeed;
int rightMotorSpeed;
int rightReverseSpeed;

// thresholds of the IR sensors which were decided based on its characterisation 
int lowThreshold = 300;
int highThreshold = 600;

int maxSpeed = 175; // maximum speed of buggy

int sensorValueLeft, sensorValueCentre, sensorValueRight; // the values of the IR sensors

LinkedList<long> path = LinkedList<long>(); // path list holds all the paths
LinkedList<long> conversePaths = LinkedList<long>(); // list holds paths that the robot will collect from
LinkedList<long> dfs = LinkedList<long>(); // dfs holds uncompleted paths while buggy is mapping (depth first search)

LinkedList<int> toBase = LinkedList<int>();
int curPathArr[8]; // holds the current path that the buggy is traversing
int pathNumber = 0; // keeps count of how many paths the buggy has currently completed
int counter = 0; // counter keeps track of which fork of the path it is currently traversing (increases at every fork)

// an integer (could probably be a boolean) - keeps track whether the buggy is currently turning or not from a fork or end of line
int rightT = 0;
int forward = 0;
int leftT = 0;

boolean endPath = false; // boolean that keeps track whether the buggy has reached the end of a path so it knows when to turn around
boolean mapping = false; // keeps track of whether the buggy is mapping or not - done at the beginning of the program
boolean findingBase = false; // checks if buggy has lost base
boolean collecting = false; // if buggy is collecting objects
boolean finishMap = false; // checks if initially buggy has finished mapping
boolean manualMode = true; // checks if buggy is in manual mode
boolean atBase = false; // keeps track if buggy is at base or not

// a linked list that keeps track of current and past positions of the buggy
// this is to be used with the dfs list to help map the paths
LinkedList<int> pos = LinkedList<int>();

int curPos = 1; // represents the position the buggy is facing (1 = north, 2 = west, 3 = south, 4 = east)

float r, g, b; // red, green, and blue values from the colour sensor

int loopCounter = 0; // keeps track of number of loops to limit distance sensor readings

void setup() {
  // put your setup code here, to run once:
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);
  // Start with drivers off, motors coasting.
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);

  Serial.begin(9600); // starts serial

  LIMIT_SWITCH_END.setDebounceTime(50);
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  servo.attach(A0);
  servo.write(servoPos);

  // checks for connection to colour sensor
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);

  // to test without having to map
  //////
  /*
  mapping = false;
  
  leftT = 1;
  rightMotorSpeed = 150;
  rightReverseSpeed = 0;
  leftMotorSpeed = 0; 
  leftReverseSpeed = 200;
  
  finishMap = false;
  counter = 0;
  pathNumber = 0;

  // red 4, green 5, yellow 6
  path.add(16);
  path.add(215);
  path.add(2214);

  path.add(2226);
  path.add(2236);
  path.add(235);
  path.add(34);
  
  // converts the first path integer into an array
  //convToArray(pathNumber);
  atBase = true;

  */
  ///////

  steer(0, 0, 0, 0);
}

// retrieves IR values
void scan() {
  sensorValueLeft = analogRead(left) * 0.96;
  sensorValueCentre = analogRead(centre);
  sensorValueRight = analogRead(right) * 1.03;
}

void movement() {
  scan(); // retrieves IR values

  //checks if the buggy is currently not turning
  if (rightT == 0 && forward == 0 && leftT == 0) {
    if (sensorValueCentre > lowThreshold) { // if buggy is centred
      // speed of motors vary depending on IR values - this allows for a much more accurate line following algorithm
      leftMotorSpeed = maxSpeed - 1000/sensorValueRight; 
      rightMotorSpeed = maxSpeed - 1000/sensorValueLeft;
      leftReverseSpeed = 0;
      rightReverseSpeed = 0;

      // if the buggy has reached a fork in its path
      if (sensorValueRight > lowThreshold && sensorValueLeft > lowThreshold && sensorValueCentre > highThreshold) {
        if (mapping) {
          // calls this function if its trying to map routes
          mappingRoutes();

          // checks if mapping has been completed (ie last turn before reaching base)
          if (dfs.size() == 0) {
            finishMap = true; // mapping is complete (but buggy is still travelling)
          }
        }
        else if (findingBase) {
          leftT = 1; // turn left if trying to find the base
          choosePath();
        }
        else { // if collecting object
          choosePath(); 
        }
      }
    }
    else{ // if buggy has come away from the line (eg buggy needs additional correcting or reached end of line)
      if (sensorValueRight > lowThreshold) { //if buggy needs additional correction to the right (should hardly happen)
        rightMotorSpeed = 0;
        rightReverseSpeed = maxSpeed - 10000/(sensorValueRight-200);
        leftMotorSpeed = maxSpeed - 4000/sensorValueRight; 
        leftReverseSpeed = 0;
      }
      else if (sensorValueLeft > lowThreshold) { // buggy needs additional correcting to the left (should hardly happen)
        leftMotorSpeed = 0;
        leftReverseSpeed = maxSpeed - 10000/(sensorValueLeft-200);
        rightMotorSpeed = maxSpeed - 4000/sensorValueLeft;
        rightReverseSpeed = 0;
      }
      else { //if buggy reaches end of path (left, centre and right sensors are all picking up white)
        if (finishMap) {
          steer(0, 0, 0, 0);
          colour = checkColour();

          // checks that it is actually at the base (7 = white)
          if (colour == 7) {
            Serial.write('B');
            // ready to turn around for next function
            leftT = 1;
            rightMotorSpeed = 150;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 200;
          
            mapping = false; // no longer mapping
            finishMap = false;
            counter = 0; // reset path values
            pathNumber = 0;
  
            atBase = true; // returned to base
          }
          else {
            // if it has not returned to base then will search for it and recommend to user that paths may be incorrect or not complete
            findTheBase();
            Serial.write('8');
          }
        }
        // uses this condition if the buggy is currently mapping the paths
        else if (mapping) {
          steer(0, 0, 0, 0);

          colour = checkColour(); // checks colour at end of path

          // checks for base ( if mapping it should not have reached base )
          if (colour == 7) {
            findTheBase(); // if at base it warns user that it may have incorrectly mapped paths
            Serial.write('8');
          }
          else {
            Serial.write(colour); // communicates to user what colour has been detected
  
            long pathCode = dfs.get(0)*10+colour; // adds colour value to pathcode integer which represents each path

            // if the buggy has reached the end of a path it will add the path integer to the list of paths (this path list can then be later manipulated)
            path.add(pathCode);
  
            Serial.write(1); // 1 integer represents to UI that path code is about to be sent over
            delay(50);
            while (pathCode != 0) {
              Serial.write(pathCode%10); // sends path code as individual integers to be easily represented on UI
              pathCode /= 10;
              delay(50);
            }
            Serial.write(0); // 0 integer represents to UI that path code has been fully sent over
            
            dfs.shift(); // removes path from dfs list
  
            // turns left 180 degrees to get back onto line
            rightMotorSpeed = 150;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 200;
            steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
            leftT = 1;
  
            // adds old position to position list
            pos.add(curPos);
            Serial.write(13);
  
            // changes the current position integer of the buggy
            if (curPos == 3) {
              curPos = 1;
            }
            else if (curPos == 1) {
              curPos = 3;
            }
            else if (curPos == 2) {
              curPos = 4;
            }
            else if (curPos == 4) {
              curPos = 2;
            }
          }
        }
        else if (findingBase) { // if buggy reaches end of path but trying to find base
          steer(0, 0, 0, 0);

          colour = checkColour();

          leftT = 1;
          rightMotorSpeed = 150;
          rightReverseSpeed = 0;
          leftMotorSpeed = 0; 
          leftReverseSpeed = 200;
          
          if (colour == 7) { // if found base - reset variables
            findingBase = false;
            atBase = true;
            manualMode = false;
            collecting = false;

            Serial.write('B');
        
            conversePaths.clear();
            for (int i=0; i<8; i++) {
              curPathArr[i] = 0;
            }
          
            counter = 0;
            pathNumber = 0;
      
            endPath = false;
            toBase.clear();
            toBase.add(8);
          }
          else { // if not at base turn around and continue searching
            Serial.write(13);
          }

        }
        else {
          // next statements are for when buggy is in collection phase
          if (curPathArr[counter] == 0) { // if array reaches a 0 this means it has reached the end of the path and is ready to pick up object
            steer(0, 0, 0, 0);

            colour = checkColour();
            
            if (curPathArr[0] != colour) { // if colour at end of path does not match mapped colour then it will not pick up object
              Serial.write('8');
            }
            else {
              if (colour == 4) { // red
                Serial.write('?'); // sends message to UI to state it has collected object
              }
              else if (colour == 5) { // green
                Serial.write('@');
              }
              else if (colour == 6) { // yellow
                Serial.write('A');
              }

              servoColour = colour;

              LIMIT_SWITCH_END.loop();
              switchState = LIMIT_SWITCH_END.getState();

              servo.write(45);
              
              while (switchState == HIGH) {
                LIMIT_SWITCH_END.loop();
                digitalWrite(in1, LOW);
                digitalWrite(in2, HIGH);
                switchState = LIMIT_SWITCH_END.getState();
              }

              digitalWrite(in1, LOW);
              digitalWrite(in2, LOW);

              for (servoPos = 45; servoPos <=100; servoPos += 1) { // in steps of 1 degree
                servo.write(servoPos);                 
                delay(35);                         
              }

              digitalWrite(in1, HIGH);
              digitalWrite(in2, LOW);
              delay(8000);
              digitalWrite(in1, LOW);
              digitalWrite(in2, LOW);
            }
            
            endPath = true;
            delay(1500);

            // turns around to head back to base
            rightMotorSpeed = 150;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 200;
            steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
            leftT = 1;
            counter--;
            changeDirection();
            Serial.write(13);
          }
          else if (curPathArr[counter] > 3) { // if buggy has returned to base
            steer(0, 0, 0, 0);
            colour = checkColour();

            if (colour == 7) { //checks that it is actually at base
              Serial.write('B');

              /*
              if (servoColour == 5) {
                steer(0, 100, 150, 0);
                delay(100);
                steer(0, 0, 0, 0);
              }
              */

              LIMIT_SWITCH_END.loop();
              switchState = LIMIT_SWITCH_END.getState();
              
              while (switchState == HIGH) { // goes down until switch is hit
                LIMIT_SWITCH_END.loop();
                digitalWrite(in1, LOW);
                digitalWrite(in2, HIGH);
                switchState = LIMIT_SWITCH_END.getState();
              }

              digitalWrite(in1, LOW);
              digitalWrite(in2, LOW);

              for (servoPos = 100;  servoPos >= 45; servoPos -= 1) { // in steps of 1 degree
                servo.write(servoPos);                 
                delay(35);                         
              }

              digitalWrite(in1, HIGH);
              digitalWrite(in2, LOW);
              delay(8000);
              digitalWrite(in1, LOW);
              digitalWrite(in2, LOW);
            
              endPath = false;
              rightMotorSpeed = 150;
              rightReverseSpeed = 0;
              leftMotorSpeed = 0; 
              leftReverseSpeed = 200;
              leftT = 1;
              toBase.clear();
              toBase.add(8);
              
              if (pathNumber < conversePaths.size()-1) { // if there are still more paths to converse will continue
                pathNumber++;
                convToArray(pathNumber);
                counter = 1;
                atBase = false;

                if (state == 4) {
                  Serial.write(';');
                }
                else if (state == 5) {
                  Serial.write('<');
                }
                else if (state == 6) {
                  Serial.write('=');
                }
                else if (state == 8) {
                  Serial.write('>');
                }
                Serial.write(13);
           
              }
              else { // if not then buggy will stop
                counter = 0;
                pathNumber = 0;
                conversePaths.clear();
                collecting = false;
                atBase = true;
              }
            }
            else { // if not buggy will go in search of base and warn user of incorrect mapping
              findTheBase();
            }
          }
          else {
            findTheBase();
          }
        }
      } 
    }
  }
  else { // dont correct until finished turning - leftT, rightT, and forward variables represent the buggy turning (so when turning does not intefer with line)
    if (sensorValueLeft > highThreshold && sensorValueCentre < lowThreshold && sensorValueRight < lowThreshold) {
      leftT = 0;
      forward = 0;
    }
    else if (sensorValueLeft < lowThreshold && sensorValueCentre > highThreshold && sensorValueRight < lowThreshold) {
      forward = 0;
    }
    else if (sensorValueLeft < lowThreshold && sensorValueCentre < lowThreshold && sensorValueRight > highThreshold) {
      rightT = 0;
      forward = 0;
    }
  }

  // steer function controls motor
  steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
  
}

void mappingRoutes() {
  
  pos.add(curPos); // pos list keeps track of buggy position

  int prevPos; // prev pos retrieves previous position in position list
  
  if (pos.size() > 1) {
    prevPos = pos.get(pos.size()-2);
  }
  else {
    prevPos = curPos;
  }

  if (!(abs(prevPos - curPos) == 2)) { // if buggy has returned back on itself (reach end of path) then does not add new potential paths to dfs list
    if (dfs.size() == 0) { // reaches first fork will add three potential paths (left, forward, right)
      dfs.add(1); // left
      dfs.add(2); // forward
      dfs.add(3); // right
    }
    else {
      int p = dfs.shift(); // works similar to dfs search algorithm removes the path it is transcending and adds partial paths back to dfs list
      dfs.add(0, p*10+3);
      dfs.add(0, p*10+2);
      dfs.add(0, p*10+1);
    }
  }
  else { // if buggy has returned on itself then remove last two positions from pos list
    pos.pop();
    pos.pop();
  }

  leftT = 1; // take left turn

  if (curPos == 4) { // goes back on itself if reached end of position
    curPos = 1;
  }
  else { // changes position as orientation is changing
    curPos += 1;
  }

  choosePath(); // chooses path
}

void choosePath() {
  if (curPathArr[counter] == 1 || leftT == 1) {
    rightMotorSpeed = 200;
    rightReverseSpeed = 0;
    leftMotorSpeed = 0; 
    leftReverseSpeed = 0;
    leftT = 1;
    Serial.write(14);
    if (!mapping) {
      toBase.add(3);
    }
  }
  else if (curPathArr[counter] == 2 || forward == 1) {
    rightMotorSpeed = 150;
    rightReverseSpeed = 0;
    leftMotorSpeed = 155; 
    leftReverseSpeed = 0;
    forward = 1;
    Serial.write(11);
    if (!mapping) {
      toBase.add(2);
    }
  }
  else if (curPathArr[counter] == 3 || rightT == 1) {
    rightMotorSpeed = 0;
    rightReverseSpeed = 0;
    leftMotorSpeed = 200; 
    leftReverseSpeed = 0;
    rightT = 1;
    Serial.write(12);
    if (!mapping) {
      toBase.add(1);
    }
  }
  else {
    findTheBase();
    Serial.write('8');
  }
  steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
  if (!endPath) {
    counter++;
  }
  else {
    counter--;
  }
}

void convToArray(int pathNo) {
  int curPath = conversePaths.get(pathNo);

  int reverseArr[8];
  
  for (int j=0; j<8; j++) {
    curPathArr[j] = 0;
    reverseArr[j] = 0;
  }
  
  int i=0;
  int y;

  // converting integers from linked list to array
  while ( curPath > 0 ) {
      y=curPath/10;
      curPathArr[i] = curPath-(10*y);
      reverseArr[i] = curPath-(10*y);
      curPath=y;
      i++;
  }

  i = 0;
  while (curPathArr[i] != 0) {
    i++;
  }
  
  int endOfPathIndex = i-1;

  // array reversal
  for (int j=0; j<endOfPathIndex; j++) {
    curPathArr[j+1] = reverseArr[endOfPathIndex-j];
  }
  
  counter++;

}

void changeDirection() {
  int i = 0;
  while (!curPathArr[i] == 0) {
    if (curPathArr[i] == 1) {
      curPathArr[i] = 3;
    }
    else if (curPathArr[i] == 3) {
      curPathArr[i] = 1;
    }
    i++;
  }
}

void steer(int m1p1, int m1p2, int m2p1, int m2p2) {
  analogWrite(MOT_A1_PIN, m1p1);
  analogWrite(MOT_A2_PIN, m1p2);
  analogWrite(MOT_B1_PIN, m2p1);
  analogWrite(MOT_B2_PIN, m2p2);
}

int checkColour() {
  int c;

  tcs.getRGB(&r, &g, &b);
  c = detectColour(r, g, b);

  int i = 0;
  while (i<10) {
    int oldCol = c;
    tcs.getRGB(&r, &g, &b);
    // could take a colour average over multiple iterations if it is not be too accurate
    c = detectColour(r, g, b);
    if (oldCol == c) {
      i++;
    }
    else {
      i=0;
    }
  }
  
  return c;
}

int detectColour( int r ,int g , int b){
 
  //colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  int c = 8;

  if(r >= 140 & g <= 75 & b <= 75) {
   // red
   c = 4;
  }
  else if(r <= 100 & g  >= 95 & b <= 100){
   // green
   c = 5;
  }
  else if(r >= 90 & g>= 60 & b <= 60){
   // yellow
   c = 6;
  }
  else if (r >= 80 && g >= 75 && b >= 65) {
    // white
    c = 7;
  }

  return c;
}

long calcDist() {
  long duration, cm;

  // Trigger the ultrasonic sensor
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);

    // Measure the echo time
  duration = pulseIn(echo, HIGH);

  // Convert the time into a distance
  cm = duration / 29 /2;

  return cm;
}

void checkStateUI(int s) {
  if (s == 48) { // left
    manualMode = true;
    steer(150, 0, 0, 150);
    atBase = false;
    mapping = false;
    collecting = false;
    findingBase = false;
  }
  else if (s == 49) { //  backwards
    manualMode = true;
    steer(150, 0, 150, 0);
    atBase = false;
    mapping = false;
    collecting = false;
    findingBase = false;
  }
  else if (s == 50) { // right
    manualMode = true;
    steer(0, 150, 150, 0);
    atBase = false;
    mapping = false;
    collecting = false;
    findingBase = false;
  }
  else if (s == 51) { // forwards
    manualMode = true;
    steer(0, 150, 0, 150);
    atBase = false;
    mapping = false;
    collecting = false;
    findingBase = false;
  }
  else if (s == 52) { // force stop
    manualMode = true;
    steer(0, 0, 0, 0);
    atBase = false;
    mapping = false;
    collecting = false;
    findingBase = false;
  }
  else if (s == 53 && !mapping && !collecting && !findingBase) { // mapping
    steer(0, 0, 0, 0);

    tcs.getRGB(&r, &g, &b);
    colour = checkColour();
    scan();
    // checks if at base
    if (colour == 7) {
      atBase = false;
      mapping = true;
      findingBase = false;
      manualMode = false;
      collecting = false;

      Serial.write('B');

      Serial.write(':');
  
      path.clear();
      conversePaths.clear();
      dfs.clear();
      toBase.clear();
      pos.clear();
      curPos = 1;
      pathNumber = 0;
      rightT = 0;
      forward = 0;
      counter = 0;
      endPath = false;

      Serial.write(13);

      rightMotorSpeed = 150;
      rightReverseSpeed = 0;
      leftMotorSpeed = 0; 
      leftReverseSpeed = 200;
      steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
      leftT = 1;
  
      for (int i=0; i<8; i++) {
        curPathArr[i] = 0;
      }
    }
    else {
      Serial.write('5');
      if (sensorValueLeft < lowThreshold && sensorValueCentre < lowThreshold && sensorValueRight < lowThreshold) {
        Serial.write('6'); // '6' represents 54
      }
      else {
        findTheBase();
      }
    }
  }
  else if (s >= 4 && s <= 8 && !mapping && !collecting) {
    steer(0, 0, 0, 0);

    tcs.getRGB(&r, &g, &b);
    colour = checkColour();
    
    if (colour == 7) {
      atBase = false;
      manualMode = false;
      collecting = true;
      mapping = false;
      findingBase = false;
  
      for (int i=0; i<8; i++) {
        curPathArr[i] = 0;
      }
  
      leftT = 1;
      rightMotorSpeed = 150;
      rightReverseSpeed = 0;
      leftMotorSpeed = 0; 
      leftReverseSpeed = 200;
    
      counter = 0;
      pathNumber = 0;

      toBase.clear();
      toBase.add(8);

      conversePaths.clear();

      Serial.write('B');

      if (s==4) {
        Serial.write(';');
      }
      else if (s==5) {
        Serial.write('<');
      }
      else if (s==6) {
        Serial.write('=');
      }
      else if (s==8) {
        Serial.write('>');
      }

      Serial.write(13);
      
      for (int i=0; i<path.size(); i++) {
        if (path.get(i) % 10 == state) {
          conversePaths.add(path.get(i));
        }
        else if (state == 8) {
          conversePaths.add(path.get(i));
        }
      }

      convToArray(pathNumber);
    }
    else {
      Serial.write('5');
    }
  }
}

void findTheBase() {
  Serial.write('7');
  findingBase = true;
  mapping = false;
  collecting = false;
  atBase = false;
  manualMode = false;
}

void loop() {
  // put your main code here, to run repeatedly:
  //loopCounter++;

  if(Serial.available() > 0){ // Checks whether data is comming from the serial port from bluetooth sensor
    state = Serial.read(); // Reads the data from the serial port
    checkStateUI(state);
  }

  if (!atBase && !manualMode) {
    movement();
  }
  else if (atBase) {
    steer(0, 0, 0, 0);
  }

  /*
  tcs.getRGB(&r, &g, &b);
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.println(" ");
  delay(100);
  */

/*
  if (loopCounter%200==0 && !atBase) {
    loopCounter = 0;
    long dis = calcDist();
    maxSpeed = 175;
    if (dis < 30) {
      maxSpeed = 175 - 400/dis;
    }
    
    while (dis < 15 && leftT == 0) {
      loopCounter++;
      if (loopCounter<1500) {
        steer(0, 0, 0, 0);
      }
      else if(loopCounter==1500 && !mapping && !endPath && collecting) {
        endPath = true;
        counter = toBase.size()-1;
        rightMotorSpeed = 150;
        rightReverseSpeed = 0;
        leftMotorSpeed = 0; 
        leftReverseSpeed = 200;
        steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
        leftT = 1;
        for (int j=0; j<toBase.size(); j++) {
          curPathArr[j]=toBase.get(j);
        }
        conversePaths.add(conversePaths.get(pathNumber));
      }
      dis = calcDist();
    }

    loopCounter = 0;
    
  }
  */
 
  delay(2);

}