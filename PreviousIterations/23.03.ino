#include <ezButton.h>
#include <LinkedList.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <Servo.h>
#include "pitches.h"

// colour sensor setup
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

// motor analogue pins
#define MOT_A1_PIN 5
#define MOT_A2_PIN 6
#define MOT_B1_PIN 3
#define MOT_B2_PIN 11

// dc motor pins for grabber
#define in2 8
#define in1 7

// speaker

int melody[] = {NOTE_CS6, NOTE_C6};
int noteDurations[] = {2, 2};

int melody1[] = {NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4};
int noteDurations1[] = {4, 8, 8, 4, 4, 4, 4, 4};

// servo setup
Servo servo;
int servoPos = 70; // servo position variable
int servoColour = 0; // variable to store colour of object held by buggy

// pressure switch setup to detect when gripper has reached bottom of the rod
ezButton LIMIT_SWITCH_END(2);
int switchState;

// ultrasonic sensor pins
const int echo = 13;
const int trig = 12;

int colour; // colour integer
int state; // reading from bluetooth sensor

int redAtBase = 0;
int greenAtBase = 0;
int yellowAtBase = 0;

// IR digital pins
int left = A3;
int centre = A2;
int right = A1;

// integers used to control speed of individual motors
int leftMotorSpeed;
int leftReverseSpeed;
int rightMotorSpeed;
int rightReverseSpeed;

// thresholds of the IR sensors
int lowThreshold = 150;
int highThreshold = 450;

int maxSpeedL = 220; // maximum speed of buggy
int maxSpeedR = 180;

int sensorValueLeft, sensorValueCentre, sensorValueRight; // the values of the IR sensors

LinkedList<long> path = LinkedList<long>(); // path list holds all the possible paths
LinkedList<long> conversePaths = LinkedList<long>(); // list holds paths that the robot will taverse when collecting objects
LinkedList<long> dfs = LinkedList<long>(); // dfs holds uncompleted paths while buggy is mapping (depth first search)
LinkedList<int> toBase = LinkedList<int>(); // holds current position of buggy during collection phase so that it can return to base in case of obstruction to the path

int curPathArr[8]; // holds the current path that the buggy is traversing
int pathNumber = 0; // keeps count of how many paths the buggy has currently completed
int counter = 0; // counter keeps track of which fork of the path it is currently traversing (increases at every fork)

// an integer - keeps track whether the buggy is currently turning or not from a fork or end of line
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
boolean willMap = false; // if rerouting to map - then will begin mapping when base is reached
boolean detection = false; // keeps track if there is an object in the way
boolean holdingObj = false; // keeps track if buggy is holding an object

// a linked list that keeps track of current and past positions of the buggy
// this is to be used with the dfs list to help map the paths
LinkedList<int> pos = LinkedList<int>();

int curPos = 1; // represents the position the buggy is facing (1 = north, 2 = west, 3 = south, 4 = east)

float r, g, b; // red, green, and blue values from the colour sensor

int loopCounter = 0; // keeps track of number of loops to limit distance sensor readings

void setup() {
  Serial.write('H');
  
  // motor pin setup
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  // motors turned off
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  Serial.begin(9600); // starts serial

  LIMIT_SWITCH_END.setDebounceTime(50);
  
  servo.attach(A0); // servo setup
  servo.write(servoPos); // set initial servo position

  // checks for connection to colour sensor
  if (tcs.begin()) {
    //Serial.println("Found sensor");
  } else {
    //Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }

  // setup of distance sensor pins
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);

  // to test without having to map
  //////
  leftT = 1;
  rightMotorSpeed = maxSpeedR;
  rightReverseSpeed = 0;
  leftMotorSpeed = 0; 
  leftReverseSpeed = maxSpeedL;
  
  finishMap = false;

  // red 4, green 5, yellow 6
  path.add(15);
  path.add(214);
  path.add(225);
  path.add(2316);
  path.add(2325);
  path.add(2334);
  path.add(36);
  
  // converts the first path integer into an array
  atBase = true;
  
  ///////

  steer(0, 0, 0, 0);
}

// retrieves IR values
void scan() {
  sensorValueLeft = analogRead(left) * 1.1;
  sensorValueCentre = analogRead(centre) * 1.1;
  sensorValueRight = analogRead(right);
}

void movement() {
  scan(); // retrieves IR values

  //checks if the buggy is currently not turning
  if (rightT == 0 && forward == 0 && leftT == 0) {
    if (sensorValueCentre > highThreshold) { // if buggy is centred
      if (sensorValueLeft < lowThreshold && sensorValueRight < lowThreshold) {
        leftMotorSpeed = maxSpeedL; 
        rightMotorSpeed = maxSpeedR;
        leftReverseSpeed = 0;
        rightReverseSpeed = 0;
      }
      else {
        // speed of motors vary depending on IR values - this allows for a much more accurate line following algorithm
        leftMotorSpeed = maxSpeedL - (4000/sensorValueRight^2); 
        rightMotorSpeed = maxSpeedR - (4000/sensorValueLeft^2);
        leftReverseSpeed = 0;
        rightReverseSpeed = 0;
      }

      // if the buggy has reached a fork in its path
      if (sensorValueRight > lowThreshold && sensorValueLeft > lowThreshold) {
        if (mapping) {
          // calls this function if its trying to map routes
          if (!finishMap) {
            mappingRoutes();
  
            // checks if mapping has been completed (ie last turn before reaching base)
            if (dfs.size() == 0) {
              finishMap = true; // mapping is complete should be now heading directly back to base
            }
          }
          else { // if buggy reaches another fork when expecting to reach base it will reroute back to base and warn user that mapping may be incorrect
            Serial.write('8'); // stored mapping may be incorrect message
            findTheBase();
            finishMap = false;
          }
        }
        else if (findingBase) { // if rerouting to base
          leftT = 1; // turn left if trying to find the base
          choosePath();
        }
        else { // if collecting object - will choose path based on values in current path array
          choosePath(); 
        }
      }
    }
    else{ // if buggy has come away from the line (eg buggy needs additional correcting or reached end of line)
      if (sensorValueRight > lowThreshold) { //if buggy needs additional correction to the right (should hardly happen)
        rightMotorSpeed = 0;
        rightReverseSpeed = maxSpeedR - (4000/sensorValueRight) + 25;
        leftMotorSpeed = maxSpeedL - 4000/sensorValueRight; 
        leftReverseSpeed = 0;
      }
      else if (sensorValueLeft > lowThreshold) { // buggy needs additional correcting to the left (should hardly happen)
        leftMotorSpeed = 0;
        leftReverseSpeed = maxSpeedL - (4000/sensorValueLeft) + 25;
        rightMotorSpeed = maxSpeedR - 4000/sensorValueLeft;
        rightReverseSpeed = 0;
      }
      else { //if buggy reaches end of path (left, centre and right sensors are all picking up white)
        if (sensorValueLeft < lowThreshold && sensorValueCentre < lowThreshold && sensorValueRight < lowThreshold) {
          if (finishMap) { // should have reached base
            steer(0, 0, 0, 0);
            colour = checkColour();
  
            // checks that it is actually at the base (7 = white)
            if (colour == 7) {
              Serial.write('B'); // returned to base message to UI
              
              // ready to turn around for next function
              leftT = 1;
              rightMotorSpeed = maxSpeedR;
              rightReverseSpeed = 0;
              leftMotorSpeed = 0; 
              leftReverseSpeed = maxSpeedL;
            
              mapping = false; // no longer mapping
              finishMap = false;
              counter = 0; // reset path values
              pathNumber = 0;
    
              atBase = true; // returned to base
            }
            else {
              // if it has not returned to base then will search for base and recommend to user that paths may be incorrect or not complete
              findTheBase();
              finishMap = false;
              Serial.write('8'); // stored mapping may be incorrect message to UI
            }
          }
          // uses this condition if the buggy is currently mapping the paths and comes to the end of a line
          else if (mapping) {
            steer(0, 0, 0, 0);
  
            colour = checkColour(); // checks colour at end of path
  
            // checks for base ( if mapping it should not have reached base as of yet )
            if (colour == 7) {
              findTheBase(); // if at base it warns user that it may have incorrectly mapped paths
              Serial.write('8'); // stored mapping may be incorrect message to UI
            }
            else {
              Serial.write(colour); // communicates to user what colour has been detected
    
              long pathCode = dfs.get(0)*10+colour; // adds colour value to pathcode integer which represents each path
  
              // path integer added to the list of paths (this path list can then be later manipulated)
              path.add(pathCode);
    
              Serial.write(1); // 1 integer represents to the UI that path code is about to be sent over
              delay(50);
              while (pathCode != 0) {
                Serial.write(pathCode%10); // sends path code as individual integers to be easily received on UI
                pathCode /= 10;
                delay(50);
              }
              Serial.write(0); // 0 integer represents to UI that path code has been fully sent over
              
              dfs.shift(); // removes path from dfs list
    
              // turns left 180 degrees to get back onto line
              rightMotorSpeed = maxSpeedR;
              rightReverseSpeed = 0;
              leftMotorSpeed = 0; 
              leftReverseSpeed = maxSpeedL;
              steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
              leftT = 1;
    
              // adds old position to position list
              pos.add(curPos);
              Serial.write(13); // turning around message to UI

              sound();
    
              // if buggy has tunred around the current position integer of the buggy should be flipped
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
  
            // turn left if not at base
            leftT = 1;
            rightMotorSpeed = maxSpeedR;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = maxSpeedL;
            
            if (colour == 7) { // if found base - reset variables
              findingBase = false;
              atBase = true;
              manualMode = false;
              collecting = false;
              mapping = false;
  
              Serial.write('B'); // returned to base message to UI
          
              conversePaths.clear();
              for (int i=0; i<8; i++) {
                curPathArr[i] = 0;
              }
            
              counter = 0;
              pathNumber = 0;
        
              endPath = false;
              toBase.clear();
              toBase.add(8);
  
              if (willMap) {
                checkStateUI(53); // if mapping had originally been selected then buggy will begin mapping phase from base
                willMap = false;
              }

              if (holdingObj) {
                dropOff();
              }
            }
            else { // if not at base turn around and continue searching
              Serial.write(13); // turning around message to UI
            }
  
          }
          else { // when buggy is in collection phase
            if (curPathArr[counter] == 0 && collecting) { // if array reaches a 0 this means it has reached the end of the path and is ready to pick up object
              steer(0, 0, 0, 0);

              if (!holdingObj) {
                if (curPathArr[0] != colour) { // if colour at end of path does not match mapped colour then it will not pick up object
                  Serial.write('8'); // stored mapping may be incorrect message to UI
                }
                if (colour == 4) { // red
                  Serial.write('?'); // picking up red message to UI
                }
                else if (colour == 5) { // green
                  Serial.write('@'); // picking up green message to UI
                }
                else if (colour == 6) { // yellow
                  Serial.write('A'); // picking up yellow message to UI
                }
                
                colour = checkColour();
                pickUpEx(); // pickup function
              }
              else {
                pickUpRe();
                
                endPath = true;
    
                // turns around to head back to base
                rightMotorSpeed = maxSpeedR;
                rightReverseSpeed = 0;
                leftMotorSpeed = 0; 
                leftReverseSpeed = maxSpeedL;
                steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
                leftT = 1;
    
                counter--;
                changeDirection(); // flips path route to return back to base
                Serial.write(13); // turning around message to UI
              }
            }
            else if (curPathArr[counter] > 3 && collecting) { // if buggy has returned to base
              steer(0, 0, 0, 0);
              delay(500);
              colour = checkColour();
  
              if (colour == 7) { //checks that it is actually at base
                Serial.write('B'); // returned to base message to UI

                if (holdingObj) {
                  dropOff();
                }
              
                endPath = false;
  
                if (servoColour == 5) { // if green
                  // ready buggy to turn right to avoid base line
                  rightMotorSpeed = 0;
                  rightReverseSpeed = maxSpeedR;
                  leftMotorSpeed = maxSpeedL; 
                  leftReverseSpeed = 0;
                  rightT = 1;
                }
                else { // if already positioned on or left of base line
                  // ready buggy to turn left again
                  rightMotorSpeed = maxSpeedR;
                  rightReverseSpeed = 0;
                  leftMotorSpeed = 0; 
                  leftReverseSpeed = maxSpeedL;
                  leftT = 1;
                }
                
                toBase.clear();
                toBase.add(8);
                
                if (pathNumber < conversePaths.size()-1) { // if there are still more paths to converse will continue to collect items
                  pathNumber++;
                  
                  convToArray(pathNumber); // converts next path integer into an array
                  counter = 1;
                  atBase = false;
                  
                  if (curPathArr[0] == 4) {
                    Serial.write(';'); // collecting red message to UI
                  }
                  else if (curPathArr[0] == 5) {
                    Serial.write('<'); // collecting green message to UI
                  }
                  else if (curPathArr[0] == 6) {
                    Serial.write('='); // collecting yellow message to UI
                  }
                  
                  Serial.write(13); // turning around message to UI
             
                }
                else { // if not then buggy will stop
                  colour = checkColour(); // buggy will have shifted off base to drop off object so necessary to recheck if on base
  
                  if (colour != 7) { // if shifted off of base
                    if (servoColour == 5) { // if placing green buggy will have shifted to the right so will be necessary to shift back to the left onto the line and ready to go again
                      rightMotorSpeed = 0;
                      rightReverseSpeed = 0;
                      leftMotorSpeed = 0; 
                      leftReverseSpeed = maxSpeedL;
                      leftT = 1;
                    }
                    else if (servoColour == 6) { // if placing yellow buggy will be shifted to the left so will be necessary to shift back to the right onto the line
                      rightMotorSpeed = 0;
                      rightReverseSpeed = maxSpeedR;
                      leftMotorSpeed = 0; 
                      leftReverseSpeed = 0;
                      rightT = 1;
                    }
                    servoColour = 0; // reset servo colour once done
                  }
                  else { // if still at base then it should be ready to go again
                    counter = 0;
                    pathNumber = 0;
                    conversePaths.clear();
                    collecting = false;
                    atBase = true;
                  }
                }
              }
              else { // if buggy is not at base it will go in search of base and warn user of incorrect mapping
                findTheBase();
              }
            }
            else { // if buggy has reached end of path when it should be approaching a fork it will relocate to base and warn user of incorrect mapping
              findTheBase();
            }
          }
        } 
      }
    }
  }
  else { // dont correct until finished turning - leftT, rightT, and forward variables represent the buggy turning (so when turning line does not intefer with buggys direction)
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
    /*
    else if (sensorValueRight < lowThreshold && sensorValueCentre > lowThreshold && sensorValueLeft > lowThreshold) {
      leftT = 0;
    }
    else if (sensorValueRight > lowThreshold && sensorValueCentre > lowThreshold && sensorValueLeft < lowThreshold) {
      rightT = 0;
    }
    */
  }

  // steer function controls motor
  steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
  
}

void mappingRoutes() {
  
  pos.add(curPos); // pos list keeps track of buggy position

  int prevPos;
  
  if (pos.size() > 1) { // prev pos retrieves previous position in position list
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
      dfs.add(0, p*10+3); // right
      dfs.add(0, p*10+2); // forward
      dfs.add(0, p*10+1); // left
    }
  }
  else { // if buggy has returned on itself then remove last two positions from pos list
    pos.pop();
    pos.pop();
  }

  leftT = 1; // take left turn

  // changes position of buggy (1 = north, 2 = west, etc)
  if (curPos == 4) { // goes back on itself if reached end of position
    curPos = 1;
  }
  else { // changes position as orientation is changing
    curPos += 1;
  }

  choosePath(); // chooses path
}

void choosePath() {
  if (curPathArr[counter] == 1 || leftT == 1) { // buggy will turn left
    rightMotorSpeed = maxSpeedR;
    rightReverseSpeed = 0;
    leftMotorSpeed = 0; 
    leftReverseSpeed = 0;
    leftT = 1;
    
    Serial.write(14); // turning left message to UI
    
    if (!mapping) {
      toBase.add(3); // will add position to list so buggy can return back to base if needed
    }
  }
  else if (curPathArr[counter] == 2 || forward == 1) { // buggy will continue straight
    rightMotorSpeed = maxSpeedR;
    rightReverseSpeed = 0;
    leftMotorSpeed = maxSpeedL; 
    leftReverseSpeed = 0;
    forward = 1;
    
    Serial.write(11); // going forwards message to UI
    
    if (!mapping) {
      toBase.add(2); // will add position to list so buggy can return back to base if needed
    }
  }
  else if (curPathArr[counter] == 3 || rightT == 1) { // buggy will turn right
    rightMotorSpeed = 0;
    rightReverseSpeed = 0;
    leftMotorSpeed = maxSpeedL; 
    leftReverseSpeed = 0;
    rightT = 1;
    
    Serial.write(12); // going right message to UI
    
    if (!mapping) {
      toBase.add(1); // will add position to list so buggy can return back to base if needed
    }
  }
  else { // if buggy should not be at a fork then will reroute to base
    findTheBase();
    Serial.write('8'); // stored mapping may be incorrect message to UI
  }
  
  steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
  
  if (!endPath) {
    counter++; // increments list to get to item
  }
  else {
    counter--; // decrements list to get back to base
  }
}

void convToArray(int pathNo) { // converts integer from conversePaths list to an array
  int curPath = conversePaths.get(pathNo); // retrieves path integer

  int reverseArr[8];
  
  for (int j=0; j<8; j++) { // clears array
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

  // array reversal - needs to be reversed as this is how the program deals with the array 
  for (int j=0; j<endOfPathIndex; j++) {
    curPathArr[j+1] = reverseArr[endOfPathIndex-j];
  }
  
  counter++;

}

void changeDirection() { // takes array and flips values for buggy to return back to base
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

void steer(int m1p1, int m1p2, int m2p1, int m2p2) { // controls dc motors
  analogWrite(MOT_A1_PIN, m1p1);
  analogWrite(MOT_A2_PIN, m1p2);
  analogWrite(MOT_B1_PIN, m2p1);
  analogWrite(MOT_B2_PIN, m2p2);
}

int checkColour() { // receives colour from colour sensor
  int c;

  tcs.getRGB(&r, &g, &b); // gets colour sensor reading
  c = detectColour(r, g, b); // checks what colour

  int i = 0;
  while (i<10) { // will repeat 10 times until detects same colour every time
    int oldCol = c;
    tcs.getRGB(&r, &g, &b);
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

int detectColour( int r ,int g , int b){ // takes r,g,b values and returns colour
 
  int c = 8;

  if(r >= 140 && g <= 80 && b <= 80) {
   // red
   c = 4;
  }
  else if(r <= 100 && g  >= 97 && b <= 100){
   // green
   c = 5;
  }
  else if(r >= 90 && r <= 140 && g>= 60 && g<= 105 && b <= 60){
   // yellow
   c = 6;
  }
  else if (r >= 70 && r <= 110 && g >= 70 && g <= 97 && b >= 60 && b <= 100) {
    // white
    c = 7;
  }

  return c; // returns colour
}

long calcDist() { // returns distance sensor reading
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

void checkStateUI(int s) { // checks values returned from bluetooth sensor
  if (s == 9) { // if user would like buggy to return to base
    if (collecting) { // if buggy is collecting
      if (!endPath) { // and buggy is on its way back to base
        Serial.write('D'); // returning to base message to UI
        endPath = true;
        counter = toBase.size()-1;
        
        rightMotorSpeed = maxSpeedR;
        rightReverseSpeed = 0;
        leftMotorSpeed = 0; 
        leftReverseSpeed = maxSpeedL;
        steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
        leftT = 1;

        // will use the values from the toBase list to return to base
        for (int j=0; j<toBase.size(); j++) {
          curPathArr[j]=toBase.get(j);
        }
        conversePaths.clear();
      }
      else {
        conversePaths.clear(); // if it is on its way back converse paths will be cleared so that it does not collect anything else
      }
    }
    else {
      findTheBase(); // if not collecting it will reroute back to base by checking every path
    }
  }
  else if (s == 48) { // manual left
    manualMode = true;
    steer(maxSpeedL, 0, 0, maxSpeedR);
    atBase = false;
    mapping = false;
    collecting = false;
    findingBase = false;
  }
  else if (s == 49) { // manual backwards
    manualMode = true;
    steer(maxSpeedL, 0, maxSpeedR, 0);
    atBase = false;
    mapping = false;
    collecting = false;
    findingBase = false;
  }
  else if (s == 50) { // manual right
    manualMode = true;
    steer(0, maxSpeedL, maxSpeedR, 0);
    atBase = false;
    mapping = false;
    collecting = false;
    findingBase = false;
  }
  else if (s == 51) { // manual forwards
    manualMode = true;
    steer(0, maxSpeedL, 0, maxSpeedR);
    atBase = false;
    mapping = false;
    collecting = false;
    findingBase = false;
  }
  else if (s == 52) { // manual force stop
    manualMode = true;
    steer(0, 0, 0, 0);
    atBase = false;
    mapping = false;
    collecting = false;
    findingBase = false;
  }
  else if (s==54 && manualMode) { // manual pick up from gripper
    pickUpEx();
    pickUpRe();
    holdingObj = false;
  }
  else if (s==55 && manualMode) { // manual drop off from gripper
    dropOff();
  }
  else if (s==56) {
    detection = true;
    sound();
    sound();
    sound();
    detection = false;
  }
  else if (s == 53 && !mapping && !collecting && !findingBase) { // user selected mapping
    steer(0, 0, 0, 0);

    tcs.getRGB(&r, &g, &b);
    colour = checkColour(); // checks colour
    scan(); // retrieves IR sensor values
    
    // checks if at base
    if (colour == 7) {
      // resets variable if at base and ready to map
      atBase = false;
      mapping = true;
      findingBase = false;
      manualMode = false;
      collecting = false;

      redAtBase = 0;
      greenAtBase = 0;
      yellowAtBase = 0;

      Serial.write(':'); // sends mapping message to UI
  
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

      Serial.write(13); // send turn around message to UI

      rightMotorSpeed = maxSpeedR;
      rightReverseSpeed = 0;
      leftMotorSpeed = 0; 
      leftReverseSpeed = maxSpeedL;
      steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
      leftT = 1;
  
      for (int i=0; i<8; i++) {
        curPathArr[i] = 0;
      }
    }
    else {
      Serial.write('5'); // not at base message to UI
      if (sensorValueLeft < lowThreshold && sensorValueCentre < lowThreshold && sensorValueRight < lowThreshold) { // if not on line and not on base buggy will not move and alert user
        Serial.write('6'); // not on line message to UI
      }
      else { // if on line and not at base - buggy will reroute to base
        findTheBase();
        willMap = true;
      }
    }
  }
  else if (s >= 4 && s <= 8 && !mapping && !collecting) { // user selecting choose colour from UI
    steer(0, 0, 0, 0);

    tcs.getRGB(&r, &g, &b);
    colour = checkColour(); // checks colour
    
    if (colour == 7) { // checks for base
      // resets variables
      atBase = false;
      manualMode = false;
      collecting = true;
      mapping = false;
      findingBase = false;
  
      for (int i=0; i<8; i++) {
        curPathArr[i] = 0;
      }
  
      leftT = 1;
      rightMotorSpeed = maxSpeedR;
      rightReverseSpeed = 0;
      leftMotorSpeed = 0; 
      leftReverseSpeed = maxSpeedL;
    
      counter = 0;
      pathNumber = 0;

      toBase.clear();
      toBase.add(8);

      conversePaths.clear();

      // will add correct colours to converse path lists
      for (int i=0; i<path.size(); i++) {
        if (path.get(i) % 10 == state) {
          conversePaths.add(path.get(i));
        }
        else if (state == 8) { // if all is selected will add all paths to converse paths list
          if (path.get(i) % 10 != 8) { // if not colour was detected at end of path buggy will not collect
            conversePaths.add(path.get(i));
          }
        }
      }

      // if list is empty (ie mapping is yet to be done or colour not present) then buggy will not move
      if (conversePaths.size() == 0) {
        findingBase = false;
        atBase = true;
        manualMode = false;
        collecting = false;
        mapping = false;
        Serial.write('C'); // colour not present message to UI
      }
      else {
        convToArray(pathNumber); // converts first path into an array
        
        if (s==4) {
          Serial.write(';'); // collecting red message to UI
        }
        else if (s==5) {
          Serial.write('<'); // collecting green message to UI
        }
        else if (s==6) {
          Serial.write('='); // collecting yellow message to UI
        }
        else if (s==8) {
          Serial.write('>'); // collecting all message to UI
          if (curPathArr[0] == 4) {
            Serial.write(';'); // collecting red message to UI
          }
          else if (curPathArr[0] == 5) {
            Serial.write('<'); // collecting green message to UI
          }
          else if (curPathArr[0] == 6) {
            Serial.write('='); // collecting yellow message to UI
          }
        }
  
        Serial.write(13); // turn around message to UI
      }
    }
    else {
      Serial.write('5'); // not at base message to UI
    }
  }
}

void findTheBase() { // resets variables for rerouting to base
  Serial.write('7'); // rerouting to base message to UI
  // resets variables 
  findingBase = true;
  mapping = false;
  collecting = false;
  atBase = false;
  manualMode = false;
}

void pickUpEx() {
  servoColour = colour; // saves colour of object so knows where to put down

  steer(maxSpeedL + 20, 0, maxSpeedR + 20, 0);
  delay(400);
  steer(0, 0, 0, 0);
  delay(300);
  
  // need to read value of switch multiple times as there was an issue taking the first reading
  for (int i=0; i<5; i++) {
    LIMIT_SWITCH_END.loop();
    switchState = LIMIT_SWITCH_END.getState(); // gets limit switch state
    delay(100);
  }

  servoPos = 80;
  servo.write(servoPos); // writes initial position to servo
  delay(500);
  
  while (switchState == HIGH) { // will spin dc motor until reached limit switch
    LIMIT_SWITCH_END.loop();
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    switchState = LIMIT_SWITCH_END.getState();
    //servo.write(servoPos);
  }

  digitalWrite(in1, LOW); // turn off dc motors
  digitalWrite(in2, LOW);

  servoPos = 30;
  servo.write(servoPos);
  delay(500);

  holdingObj = true;
}

void pickUpRe() { // code to pick up objects

  servoPos = 80;
  servo.write(servoPos); // close gripper
  delay(500);

  // will move motor back up for 10 seconds
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(10000);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

}

void dropOff() {
  if (servoColour == 4) { // red
    if (redAtBase >= 2) {
      steer(0, maxSpeedL, maxSpeedR, 0); // rotates slightly to the right for additional objects
      delay(redAtBase*200 - 400);
    }
    else {
      steer(200, 0, 0, 150); // rotates slightly to left for first two objects
      delay(400 - redAtBase*200);
    }
    steer(0, 0, 0, 0);
    redAtBase++; // keeps track of number of red objects at base
  }
  else if (servoColour == 5) { // green
    steer(0, maxSpeedL, maxSpeedR, 0); // rotates to right to drop off object
    delay(750 - greenAtBase*150);
    steer(0, 0, 0, 0);
    greenAtBase++; // keeps track of number of green objects at base
  }
  else if (servoColour == 6) {
    steer(maxSpeedL, 0, 0, maxSpeedR); // rotates to left to drop off object
    delay(750 - yellowAtBase*150);
    steer(0, 0, 0, 0);
    yellowAtBase++; // keeps track of number of yellow objects at base
  }
  
  // need to read value of switch multiple times as there was an issue taking the first reading
  steer(0, maxSpeedL, 0, maxSpeedR);
  delay(200);
  steer(0, 0, 0, 0);
  delay(500);
  
  for (int i=0; i<5; i++) {
    LIMIT_SWITCH_END.loop();
    switchState = LIMIT_SWITCH_END.getState(); // gets limit switch state
    delay(100);
  }

  servoPos = 80;
  servo.write(servoPos); // writes initial position to servo

  delay(200);
  
  while (switchState == HIGH) { // goes down until switch is hit
    LIMIT_SWITCH_END.loop();
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    switchState = LIMIT_SWITCH_END.getState();
    servo.write(servoPos);
  }
  
  digitalWrite(in1, LOW); // turns off motor
  digitalWrite(in2, LOW);

  // opens up servo to place object
  servoPos = 30;
  servo.write(servoPos);
  delay(500);

  steer(maxSpeedL, 0, maxSpeedR, 0);
  delay(500);
  steer(0, 0, 0, 0);
  delay(500);
  
  servoPos = 80;
  servo.write(servoPos);
  delay(500);

  // motor turns grabber back in for 13 seconds
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(10000);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  Serial.write('F'); // placing down object message to UI

  holdingObj = false;
}

void sound() {
  int numNotes;
  if (detection) {
    numNotes = 2;
  }
  else {
    numNotes = 8;
  }
  for (int thisNote = 0; thisNote < numNotes; thisNote++) {
    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration;
    if (detection) {
      noteDuration = 1000/noteDurations[thisNote];
      tone(10, melody[thisNote],noteDuration);
    }
    else {
      noteDuration = 1000/noteDurations1[thisNote];
      tone(10, melody1[thisNote],noteDuration);
    }

    // to distinguish the notes, set a minimum time between them.
    // the note’s duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(10);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  loopCounter++;

  if(Serial.available() > 0){ // Checks whether data is comming from the serial port from bluetooth sensor
    state = Serial.read(); // Reads the data from the serial port
    checkStateUI(state);
  }

  if (!atBase && !manualMode) { // line following program
    movement();
  }
  else if (atBase) {
    steer(0, 0, 0, 0); // will not move if at base
  }

  if (loopCounter%200==0) { // will only check for distance every 200 iterations of the loop otherwise there was too much delay from the sensor
    servo.write(servoPos); // in industry it may be wise to keep servo position updated more regularly to ensure buggy would not drop any barrels 
  }

  if (loopCounter%200==0 && !atBase) { // will only check for distance every 200 iterations of the loop otherwise there was too much delay from the sensor
    if (collecting) { // locks servo position every 200 iterations of the loop - saves on power by not locking every single iteration
      servo.write(servoPos); // in industry it may be wise to keep servo position updated more regularly to ensure buggy would not drop any barrels 
    }
    if (!atBase) {
      loopCounter = 0;
      long dis = calcDist();
   
      if (dis < 30) { // will slow down speed until it reaches object
        maxSpeedL = 220 - 700/dis;
        maxSpeedR = 180 - 700/dis;
        
        if (detection == false) {
          Serial.write('G'); // object detected message to UI
          detection = true;
        }

        sound();
      
        while (dis < 15 && leftT == 0) {
          sound();
          loopCounter++;
          if (loopCounter<1500) { 
            steer(0, 0, 0, 0); // buggy will stop if within 15cm
          }
          else if(loopCounter==1500 && !mapping && !endPath && collecting) { // if been stopped for 1500 iterations buggy will turn around and return to base
            Serial.write('D'); // returning to base message to UI
            endPath = true;
            counter = toBase.size()-1;
            
            rightMotorSpeed = maxSpeedR;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = maxSpeedL;
            steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
            leftT = 1;
            
            for (int j=0; j<toBase.size(); j++) {
              curPathArr[j]=toBase.get(j);
            }
            conversePaths.add(conversePaths.get(pathNumber));
          }
          dis = calcDist();
        }
      }
      else {
        detection = false;
        maxSpeedL = 220;
        maxSpeedR = 180;
      }
  
      loopCounter = 0;

    }
    
  }
 
  delay(2);

}
