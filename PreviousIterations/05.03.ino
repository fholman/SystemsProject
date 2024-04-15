#include <LinkedList.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

// motor analogue pins
#define MOT_A1_PIN 5
#define MOT_A2_PIN 6
#define MOT_B1_PIN 3
#define MOT_B2_PIN 11

// ultrasonic sensor pins
const int echo = 12;
const int trig = 11;

boolean initReturn = false;
boolean findingBase = false;

int colour; 
int state;

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
int lowThreshold = 250;
int highThreshold = 500;
// maximum speed of buggy
int maxSpeed = 225;

// the values of the IR sensors
int sensorValueLeft, sensorValueCentre, sensorValueRight;

boolean finishMap = true;

boolean manualMode = false;

boolean atBase = false;

// path list holds all the paths
LinkedList<long> path = LinkedList<long>();
// list holds paths that robot will use
LinkedList<long> conversePaths = LinkedList<long>();
// dfs holds uncompleted paths while buggy is mapping
LinkedList<long> dfs = LinkedList<long>();

LinkedList<int> toBase = LinkedList<int>();
// holds the current path that the buggy is traversing
int curPathArr[8];
// keeps count of how many paths the buggy has currently completed
int pathNumber = 0;
// counter keeps track of which fork of the path it is currently traversing (increases at every fork)
int counter = 0;
// an integer (could probably be a boolean) - keeps track whether the buggy is currently turning or not from a fork or end of line
int rightT = 0;
int forward = 0;
int leftT = 0;
// boolean that keeps track whether the buggy has reached the end of a path so it knows when to turn around
boolean endPath = false;
// turns true when buggy has traversed every path in the path list
boolean completed = false;
// keeps track of whether the buggy is mapping or not - done at the beginning of the program
boolean mapping = false;
boolean collecting = false;

// a linked list that keeps track of current and past positions of the buggy
// this is to be used with the dfs list to help map the paths
LinkedList<int> pos = LinkedList<int>();
// represents the position the buggy is facing (1 = north, 2 = west, 3 = south, 4 = east)
int curPos = 1;

float r, g, b;

int loopCounter = 0;

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

  Serial.begin(9600);

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
  rightMotorSpeed = 200;
  rightReverseSpeed = 0;
  leftMotorSpeed = 0; 
  leftReverseSpeed = 200;
  
  finishMap = false;
  counter = 0;
  pathNumber = 0;

  // red 4, green 5, yellow 6
  path.add(15);
  path.add(214);
  path.add(225);
  path.add(2316);
  path.add(2325);
  path.add(2334);
  path.add(36);
  
  // converts the first path integer into an array
  //convToArray(pathNumber);
  atBase = true;
  */
  
  ///////

  // starts it in manual mode - must click map to start
  manualMode = true;
  steer(0, 0, 0, 0);
  completed = true;
  atBase = false;
}

// retrieves IR values
void scan() {
  sensorValueLeft = analogRead(left) * 0.96;
  sensorValueCentre = analogRead(centre);
  sensorValueRight = analogRead(right) * 1.03;
}

void movement() {
  // retrieves IR values
  scan();

  //if the buggy is currently not turning
  if (rightT == 0 && forward == 0 && leftT == 0) {
    if (sensorValueCentre > lowThreshold) { //if buggy is centred
      leftMotorSpeed = maxSpeed - 4000/sensorValueRight; 
      rightMotorSpeed = maxSpeed - 4000/sensorValueLeft;
      leftReverseSpeed = 0;
      rightReverseSpeed = 0;

      // if the buggy has reached a fork in its path
      if (sensorValueRight > lowThreshold && sensorValueLeft > lowThreshold && sensorValueCentre > highThreshold) {
        if (mapping) {
          // calls this function if its trying to map routes
          mappingRoutes();

          if (dfs.size() == 0) {
            mapping = false;
            finishMap = true;
          }
        }
        else if (findingBase) {
          leftT = 1;
          choosePath();
        }
        else {
          // calls this function if it is trying to locate an object
          choosePath(); 
        }
      }
    }
    else{ // if buggy has come away from the line (eg buggy needs additional correcting, buggy has come across an object (this will change), buggy is at base (will also change))
      if (sensorValueRight > lowThreshold) { //if buggy needs additional correction to the right - should hardly happen
        rightMotorSpeed = 0;
        rightReverseSpeed = maxSpeed - 10000/(sensorValueRight-200);
        leftMotorSpeed = maxSpeed - 4000/sensorValueRight; 
        leftReverseSpeed = 0;
      }
      else if (sensorValueLeft > lowThreshold) { // buggy needs additional correcting to the left - should hardly happen
        leftMotorSpeed = 0;
        leftReverseSpeed = maxSpeed - 10000/(sensorValueLeft-200);
        rightMotorSpeed = maxSpeed - 4000/sensorValueLeft;
        rightReverseSpeed = 0;
      }
      else { //if buggy reaches end of path (so left, centre and right sensors are all picking up white
        // uses this condition if the buggy is currently mapping the paths
        if (mapping) {
          // if the buggy has reached the end of a path it will add the path integer to the list of paths (this path list can then be later manipulated)
          tcs.getRGB(&r, &g, &b);

          steer(0, 0, 0, 0);
          
          int i = 0;
          while (i<25) {
            int oldCol = colour;
            tcs.getRGB(&r, &g, &b);
            // could take a colour average over multiple iterations if it is not be too accurate
            colour = detectColour(r, g, b);
            if (oldCol == colour) {
              i++;
            }
            else {
              i=0;
            }
          }

          // to check that colour is correct
          Serial.write(colour);
          delay(500);

          long pathCode = dfs.get(0)*10+colour;

          path.add(pathCode);

          Serial.write(1);
          delay(100);
          while (pathCode != 0) {
            Serial.write(pathCode%10);
            pathCode /= 10;
            delay(100);
          }
          Serial.write(0);
          
          dfs.shift();

          // turns left 180 degrees to get back onto line
          rightMotorSpeed = 200;
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
        else if (findingBase) {
          tcs.getRGB(&r, &g, &b);
          colour = detectColour(r, g, b);
          if (colour == 7) {
            //Serial.write('7');
            findingBase = false;
          }
          else {
            rightMotorSpeed = 200;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 200;
            steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
            leftT = 1;
          }
        }
        else {
          if (finishMap) {
            // this means it has reached base after finishing mapping the routes
            initReturn = true;
            leftT = 1;
            rightMotorSpeed = 200;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 200;
          
            finishMap = false;
            counter = 0;
            pathNumber = 0;

            // converts the first path integer into an array
            //convToArray(pathNumber);
            atBase = true;
          }
          else if (curPathArr[counter] == 0) {
            // code for arm should go in here
            
            endPath = true;
            rightMotorSpeed = 0;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 0;
            steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
            delay(1500);
            
            rightMotorSpeed = 200;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 200;
            steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
            leftT = 1;
            counter--;
            changeDirection();
            Serial.write(13);
          }
          else if (curPathArr[counter] > 3) {
            initReturn = true;
            endPath = false;
            rightMotorSpeed = 200;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 200;
            leftT = 1;
            atBase = true;
            
            if (pathNumber < conversePaths.size()-1) {
              pathNumber++;
              convToArray(pathNumber);
              Serial.write(13);
            }
            else {
              //completed = true;

              leftT = 1;
              rightMotorSpeed = 200;
              rightReverseSpeed = 0;
              leftMotorSpeed = 0; 
              leftReverseSpeed = 200;
            
              counter = 0;
              pathNumber = 0;
  
              // converts the first path integer into an array
              //convToArray(pathNumber);
              atBase = true;
              conversePaths.clear();

            }

            counter = 0;
          }
        }
      } 
    }
  }
  else { // dont correct until finished turning
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
  
  steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
  
}

void mappingRoutes() {
  
  pos.add(curPos);

  //int curPos = pos.get(pos.size()-1); // receieve last position
  //Serial.println(curPos);
  int prevPos;
  
  if (pos.size() > 1) {
    prevPos = pos.get(pos.size()-2);
  }
  else {
    prevPos = curPos;
  }

  if (!(abs(prevPos - curPos) == 2)) {
    if (dfs.size() == 0) {
      dfs.add(1);
      dfs.add(2);
      dfs.add(3);
    }
    else {
      int p = dfs.shift();
      dfs.add(0, p*10+3);
      dfs.add(0, p*10+2);
      dfs.add(0, p*10+1);
    }
  }
  else {
    pos.pop();
    pos.pop();
  }

  leftT = 1;

  if (curPos == 4) {
    curPos = 1;
  }
  else {
    curPos += 1;
  }

  choosePath();
}

void choosePath() {
  if (curPathArr[counter] == 1 || leftT == 1) {
    rightMotorSpeed = 170;
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
    rightMotorSpeed = 170;
    rightReverseSpeed = 0;
    leftMotorSpeed = 210; 
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
    leftMotorSpeed = 210; 
    leftReverseSpeed = 0;
    rightT = 1;
    Serial.write(12);
    if (!mapping) {
      toBase.add(1);
    }
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

int detectColour( int r ,int g , int b){
 
  //colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  colour = 8;

  if(r >= 140 & g <= 75 & b <= 75) {
   // red
   colour = 4;
  }
  else if(r <= 100 & g  >= 95 & b <= 100){
   // green
   colour = 5;
  }
  else if(r >= 90 & g>= 60 & b <= 60){
   // yellow
   colour = 6;
  }
  else if (r >= 80 && g >= 75 && b >= 65) {
    // white
    colour = 7;
  }

  return colour;
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
    steer(200, 0, 0, 200);
    completed = true;
    atBase = false;
    mapping = false;
    collecting = false;
  }
  else if (s == 49) { //  backwards
    manualMode = true;
    steer(200, 0, 200, 0);
    completed = true;
    atBase = false;
    mapping = false;
    collecting = false;
  }
  else if (s == 50) { // right
    manualMode = true;
    steer(0, 200, 200, 0);
    completed = true;
    atBase = false;
    mapping = false;
    collecting = false;
  }
  else if (s == 51) { // forwards
    manualMode = true;
    steer(0, 200, 0, 200);
    completed = true;
    atBase = false;
    mapping = false;
    collecting = false;
  }
  else if (s == 52) { // force stop
    manualMode = true;
    steer(0, 0, 0, 0);
    completed = true;
    atBase = false;
    mapping = false;
    collecting = false;
  }
  else if (s == 53 && !mapping) { // mapping
    tcs.getRGB(&r, &g, &b);
    colour = detectColour(r, g, b);
    if (colour == 7) {
      scan();
      if (sensorValueLeft < lowThreshold && sensorValueCentre < lowThreshold && sensorValueRight < lowThreshold) {
        Serial.write('6'); // '6' represents 54
      }
      else {
        completed = false;
        atBase = false;
        mapping = true;
    
        path.clear();
        conversePaths.clear();
        dfs.clear();
        toBase.clear();
        pos.clear();
        curPos = 1;
        pathNumber = 0;
        rightT = 0;
        forward = 0;
        leftT = 0;
        counter = 0;
        endPath = false;

        rightMotorSpeed = 200;
        rightReverseSpeed = 0;
        leftMotorSpeed = 0; 
        leftReverseSpeed = 200;
        steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
        leftT = 1;
    
        for (int i=0; i<8; i++) {
          curPathArr[i] = 0;
        }
      }
    }
    else {
      Serial.write('5');
    }
  }
  else if (s >= 4 && s <= 8 && !mapping && !collecting) {
    tcs.getRGB(&r, &g, &b);
    colour = detectColour(r, g, b);
    if (colour == 7) {
      atBase = true;
      manualMode = false;
      completed = false;
      collecting = true;
  
      conversePaths.clear();
      for (int i=0; i<8; i++) {
        curPathArr[i] = 0;
      }
  
      leftT = 1;
      rightMotorSpeed = 210;
      rightReverseSpeed = 0;
      leftMotorSpeed = 0; 
      leftReverseSpeed = 170;
    
      counter = 0;
      pathNumber = 0;
    }
    else {
      Serial.write('5');
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  loopCounter++;
  
  if(Serial.available() > 0){ // Checks whether data is comming from the serial port from bluetooth sensor
    state = Serial.read(); // Reads the data from the serial port
    checkStateUI(state);
  }

  if (!completed && !atBase) {
    movement();
  }
  
  if (atBase) {
    if (initReturn) {
      tcs.getRGB(&r, &g, &b);
      colour = detectColour(r, g, b);
      if (colour != 7) {
        Serial.write('7');
        findingBase = true;
        mapping = false;
        collecting = false;
        atBase = false;
        completed = false;
      }
      initReturn = false;
    }
    
    toBase.clear();
    toBase.add(8);
    steer(0, 0, 0, 0);
    if (state != 0 && conversePaths.size()==0) {
      conversePaths.clear();
      for (int i=0; i<path.size(); i++) {
        if (path.get(i) % 10 == state) {
          conversePaths.add(path.get(i));
        }
        else if (state == 8) {
          conversePaths.add(path.get(i));
        }
      }
      if (conversePaths.size() != 0) {
        atBase = false;
        convToArray(pathNumber);
        Serial.write(13);
      }
    }

    if (conversePaths.size() != 0) {
      atBase = false;
    }
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
    long dis = calcDist();
    //Serial.println(dis);
    maxSpeed = 175;
    if (dis < 30) {
      maxSpeed = 175 - 400/dis;
    }

    int disT=0;
    
    while (dis < 15) {
      disT++;
      if (disT<1500) {
        steer(0, 0, 0, 0);
      }
      else if(disT==1500 && !mapping && !endPath) {
        endPath = true;
        counter = toBase.size()-1;
        rightMotorSpeed = 200;
        rightReverseSpeed = 0;
        leftMotorSpeed = 0; 
        leftReverseSpeed = 200;
        steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
        leftT = 1;
        delay(100);
        for (int j=0; j<toBase.size(); j++) {
          curPathArr[j]=toBase.get(j);
        }
        conversePaths.add(conversePaths.get(pathNumber));
      }
      dis = calcDist();
    }
    
  }

  */
 
  delay(2);

}
