#define MOT_A1_PIN 5
#define MOT_A2_PIN 6
#define MOT_B1_PIN 9
#define MOT_B2_PIN 10

int left = A3;
int centre = A2;
int right = A1;

int leftMotorSpeed;
int leftReverseSpeed;
int rightMotorSpeed;
int rightReverseSpeed;

int lowThreshold = 300;
int highThreshold = 700;
const int maxSpeed = 175;

int path[] = {17, 126, 229, 326, 38};
int dfs[10];
int curPathArr[8];
int pathNumber = 0;
int counter = 0;
int rightT = 0;
int forward = 0;
int leftT = 0;
boolean endPath = false;
boolean completed = false;

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

  convToArray(pathNumber);
}

void scan() {
  int sensorValueLeft = analogRead(left) * 0.96;
  int sensorValueCentre = analogRead(centre);
  int sensorValueRight = analogRead(right) * 1.03;
  movement(sensorValueLeft, sensorValueCentre, sensorValueRight);
}

void movement(int sensorValueLeft, int sensorValueCentre, int sensorValueRight) {
  //int selectSide = 0; //0 is nothing, 1 is left, 2 is right
  if (rightT == 0 && forward == 0 && leftT == 0) {
    if (sensorValueCentre > lowThreshold) { //if buggy is centred
      leftMotorSpeed = maxSpeed - 4000/sensorValueRight; 
      rightMotorSpeed = maxSpeed - 4000/sensorValueLeft;
      leftReverseSpeed = 0;
      rightReverseSpeed = 0;
  
      if ((sensorValueRight > lowThreshold && sensorValueLeft > lowThreshold) && sensorValueCentre > highThreshold) { //if buggy reaches a fork
        if (curPathArr[counter] == 1) {
          rightMotorSpeed = 150;
          rightReverseSpeed = 0;
          leftMotorSpeed = 0; 
          leftReverseSpeed = 0;
          leftT = 1;
        }
        else if (curPathArr[counter] == 2) {
          rightMotorSpeed = 130;
          rightReverseSpeed = 0;
          leftMotorSpeed = 135; 
          leftReverseSpeed = 0;
          forward = 1;
        }
        else if (curPathArr[counter] == 3) {
          rightMotorSpeed = 0;
          rightReverseSpeed = 0;
          leftMotorSpeed = 150; 
          leftReverseSpeed = 0;
          rightT = 1;
        }
        steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
        if (!endPath) {
          counter++;
        }
        else {
          counter--;
        }
      }
    }
    else{
      if (sensorValueRight > lowThreshold) { //if buggy needs additional correction
        rightMotorSpeed = 0;
        rightReverseSpeed = maxSpeed - 10000/(sensorValueRight-200);
        leftMotorSpeed = maxSpeed - 4000/sensorValueRight; 
        leftReverseSpeed = 0;
      }
      else if (sensorValueLeft > lowThreshold) {
        leftMotorSpeed = 0;
        leftReverseSpeed = maxSpeed - 10000/(sensorValueLeft-200);
        rightMotorSpeed = maxSpeed - 4000/sensorValueLeft;
        rightReverseSpeed = 0;
      }
      else { //if buggy reaches end of path
        if (curPathArr[counter] == 0) {
          endPath = true;
          rightMotorSpeed = 0;
          rightReverseSpeed = 0;
          leftMotorSpeed = 0; 
          leftReverseSpeed = 0;
          steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
          delay(1500);
          rightMotorSpeed = 90;
          rightReverseSpeed = 0;
          leftMotorSpeed = 0; 
          leftReverseSpeed = 90;
          steer(leftReverseSpeed, leftMotorSpeed, rightReverseSpeed, rightMotorSpeed);
          leftT = 1;
          counter--;
          changeDirection();
          Serial.println(curPathArr[counter]);
        }
        else if (curPathArr[counter] > 3) {
          endPath = false;
          rightMotorSpeed = 90;
          rightReverseSpeed = 0;
          leftMotorSpeed = 0; 
          leftReverseSpeed = 90;
          leftT = 1;
          if (pathNumber < 4) {
            pathNumber++;
          }
          else {
            completed = true;
            rightMotorSpeed = 200;
            rightReverseSpeed = 0;
            leftMotorSpeed = 0; 
            leftReverseSpeed = 200;
          }
          convToArray(pathNumber);
          counter = 0;
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
  
}

void convToArray(int pathNo) {
  int curPath = path[pathNo];
  for (int j=0; j<8; j++) {
    curPathArr[j] = 0;
  }
  int i=0;
  int y;
  while ( curPath > 0 ) {
      y=curPath/10;
      curPathArr[i] = curPath-(10*y);
      curPath=y;
      i++;
  }
  counter++;

  Serial.println(curPathArr[counter]);
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

void loop() {
  // put your main code here, to run repeatedly:
  if (!completed) {
    scan();
  }
  delay(2);

}
