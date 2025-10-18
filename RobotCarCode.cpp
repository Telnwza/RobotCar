#include "Arduino.h"

#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_E5  659
#define REST      0

#define Ena     3
#define inA     4
#define inB     5
#define inC     6
#define inD     7
#define Enb     9
#define Buzzer  10
#define PIN_TRIG 13
#define PIN_ECHO 12

const int Speed = 120;
const int White = 250;
const int Ttune = 20;
const int RMAX = 6;
const int CMAX = 6;
int ischeck = 0, isbacktostart = 0, boxR = 2, boxC = 2;
const int dr[4] = {-1, 0, 1, 0};
const int dc[4] = { 0, 1, 0,-1};

enum Cell   : int8_t  { FREE=0, OBST=1, BOX=2};
enum Heading: uint8_t { NORTH=0, EAST=1, SOUTH=2, WEST=3 };

int8_t grid[RMAX][CMAX];
int visited[RMAX][CMAX][4];
const int MAX_VISIT = 2;

struct Pos {
  int row, col;
  Heading heading;
} current;

void beep(int time){
  for(int i=0;i<time;i++){ tone(Buzzer,3000,100); delay(200); noTone(Buzzer); }
}

void playsong(){
  int tempo = 150;
  int buzzer = 10;
  int melody[] = {
    NOTE_A4,5,NOTE_A4,5,NOTE_A4,5, NOTE_E5,3 ,REST,4,NOTE_E5,5,NOTE_E5,4, NOTE_C5,4, NOTE_B4,4,
    NOTE_A4,5,NOTE_A4,5,NOTE_A4,5, NOTE_E5,3 ,REST,4,NOTE_E5,5,NOTE_E5,4, NOTE_C5,4, NOTE_B4,4,
    NOTE_A4,5,NOTE_A4,5,NOTE_A4,5, NOTE_E5,3 ,REST,4,NOTE_E5,5,NOTE_E5,4, NOTE_C5,4, NOTE_B4,4,
    NOTE_A4,5,NOTE_A4,5,NOTE_A4,5, NOTE_E5,3 ,REST,4,NOTE_E5,5,NOTE_E5,4, NOTE_C5,4, NOTE_B4,4,
  };
  int notes = sizeof(melody) / sizeof(melody[0]) / 2;
  int wholenote = (60000 * 4) / tempo;
  int divider = 0, noteDuration = 0;
  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
    divider = melody[thisNote + 1];
    if (divider > 0) {
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5;
    }
    tone(buzzer, melody[thisNote], noteDuration * 0.9);
    delay(noteDuration);
    noTone(buzzer);
  }
}

void readSensors(int s[5]){
  int raw[5];
  raw[0]=analogRead(A0);
  raw[1]=analogRead(A1);
  raw[2]=analogRead(A3);
  raw[3]=analogRead(A4);
  raw[4]=analogRead(A5);
  for(int i=0;i<5;i++){
    if(raw[i] >= White) s[i]=1;
    else s[i]=0;
  }
}

void printRawSensors() {
  int raw[5];
  raw[0] = analogRead(A0);
  raw[1] = analogRead(A1);
  raw[2] = analogRead(A3);
  raw[3] = analogRead(A4);
  raw[4] = analogRead(A5);

  Serial.print("RAW: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(raw[i]);
    if (i < 4) Serial.print(", ");
  }
  Serial.println();
}

bool atIntersection(const int s[5]) {
  bool center = (s[2] == 0);
  bool left   = (s[0] == 0 || s[1] == 0);
  bool right  = (s[3] == 0 || s[4] == 0);

  if (center && !left && !right) { go(Speed, Speed); return false; }
  if (center && left  && !right){ go(Speed - Ttune, Speed + Ttune); return false; }
  if (center && right && !left) { go(Speed + Ttune, Speed - Ttune); return false; }
  if (left    && !right)          { go(Speed - Ttune*1.5, Speed + Ttune*1.5); return false; }
  if (right   && !left)           { go(Speed + Ttune*1.5, Speed - Ttune*1.5); return false; }
  if (!center && !left && !right){ go(Speed, Speed); return false; }
  return (center && left && right);
}

void goB() {
  int s[5];
  do readSensors(s);
  while (!atIntersection(s));
  gosec(220);
  stopB();
  tone(Buzzer, 500, 100); delay(150); noTone(Buzzer);
}

void go(int left,int right){
  analogWrite(Ena,left); analogWrite(Enb,right);
  digitalWrite(inA,1); digitalWrite(inB,0);
  digitalWrite(inC,1); digitalWrite(inD,0);
}

void back(int left,int right){
  analogWrite(Ena,left); analogWrite(Enb,right);
  digitalWrite(inA,0); digitalWrite(inB,1);
  digitalWrite(inC,0); digitalWrite(inD,1);
}

void stop(){
  analogWrite(Ena,0); analogWrite(Enb,0);
  digitalWrite(inA,0); digitalWrite(inB,0);
  digitalWrite(inC,0); digitalWrite(inD,0);
}

void stopB(){
  analogWrite(Ena,Speed); analogWrite(Enb,Speed);
  digitalWrite(inA,0); digitalWrite(inB,1);
  digitalWrite(inC,0); digitalWrite(inD,1);
  delay(50);
  stop();
}

void stopF(){
  go(Speed,Speed);
  delay(50);
  stop();
}

void gosec(unsigned long milsec){
  int s[5];
  unsigned long t0 = millis();
  while (millis() - t0 < milsec) {
    readSensors(s);
    if (atIntersection(s)) go(Speed, Speed);
  }
  stopB();
}

void backsec(unsigned long milsec){
  unsigned long t0 = millis();
  while (millis() - t0 < milsec) {
    back(Speed,Speed);
  }
  stopF();
}

void tr(){
  int s[5];
  digitalWrite(inA,1); digitalWrite(inB,0);
  digitalWrite(inC,0); digitalWrite(inD,1);
  analogWrite(Ena,100); analogWrite(Enb,100);
  unsigned long t0 = millis();
  do{ readSensors(s); }while((!s[2] && s[1] && s[3]) || (millis()-t0 <= 300UL));
  do{ readSensors(s); }while(!(!s[2] && s[1] && s[3]));
  digitalWrite(inA,0); digitalWrite(inB,1);
  digitalWrite(inC,1); digitalWrite(inD,0);
  delay(100);
  stop(); delay(100);
}

void tl(){
  int s[5];
  digitalWrite(inA,0); digitalWrite(inB,1);
  digitalWrite(inC,1); digitalWrite(inD,0);
  analogWrite(Ena,100); analogWrite(Enb,100);
  unsigned long t0 = millis();
  do{ readSensors(s); }while((!s[2] && s[1] && s[3]) || (millis()-t0 <= 300UL));
  do{ readSensors(s); }while(!(!s[2] && s[1] && s[3]));
  digitalWrite(inA,1); digitalWrite(inB,0);
  digitalWrite(inC,0); digitalWrite(inD,1);
  delay(100);
  stop(); delay(100);
}

void rotateTo(Heading nextDir){
  int diff = ((int)nextDir - (int)current.heading + 4) % 4;
  if(diff==1) { tr(); }
  else if(diff==3) { tl(); }
  else if(diff==2){ tl(); delay(100); tl(); }
  current.heading = nextDir;
}

long readUltrasonicCM(){
  digitalWrite(PIN_TRIG,LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG,HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG,LOW);
  long us = pulseIn(PIN_ECHO,HIGH,25000);
  if(us==0) return -1;
  return us/58;
}

bool inBoundsRC(int r,int c){ return (r>=0 && r<RMAX && c>=0 && c<CMAX); }
bool inBoundsIN(int r,int c){ return (r>=1 && r<=4 && c>=1 && c<=4); }
bool isAtRC(int r,int c){ return (current.row==r && current.col==c); }
void setCellRC(int r,int c, Cell t){ if(inBoundsRC(r,c)) grid[r][c] = t; }

Heading turnRight(Heading h){ return (Heading)((h+1)%4); }
Heading turnLeft (Heading h){ return (Heading)((h+3)%4); }
Heading turnBack (Heading h){ return (Heading)((h+2)%4); }

void pushU(int steps){
  grid[boxR][boxC] = FREE;
  rotateTo(NORTH);
  for(int i=0;i<steps;i++){
    goOneCell();
    boxR--;
  }
  gosec(750);
  backsec(450);
  grid[boxR][boxC] = BOX;
}

void pushD(int steps){
  grid[boxR][boxC] = FREE;
  rotateTo(SOUTH);
  for(int i=0;i<steps;i++){
    goOneCell();
    boxR++;
  }
  gosec(750);
  backsec(450);
  grid[boxR][boxC] = BOX;
}

void pushR(int steps){
  grid[boxR][boxC] = FREE;
  rotateTo(EAST);
  for(int i=0;i<steps;i++){
    goOneCell();
    boxC++;
  }
  gosec(750);
  backsec(450);
  grid[boxR][boxC] = BOX;
}

void pushL(int steps){
  grid[boxR][boxC] = FREE;
  rotateTo(WEST);
  for(int i=0;i<steps;i++){
    goOneCell();
    boxC--;
  }
  gosec(750);
  backsec(450);
  grid[boxR][boxC] = BOX;
}

void initGrid(){
  for(int r=0;r<RMAX;r++){
    for(int c=0;c<CMAX;c++){
      grid[r][c] = FREE;
      for(int d=0;d<4;d++){
        visited[r][c][d] = 0;
      }
    }
  }
  setCellRC(2,1,OBST);
  setCellRC(3,4,OBST);
  setCellRC(2,2,BOX);

  current.row = 5;
  current.col = 0;
  current.heading = NORTH;
}

void goOneCell(){
  goB();
  current.row += dr[current.heading];
  current.col += dc[current.heading];
}

void handleGo(int in,int d){
  Heading nextDir;
  nextDir = decideNext(in, d);
  visited[current.row][current.col][nextDir]++;
  rotateTo(nextDir);
  delay(150);
  long cm = readUltrasonicCM();
  if (cm>0 && cm < 18) {
    int fr = current.row + dr[current.heading];
    int fc = current.col + dc[current.heading];
    if (inBoundsRC(fr,fc)) grid[fr][fc] = OBST;
    rotateTo( turnBack(current.heading) );
    delay(200);
  } else {
    goOneCell();
  }
}

Heading decideNext(int in,int d){
  Heading rightDir = turnRight(current.heading);
  Heading fwdDir   = current.heading;
  Heading leftDir  = turnLeft(current.heading);
  if(d == 0){
  if (canGo(rightDir, in)) return rightDir;
  if (canGo(fwdDir  , in)) return fwdDir;
  if (canGo(leftDir , in)) return leftDir;}
  else{
    if (canGo(leftDir , in)) return leftDir;
    if (canGo(fwdDir  , in)) return fwdDir;
    if (canGo(rightDir, in)) return rightDir;
  }
  return turnBack(current.heading);
}

bool canGo(Heading target, int in){
  int nr = current.row + dr[target];
  int nc = current.col + dc[target];
  if(in == 0){
    if(!inBoundsRC(nr,nc)) return false;
    if(grid[nr][nc]==OBST || grid[nr][nc]==BOX) return false;
    if(visited[current.row][current.col][target] >= MAX_VISIT) return false;
    return true;
  }else{
    if (inBoundsIN(nr, nc) && grid[nr][nc] != OBST && grid[nr][nc] != BOX) return true;
    return false;
  }
}

void setup(){
  for(int i=3;i<=11;i++){pinMode(i,OUTPUT); digitalWrite(i,0); }
  pinMode(2,INPUT_PULLUP);
  pinMode(Buzzer,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
  Serial.begin(9600);
  initGrid();
  beep(2);

  while(1){ //start button
    if(!digitalRead(2)){
      delay(50);
        if(!digitalRead(2)){
          Serial.println("Ready"); beep(1);
          break;
      }
    }
  }

  while(!isAtRC(0,5)) handleGo(0,1); //checkpoint
  beep(2); delay(3000);

  while(isAtRC(0,5)) handleGo(0,0);

  if((grid[1][0]==OBST || grid[2][0]==OBST || grid[3][0]==OBST) 
  && (grid[0][1]==OBST || grid[0][2]==OBST || grid[0][3]==OBST))
  {
    while(!isAtRC(1,2)) handleGo(1,0);
    pushD(1);
    rotateTo(EAST);
    while(!isAtRC(3,1)) handleGo(1,0);
    pushR(1);
    rotateTo(SOUTH);
    while(!isAtRC(4,3)) handleGo(1,1);
    pushU(2);
    rotateTo(WEST);
    while(!isAtRC(1,2)) handleGo(1,0);
    pushR(1);
  }else if(grid[0][1]==OBST || grid[0][2]==OBST || grid[0][3]==OBST){
    setCellRC(2,4,OBST);
    setCellRC(4,3,OBST);

    while(!isAtRC(3,2)) handleGo(1,1);
    pushU(1);
    rotateTo(SOUTH);
    while(!isAtRC(1,1)) handleGo(0,0);
    pushR(2);
  }else {
    setCellRC(2,4,OBST);
    setCellRC(4,3,OBST);

    while(!isAtRC(3,2)) handleGo(1,1);
    pushU(1);
    rotateTo(EAST);
    while(!isAtRC(1,1)) handleGo(0,1);
    pushR(2);
  }

  setCellRC(4,3,FREE);
  setCellRC(2,4,OBST);
  setCellRC(4,4,OBST);
  setCellRC(5,2,OBST);
  setCellRC(3,0,OBST);

  //back home
  while(!isAtRC(4,1)) handleGo(1,1); 
  while(!isAtRC(5,0)) handleGo(0,1); //home
  
  stop();
  beep(3);

  stop();
  Serial.println("Mission Complete");
  playsong();
}

void loop(){
  
}

void path1(){
  while(!isAtRC(0,5)) handleGo(0,0); //checkpoint
  stop();
  beep(2); delay(3000);

  while(isAtRC(0,5)) handleGo(0,0);

  //push box
  while(!isAtRC(1,2)) handleGo(1,0);
  pushD(1);
  rotateTo(EAST);
  while(!isAtRC(3,1)) handleGo(1,0);
  pushR(1);
  rotateTo(SOUTH);
  while(!isAtRC(4,3)) handleGo(1,1);
  pushU(2);
  rotateTo(WEST);
  while(!isAtRC(1,2)) handleGo(1,0);
  pushR(1);

  setCellRC(2,4,OBST);
  setCellRC(4,4,OBST);
  setCellRC(5,2,OBST);
  setCellRC(3,0,OBST);

  //back home
  while(!isAtRC(4,1)) handleGo(1,1); 
  while(!isAtRC(5,0)) handleGo(0,1); //home
}

// while(isbacktostart == 0){
//     if (isAtRC(0,5) && ischeck == 0) {
//       stop();
//       beep(2); delay(3000);
//       Serial.println("At Checkpoint");
//       ischeck = 1;
//     }
//     if (isAtRC(5,0) && ischeck == 1) {
//       stop();
//       beep(3); delay(3000);
//       Serial.println("Back to Start");
//       isbacktostart = 1;
//       break;
//     }
//     handleGo(0,0);
//   }

 // if((grid[1][0]==OBST || grid[2][0]==OBST || grid[3][0]==OBST) 
  // && (grid[0][1]==OBST || grid[0][2]==OBST || grid[0][3]==OBST))
  // {
  //   rotateTo(EAST);
  //   while(!isAtRC(1,2)) handleGo(1,1);
  //   pushD(1);
  //   rotateTo(EAST);
  //   while(!isAtRC(3,1)) handleGo(1,0);
  //   pushR(1);
  //   rotateTo(SOUTH);
  //   while(!isAtRC(4,3)) handleGo(1,1);
  //   pushU(2);
  //   rotateTo(WEST);
  //   while(!isAtRC(1,2)) handleGo(1,0);
  //   pushR(1);
  // }else if(grid[0][1]==OBST || grid[0][2]==OBST || grid[0][3]==OBST){
  //   pushU(1);
  //   rotateTo(SOUTH);
  //   while(!isAtRC(1,1)) handleGo(0,0);
  //   pushR(2);
  // }else {
  //   pushU(1);
  //   rotateTo(EAST);
  //   while(!isAtRC(1,1)) handleGo(0,1);
  //   pushR(2);
  // }

  // if(grid[5][1]==OBST){
  //   rotateTo(NORTH);
  //   goOneCell();
  //   rotateTo(EA©©ST);
  //   goOneCell();
  // }else {
  //   rotateTo(EAST);
  //   goOneCell();
  // }