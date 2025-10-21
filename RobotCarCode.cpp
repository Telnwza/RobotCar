#include "Arduino.h"

//Note freq
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_E5  659
#define REST      0

//Set pinout
#define Ena     3
#define inA     4
#define inB     5
#define inC     6
#define inD     7
#define Enb     9
#define Buzzer  10
#define PIN_TRIG 13
#define PIN_ECHO 12

//ประกาศตัวแปรที่ใช้หลักๆใน code
const int Speed = 120; //ความเร็ว
const int White = 250; //treshold แสง
const int Ttune = 20;  //ความแรงในการหมุน
const int RMAX = 6;  // ขนาดตาราง
const int CMAX = 6;
int boxR = 2, boxC = 2; //ตัวแปรเก็บตำแหน่งกล่อง
const int dr[4] = {-1, 0, 1, 0}; //เก็บค่าเมื่อขยับไปตามทิศต่างๆ เอาไว้ใช้กับ Heading
const int dc[4] = { 0, 1, 0,-1};

enum Cell : int8_t  { FREE=0, OBST=1, BOX=2};  //datatype เอาไว้บอกว่าcellนั้นคืออะไร
enum Heading : uint8_t { NORTH=0, EAST=1, SOUTH=2, WEST=3 }; //ตัวเลขแทนทิศที่หุ่นหันหน้าไป

int8_t grid[RMAX][CMAX]; //เก็บว่าแต่ละcellคืออะไร
int visited[RMAX][CMAX][4];
const int MAX_VISIT = 2; //ตั้งให้เดินซ้ำ cell เดิมได้ไม่เกิน 2 รอบ กันเดินวนซ้ำ

struct Pos { //struct เก็บตำแหน่งปัจจุบันของหุ่น row col และ ทิศที่หันหน้าไป
  int row, col;
  Heading heading;
} current;

void beep(int time){ //สั่งให้ buzzer ส่งเสียงกี่ครั้ง
  for(int i=0;i<time;i++){ tone(Buzzer,3000,100); delay(200); noTone(Buzzer); }
}

void playsong(){ //เล่นเพลง
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

//รับ array มาแล้วอ่านค่าของ sensor แล้วแปลงค่าให้เป็น ขาวหรือดำ ใส่ array นั้นกลับไป
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

//สำหรับ debug print ค่า sensor
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

//ฟังก์ชั่นเอาไว้ดูว่าถึงแยกหรือยัง คืนค่า 0,1
//พร้อมกับ track line ไปในตัว
bool atIntersection(const int s[5]) {
  //ใช้ค่าแค่ 3 อัน โดยให้ เซนเซอร์ด้านเดียวกันมีค่าเท่ากัน
  bool center = (s[2] == 0);
  bool left   = (s[0] == 0 || s[1] == 0);
  bool right  = (s[3] == 0 || s[4] == 0);

  if (center && !left && !right) { go(Speed, Speed); return false; }
  if (center && left && !right){ go(Speed - Ttune, Speed + Ttune); return false; }
  if (center && right && !left) { go(Speed + Ttune, Speed - Ttune); return false; }
  if (left && !right) { go(Speed - Ttune*1.5, Speed + Ttune*1.5); return false; }
  if (right && !left) { go(Speed + Ttune*1.5, Speed - Ttune*1.5); return false; }
  if (!center && !left && !right){ go(Speed, Speed); return false; }
  return (center && left && right);
}

//เดินtrackเส้นไปจนเจอแยก แล้วเดินต่อให้ล้ออยู่กลางเส้น
void goB() {
  int s[5];
  do readSensors(s);
  while (!atIntersection(s));
  gosec(220);
  stopB();
  tone(Buzzer, 500, 100); delay(150); noTone(Buzzer);
}

//เปิดมอเตอร์เดินหน้า รับspeed ซ้าย-ขวา
void go(int left,int right){
  analogWrite(Ena,left); analogWrite(Enb,right);
  digitalWrite(inA,1); digitalWrite(inB,0);
  digitalWrite(inC,1); digitalWrite(inD,0);
}

//เปิดมอเตอร์เดินถอยหลัง รับspeed ซ้าย-ขวา
void back(int left,int right){
  analogWrite(Ena,left); analogWrite(Enb,right);
  digitalWrite(inA,0); digitalWrite(inB,1);
  digitalWrite(inC,0); digitalWrite(inD,1);
}

//ปิดมอเตอร์
void stop(){
  analogWrite(Ena,0); analogWrite(Enb,0);
  digitalWrite(inA,0); digitalWrite(inB,0);
  digitalWrite(inC,0); digitalWrite(inD,0);
}

//หยุดจากเดินหน้า โดยถอยหลังนิดนึงเพื่อให้หุ่นยนต์ไม่ไหลต่อ
void stopB(){
  analogWrite(Ena,Speed); analogWrite(Enb,Speed);
  digitalWrite(inA,0); digitalWrite(inB,1);
  digitalWrite(inC,0); digitalWrite(inD,1);
  delay(50);
  stop();
}

//หยุดจากถอยหลัง โดยเดินหน้านิดนึงเพื่อให้หุ่นยนต์ไม่ไหลต่อ
void stopF(){
  go(Speed,Speed);
  delay(50);
  stop();
}

//เดิน track เส้นเป็นเวลากี่ ms
void gosec(unsigned long milsec){
  int s[5];
  unsigned long t0 = millis();
  while (millis() - t0 < milsec) {
    readSensors(s);
    if (atIntersection(s)) go(Speed, Speed);
  }
  stopB();
}

//เดินถอยหลังเฉยๆเป็นเวลากี่ ms
void backsec(unsigned long milsec){
  unsigned long t0 = millis();
  while (millis() - t0 < milsec) {
    back(Speed,Speed);
  }
  stopF();
}

//เลี้ยวขวา โดยใช้เส้นเป็นตัวตัดสินว่าเลี้ยวถึงแล้วหรือยัง
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

//เลี้ยวซ้าย โดยใช้เส้นเป็นตัวตัดสินว่าเลี้ยวถึงแล้วหรือยัง
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

//หันหุ่นไปตามทิศที่ต้องการ โดยการดูทิศปัจจุบันแล้วหารหาเศษกับทิศที่จะไป ดูผลต่างแล้วหันหุ่นไปตามที่ต้องการ
void rotateTo(Heading nextDir){
  int diff = ((int)nextDir - (int)current.heading + 4) % 4;
  if(diff==1) { tr(); }
  else if(diff==3) { tl(); }
  else if(diff==2){ tl(); delay(100); tl(); }
  current.heading = nextDir;
}

//อ่านค่าของ ultrasonic เป็น CM
long readUltrasonicCM(){
  digitalWrite(PIN_TRIG,LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG,HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG,LOW);
  long us = pulseIn(PIN_ECHO,HIGH,25000);
  if(us==0) return -1;
  return us/58;
}

//ฟังก์ชั่นเอาไว้เช็คต่างๆ 
bool inBoundsRC(int r,int c){ return (r>=0 && r<RMAX && c>=0 && c<CMAX); } //ดูว่าตำแหน่งที่ส่งไปอยู่ใน map ไหม
bool inBoundsIN(int r,int c){ return (r>=1 && r<=4 && c>=1 && c<=4); } //ดูว่าตำแหน่งที่ส่งไปอยู่ในกรอบในไหม 1,1-4,4
bool isAtRC(int r,int c){ return (current.row==r && current.col==c); } //ดูว่าตำแหน่งปัจจุบันอยู่ที่ตำแหน่งที่ส่งไปไหม
void setCellRC(int r,int c, Cell t){ if(inBoundsRC(r,c)) grid[r][c] = t; } //set cell ว่าตำแหน่งนั้นจะให้เป็นอะไร

//ฟังก์ชั่นไว้หมุนทิศจากทิศที่ใส่มาเฉยๆ ไม่ได้หมุนหุ่นยนต์
Heading turnRight(Heading h){ return (Heading)((h+1)%4); }
Heading turnLeft (Heading h){ return (Heading)((h+3)%4); }
Heading turnBack (Heading h){ return (Heading)((h+2)%4); }

//ฟังก์ชั่นในการดันกล่อง โดยดันกล่องให้เจอเส้น แล้วดันต่ออีกหน่อย พร้อมทั้ง update ตำแหน่งกล่อง ในmap
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

//clear ค่าใน grid และset ตำแหน่งสิ่งกีดขวางกับกล่อง
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

//สั่งใช้คำสั่ง goB แต่มีการ update ตำแหน่งด้วย
void goOneCell(){
  goB();
  current.row += dr[current.heading];
  current.col += dc[current.heading];
}

/*
ฟังก์ชั่นหลักในการใช้เดิน โดยจะตัดสินใจเดินและเดินไป 1 ช่องในทิศทางที่เดินได้ พร้อมตรวจจับสิ่งกีดขวางด้านหน้า 1 cell 
โดยรับ input 2 ค่า คือ in ดูว่าจะให้เดินเฉพาะขอบในไหม และ d คือ ดูว่าจะให้เดินชิดขวาหรือซ้าย 
handleGo(ชิดขอบในไหม,ชิดซ้ายไหม) {0,1}
*/
void handleGo(int in,int d){
  Heading nextDir;
  nextDir = decideNext(in, d); //คิดว่าจะเดินไปทางไหน คืนค่ามาเป็นทิศ
  visited[current.row][current.col][nextDir]++; //set ว่าตำแหน่งปัจจุบันเคยเดินมาแล้ว+1
  rotateTo(nextDir); //หันหน้าไปตามทิศที่จะเดิน
  delay(150);
  long cm = readUltrasonicCM(); //เช็คว่าด้านหน้ามีสิ่งกีดขวางไหม
  if (cm>0 && cm < 18) { //ถ้ามีก็ set ว่าข้างหน้าเป็นสิ่งกีดขวาง และหันหลังกลับ
    int fr = current.row + dr[current.heading]; 
    int fc = current.col + dc[current.heading];
    if (inBoundsRC(fr,fc)) grid[fr][fc] = OBST;
    rotateTo( turnBack(current.heading) );
    delay(200);
  } else {
    goOneCell(); // ถ้าไม่มีอะไรก็เดินไปข้างหน้า1ช่อง
  }
}

//ดูว่าจะเดินไปทิศไหนได้ โดยรับค่าต่อมาจาก handlego
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

//เช็คว่าในทิศที่ Input มาเดินไปได้ไหม
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

//ฟังก์ชั่นหลักของหุ่นยนตฺ์ทำงานใน setup
void setup(){
  //set IO Pin
  for(int i=3;i<=11;i++){pinMode(i,OUTPUT); digitalWrite(i,0); }
  pinMode(2,INPUT_PULLUP);
  pinMode(Buzzer,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
  Serial.begin(9600);
  initGrid(); //reset grid
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

  while(!isAtRC(0,5)) handleGo(0,1); //เดินไป checkpoint ชิดซ้าย
  beep(2); delay(3000); //ส่งเสียง หยุด 3 วิ

  while(isAtRC(0,5)) handleGo(0,0); //เดินออกจากจุด checkpoint ไปในทางที่เดินได้

  if((grid[1][0]==OBST || grid[2][0]==OBST || grid[3][0]==OBST) //มีสิ่งกีดขวางทั้ง ด้านซ้ายและด้านบน
  && (grid[0][1]==OBST || grid[0][2]==OBST || grid[0][3]==OBST))
  {
    //เส้นทางที่ 1
    while(!isAtRC(1,2)) handleGo(1,0); //เดินชิดขวาขอบในไปที่ตำแหน่ง 1,2
    pushD(1); //ดันกล่องลง 1 ช่อง
    rotateTo(EAST); //หมุนหุ่นไปทางทิศตะวันออก (ขวา)
    while(!isAtRC(3,1)) handleGo(1,0); //เดินชิดขวาขอบในไปตำแหน่ง 3,1
    pushR(1); //ดันกล่องไปด้านขวา 1 ช่อง
    rotateTo(SOUTH); //หันหน้าไปทางทิศใต้ (ลง)
    while(!isAtRC(4,3)) handleGo(1,1); //เดินชิดซ้ายขอบในไปที่ตำแหน่ง 4,3
    pushU(2); //ดันกล่องขึ้น 2 ช่อง
    rotateTo(WEST); //หันหน้าไปทิศตะวันตก (ซ้าย)
    while(!isAtRC(1,2)) handleGo(1,0); //เดินชิดขวาขอบในไปที่ตำแหน่ง 1,2 
    pushR(1);
  }else if(grid[0][1]==OBST || grid[0][2]==OBST || grid[0][3]==OBST){ //มีสิ่งกีดขวางด้านบน
    //เส้นทางที่ 2
    //set ให้ไม่ต้องเดินไปในช่อง 2,4:4,3 
    setCellRC(2,4,OBST);
    setCellRC(4,3,OBST);

    while(!isAtRC(3,2)) handleGo(1,1); //เดินชิดซ้ายขอบในไปที่ตำแหน่ง 3,2
    pushU(1); //ดันกล่องขึ้น 1 ช่อง
    rotateTo(SOUTH); //หันหน้าไปทางทิศใต้ (ลง)
    while(!isAtRC(1,1)) handleGo(0,0); //เดินชิดขวาไปที่ตำแหน่ง 1,1
    pushR(2); //ดันกล่องไปทางขวา 2 ช่อง
  }else {
    //เส้นทางที่ 3 (default)
    //set ให้ไม่ต้องเดินไปในช่อง 2,4:4,3 
    setCellRC(2,4,OBST);
    setCellRC(4,3,OBST);

    while(!isAtRC(3,2)) handleGo(1,1); //เดินชิดซ้ายขอบในไปที่ตำแหน่ง 3,2
    pushU(1); //ดันกล่องขึ้น 1 ช่อง
    rotateTo(EAST); //หันหน้าไปทิศตะวันออก (ขวา)
    while(!isAtRC(1,1)) handleGo(0,1); //เดินชิดซ้ายไปที่ตำแหน่ง 1,1
    pushR(2); //ดันกล่องไปทางขวา 2 ช่อง
  }

  //set ตำแหน่งให้ไม่ต้องเดินเข้าไปเช็คในช่องที่ไม่ต้องการ
  setCellRC(4,3,FREE);
  setCellRC(2,4,OBST);
  setCellRC(4,4,OBST);
  setCellRC(5,2,OBST);
  setCellRC(3,0,OBST);

  //เดินกลับจุดเริ่มต้น
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

// void path1(){
//   while(!isAtRC(0,5)) handleGo(0,0); //checkpoint
//   stop();
//   beep(2); delay(3000);

//   while(isAtRC(0,5)) handleGo(0,0);

//   //push box
//   while(!isAtRC(1,2)) handleGo(1,0);
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

//   setCellRC(2,4,OBST);
//   setCellRC(4,4,OBST);
//   setCellRC(5,2,OBST);
//   setCellRC(3,0,OBST);

//   //back home
//   while(!isAtRC(4,1)) handleGo(1,1);
//   while(!isAtRC(5,0)) handleGo(0,1); //home
// }

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
