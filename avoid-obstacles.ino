/*  Avoid Obstacles
 *  Move to avoid obstacles. 
 */


const int p_ftl = 30;
const int p_ftr = 31;
const int p_fbl = 32;
const int p_fbr = 33;
const int p_btl = 34;
const int p_btr = 35;
const int p_bbl = 36;
const int p_bbr = 37;
const int p_l = 38;
const int p_r = 39;
const int lMotor = 9;
const int rMotor = 10;


const int safeDistance = 10; //Safe Forward distance
const int correctingGap = 5; //??
const int tTurn=256;//?? //Time to make 90 Degree Turn
const int tPass=100; //time it takes for robot to clear corner.

void setup() {
  Serial.begin(9600);
  pinMode(lMotor, OUTPUT);
  pinMode(rMotor, OUTPUT);
  forward(); //moves robot forward
  
}

void loop() {
 avoid();
 correct();
  
}


//MOVEMENT HELPERS//
///////////////////
 //MOVE FORWARD
 
void forward(){
  digitalWrite(lMotor,HIGH);  //Turn both motors on.
  digitalWrite(rMotor,HIGH);
}

void stopp(){
  digitalWrite(lMotor,LOW);  //Turn both motors on.
  digitalWrite(rMotor,LOW);
  
}


///////////////////////
//AVOIDANCE HELPERS
///////////////////////

//CORRECT FORWARD//

void correct(){
  
int margin=1; //mm, will set the tolerance on difference between the first and second ping. 
      //There might be noise so don't want to get stuck in endless loop of correcting due to noise.
int correctingTime=20; //ms , time spent with one engine off to adjust angle.
int checkTime=2; //ms , time between distance checks, used to see if were approaching side or not., 
 
  if (get_dist(p_r)<safeDist || get_dist(p_l)<safeDist){
    if( get_dist(p_l)>get_dist(p_r) ){ //means right side is closer CORRECTS TO THE LEFT
      int check1=get_dist(p_r);
      delay(1);
      int check2=get_dist(p_r); //check 2 shouldnt be too much closer than check 1, otherwise we are quickly approaching a wall to the side.
      while(check2<(check1)){
        analogWrite(lMotor,0);
        delay(correctingTime);
        analogWrite(lMotor,analogInput/1.1);
        check1 = get_dist(p_r);
        delay(checkTime);
        check2 = get_dist(p_r);
      }
    }
    if( get_dist(p_r)>get_dist(p_l) ){ //means left side is closer CORRECTS TO THE LEFT
      int check1=get_dist(p_l);
      delay(1);
      int check2=get_dist(p_l); //check 2 shouldnt be too much closer than check 1, otherwise we are quickly approaching a wall to the side.
      while(check2<(check1)){
        analogWrite(rMotor,0);
        delay(correctingTime);
        analogWrite(rMotor,analogInput);
        check1 = get_dist(p_l);
        delay(checkTime);
        check2 = get_dist(p_l);
      }
    } 
  }
   
}

//AVOID//
//Checks front for obstacle and avoids obstacle to the left or to the right.

void avoid() {
  if (getPingFront()<=safeDistance){ //checking for obstacle
    stopp();
    if (get_l() > 250  & get_r() >250){ //making case for when either sensor has no input - i.e. nothing is within range on either side.
    avoidRight(); //default righty.
    }
    if (get_l() > get_r()) {
    avoidLeft(); //if left distance is greater than right, go left.
    }
    if (get_r() > get_l()) {
    avoidRight(); //if right distance is greater than left, go right.
    }   
  forward();
  }
}


//AVOID TO THE RIGHT
void avoidRight(){
int sideDistance=0;

boolean frontSafe = false;  
digitalWrite(rMotor,LOW); // avoid right.
digitalWrite(lMotor,HIGH);
  while(frontSafe==false){ //while front unsafe rotate
  int frontDistance=getPingFront();
    if (frontDistance>=safeDistance){ 
    frontSafe=true; //when becomes safe stop while loop.
    }
  }
    sideDistance=0;
    boolean sideSafe = false;
    digitalWrite(rMotor,HIGH);
    digitalWrite(lMotor,HIGH);
    while (sideSafe==false){
      sideDistance=get_l();
      avoid();
      if (sideDistance>=safeDistance){
        sideSafe=true;
        delay(tPass);
      }
    }
    digitalWrite(lMotor,LOW);
    delay(tTurn);
    digitalWrite(lMotor,HIGH);
}

//AVOID LEFT//

void avoidLeft(){
int sideDistance=0;

boolean frontSafe = false;  
digitalWrite(rMotor,HIGH); // avoid left.
digitalWrite(lMotor,LOW);
  while(frontSafe==false){ //while front unsafe rotate
  int frontDistance=getPingFront();
    if (frontDistance>=safeDistance){ 
    frontSafe=true; //when becomes safe stop while loop.
    
    }
  }
    sideDistance=0;
    boolean sideSafe = false;
    digitalWrite(rMotor,HIGH);
    digitalWrite(lMotor,HIGH);
    while (sideSafe==false){
      sideDistance=get_r();
      avoid();
      if (sideDistance>=safeDistance){
        sideSafe=true;
        delay(tPass);
      }
    }
    digitalWrite(lMotor,LOW);
    delay(tTurn);
    digitalWrite(lMotor,HIGH);
}

//PING HELPERS//////
//////////////////////
///FRONT PING///

int getPingFront(){
  int sense1 = get_ftl();
  int sense2 = get_ftr();
  int sense3 = get_fbr();
  int sense4 = get_fbl();
  return (sense1+sense2+sense3+sense4)/4;
}

//BACK PING////
int getPingBack(){
  int sense1 = get_btl();
  int sense2 = get_btr();
  int sense3 = get_bbr();
  int sense4 = get_bbl();
  return (sense1+sense2+sense3+sense4)/4;
}


//Front Top Left
int get_ftl() {
  unsigned long microsec;
  unsigned long zero = micros();
  unsigned long tim2 = 0;
  pinMode(p_ftl, OUTPUT);        //make port an output
  digitalWrite(p_ftl, HIGH);     //transmit signal
  
  while (microsec - zero < 5) {       //wait 5 microseconds
    microsec = micros();
  }

  pinMode(p_ftl, INPUT);         //change port to input

  while (digitalRead(p_ftl) == LOW) { //wait until signal is read
  }
  unsigned long tim = micros();

  while (digitalRead(p_ftl) == HIGH) { //measure echo pulse width
    tim2 = micros();
  }
  unsigned long timm = tim2 - tim;
  int dist = (timm / 58); //convert microseconds to centimeters
  //if (dist > 250) {dist = -1;} //Too far away or bad signal
  return dist;
}

//Front Top Right
int get_ftr() {
  unsigned long microsec;
  unsigned long zero = micros();
  unsigned long tim2 = 0;
  pinMode(p_ftr, OUTPUT);        //make port an output
  digitalWrite(p_ftr, HIGH);     //transmit signal
  
  while (microsec - zero < 5) {       //wait 5 microseconds
    microsec = micros();
  }

  pinMode(p_ftr, INPUT);         //change port to input

  while (digitalRead(p_ftr) == LOW) { //wait until signal is read
  }
  unsigned long tim = micros();

  while (digitalRead(p_ftr) == HIGH) { //measure echo pulse width
    tim2 = micros();
  }
  unsigned long timm = tim2 - tim;
  int dist = (timm / 58); //convert microseconds to centimeters
  //if (dist > 250) {dist = -1;} //Too far away or bad signal
  return dist;
}

//Front Bottom Left
int get_fbl() {
  unsigned long microsec;
  unsigned long zero = micros();
  unsigned long tim2 = 0;
  pinMode(p_fbl, OUTPUT);        //make port an output
  digitalWrite(p_fbl, HIGH);     //transmit signal
  
  while (microsec - zero < 5) {       //wait 5 microseconds
    microsec = micros();
  }

  pinMode(p_fbl, INPUT);         //change port to input

  while (digitalRead(p_fbl) == LOW) { //wait until signal is read
  }
  unsigned long tim = micros();

  while (digitalRead(p_fbl) == HIGH) { //measure echo pulse width
    tim2 = micros();
  }
  unsigned long timm = tim2 - tim;
  int dist = (timm / 58); //convert microseconds to centimeters
  //if (dist > 250) {dist = -1;} //Too far away or bad signal
  return dist;
}

//Front Bottom Right
int get_fbr() {
  unsigned long microsec;
  unsigned long zero = micros();
  unsigned long tim2 = 0;
  pinMode(p_fbr, OUTPUT);        //make port an output
  digitalWrite(p_fbr, HIGH);     //transmit signal
  
  while (microsec - zero < 5) {       //wait 5 microseconds
    microsec = micros();
  }

  pinMode(p_fbr, INPUT);         //change port to input

  while (digitalRead(p_fbr) == LOW) { //wait until signal is read
  }
  unsigned long tim = micros();

  while (digitalRead(p_fbr) == HIGH) { //measure echo pulse width
    tim2 = micros();
  }
  unsigned long timm = tim2 - tim;
  int dist = (timm / 58); //convert microseconds to centimeters
  //if (dist > 250) {dist = -1;} //Too far away or bad signal
  return dist;
}

//Back Top Left
int get_btl() {
  unsigned long microsec;
  unsigned long zero = micros();
  unsigned long tim2 = 0;
  pinMode(p_btl, OUTPUT);        //make port an output
  digitalWrite(p_btl, HIGH);     //transmit signal
  
  while (microsec - zero < 5) {       //wait 5 microseconds
    microsec = micros();
  }

  pinMode(p_btl, INPUT);         //change port to input

  while (digitalRead(p_btl) == LOW) { //wait until signal is read
  }
  unsigned long tim = micros();

  while (digitalRead(p_btl) == HIGH) { //measure echo pulse width
    tim2 = micros();
  }
  unsigned long timm = tim2 - tim;
  int dist = (timm / 58); //convert microseconds to centimeters
  //if (dist > 250) {dist = -1;} //Too far away or bad signal
  return dist;
}

//Back Top Right
int get_btr() {
  unsigned long microsec;
  unsigned long zero = micros();
  unsigned long tim2 = 0;
  pinMode(p_btr, OUTPUT);        //make port an output
  digitalWrite(p_btr, HIGH);     //transmit signal
  
  while (microsec - zero < 5) {       //wait 5 microseconds
    microsec = micros();
  }

  pinMode(p_btr, INPUT);         //change port to input

  while (digitalRead(p_btr) == LOW) { //wait until signal is read
  }
  unsigned long tim = micros();

  while (digitalRead(p_btr) == HIGH) { //measure echo pulse width
    tim2 = micros();
  }
  unsigned long timm = tim2 - tim;
  int dist = (timm / 58); //convert microseconds to centimeters
  //if (dist > 250) {dist = -1;} //Too far away or bad signal
  return dist;
}

//Back Bottom Right
int get_bbr() {
  unsigned long microsec;
  unsigned long zero = micros();
  unsigned long tim2 = 0;
  pinMode(p_bbr, OUTPUT);        //make port an output
  digitalWrite(p_bbr, HIGH);     //transmit signal
  
  while (microsec - zero < 5) {       //wait 5 microseconds
    microsec = micros();
  }

  pinMode(p_bbr, INPUT);         //change port to input

  while (digitalRead(p_bbr) == LOW) { //wait until signal is read
  }
  unsigned long tim = micros();

  while (digitalRead(p_bbr) == HIGH) { //measure echo pulse width
    tim2 = micros();
  }
  unsigned long timm = tim2 - tim;
  int dist = (timm / 58); //convert microseconds to centimeters
  //if (dist > 250) {dist = -1;} //Too far away or bad signal
  return dist;
}

//Back Bottom Left
int get_bbl() {
  unsigned long microsec;
  unsigned long zero = micros();
  unsigned long tim2 = 0;
  pinMode(p_bbl, OUTPUT);        //make port an output
  digitalWrite(p_bbl, HIGH);     //transmit signal
  
  while (microsec - zero < 5) {       //wait 5 microseconds
    microsec = micros();
  }

  pinMode(p_bbl, INPUT);         //change port to input

  while (digitalRead(p_bbl) == LOW) { //wait until signal is read
  }
  unsigned long tim = micros();

  while (digitalRead(p_bbl) == HIGH) { //measure echo pulse width
    tim2 = micros();
  }
  unsigned long timm = tim2 - tim;
  int dist = (timm / 58); //convert microseconds to centimeters
  //if (dist > 250) {dist = -1;} //Too far away or bad signal
  return dist;
}

//Left
int get_l() {
  unsigned long microsec;
  unsigned long zero = micros();
  unsigned long tim2 = 0;
  pinMode(p_l, OUTPUT);        //make port an output
  digitalWrite(p_l, HIGH);     //transmit signal
  
  while (microsec - zero < 5) {       //wait 5 microseconds
    microsec = micros();
  }

  pinMode(p_l, INPUT);         //change port to input

  while (digitalRead(p_l) == LOW) { //wait until signal is read
  }
  unsigned long tim = micros();

  while (digitalRead(p_l) == HIGH) { //measure echo pulse width
    tim2 = micros();
  }
  unsigned long timm = tim2 - tim;
  int dist = (timm / 58); //convert microseconds to centimeters
  //if (dist > 250) {dist = -1;} //Too far away or bad signal
  return dist;
}

//Right
int get_r() {
  unsigned long microsec;
  unsigned long zero = micros();
  unsigned long tim2 = 0;
  pinMode(p_r, OUTPUT);        //make port an output
  digitalWrite(p_r, HIGH);     //transmit signal
  
  while (microsec - zero < 5) {       //wait 5 microseconds
    microsec = micros();
  }

  pinMode(p_r, INPUT);         //change port to input

  while (digitalRead(p_r) == LOW) { //wait until signal is read
  }
  unsigned long tim = micros();

  while (digitalRead(p_r) == HIGH) { //measure echo pulse width
    tim2 = micros();
  }
  unsigned long timm = tim2 - tim;
  int dist = (timm / 58); //convert microseconds to centimeters
  //if (dist > 250) {dist = -1;} //Too far away or bad signal
  return dist;
}
