#include <Servo.h>

Servo thrservo;
Servo turnservo;
int thr=1500;
int turn=1500;
int thr2=1500;
int turn2=1500;
int DIR[2];
int i=0;

void setup() {
  Serial.begin(9600);
  
  thrservo.attach(11);
  turnservo.attach(3);
  thrservo.write(1500); 
  turnservo.write(1500);
   
}

void loop() {
  i=i+1;
  while (Serial.available() > 0) {
     //decifer the incoming serial string
     String thrstr = Serial.readStringUntil(',');
     String turnstr = Serial.readStringUntil('\n');
      
     thr= thrstr.toInt() ;//string to integer
     turn= turnstr.toInt() ;

     //cap velocity to prevent run away robot
     if(thr>1680){
      thr=1680;
     }
     else if(thr<1300){
      thr=1300;
     }
     if(turn>2000){
      turn=2000;
     }
     else if(turn<1000){
      turn=1000;
     }
     
     
     //thrservo.write(thr); 
     //turnservo.write(turn); 
     Serial.println(turn);
     i=0;
  }
  delay(3);

  //iterate to get to goal this keeps the
  //ESC from faulting
  if(thr!=thr2){
      thr2=((thr-thr2)/abs(thr-thr2))+thr2;
  }
  if(turn!=turn2){
      turn2=((turn-turn2)/abs(turn-turn2))+turn2;
     }
  //if serial is lost set throttle to 1500
  if (i>650){
    thr=1500;
    thr2=1500;
    //thrservo.write(thr);
  }
      //write to servo and esc
     thrservo.write(thr2); 
     turnservo.write(turn2); 
  }

  
  

