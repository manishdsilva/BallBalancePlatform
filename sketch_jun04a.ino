#include <VarSpeedServo.h> 
#include <String.h>
 
VarSpeedServo Servo1;    // create servo object to control a servo 
VarSpeedServo Servo2;  
VarSpeedServo Servo3;
VarSpeedServo Servo4;

int x_angle = 10;
int y_angle = -10;
int i ;
String angle;
int a,b,u,v,w,x,y,z;
unsigned long previousMillis = 0;
int interval = 100;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Servo1.attach(3);      // attach the signal pin of servo to pin9 of arduino
  Servo2.attach(5);
  Servo3.attach(6);      // attach the signal pin of servo to pin9 of arduino
  Servo4.attach(9);

  Servo1.write(90);
  Servo2.write(90);
  Servo3.write(90);
  Servo4.write(90);
 // delay(100);
 //Serial.setTimeout(50);
}

void loop() 
{  
  //unsigned long currentMillis = millis();
     i=0;
  if((Serial.available()>0)) //&& ((currentMillis-previousMillis)<interval))
  {   
      //Serial.println("Receiving Data at Arduino");
      angle = Serial.readStringUntil('*');
      //Serial.println(angle);
      //l = strlen(angle);
      
      //for(i=0; angle[i] !='\0';i=i+6)
      //{
      
        //Serial.println(angle);
        //Serial.println(angle[0]);
        //Serial.println(angle[1]);
        
        u = angle[i]-'0';
        v = angle[i+1]-'0';
        w = angle[i+2]-'0';
        x = angle[i+3]-'0';
        y = angle[i+4]-'0';
        z = angle[i+5]-'0';
        
        a = (u*100)+ (v*10) + w;
        b = (x*100)+ (y*10) + z;
        
        //Serial.println(a);
  
        Servo1.slowmove(a,200);
        Servo3.slowmove(180-a,200);                                     
        Servo2.slowmove(b,200);
        Servo4.slowmove(180-b,200);
    
      //}
      //angle = Serial.readString();
     
     

  /*if(i%2 == 0)
  {
    x_angle = -10;
    y_angle = 10;
    
  }
  else
  {
    x_angle = 10;
    y_angle = -10;
  }
  if(i<16)
  {
  Servo_yDir();
  Servo_xDir();
  delay(2000);
  i++ ;
  }*/
}
}
