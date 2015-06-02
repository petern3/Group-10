/*
This sample code is for testing the 2 stepper motors
The rotation velocity can be adjusted by the code switch
Microcontroller: Arduino UNO 
*/
int M1dirpin = 31;
int M1steppin = 30;
int M2dirpin = 33;
int M2steppin = 32;

int M3dirpin = 35;
int M3steppin = 34;
int M4dirpin = 37;
int M4steppin = 36;

void setup()
{
  pinMode(M1dirpin,OUTPUT);
  pinMode(M1steppin,OUTPUT);
  pinMode(M2dirpin,OUTPUT);
  pinMode(M2steppin,OUTPUT);
  
  pinMode(M3dirpin,OUTPUT);
  pinMode(M3steppin,OUTPUT);
  pinMode(M4dirpin,OUTPUT);
  pinMode(M4steppin,OUTPUT);
}
void loop()
{
  int j;
  delayMicroseconds(2);
  digitalWrite(M1dirpin,LOW);
  //digitalWrite(M2dirpin,LOW);
  //digitalWrite(M3dirpin,LOW);
  //digitalWrite(M4dirpin,LOW);
  for(j=0;j<=5000;j++){
    digitalWrite(M1steppin,LOW);
    //digitalWrite(M2steppin,LOW);
    //digitalWrite(M3steppin,LOW);
    //digitalWrite(M4steppin,LOW);
    delayMicroseconds(2);
    digitalWrite(M1steppin,HIGH);
    //digitalWrite(M2steppin,HIGH);
    //digitalWrite(M3steppin,HIGH);
    //digitalWrite(M4steppin,HIGH);
    delay(1);
  }
}
