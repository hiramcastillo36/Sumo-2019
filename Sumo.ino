#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int pw[]={0,5,6}; //Pines de los motores
const int rg[]={0,7,4};
const int lf[]={0,8,9};

int trig[]= {0, 2,12}; //Pines de los ultrasonicos
int echo[]= {0, 3,11}; //El primero es del ultrasonico izquierdo
int us[4];
// defines variables
long duration;
int distance;
bool start=0, usR=0, usL=0, last_us;
int last, dir=0;
void setup(){
  pinMode(0,INPUT_PULLUP);  //Pines de los botones
  pinMode(10,INPUT_PULLUP);  
  for(int i=1; i<=2; i++){
    pinMode(pw[i],OUTPUT);
    pinMode(rg[i],OUTPUT);
    pinMode(lf[i],OUTPUT);

    pinMode(trig[i],OUTPUT);
    pinMode(echo[i],INPUT);
  }
  Serial.begin(9600); // Starts the serial communication
}

void loop(){
 while(!start){ //Mientras no se haya presionado ningun boton
    if(!digitalRead(0)){  //Para ir a la derecha
      Serial.println("AAAAA");
      delay(5000);
      girarderecha(90);
      delay(350); //Delay para girar
      //avanza();
      //delay(200);
      start=1;
      dir=0;
    }
    if(!digitalRead(10)){  //Para ir a la izquierda
      Serial.println("AAAAA");
      delay(5000);
      girarizq(90);
      delay(300); //Delay para girar 220
      //avanza();
      //delay(200);
      start=1;
      dir=1;
    }
  }

  for(int i=1; i<=1; i++){ //Leer ultrasonicos
    digitalWrite(trig[i], LOW);
    delayMicroseconds(2);
    digitalWrite(trig[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trig[i], LOW);
    us[i]= pulseIn(echo[i], HIGH);
    us[i]= us[i]/29/2;
  }
  usL= us[1]<40&&us[1]>0? 1:0; //Si los ultrasonicos detectan menor de 50 cm
  usR= us[2]<40&&us[2]>0? 1:0;
  
  if(usL)avanza();
  else {
    if(dir==1) girarizq(90);
    else girarderecha(90);  
   
  }

}

void avanza(){
    digitalWrite(7,HIGH);
digitalWrite(8,LOW);
analogWrite(5,175);

digitalWrite(4,HIGH);
digitalWrite(9,LOW);
analogWrite(6,175);

}
void girarderecha(int sp){
   digitalWrite(7,LOW);
digitalWrite(8,HIGH);
analogWrite(5,sp);
digitalWrite(4,HIGH);
digitalWrite(9,LOW);
analogWrite(6,sp);

}
void girarizq(int sp){
  digitalWrite(7,HIGH);
  digitalWrite(8,LOW);
  analogWrite(5,sp);
  digitalWrite(4,LOW);
  digitalWrite(9,HIGH);
  analogWrite(6,sp);
}
/*
void motor(int n, int sp){ //n= numero del motor, sp= velocidad
  bool dir= sp>0? 1:0;
  analogWrite(pw[n], abs(sp));
  digitalWrite(rg[n], !dir);
  digitalWrite(lf[n], dir);
}

void spin(int dir){
  for(int i=1; i<=2; i++) 
    motor(i, 100*dir); //Cambiar esto (Velocidad del giro)
}

void fwd(){
  motor(1, 255); //Mover los motores hacia adelante
  motor(2, -255);
}*/
