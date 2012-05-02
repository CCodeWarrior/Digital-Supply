/* * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 7
 * LCD D5 pin to digital pin 6
 * LCD D6 pin to digital pin 5
 * LCD D7 pin to digital pin 4
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
*/

#include<avr/interrupt.h>
#include <LiquidCrystal.h>

#define lecturaSalida A0
#define referencia    A5

#define kp  5
#define ki  0.1
#define kd  0.1

#define boton  2
#define pwmCh  9

LiquidCrystal lcd(12, 11, 7, 6, 5, 4);

int ovf=0;
int contador = 0;  //debug

unsigned int refCh = 255;
unsigned int salCh = 123;
unsigned char btVisualizar=1;
unsigned char btvisAntes=1;
unsigned char btPresionado=1;


unsigned char digito2 = 0;
unsigned char digito1 = 0;
unsigned char digito0 = 0;

unsigned char estado = 0;

int error = 0;
float error_0 = 0;
float dTerm = 0;
float iTerm = 0;
float pTerm = 0;
float PID = 0;
float salidaPWM = 0;

void setup(){
  lcd.begin(16, 2);
  pinMode(boton,INPUT);
  pinMode(13,INPUT);
  digitalWrite(boton,HIGH);
  analogWrite(pwmCh,255);  //estado inicial del PWM


  //Configuracion del timer2 para interrupcion por overflow

  ASSR   |= (1<<AS2);
  TCCR2A = 0;
  TCCR2B |= (1<<CS22)|(1<<CS21)|(0<<CS20);
  TIMSK2 |= (1<<TOIE2);
  TCNT2 = 0;
//************************************  

  
  Serial.begin(9600);  //debug  
}



void loop(){
  entradas();
  MEF();
  salidas();

}


void entradas(){

  btvisAntes = btVisualizar;
  btVisualizar = digitalRead(boton);
  if (!btvisAntes &&  btVisualizar) btPresionado = 1;
  else  btPresionado = 0;
}


void MEF(){
}



void salidas(){
  float refDisp = 0;
  float salDisp = 0;
  refDisp = refCh;
  salDisp = salCh;
  refDisp = refDisp*12/1023;
  salDisp = salDisp*12/1023;
  lcd.clear();
  lcd.print("Ref: ");
  if(refDisp > 10)   lcd.setCursor(11, 0);
  else   lcd.setCursor(12, 0);
  lcd.print(refDisp, DEC);
  lcd.setCursor(0,1);
  lcd.print("Salida: ");
  if(salDisp > 10)   lcd.setCursor(11, 1);
  else   lcd.setCursor(12, 1);
  lcd.print(salDisp, DEC);
  Serial.print(refCh, DEC);
  Serial.print(", ");
  Serial.println(salCh, DEC);
  Serial.print("Error: ");
  Serial.println(error, DEC);
  delay(1000);
  
  

  
}

void convertBCD(unsigned int x){
        digito0 = x % 10;
        x=x/10;
        digito1 = x%10;
        x=x/10;
        digito2 = x%1000;
}

//ISR
ISR(TIMER2_OVF_vect){
  ovf++;
  
  if(ovf>=5){
    
    
    ovf=0;
  //lazo de control
    refCh = analogRead(referencia);
    salCh = analogRead(lecturaSalida);
  
    error_0=error;
    error = refCh - salCh;
  
    pTerm = kp*error;
    dTerm = kd*(error - error_0);
    iTerm = iTerm + ki*error;
    
    PID = pTerm + dTerm + iTerm;
    
    salidaPWM = salidaPWM + PID;
   
    if(salidaPWM < 102) salidaPWM = 100;
    else if(salidaPWM > 921) salidaPWM = 920;
    
    analogWrite(pwmCh, 255 - salidaPWM/4);   
  }

}




