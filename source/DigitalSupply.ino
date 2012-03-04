#include<avr/interrupt.h>

#define lecturaSalida A0
#define referencia    A1

#define kp  1
#define ki  1
#define kd  1

#define boton  2
#define pwmCh  3

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
int error_0 = 0;
int dTerm = 0;
int iTerm = 0;
int pTerm = 0;
int PID = 0;
int salidaPWM = 0;

void setup(){
  pinMode(boton,INPUT);
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
  //refCh = analogRead(referencia);
  //salCh = analogRead(lecturaSalida);
  btvisAntes = btVisualizar;
  btVisualizar = digitalRead(boton);
  if (!btvisAntes &&  btVisualizar) btPresionado = 1;
  else  btPresionado = 0;
}


void MEF(){
    switch(estado){
      case 0:
        convertBCD(salCh);
        if(btPresionado) estado = 1;
      break;
      
      case 1:
        convertBCD(refCh);
        if(btPresionado) estado = 0;
        
      break;
      
      default:
      break;      
    }
}


void convertBCD(unsigned int x){
        digito0 = x % 10;
        x=x/10;
        digito1 = x%10;
        x=x/10;
        digito2 = x%1000;
        
}

void salidas(){
//
//Serial.print(digito1, DEC);
//Serial.println(digito0, DEC);
delay(10);
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
