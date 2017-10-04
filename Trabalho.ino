#include <Servo.h> 

int ENA = 3;
int ENB = 5; 
int IN1 = 7;
int IN2 = 8;
int IN3 = 6;
int IN4 = 9;
int sensorCentro = 2;
int sensorEsquerda = 12;
int sensorDireita = 4;
int trigger =1;
int echo = 11;
//int servo = 10;


int v = 0;
int leituraEsquerda;
int leituraDireita ;
int leituraCentro;
//int erro = 0;
int x,y;
//int constante = 87; //120constante
//float kp = 1.80;//1.8 // 1.77
//float ki = 0.0007;//0.0007
//int kd = 0;//2
int dT = 0;
long tempoFinal=0;
long tempo = 0 ;
int ultimaLeitura;
float integral ; // armazena o valor do integrativo
Servo servo;

void setup() {
 pinMode(ENA,OUTPUT);
 pinMode(ENB,OUTPUT);
 pinMode(IN1,OUTPUT);
 pinMode(IN2,OUTPUT);
 pinMode(IN3,OUTPUT);
 pinMode(IN4,OUTPUT);
 pinMode(trigger,OUTPUT);
 pinMode(echo,INPUT);
 pinMode(sensorCentro,INPUT);
 pinMode(sensorEsquerda,INPUT);
 pinMode(sensorDireita,INPUT);
 digitalWrite(ENA,LOW);
 digitalWrite(ENB,LOW);
 digitalWrite(IN1,LOW);
 digitalWrite(IN2,LOW);
 digitalWrite(IN3,LOW);
 digitalWrite(IN4,LOW);
 servo.attach(10);
 Serial.begin(9600);
 tempoFinal = millis();
 servo.write(45);
}

void frente(int velocidadeMotor1, int velocidadeMotor2){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  //velocidadeMotor1 = velocidadeMotor1 * 0.825;
  analogWrite(ENA,velocidadeMotor1);
  analogWrite(ENB,velocidadeMotor2);
}

void esquerda(int velocidadeMotor1, int velocidadeMotor2){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  //velocidadeMotor1 = velocidadeMotor1 * 0.825;
  analogWrite(ENA,velocidadeMotor1);
  analogWrite(ENB,velocidadeMotor2);//70 65
    
}

void direita(int velocidadeMotor1, int velocidadeMotor2){
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite(ENA,velocidadeMotor1);//
  analogWrite(ENB,velocidadeMotor2);
  
}
void re(int velocidadeMotor1, int velocidadeMotor2){
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  //velocidadeMotor1 = velocidadeMotor1 * 0.825;
  analogWrite(ENA,velocidadeMotor1);
  analogWrite(ENB,velocidadeMotor2);
}
void parar(){
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}
void limiteXY(){
  if(x >= 200){
    x = 180;
  }
  if(y >= 200){
    y = 180;
  }  

  if(x < -180){
    x = -180;
  }
  if(y < -180){
   y = -180;
  }
}
void control(){
 // limiteXY();
  if(x >= 0 && y>= 0){
    frente(x,y); 
    //Serial.println("frente");  
  }else if (x < 0 && y> 0){
    //Serial.println("direita"); 
    direita(-x,y);
  }else if(x > 0 && y< 0){
    //Serial.println("esquerda"); 
    esquerda(x,-y);
  }
}
/*void definirErro2(int setPoint){
  ultimaLeitura = v;
  if((leituraEsquerda+leituraCentro+leituraDireita ) !=0 && (leituraEsquerda+leituraCentro+leituraDireita ) !=3){
    v = (300*leituraEsquerda + 200*leituraCentro + 100*leituraDireita )/(leituraEsquerda+leituraCentro+leituraDireita );
  
  erro =  setPoint - v;
  integral += (erro*dT*ki);
  x = constante + ((erro*kp) + (kd*(v - ultimaLeitura)/dT) + integral); // com kp igual a 5 o negativo no maximo chega 130 
  y = constante - ((erro*kp) + (kd*(v - ultimaLeitura)/dT) + integral); // so falta ajustar
  }
}

void leitura(){
  leituraEsquerda = digitalRead(sensorEsquerda);
  leituraDireita = digitalRead(sensorDireita);
  leituraCentro = digitalRead(sensorCentro);
}*/
void definirTempo(){
  tempo = tempoFinal;
  tempoFinal = millis();
  dT = (tempoFinal - tempo);
}



int dispararPulso(int pinEcho, int pinTrigger){
  float tempo;
  digitalWrite(pinTrigger,HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrigger,LOW);
  tempo = pulseIn(pinEcho,HIGH);
  return tempo;
}

int calcularDistancia(int pinEcho, int pinTrigger){
  return dispararPulso(pinEcho,pinTrigger)/29.4/2;
}

int distanciaDireita;
int distanciaEsquerda;
int distanciaCentro;

void leituraDeArea(){
  distanciaDireita = 0;
  distanciaCentro = 0;
  distanciaEsquerda = 0;
  //delay(1000);
  //servo.write(45);
  //delay(800);
 /* while(distanciaDireita == 0 ){
    distanciaDireita = calcularDistancia(echo,trigger);
    delay(5);
    Serial.println(distanciaDireita);
  }
  servo.write(90);
  delay(800);
  while(distanciaCentro == 0){
    distanciaCentro = calcularDistancia(echo,trigger);
    delay(5);
    Serial.println(distanciaCentro);
  }*/
  //servo.write(135);
  //delay(800);
  while(distanciaEsquerda == 0){
    distanciaEsquerda = calcularDistancia(echo,trigger);
    delay(5);
    Serial.println(distanciaEsquerda);
  }
  
  
}
  
int distancia;
int ultimaDistancia;
int erro,dt;
int kd=0,kp=3;
float i,ki = 0.000;
int cons = 120;  
  
void definirErro(int setPoint){
  
  ultimaDistancia = distanciaEsquerda;
  leituraDeArea();
 // Serial.println(distanciaEsquerda);
  if(distanciaEsquerda > 0){
    erro =  setPoint - distanciaEsquerda;
    i += (erro*dT*ki);
    x = cons + ((erro*kp) + (kd*(distanciaEsquerda - ultimaDistancia)/dT) + i);
    y = cons - ((erro*kp) + (kd*(distanciaEsquerda - ultimaDistancia)/dT) + i);
  }
 /* Serial.println("-----");
  Serial.println(x);
  Serial.println(y);
  Serial.println("TO AQUI");
  /*if(distancia == 0){
    x = 10;
    y = 15;
  }*/
}  

void loop() {
  definirTempo();
  definirErro(30);
  control();
  //delay(5);

}
