int ENA = 3;
int ENB = 5; 
int IN1 = 7;
int IN2 = 8;
int IN3 = 6;
int IN4 = 9;
int sensorCentro = 2;
int sensorEsquerda = 12;
int sensorDireita = 4;

int v = 0;
int leituraEsquerda;
int leituraDireita ;
int leituraCentro;
int erro = 0;
int x,y;
int constante = 120; //constante
float kp = 4;
float ki = 0.0001;//4
float kd = 1;//2
int dT = 0;
long tempoFinal=0;
long tempo = 0 ;
int ultimaLeitura;
int integral ; // armazena o valor do integrativo

void setup() {
 pinMode(ENA,OUTPUT);
 pinMode(ENB,OUTPUT);
 pinMode(IN1,OUTPUT);
 pinMode(IN2,OUTPUT);
 pinMode(IN3,OUTPUT);
 pinMode(IN4,OUTPUT);
 pinMode(sensorCentro,INPUT);
 pinMode(sensorEsquerda,INPUT);
 pinMode(sensorDireita,INPUT);
 digitalWrite(ENA,LOW);
 digitalWrite(ENB,LOW);
 digitalWrite(IN1,LOW);
 digitalWrite(IN2,LOW);
 digitalWrite(IN3,LOW);
 digitalWrite(IN4,LOW);
 tempoFinal = millis();
 //Serial.begin(9600);
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
  analogWrite(ENB,velocidadeMotor2);
}

void direita(int velocidadeMotor1, int velocidadeMotor2){
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite(ENA,velocidadeMotor1);
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
  if(x >= 255){
    x = 200;
  }
  if(y >= 255){
    y = 200;
  }  
  if(x < -255){
    x = -200;
  }
  if(y < -255){
    y = -200;
  }
}
void control(){
  limiteXY();
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
void definirErro(int setPoint){
  ultimaLeitura = v;
  if((leituraEsquerda+leituraCentro+leituraDireita ) !=0){
    v = (300*leituraEsquerda + 200*leituraCentro + 100*leituraDireita )/(leituraEsquerda+leituraCentro+leituraDireita );
  }
  erro =  setPoint - v;
  integral += (erro*dT*ki);
  x = constante + ((erro*kp) + (kd*(v - ultimaLeitura)/dT) + integral); // com kp igual a 5 o negativo no maximo chega 130 
  y = constante - ((erro*kp) + (kd*(v - ultimaLeitura)/dT) + integral); // so falta ajustar
}

void leitura(){
  leituraEsquerda = digitalRead(sensorEsquerda);
  leituraDireita = digitalRead(sensorDireita);
  leituraCentro = digitalRead(sensorCentro);
}


void definirTempo(){
  tempo = tempoFinal;
  tempoFinal = millis();
  dT = (tempoFinal - tempo);
}


void loop() {
  definirTempo();
  leitura();
  definirErro(200);
  control();
 

}
