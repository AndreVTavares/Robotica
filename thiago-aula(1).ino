int ENA = 3;
int ENB = 5; 
int IN1 = 7;
int IN2 = 8;
int IN3 = 6;
int IN4 = 9;
int c = 2;
int l = 12;
int r = 4;

void setup() {
 pinMode(ENA,OUTPUT);
 pinMode(ENB,OUTPUT);
 pinMode(IN1,OUTPUT);
 pinMode(IN2,OUTPUT);
 pinMode(IN3,OUTPUT);
 pinMode(IN4,OUTPUT);
 pinMode(c,INPUT);
 pinMode(l,INPUT);
 pinMode(r,INPUT);
 digitalWrite(ENA,LOW);
 digitalWrite(ENB,LOW);
 digitalWrite(IN1,LOW);
 digitalWrite(IN2,LOW);
 digitalWrite(IN3,LOW);
 digitalWrite(IN4,LOW);
 Serial.begin(9600);
}

void frente(int velocidadeMotor1, int velocidadeMotor2){
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  velocidadeMotor1 = velocidadeMotor1 * 0.825;
  analogWrite(ENA,velocidadeMotor1);
  analogWrite(ENB,velocidadeMotor2);
    Serial.println(velocidadeMotor1);
  Serial.println(velocidadeMotor2);
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
  velocidadeMotor1 = velocidadeMotor1 * 0.825;
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
  int v = 0;
  int lr;
  int rr;
  int cr;
  int erro = 0;
  int x,y;
  int cons = 120; //constante
  float kp = 4;
  int ki = 0.0001;//4
  int kd = 1;//2
  int dt = 0;
  int tempFinal=0;
  int temp = 0 ;
  int ult;
void check(){
  Serial.println(digitalRead(c));
  Serial.println(digitalRead(l));
  Serial.println(digitalRead(r));
}

void control(){
  if(x >= 255){
    x = 255;
  }
  if(y >= 255){
    y = 255;
  }  

  if(x < -255){
    x = -255;
  }
  if(y < -255){
    y = -255;
  }
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
int i ;
void definirErro(int setPoint){
  ult = v;
  if((lr+cr+rr) !=0){
    v = (300*lr + 200*cr + 100*rr)/(lr+cr+rr);
  }
  erro =  setPoint - v;
  i += (erro*dt*ki);
  x = cons + ((erro*kp) + (kd*(v - ult)/dt) + i);
  y = cons - ((erro*kp) + (kd*(v - ult)/dt) + i);
}

void leitura(){
  lr = digitalRead(l);
  rr = digitalRead(r);
  cr = digitalRead(c);
}



void loop() {
  temp = tempFinal;
  tempFinal = millis();
  dt = (tempFinal - temp);
  leitura();
  definirErro(200);
  control();
 

}
