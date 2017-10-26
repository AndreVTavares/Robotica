int ENA = 3;
int ENB = 5; 
int IN1 = 7;
int IN2 = 8;
int IN3 = 6;
int IN4 = 9;

int sensorCentroEsquerda = 12;
int sensorCentroDireita = 11;
int sensorEsquerda = 13;
int sensorDireita = 10;


int leituraEsquerda;
int leituraDireita ;
int leituraCentroEsquerda;
int leituraCentroDireita;

double erro = 0;
double input = 0;
double errSum, ultimoErro,dInput, kp, ki, kd;
double dT = 0;
long tempoFinal=0;
long tempo = 0 ;
double ultimaLeitura;

int x,y;
int constante = 200; //120constante



void frente(int velocidadeMotor1, int velocidadeMotor2){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  analogWrite(ENA,velocidadeMotor1);
  analogWrite(ENB,velocidadeMotor2);
}

void esquerda(int velocidadeMotor1, int velocidadeMotor2){
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
  analogWrite(ENA,velocidadeMotor1);
  analogWrite(ENB,velocidadeMotor2);//70 65
    
}

void direita(int velocidadeMotor1, int velocidadeMotor2){
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
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
  if(x >= 230){
    x = 230;
  }
  if(y >= 230){
    y = 230;
  }  

  if(x < -180){
    x = -180;
  }
  if(y < -180){
   y = -180;
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


int amostra = 1000; //1 sec
void definirErro(double setpoint){
  definirTempo();
  if(dT>=amostra &&( leituraEsquerda || leituraDireita )){
    input = (-250*leituraEsquerda + 250*leituraDireita);
    erro = setpoint - input;
    if(errSum >= 400){
      errSum = 0;
    }
    errSum += (erro);//* dT);
    dInput = (input - ultimaLeitura );//- ultimoErro) / dT;
    //Serial.println();
    x = constante + (kp * erro + ki * errSum - kd * dInput);
    y = constante - (kp * erro + ki * errSum -   kd * dInput);
    ultimoErro = erro;
    ultimaLeitura = input;
    dT = 0;
  }
}

double erro2 = 0;
double input2 = 0;
double errSum2, ultimoErro2, kp2, ki2, kd2;
double ultimaLeitura2;

void definirErro3(double setpoint){
  definirTempo();
  if(dT>=amostra){
    setAmostra(30);
    if(( leituraEsquerda || leituraDireita )){
      input = (-250*leituraEsquerda + 250*leituraDireita);
      erro = setpoint - input;
      if(errSum >= 400){
        errSum = 0;
      }
      errSum += (erro);//* dT);
      dInput = (input - ultimaLeitura );//- ultimoErro) / dT;
      //Serial.println();
      x = constante + (kp * erro + ki * errSum - kd * dInput);
      y = constante - (kp * erro + ki * errSum -   kd * dInput);
      ultimoErro = erro;
      ultimaLeitura = input;
      setAmostra(200);
    }else if((leituraCentroEsquerda || leituraCentroDireita)){
      input = (-350*leituraCentroEsquerda + 350*leituraCentroDireita);
      erro = setpoint - input;
      if(errSum2 >= 400){
        errSum2 = 0;
      }
      errSum2 += (erro);//* dT);
      dInput = (input - ultimaLeitura2);//- ultimoErro) / dT;
      //Serial.println();
      x = constante + (kp2 * erro + ki2 * errSum2 - kd2 * dInput);
      y = constante - (kp2 * erro + ki2 * errSum2 -   kd2 * dInput);
      ultimoErro2 = erro;
      ultimaLeitura2 = input;
      setAmostra(300);
    }else{
      x = constante;
      y = constante;
    }
    dT = 0;
  }
}



int amostra2=1000;

void definirErro2(double setpoint){
  if(dT>=amostra2 && (leituraCentroEsquerda || leituraCentroDireita)){
    input = (-350*leituraCentroEsquerda + 350*leituraCentroDireita);
    erro = setpoint - input;
    if(errSum2 >= 400){
      errSum2 = 0;
    }
    errSum2 += (erro);//* dT);
    dInput = (input - ultimaLeitura2);//- ultimoErro) / dT;
    //Serial.println();
    x = constante + (kp2 * erro + ki2 * errSum2 - kd2 * dInput);
    y = constante - (kp2 * erro + ki2 * errSum2 -   kd2 * dInput);
    ultimoErro2 = erro;
    ultimaLeitura2 = input;
    dT = 0;
  }
}


void setTunings(double Kp, double Ki, double Kd){
    double amostraEmSeg = ((double)amostra)/1000;
    kp = Kp;
    ki = Ki * amostraEmSeg;
    kd = Kd / amostraEmSeg;
}

void setTunings2(double Kp, double Ki, double Kd){
    double amostraEmSeg = ((double)amostra)/1000;
    kp2 = Kp;
    ki2 = Ki * amostraEmSeg;
    kd2 = Kd / amostraEmSeg;
}


void setAmostra(int novaAmostra){
  if (novaAmostra > 0){
    double ratio = (double)novaAmostra / (double)amostra;
    ki *= ratio;
    kd /= ratio;
    ki2 *= ratio;
    kd2 /= ratio;
    amostra = (unsigned long)novaAmostra;
  }
}

void leitura(){
  leituraEsquerda = digitalRead(sensorEsquerda);
  leituraDireita = digitalRead(sensorDireita);
  leituraCentroDireita = digitalRead(sensorCentroDireita);
  leituraCentroEsquerda = digitalRead(sensorCentroEsquerda);
}
void definirTempo(){
  tempo = tempoFinal;
  tempoFinal = millis();
  dT += (double)(tempoFinal - tempo);
}

void setup() {
 pinMode(ENA,OUTPUT);
 pinMode(ENB,OUTPUT);
 pinMode(IN1,OUTPUT);
 pinMode(IN2,OUTPUT);
 pinMode(IN3,OUTPUT);
 pinMode(IN4,OUTPUT);
 pinMode(sensorCentroDireita,INPUT);
 pinMode(sensorCentroEsquerda,INPUT);
 pinMode(sensorEsquerda,INPUT);
 pinMode(sensorDireita,INPUT);
 
 digitalWrite(ENA,LOW);
 digitalWrite(ENB,LOW);
 digitalWrite(IN1,LOW);
 digitalWrite(IN2,LOW);
 digitalWrite(IN3,LOW);
 digitalWrite(IN4,LOW);
 tempoFinal = millis();
 setAmostra(30);
 setTunings(3.5, 0.8, 0);
 setTunings2(1.8, 0.3, 0);
// setAmostra2(30);
 
 
 //Serial.begin(9600);
}

void loop() {
  leitura();
  definirErro3(0);
  control();
  

}
