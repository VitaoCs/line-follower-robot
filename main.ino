/* Definição de pinagem da placa para os motores */
#define engineLeftDirection1 32
#define engineLeftDirection2 33
#define engineLeftVelocity 27
#define engineRightDirection1 25
#define engineRightDirection2 26
#define engineRightVelocity 14

/* Definição de pinagem da placa para os sensores infravermelhos*/
#define sensor1 23
#define sensor2 22
#define sensor3 21
#define sensor4 19
#define sensor5 18

/* Definição das constantes para o sinal PWM */
const int freq = 30000;
const int ledChannel = 0;
const int ledChanne2 = 1;
const int resolution = 8;

/* Vetor para leitura dos sensores infravermelhos */
int LFSensor[5]={0, 0, 0, 0, 0};

/* Vetor de erros dos sensores infravermelhos */
const int trackBias[5] = {-6,-3, 0, 3, 6};

/* Velocidades dos motores */
int mean = 175;               // velocidade base
int velocidade1, velocidade2; // variaveis de velocidade para cada motor

/* Variáveis de controle para o PID */
int fimDePista = 0;           // flag para identificar final da pista 
int realFim = 0;              // flag para identificar o final real do circuito
int lineOut = 0;              // flag para identificar saida da pista
bool liberaControle = false;  // flag para ativar controlador PID

/* Parâmetros do PID */
float error = 0;
float Kp = 6;
int Ki = 0;
float Kd = 3;
float deltaT = 0.1;           // período de aquisição dos valores dos sensores
float h1 = deltaT/2;          // parâmetros do PID que carregam deltaT
float h2 = 1/deltaT;
float b0, b1, b2;             // variáveis para calculo do PID
float previousErrorOne = 0;   // ultimo erro calculado [ e(n-1) ]
float previousErrorTwo = 0;   // penúltimo erro calculado [ e(n-2) ]
float PIDvalue = 0;           // resposta do controlador calculada [ u(n) ]
float PIDLastvalue = 0;       // ultima resposta do controlador calculada [ u(n-1) ]

/* Variaveis para interrupção do contador/timer */
volatile int interruptCounter;
int totalInterruptCounter;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/* Interrupção do timer */
void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&timerMux);
    liberaControle = true;
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
}

/* Definição dos outputs/inputs da pinagem da placa */
void initPinouts() {
    Serial.print("Setting pinnouts..........\n");
    pinMode(engineLeftDirection1, OUTPUT);
    pinMode(engineLeftDirection2, OUTPUT);
    pinMode(engineRightDirection1, OUTPUT);
    pinMode(engineRightDirection2, OUTPUT);
    
    pinMode(sensor1, INPUT);
    pinMode(sensor2, INPUT);
    pinMode(sensor3, INPUT);
    pinMode(sensor4, INPUT);
    pinMode(sensor5, INPUT);
    
    // Configurando funcionalidade do PWM
    ledcSetup(ledChannel, freq, resolution);
    ledcSetup(ledChanne2, freq, resolution);
    
    // Anexando os canais de PWM aos motores
    ledcAttachPin(engineLeftVelocity, ledChannel);
    ledcAttachPin(engineRightVelocity, ledChanne2);
    Serial.print("End setting pinnouts..........\n");
}

/* Configurando valores iniciais para a corrida */
void initRacing() {
    delay(500);
    digitalWrite(engineLeftDirection1,HIGH);
    digitalWrite(engineLeftDirection2,LOW);
    digitalWrite(engineRightDirection1,LOW);
    digitalWrite(engineRightDirection2,HIGH);
    ledcWrite(ledChannel, 160);
    ledcWrite(ledChanne2, 160);
}

/* Inicia timer por interrupção */
void initTimer() {
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 50000, true);
    timerAlarmEnable(timer);
}

/* Escreve o valor de leitura dos sensores no vetor */
void readPosition() {
    LFSensor[0] = digitalRead(sensor1);
    LFSensor[1] = digitalRead(sensor2);
    LFSensor[2] = digitalRead(sensor3);
    LFSensor[3] = digitalRead(sensor4);
    LFSensor[4] = digitalRead(sensor5);

    lineOut = 0;

    // Verifica final de pista e da corrida
    if(((LFSensor[0]== 1 )&&(LFSensor[1]== 1 )&&(LFSensor[2]== 1 )&&(LFSensor[3]== 1 )&&(LFSensor[4]== 1 ))){
      if(realFim == 3){
        fimDePista += 1;
      }else{
        realFim += 1;
      }
    // Verifica saida da pista
    }else if(((LFSensor[0]== 0 )&&(LFSensor[1]== 0 )&&(LFSensor[2]== 0 )&&(LFSensor[3]== 0 )&&(LFSensor[4]== 0 ))){
        lineOut = 1;
    }else realFim = 0;
}

/* Calcula o erro do controle baseado na leitura dos sensores */
void calcError() {
    int i;
    int activeSensors = 0;
    error = 0;

    for(i=0;i<5;i++)
        activeSensors += LFSensor[i];
    // Calcula erro total dos sensores considerando os sensores ativos
    for(i=0;i<5;i++)
        error += trackBias[i]*LFSensor[i];
    
    // Se sair da pista, mantém o ultimo erro lido [ e(n-1) ]
    error = lineOut ? previousErrorOne : error/activeSensors;
}

/* Faz o cálculo do sinal de controle PID */
void calculatePID() {
    // Calculo das variáveis do controlador
    b0 = Kp + (Ki*h1) +(Kd*h2);
    b1 = -Kp + (Ki*h1) - (2*Kd*h2);
    b2 = Kd*h2;
    // Verifica se erro é numérico (ou se retorna NAN/null por erro de leitura dos sensores)
    error = isnan(error) ? 0 : error;

    // Calculo do sinal de controle [ u(n) ] considerando o erro atual [ e(n) ], ultimo erro [ e(n-1) ],
    // penúltimo erro [ e(n-2) ] e ultimo esforço de controle calculado [ u(n-1) ]
    PIDvalue = (error*b0) + (previousErrorOne*b1) + (previousErrorTwo*b2) + PIDLastvalue;

    // Verifica se sinal de controle é numérico (ou se retorna NAN/null por erro de cálculo)
    PIDvalue = isnan(PIDvalue) ? 0 : PIDvalue;

    // Atualiza valores antigos dos erros e esforço de controle
    previousErrorTwo = previousErrorOne;
    previousErrorOne = error;
    PIDLastvalue = PIDvalue;
}

/* Zera a velocidade dos motores para parada */
void endRace() {
    ledcWrite(ledChannel, 0);
    ledcWrite(ledChanne2, 0);
}

/* Escreve nos motores a média das velocidades +- o controle PID */
void limiteVelocidade(){
    // Limita a velocidade máxima dos motores em 220 de 255 do sinal PWM
    velocidade1 = (mean + PIDvalue) > 220 ? 220 : (mean + PIDvalue);
    velocidade2 = (mean - PIDvalue) > 220 ? 220 : (mean - PIDvalue);
}

/* Faz o setup da placa */
void setup() {
    // Inicia comunicação serial e chama métodos de configurações iniciais
    Serial.begin(115200);
    initPinouts();
    initRacing();
    initTimer();
}

/* Repete a função loop enquanto o controlador estiver ligado */
void loop() {
    unsigned long start = micros();
    
    // Verifica se o carrinho está no final da pista
    if(fimDePista != 2) {
        // Lê a posição do carrinho e calcula o erro
        readPosition();
        calcError();

        // Verifica flag de controle atualizada pela interrupção do contador/timer
        if(liberaControle){
            calculatePID();
            liberaControle = false;
        }
        // Atualiza velocidade dos motores
        limiteVelocidade();
        ledcWrite(ledChannel, velocidade1);
        ledcWrite(ledChanne2, velocidade2);
    }
    else {
        endRace();
    }
    delay(50);
}
