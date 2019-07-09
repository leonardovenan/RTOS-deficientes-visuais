#include <Arduino_FreeRTOS.h>
#include <Arduino.h>
#include <TM1637Display.h>
TM1637Display display(6, 7);

int cont = 0;//contador para auxiliar numero a ser mostrado no display
int nota = 0;
int pino_buzzer = 5;
int t_vd=20;//tempo de verde
int t_am=5;//tempo de amarelo
int t_vm=20;//tempo de vermelho
int tempo_via[] = {t_vd, t_vd + t_am, t_vd + t_am + t_vm};//vetor com os tempos do semáforo
int tempo_pedestre[] = {tempo_via[2] - tempo_via[1], tempo_via[1] - tempo_via[0]};

int estado = 0;//0=verde,1=amarelo,2=vermelho
int estado_pedestre = 2;//0=verde,1=verde piscando,2=vermelho

int verde = 13;//pino para sinal verde da via
int amarelo = 12;//pino para sinal amarelo da via
int vermelho = 8;//pino para sinal vermelho da via
int verde_pedestre = 4;//pino para sinal verde do pedestre
int vermelho_pedestre = 2;//pino para sinal vermelho do pedestre
int pino_ativo=10;//pino para acender LED de atividade

int ativo = 0;//Flag para funcionamento do sistema. Quando 0, o sistema fica em espera.
int buzz = 0;
int freq_localizacao=950;
int dur_localizacao=60;
int ciclo_localizacao=2000;
int freq[]={2000,1500,1000,500};
int cont_freq=0;
int freq_inicial=2000;
int freq_final=500;
int dur_op=160;
int ciclo_op=1000;



// define two tasks for Blink & AnalogRead
void TaskBlink(void *pvParameters);
void TaskDisplay7Seg(void *pvParameters);
void TaskBuzzer(void *pvParameters);
void TaskAnalogRead(void *pvParameters);

// the setup function runs once when you press reset or power the board
void setup()
{

    // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
    display.setBrightness(0x0f);
    display.clear();
    pinMode(verde, OUTPUT);
    pinMode(amarelo, OUTPUT);
    pinMode(vermelho, OUTPUT);
    pinMode(verde_pedestre, OUTPUT);
    pinMode(vermelho_pedestre, OUTPUT);
    pinMode(pino_ativo, OUTPUT);

    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
    }

    // Now set up two tasks to run independently.
    xTaskCreate(
        TaskBlink, (const portCHAR *)"Blink" // A name just for humans
        ,
        128 // This stack size can be checked & adjusted by reading the Stack LOWwater
        ,
        NULL, 1 // Priority, with 3 (configMAX_PRIORITIES - 1) being the LOWest, and 0 being the HIGHest.
        ,
        NULL);

    xTaskCreate(
        TaskDisplay7Seg, (const portCHAR *)"Display 7 Seg" // A name just for humans
        ,
        128 // This stack size can be checked & adjusted by reading the Stack LOWwater
        ,
        NULL, 0 // Priority, with 3 (configMAX_PRIORITIES - 1) being the LOWest, and 0 being the HIGHest.
        ,
        NULL);

    xTaskCreate(
        TaskBuzzer, (const portCHAR *)"Buzzer" // A name just for humans
        ,
        128 // This stack size can be checked & adjusted by reading the Stack LOWwater
        ,
        NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the LOWest, and 0 being the HIGHest.
        ,
        NULL);

    xTaskCreate(
        TaskAnalogRead, (const portCHAR *)"AnalogRead", 128 // Stack size
        ,
        NULL, 3 // Priority
        ,
        NULL);

    // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
    // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBlink(void *pvParameters) // This is a task.
{
    (void)pvParameters;

    digitalWrite(verde, LOW);
    digitalWrite(vermelho, LOW);
    digitalWrite(amarelo, LOW);
    digitalWrite(verde_pedestre, LOW);
    digitalWrite(vermelho_pedestre, LOW);

    // initialize digital LED_BUILTIN on pin 13 as an output.

    for (;;) // A Task shall never return or exit.
    {
        if (ativo)
        {
            analogWrite(pino_ativo, 255);
            switch (estado)
            {
            case 0:
                digitalWrite(verde, HIGH);
                digitalWrite(vermelho, LOW);
                digitalWrite(vermelho_pedestre, HIGH);
                digitalWrite(verde_pedestre, LOW);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
            case 1:
                digitalWrite(verde, LOW);
                digitalWrite(amarelo, HIGH);
                digitalWrite(vermelho, LOW);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
            case 2:
                digitalWrite(amarelo, LOW);
                digitalWrite(vermelho, HIGH);
                digitalWrite(vermelho_pedestre, LOW);
                if (estado_pedestre!=1)
                {
                    digitalWrite(verde_pedestre, HIGH);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    digitalWrite(verde_pedestre, LOW);
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                    digitalWrite(verde_pedestre, HIGH);
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                    digitalWrite(verde_pedestre, LOW);
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                    digitalWrite(verde_pedestre, HIGH);
                    vTaskDelay(250 / portTICK_PERIOD_MS);
                }
                break;
            }
            cont++;
            if (cont == tempo_via[estado])
                estado++;
            if (cont == (tempo_via[2])-t_am-1){
                estado_pedestre=1;
            }
            if (estado > 2)
            {
                estado = 0;
                cont = 0;
                display.clear();
                ativo = 0;
                estado_pedestre=2;
                
            }
            
        }
        else{
            analogWrite(pino_ativo, 0);
            digitalWrite(verde, HIGH);
            digitalWrite(vermelho, LOW);
            digitalWrite(vermelho_pedestre, HIGH);
            digitalWrite(verde_pedestre, LOW);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

    }
}

void TaskDisplay7Seg(void *pvParameters)
{
    (void) pvParameters;
    for (;;)
    {
        if (cont < tempo_via[0]&&ativo)
        {
            display.showNumberDec((tempo_via[0] - cont - 1), false,2,0);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }else if (cont>(tempo_via[1]-1)&&cont<(tempo_via[2])&&ativo){
            display.showNumberDec((tempo_via[2] - cont - 1), false);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }else {
          display.clear();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

    }
}
void TaskBuzzer(void *pvParameters)
{
  (void) pvParameters;
    for (;;)
    {
      if(estado<2){
        if (cont<(tempo_via[1]-1)){
          tone(pino_buzzer,freq_localizacao,dur_localizacao);
          vTaskDelay((ciclo_localizacao/2-dur_localizacao) / portTICK_PERIOD_MS);
        }
        else{
          tone(pino_buzzer,freq[cont_freq],dur_op);
          vTaskDelay(dur_op / portTICK_PERIOD_MS);
          cont_freq++;
          if (cont_freq>3){
            cont_freq=0;
            vTaskDelay((ciclo_op-4*160) / portTICK_PERIOD_MS);
            }
          }
          vTaskDelay((ciclo_localizacao/2-dur_localizacao) / portTICK_PERIOD_MS);
      }
      else{
         if (estado_pedestre!=1){
            tone(pino_buzzer,freq_inicial,dur_op);
            vTaskDelay(dur_op / portTICK_PERIOD_MS);
            tone(pino_buzzer,freq_final,dur_op);
            vTaskDelay((ciclo_op-dur_op) / portTICK_PERIOD_MS);
          }
         else{
            tone(pino_buzzer,freq_inicial,dur_op);
            vTaskDelay(dur_op / portTICK_PERIOD_MS);
            tone(pino_buzzer,freq_final,dur_op);
            vTaskDelay((ciclo_op/2-dur_op) / portTICK_PERIOD_MS);
         }
       }
    }
}

void TaskAnalogRead(void *pvParameters) // This is a task.
{
    (void)pvParameters;

    /*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

    for (;;)
    {
        // read the input on analog pin 0:
        int sensorValue = analogRead(A0);
        if (sensorValue>100&&!ativo) {ativo=1; buzz=1; }
        if (sensorValue>100&&ativo){
          tone(pino_buzzer,3000,dur_op);
          vTaskDelay(ciclo_op / portTICK_PERIOD_MS);
          }
        // print out the value you read:
        //Serial.println(sensorValue);
        vTaskDelay(100/portTICK_PERIOD_MS); // one tick delay (15ms) in between reads for stability
    }
}
