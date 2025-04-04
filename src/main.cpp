#include <Arduino.h>
/*************************************************************
  Blynk library is licensed under MIT license
 *************************************************************
  Blynk.Edgent implements:
  - Blynk.Inject - Dynamic WiFi credentials provisioning
  - Blynk.Air    - Over The Air firmware updates
  - Device state indication using a physical LED
  - Credentials reset using a physical Button
 *************************************************************/
/* 
   Firmware para o  hardware KC 868-A6 v1.3 ou compatível

 - no template do Blynk Console setar para sincronizar hora start, hora stop (V40), modo (V49), etc...
 - Blynk.syncVirtual (V40, V49);     // sincroniza datastream agendamento e modo de operação
 - em Settings.h adicionado:
    46  #define BOARD_LED_PIN               2
    47  #define BOARD_LED_INVERSE           false
    48  #define BOARD_LED_BRIGHTNESS        128

 - em ResetButton.h adicionado:
     7  int flagSetRTC = 0;
    33  DEBUG_PRINT("Botão pressionado rapidamente, o relógio será recalibrado pelo NTP!");
    34  flagSetRTC = 1;

 - em Settings.h comentada linha 41 - warning led

 - RTC: ajuste automático via NTP com um click rápido no GPOI0 ou em horario definido
*/
// sensor MODBUS 4x1 - Slave ID 32
// sensor MODBUS CWT - Slave ID 01

#define BLYNK_TEMPLATE_ID        "TMPL2r6MI0Ptc"
#define BLYNK_TEMPLATE_NAME      "Automação Silo"
#define Slave_ID_EXT             32 // sensor 4x1

//#define BLYNK_TEMPLATE_ID        "TMPL2NIQXip7y"
//#define BLYNK_TEMPLATE_NAME      "Área de Teste"
//#define Slave_ID_EXT             1 // sensor CWT

#define BLYNK_FIRMWARE_VERSION   "0.1.6"
//#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG   
//#define APP_DEBUG

#define USE_ESP32_DEV_MODULE
#include "time.h"                               // biblioteca para ajuste e controle do tempo (NTP)
#include "BlynkEdgent.h"
#include "HardwareSerial.h"
#include "SPI.h"
#include "RTClib.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Wire.h"
#include <Preferences.h>                        // biblioteca para salvar dados na NVS
#include "HeartBeat.h"
#include "uptime_formatter.h"                   // imprimir uptime do sistema

unsigned char output_PLC = 0b11111111;          // variavel para controle dos relês de saída - iniciam tudos desligados (0 liga)

RTC_DS1307           RTC;                       // RTC DS1307 - Address I2C (0x68)
Preferences  preferences;
unsigned int  counterRST;                       // contador de reset's
bool    sendBlynk = true;

  // ATENÇÃO AO WATCHDOG; AJUSTAR COM MARGEM DE ACORDO COM O TEMPO ABAIXO !!!
int tempoStart  =     10; // tempo de espera em segundos para o inicio do sistema a cada reset minimo 10 segundos para RTC start
int timerON     =      0; // usado para mostrar só uma vez a cada reset a tela inicial no diplay com a logomarca
int minAtualiza =      0; // usado para enviar dados ao servidor a cada 15 minutos, começa a enviar no minAtualiza
int BotaoRESET;           // BotaoRESET = Virtual do APP

char monthString[25] = {"010203040506070809101112"};                // modelo 1
int  monthIndex[122] = {0,2,4,8,10,12,14,16,17,18,20,22};

//char monthString[37]= {"JANFEVMARABRMAIJUNJULAGOSETOUTNOVDEZ"};   // modelo 2
//int  monthIndex[122] ={0,3,6,9,12,15,18,21,24,27,30,33};
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

#define I2C_SCL 15                             // SCL (ESP32 default GPIO 22)
#define I2C_SDA 4                              // SDA (ESP32 default GPIO 21)

#define rearme_PIN 12                          // GPIO12 pino para rearme do quadro
//#define CANAL_DAC0 25                        // Definição do canal DAC GPIO 25 - Saida DAC_01 da KC868-A6 
//#define CANAL_DAC1 26                        // Definição do canal DAC GPIO 26 - Saida DAC_02 da KC868-A6

Adafruit_SSD1306 display(128, 64, &Wire, -1);

//-------------------------------------  NTP Server time ----------------------------------------------
const char* ntpServer = "br.pool.ntp.org";      // "pool.ntp.org"; "a.st1.ntp.br";
const long  gmtOffset_sec = -10800;             // -14400; Fuso horário em segundos (-03h = -10800 seg)
const int   daylightOffset_sec = 0;             // ajuste em segundos do horario de verão

//int flagSetRTC = 0;                           // usada para enviar para a rotina de ajuste automatico do RTC
char* ntpTime;
char* RTC_Time;
//-----------------------------------------------------------------------------------------------------

int currentSec;
int currentMin;
int currentHour;
int currentDay;
int currentMonth;
int currentYear;
int currentSecDAY;
String currentWday    = "-1";

// ******************************************** //
//            Controle do Silo 1
// ******************************************** //
// string de modo "start" de operação para mostrar no display
String StrModo_S1     = "------";               
String old_StrModo_S1 = "old";
// variaves para controle de horário de liga/desliga
int hora_PGM_ON1;
int min_PGM_ON1;
int sec_PGM_ON1;
int HoraLiga1;
int HoraDESliga1;
int hora_PGM_OFF1;
int min_PGM_OFF1;
int sec_PGM_OFF1;
int HoraOn_PGMMem1;
int HoraOff_PGMMem1;
int HoraLigaPGM1;
int HoraDESLigaPGM1;
String DiaSemPGM1;
int WdayON1;
// vinculo com app
int forcaLiga1 = 0;
int forcaDESLiga1 = 0;
// uso geral
int varModoOper1;
int cicloON_1 = 0;                // usado na rotina de acionamentos por agendamento
int cicloOFF_1 = 0; 
int timer_Motor1 = 0;             // contador de um em um segundo chamado pelo na rotina Main2
bool statusMotor1;                // motor do Silo 1 esta em ON (true) ou OFF (false)
bool oldStatusMotor1;             // usado para comparar e gravar na memória o estado do motor VAMOS USAR ?!
int setUmidade1;                  // busca valor da memória - Setup de umidade para controle em modo Automatico(2)
unsigned int tempoAtivacao1 = 60; // é o tempo de espera em segundos após o comandos de ligar os motores 

// ******************************************** //
//            Controle do Silo 2
// ******************************************** //
// string de modo "start" de operação para mostrar no display
String StrModo_S2     = "------";               
String old_StrModo_S2 = "old";
// variaves para controle de horário de liga/desliga
int hora_PGM_ON2;
int min_PGM_ON2;
int sec_PGM_ON2;
int HoraLiga2;
int HoraDESliga2;
int hora_PGM_OFF2;
int min_PGM_OFF2;
int sec_PGM_OFF2;
int HoraOn_PGMMem2;
int HoraOff_PGMMem2;
int HoraLigaPGM2;
int HoraDESLigaPGM2;
String DiaSemPGM2;
int WdayON2;
// vinculo com app
int forcaLiga2 = 0;
int forcaDESLiga2 = 0;
// uso geral
int varModoOper2;
int cicloON_2 = 0;                // usado na rotina de acionamentos por agendamento
int cicloOFF_2 = 0; 
int timer_Motor2 = 0;             // contador de um em um segundo chamado pelo na rotina Main2
bool statusMotor2;                // motor do Silo 1 esta em ON (true) ou OFF (false)
bool oldStatusMotor2;             // usado para comparar e gravar na memória o estado do motor VAMOS USAR ?!
int setUmidade2;                  // busca valor da memória - Setup de umidade para controle em modo Automatico(2)
unsigned int tempoAtivacao2 = 120; // é o tempo de espera em segundos após o comandos de ligar os motores 

// ******************************************** //
//            Controle do Silo 3
// ******************************************** //
// string de modo "start" de operação para mostrar no display
String StrModo_S3     = "------";               
String old_StrModo_S3 = "old";
// variaves para controle de horário de liga/desliga
int hora_PGM_ON3;
int min_PGM_ON3;
int sec_PGM_ON3;
int HoraLiga3;
int HoraDESliga3;
int hora_PGM_OFF3;
int min_PGM_OFF3;
int sec_PGM_OFF3;
int HoraOn_PGMMem3;
int HoraOff_PGMMem3;
int HoraLigaPGM3;
int HoraDESLigaPGM3;
String DiaSemPGM3;
int WdayON3;
// vinculo com app
int forcaLiga3 = 0;
int forcaDESLiga3 = 0;
// uso geral
int varModoOper3;
int cicloON_3 = 0;                // usado na rotina de acionamentos por agendamento
int cicloOFF_3 = 0; 
int timer_Motor3 = 0;             // contador de um em um segundo chamado pelo na rotina Main2
bool statusMotor3;                // motor do Silo 1 esta em ON (true) ou OFF (false)
bool oldStatusMotor3;             // usado para comparar e gravar na memória o estado do motor VAMOS USAR ?!
int setUmidade3;                  // busca valor da memória - Setup de umidade para controle em modo Automatico(2)
unsigned int tempoAtivacao3 = 180; // é o tempo de espera em segundos após o comandos de ligar os motores 

// ******************************************** //

// ----------------------------------- SETUP Watchdog e ResetReason ----------------------------------- 
#include "soc/rtc_wdt.h"
#define WDT_TIMEOUT   120000               // WDT miliseconds (max 120000 segundo manual Espressif)
//#define Heartbeat_PIN 13                 // monitoração de "batimentos" para o watchdog de hardware

//Converte esp_reset_reason para string
const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN RESET";
    case ESP_RST_POWERON:   return "POWER ON RESET";        //Power on or RST pin toggled
    case ESP_RST_EXT:       return "EXTERN PIN RESET";      //External pin - not applicable for ESP32
    case ESP_RST_SW:        return "SOFTWARE REBOOT";       //esp_restart()
    case ESP_RST_PANIC:     return "CRASH RESET";           //Exception/panic
    case ESP_RST_INT_WDT:   return "INTERRUPT WATCHDOG";    //Interrupt watchdog (software or hardware)
    case ESP_RST_TASK_WDT:  return "TASK WATCHDOG";         //Task watchdog
    case ESP_RST_WDT:       return "RTC WATCHDOG";          //Other watchdog (RTC)
    case ESP_RST_DEEPSLEEP: return "SLEEP RESET";           //Reset after exiting deep sleep mode
    case ESP_RST_BROWNOUT:  return "BROWNOUT RESET";        //Brownout reset (software or hardware)
    case ESP_RST_SDIO:      return "RESET OVER SDIO";       //Reset over SDIO
    default:                return "";
  }
}

void ResetReason(void) {
  esp_reset_reason_t r = esp_reset_reason();
  //if (r == ESP_RST_POWERON) {delay(100);}                 // se habilitar espera o monitor serial inicializar
  Serial.printf("Reset reason:     %i - %s\r\n\r\n", r, resetReasonName(r));
  // se reiniciar por POWER ON RESET vai iniciar o app em operacao no modo Manual
  if (r == 1){
    varModoOper1 = 0;
    varModoOper2 = 0;
    varModoOper3 = 0;
    preferences.begin  ("my-app", false);                       // inicia 
    preferences.putUInt("varModoOper1", varModoOper1);          // grava na NVS
    preferences.putUInt("varModoOper2", varModoOper2);
    preferences.putUInt("varModoOper3", varModoOper3);
    preferences.end();
    }
}
// -----------------------------------  Fim Watchdog e ResetReason ----------------------------------- 

#include <ModbusMaster.h>
ModbusMaster ExtSensor;           // Sensor externo 4x1 (antigo node2) 
//ModbusMaster ExtSensorCWT;           // Sensor externo CWT

// sensor MODBUS 4x1 - Slave ID 32
// sensor MODBUS CWT - Slave ID 01
int TempExt    = 0;    int UmiExt = 0; 
//int PresaoExt  = 0;    int LuxExt = 0;
 

// ------ protótipo de funções ------
//void heartBeat(void);
void timerStart(void);
void setRTC(void);
void sendLogReset(void);
void ComandoOutput(void);
void timerButtonAPP(void);
void getDataHora(void);
void MODBUS_Sensor(void);
// ------         FIM          ------

void Main2(){
  unsigned long tempo_start = millis();      // usado no final para imprimir o tempo de execução dessa rotina

  rtc_wdt_feed();                             // a cada "loop executado" reseta o contador do watchdog
    // ------ testa o estado das entradas ------
  Wire.requestFrom(0x22, 1);                  // requisita do endereço 0x22      
    unsigned char inputPCF = Wire.read();     // carrega o valor lido em inputPCF, variavel de 8 bits
    Wire.endTransmission();                   // transfere o valor para as variaveis de sinalizacao e controle
       statusMotor1 = inputPCF & (1<<0);      // se bit 0 = 0 statusMotor1 = 0 (false) - Estado do motor
    bool modoOper_1 = inputPCF & (1<<1);      // se bit 1 = 0 modoOper_1 = 0 (false) - Modo:local ou app
       statusMotor2 = inputPCF & (1<<2);      // se bit 0 = 0 statusMotor1 = 0 (false) - Estado do motor
    bool modoOper_2 = inputPCF & (1<<3);      // se bit 1 = 0 modoOper_1 = 0 (false) - Modo:local ou app
       statusMotor3 = inputPCF & (1<<4);      // se bit 0 = 0 statusMotor1 = 0 (false) - Estado do motor
    bool modoOper_3 = inputPCF & (1<<5);      // se bit 1 = 0 modoOper_1 = 0 (false) - Modo:local ou app

  DateTime now = RTC.now();
  currentSec   = now.second();
  currentMin   = now.minute();
  currentHour  = now.hour();
  currentDay   = now.day();
  currentMonth = now.month();
  currentYear  = now.year();
  int RTCWday  = now.dayOfTheWeek();          // Day of week do RTC: 0 (Domingo) até 6 (Sábado)
                                              //              Blynk: 1 (Segunda) até 7 (Domingo)
  if (RTCWday == 0) {currentWday = 7;}        // compatibiliza os dias da semana com a string do Blynk 
      else {currentWday = RTCWday;}

  currentSecDAY = ((currentHour * 3600) + (currentMin * 60) + currentSec);   // calcula o Segundo do dia atual
  //Serial.printf("Segundos do dia atual: %u\n", currentSecDAY);
  // Serial.println("Up time: " + uptime_formatter::getUptime());
  //Blynk.virtualWrite(V10, uptime_formatter::getUptime());                    // envia ao Blynk a informação
  Serial.println("-------------------------------------------------------------");
  Serial.print("");

  char RTC_Time[64];                         //Cria uma string formatada da estrutura "timeinfo"
  sprintf(RTC_Time, "%02d.%02d.%04d  -  %02d:%02d:%02d", currentDay, currentMonth, currentYear, currentHour, currentMin, currentSec);
  Serial.print("Data/hora do sistema:   ");
  Serial.println(RTC_Time);
  Blynk.virtualWrite(V46, RTC_Time);          // envia ao Blynk a informação de data, hora e minuto do RTC
  
	Serial.print("Inputs BIN:             ");
	Serial.println(inputPCF, BIN);              // mostra no terminal o estado das entradas em binário

  long rssi = WiFi.RSSI();
  Serial.print("RF Signal Level:        ");
  Serial.println(rssi);                       // Escreve o indicador de nível de sinal Wi-Fi
  //Blynk.virtualWrite(V54, rssi);              // Envia ao Blynk informação RF Signal Level

  Serial.print("SETUP de Umidade Silo 1: ");
  Serial.println(setUmidade1);                      
  Blynk.virtualWrite(V38, setUmidade1);

  Serial.print("SETUP de Umidade Silo 2: ");
  Serial.println(setUmidade2);                   
  Blynk.virtualWrite(V66, setUmidade2);

  Serial.print("SETUP de Umidade Silo 3: ");
  Serial.println(setUmidade3);                 
  Blynk.virtualWrite(V86, setUmidade3);       // Envia ao Blynk informação do setup da umidade  

  // mostra no display:  rssi
  display.clearDisplay();
  display.setTextSize(1); 
  display.setCursor(0, 57);                   // coluna, linha 68, 57 sem rssi (44,57);

  display.print("U:");
  display.print(UmiExt);        // umidade externa
  display.print('%');
  display.print(" ");
  display.print(rssi);
  display.print(" ");

/*     Desabilitado o desenho das 4 barras indicadoras de nivel de RF
//                  (coluna, linha, largura, altura, cor) 
  if (rssi > -55 & rssi < -3) { 
    display.fillRect( 40, 25, 4, 17,WHITE);      // sinal muito bom
    display.fillRect( 33, 29, 4, 13,WHITE);
    display.fillRect( 26, 33, 4,  9,WHITE);
    display.fillRect( 19, 37, 4,  5,WHITE);
  } else if (rssi < -55 & rssi > -70) {
    display.drawRect( 40, 25, 4, 17,WHITE);      // sinal bom
    display.fillRect( 33, 29, 4, 13,WHITE);
    display.fillRect( 26, 33, 4,  9,WHITE);
    display.fillRect( 19, 37, 4,  5,WHITE);
  } else if (rssi < -70 & rssi > -78) {
    display.drawRect( 40, 25, 4, 17,WHITE);      // sinal razoável
    display.drawRect( 33, 29, 4, 13,WHITE);
    display.fillRect( 26, 33, 4,  9,WHITE);
    display.fillRect( 19, 37, 4,  5,WHITE);
  } else if (rssi < -78 & rssi > -82) {
    display.drawRect( 40, 25, 4, 17,WHITE);      // sinal baixo
    display.drawRect( 33, 29, 4, 13,WHITE);
    display.drawRect( 26, 33, 4,  9,WHITE);
    display.fillRect( 19, 37, 4,  5,WHITE);
  } else {
    display.drawRect( 40, 25, 4, 17,WHITE);      // sem sinal
    display.drawRect( 33, 29, 4, 13,WHITE);
    display.drawRect( 26, 33, 4,  9,WHITE);
    display.drawRect( 19, 37, 4,  5,WHITE);
  }
  */
  // mostra State Blynk
  display.print(StrStateBlynk);
  Serial.print("Status Blynk Service:   "); Serial.println(StrStateBlynk);  

  display.setCursor(0,22);                     // coluna, linha 
  display.print("Silo 1:");
  display.print(setUmidade1);                  // umidade ajustada para comando automático
  display.print("  T:");
  display.print(timer_Motor1);    
  display.print(" ");
  display.println(StrModo_S1);

  display.setCursor(0,32);  
  display.print("Silo 2:");
  display.print(setUmidade2);        
  display.print("  T:");
  display.print(timer_Motor2);    
  display.print(" ");
  display.println(StrModo_S2);

  display.setCursor(0,42);  
  display.print("Silo 3:");
  display.print(setUmidade3);   
  display.print("  T:");
  display.print(timer_Motor3);    
  display.print(" ");
  display.println(StrModo_S3);

  // mostra hora:minutos:segundos no display
  display.setCursor(15,0);                 //(32,1);
  display.setTextSize(2);
  if(currentHour < 10){
    display.print(' ');
    display.print(currentHour);
    } else {display.print(currentHour, DEC);}
  display.print(":");
  if(currentMin < 10){
    display.print('0');
    display.print(currentMin);
    } else {display.print(currentMin);}
  display.print(":");
  if(currentSec < 10){
    display.print('0');
    display.print(currentSec);
    } else {display.print(currentSec);}
  display.display();            // mostra na tela

// *************************************************** //
//            Loop de verificações do Silo 1
// *************************************************** //

  // ------    Verifica Modo de Operação - Local ou Via celular - Remoto ou Agenda   ------
  Serial.println("===============  SILO 1 ===============");
  Serial.print("Operação em modo:      ");
  if (modoOper_1 == false){                  // Lê IN-2 > || B11111101
      Serial.println("Via aplicativo");
      //Blynk.virtualWrite(V48, 0);          // Envia ao Blynk informação - Modo Remoto
      Serial.print("Comando de saída via : ");
      StrModo_S1="MANUAL"; // validar depois de ler da memoria
      switch (varModoOper1){
          case 0: Serial.println("MANUAL APP");      Blynk.virtualWrite(V49, 0); StrModo_S1="MANUAL"; break;  
          case 1: Serial.println("Agendamento APP"); Blynk.virtualWrite(V49, 1); StrModo_S1="AGENDA"; break; 
          case 2: Serial.println("AUTOMATICO  APP"); Blynk.virtualWrite(V49, 2); StrModo_S1="AUTO"  ; break;
          }
      } else {
      StrModo_S1="LOCAL";
      Serial.println(StrModo_S1);               
      //Blynk.virtualWrite(V48, 1);          // Envia ao Blynk informação - Modo Local
      Blynk.virtualWrite(V49, 3);            // sinalização no app inetrruptor segmentado
      timer_Motor1 = 1;                      // faz o led no app ficar apagado conforme estado do motor
      }
  // se mudou o modo de operacao envia ao Blynk
  if (StrModo_S1 != old_StrModo_S1){
    Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " Silo 1 -",StrModo_S1);
    old_StrModo_S1 = StrModo_S1;}
  // ------   Verifica Estado de Operação do Motor - ON ou OFF    ------
  Serial.print("Estado do motor:       ");
  if (statusMotor1){                              // vem do IN-1
      Serial.println("DESLIGADO");              
      Blynk.virtualWrite(V44, 0);                 // Envia ao Blynk informação - Motor OFF
      cicloON_1 = 0;                              // habilita os pulsos de on motor
      //Blynk.virtualWrite(V43, "MOTOR DESLIGADO");
      } else {
        Serial.println("LIGADO");
        //Blynk.setProperty(V44, "color", "#EB4E45"); // verde "#00FF6E"           
        Blynk.virtualWrite(V44, 1);               // Envia ao Blynk informação - Motor ON
        cicloOFF_1 = 0;                           // habilita os pulsos de off motor
        //Blynk.virtualWrite(V43, "MOTOR LIGADO");
      }
  //  se o estado de funcionamento do motor mudou grava na memória
  if (statusMotor1 != oldStatusMotor1){
      timer_Motor1 = 0;
      preferences.begin  ("my-app", false); 
      preferences.putBool("MemMotorState1", statusMotor1);   // true  = motor off
      preferences.end();                                     // false = motor on
      oldStatusMotor1 = statusMotor1;

      if (statusMotor1){                              // vem do IN-1
        Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 1 - Motor Desligado");
        } else {
          Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 1 - Motor Ligado");
        }
      //Serial.println("Estado do motor alterou e foi gravado na memória...");
      //delay (5000);
      }

// *************************************************** //
//            Loop de verificações do Silo 2
// *************************************************** //

  // ------    Verifica Modo de Operação - Local ou Via celular - Remoto ou Agenda   ------
  Serial.println("===============  SILO 2 ===============");
  Serial.print("Operação em modo:      ");
  if (modoOper_2 == false){                  // Lê IN-4
      Serial.println("Via aplicativo");
      //Blynk.virtualWrite(V68, 0);          // Envia ao Blynk informação - Modo Remoto
      Serial.print("Comando de saída via : ");
      StrModo_S2="MANUAL"; // validar depois de ler da memoria
      switch (varModoOper2){
          case 0: Serial.println("MANUAL APP");      Blynk.virtualWrite(V60, 0); StrModo_S2="MANUAL"; break;  
          case 1: Serial.println("Agendamento APP"); Blynk.virtualWrite(V60, 1); StrModo_S2="AGENDA"; break; 
          case 2: Serial.println("AUTOMATICO  APP"); Blynk.virtualWrite(V60, 2); StrModo_S2="AUTO"  ; break;
          } 
      } else {
      StrModo_S2="LOCAL";
      Serial.println(StrModo_S2);               
      //Blynk.virtualWrite(V68, 1);          // Envia ao Blynk informação - Modo Local
      Blynk.virtualWrite(V60, 3);            // sinalização no app interruptor segmentado
      timer_Motor2 = 1;
      }
  // se mudou o modo de operacao envia ao Blynk
  if (StrModo_S2 != old_StrModo_S2){
    Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " Silo 2 -",StrModo_S2);
    old_StrModo_S2 = StrModo_S2;}
  // ------   Verifica Estado de Operação do Motor - ON ou OFF    ------
  Serial.print("Estado do motor:       ");
  if (statusMotor2){                              // vem do IN-3
      Serial.println("DESLIGADO");              
      Blynk.virtualWrite(V72, 0);                 // Envia ao Blynk informação - Motor OFF
      cicloON_2 = 0;                              // habilita os pulsos de on motor
      //Blynk.virtualWrite(V64, "MOTOR DESLIGADO");
      } else {
        Serial.println("LIGADO");               
        Blynk.virtualWrite(V72, 1);               // Envia ao Blynk informação - Motor ON
        cicloOFF_2 = 0;                           // habilita os pulsos de off motor
        //Blynk.virtualWrite(V64, "MOTOR LIGADO");
      }
  //  se o estado de funcionamento do motor mudou grava na memória
  if (statusMotor2 != oldStatusMotor2){
      timer_Motor2 = 0;
      preferences.begin  ("my-app", false); 
      preferences.putBool("MemMotorState2", statusMotor2);   // true  = motor off
      preferences.end();                                    // false = motor on
      oldStatusMotor2 = statusMotor2;

      if (statusMotor2){                              // vem do IN-3
        Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 2 - Motor Desligado");
        } else {
          Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 2 - Motor Ligado");
        }
      //Serial.println("Estado do motor alterou e foi gravado na memória...");
      //delay (5000);
      }

// *************************************************** //
//            Loop de verificações do Silo 3
// *************************************************** //

  // ------    Verifica Modo de Operação - Local ou Via celular - Remoto ou Agenda   ------
  Serial.println("===============  SILO 3 ===============");
  Serial.print("Operação em modo:      ");
  if (modoOper_3 == false){                  // Lê IN-6 
      Serial.println("Via aplicativo");
      //Blynk.virtualWrite(V88, 0);          // Envia ao Blynk informação - Modo Remoto
      Serial.print("Comando de saída via : ");
      StrModo_S3="MANUAL"; // validar depois de ler da memoria
      switch (varModoOper3){
          case 0: Serial.println("MANUAL APP");      Blynk.virtualWrite(V80, 0); StrModo_S3="MANUAL"; break;  
          case 1: Serial.println("Agendamento APP"); Blynk.virtualWrite(V80, 1); StrModo_S3="AGENDA"; break; 
          case 2: Serial.println("AUTOMATICO  APP"); Blynk.virtualWrite(V80, 2); StrModo_S3="AUTO"  ; break;
          } 
      } else {
      StrModo_S3="LOCAL";
      Serial.println(StrModo_S3);               
      //Blynk.virtualWrite(V88, 1);          // Envia ao Blynk informação - Modo Local
      Blynk.virtualWrite(V80, 3);            // sinalização no app interruptor segmentado
      timer_Motor3 = 1;
      }
  // se mudou o modo de operacao envia ao Blynk
  if (StrModo_S3 != old_StrModo_S3){
    Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " Silo 3 -",StrModo_S3);
    old_StrModo_S3 = StrModo_S3;}
  // ------   Verifica Estado de Operação do Motor - ON ou OFF    ------
  Serial.print("Estado do motor:       ");
  if (statusMotor3){                              // vem do IN-5
      Serial.println("DESLIGADO");              
      Blynk.virtualWrite(V92, 0);                 // Envia ao Blynk informação - Motor OFF
      cicloON_3 = 0;                              // habilita os pulsos de on motor
      //Blynk.virtualWrite(V84, "MOTOR DESLIGADO");
      } else {
        Serial.println("LIGADO");               
        Blynk.virtualWrite(V92, 1);               // Envia ao Blynk informação - Motor ON
        cicloOFF_3 = 0;                           // habilita os pulsos de off motor
        //Blynk.virtualWrite(V84, "MOTOR LIGADO");
      }
  //  se o estado de funcionamento do motor mudou grava na memória
  if (statusMotor3 != oldStatusMotor3){
      timer_Motor3 = 0;
      preferences.begin  ("my-app", false); 
      preferences.putBool("MemMotorState3", statusMotor3);   // true  = motor off
      preferences.end();                                    // false = motor on
      oldStatusMotor3 = statusMotor3;

      if (statusMotor3){                              // vem do IN-5
        Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 3 - Motor Desligado");
        } else {
          Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 3 - Motor Ligado");
        }
      //Serial.println("Estado do motor alterou e foi gravado na memória...");
      //delay (5000);
      }

// *************************************************** //

  int rstTimer = WDT_TIMEOUT/1000;                          // 120000 miliseconds de WDT_TIMEOUT = 2 minutos
  while (BotaoRESET == 1) {                                 // BotaoRESET = 1 força e entrada na rotina do watchdog
        Serial.println("Botão de RESET no APP pressionado...");
        Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " App força RESET em ", rstTimer);
          if ((WDT_TIMEOUT/1000) - rstTimer == 2) {
            Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " Calibrando o relógio interno...");
            setRTC();
            //ESP.restart();
            }
        rstTimer --;
        delay(900);   // faz um loop aqui... ATENÇÃO não alimenta o software watchdog de propósito!
        }

                     // até aqui usa aproximadamente 1300ms
  sendLogReset();    // depois de executar uma vez manda o log 
  getDataHora();     // usa aproximadamente  10ms
  ComandoOutput();   // usa aproximadamente 500ms
  MODBUS_Sensor();   // usa aproximadamente  50ms

  //Envio dos dados que serão armazenados. Tenta enviar por até 60 segundos.
  if (currentMin == minAtualiza && currentSec > 0){
    Blynk.beginGroup();                             // https://docs.blynk.io/en/blynk-library-firmware-api/virtual-pins
      Blynk.virtualWrite(V0, UmiExt);             
      Blynk.virtualWrite(V1, TempExt);
    Blynk.endGroup();
    //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " LOG de Temp. e Umidade enviado");
    minAtualiza = minAtualiza + 15;                 // soma 15 a cada 15 minutos
    if (minAtualiza > 50) {minAtualiza = 0;}        // minAtualiza usado para enviar a cada 15 minutos
  }

  Serial.printf("Período (ms) do Main2: %u\n", (millis() - tempo_start)); // cálculo do tempu utilizado até aqui

}

void MODBUS_Sensor(){

  // SENSOR EXTERNO CWT = ExtSensorCWT
  // SENSOR EXTERNO 4x1 = ExtSensor4x1
  uint8_t result2 = ExtSensor.readHoldingRegisters( 0, 2 );
  Serial.print("\n"); Serial.println("Sensor Externo ");
  Serial.print("Error = "); Serial.println( result2 );   // 0: ok, 226: falha
  Blynk.virtualWrite(V50, result2);                      // envia Status do Sensor MODBUS (0 = OK, 226 = falha)
  
  if (result2 == ExtSensor.ku8MBSuccess){
    UmiExt    = (ExtSensor.getResponseBuffer(0)/10);
    TempExt   = (ExtSensor.getResponseBuffer(1)/10);

    Blynk.virtualWrite(V51, UmiExt);                     // Envia ao Blynk a informação do SENSOR EXTERNO 4x1
    Blynk.virtualWrite(V52, TempExt);
    Serial.print("Umidade Ext.:     "); Serial.print(UmiExt); Serial.println(" %");
    Serial.print("Temperatura Ext.: "); Serial.print(TempExt); Serial.println(" C");
  } Serial.print("\n"); delay(5);

}

void sendLogReset(){
  // envia razao do reset para o monitor serial e servidor
  if ((servicoIoTState==4) && (sendBlynk)){
    Serial.print("               BLYNK:  RODANDO COM SUCESSO!");
    esp_reset_reason_t r = esp_reset_reason();
    Serial.printf("\r\nReset reason %i - %s\r\n", r, resetReasonName(r));
    Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, "",resetReasonName(r), " ",counterRST);
    Blynk.virtualWrite(V53, counterRST);                        // envia para tela do app
    Blynk.syncVirtual (V40, V69, V89);                          // sincroniza datastream de agendamentos
    delay(500);
    // se reiniciar por (1) POWER ON RESET
    if (r == 1){
      Blynk.virtualWrite(V2, 255);                        // envia 1 para sinalização push no app via automação
      delay(3000);
      Blynk.virtualWrite(V2, 0);                          // envia 0 para "des_sinalizar" na automação
      //Blynk.logEvent("falha_de_energia", String("Teste - Falha de Energia!"));
      Blynk.logEvent("falha_de_energia");                 // registra o evento falha_de_energia no servidor
      }
    sendBlynk = false;                                    // garante que só envia uma vez essas informações
  }
}

void NTPserverTime(){          // habilitar no SETUP se quiser depurar na serial o horário recebido da internet
  struct tm timeinfo;
     //  "%u, %d.%m.%Y %H:%M:%S"  = 1, 17.08.2021 14:33:33
     //  %u = dia da semana em decimal, range 1 a 7, Segunda = 1.
     //  https://www.ibm.com/docs/en/z-workload-scheduler/9.5.0?topic=troubleshooting-date-time-format-reference-strftime
  Serial.print("NTP: ");
  if(!getLocalTime(&timeinfo)){
    Serial.println(" falha ao sincronizar NTP!");
    } else { 
      Serial.print(&timeinfo, "%u, %d.%m.%Y %H:%M:%S");
      Serial.println(" hora recebida da internet.");}
}

void setRTC(){                        // comandos de set no DS1307
struct tm timeinfo;
     //  "%u, %d.%m.%Y %H:%M:%S"  = 1, 17.08.2021 14:33:33
     //  %u = dia da semana em decimal, range 1 a 7, Segunda = 1.
     //  https://www.ibm.com/docs/en/z-workload-scheduler/9.5.0?topic=troubleshooting-date-time-format-reference-strftime
Serial.print("NTP: ");
if(!getLocalTime(&timeinfo)){
    Serial.println(" falha ao sincronizar NTP!");
    for (int i=9; i > 0; i= i-1) {
       delay(960);                    // 9 segundos no laço for considerando tempo de escrita no display
         // mostra no display
       display.clearDisplay(); 
       display.setCursor(25, 15);
       display.setTextSize(2);
       display.print("FALHA");
       display.setCursor(10, 35);      // coluna, linha 
       display.print("INTERNET");
       display.setCursor(110, 5); 
       display.print(i);
       display.display();             // mostra na tela
       }
    } else { 
  Serial.print(&timeinfo, "%u, %d.%m.%Y %H:%M:%S");
  Serial.println(" hora recebida da internet.");

  time_t now;                         // this is the epoch
  tm tm;                              // the structure tm holds time information in a more convient way
  time(&now);                         // read the current time
  //delay(50); 
  localtime_r(&now, &tm);             // update the structure tm with the current time
  //delay(50); 
  int ye = tm.tm_year + 1900;
  int mo = tm.tm_mon + 1;
  int da = tm.tm_mday;
  int ho = tm.tm_hour;
  int mi = tm.tm_min;
  int se = tm.tm_sec +1;

  //delay(50); 
  RTC.adjust(DateTime( ye , mo , da , ho , mi , se )); 
  delay(50); 

  preferences.begin  ("my-app", false);              // inicia 
  preferences.putUInt("counterRST", 0);              // grava em Preferences/My-app/counterRST, counterRST
  counterRST = preferences.getUInt("counterRST", 0); // Le da NVS
  preferences.end();
  delay(50);
  Serial.println("A data e hora foram recalibradas no relógio interno, e o contador de RESETs foi zerado!");
  Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " Relógio calibrado");
  flagSetRTC = 0;

  for (int i=20; i > 0; i= i-1) {      // i= tempo que fica mostrando no display
       delay(960);                     // ex.: adianta 13 segundos por dia
         // mostra no display          // espera por pelo menos 14 segundos aqui para não calibrar mais de uma vez
       display.clearDisplay();         // quanto maior o tempo mais garantido que não vai repetir a calibração
       display.setCursor(25, 25);
       display.setTextSize(2);
       display.print("CAL..");
       display.setCursor(96, 25);     // coluna, linha 
       display.print(i);
       display.display();             // mostra na tela
       }
  }
}

void failMSG(String HW_status) {
  // mensagem de falha nos disposotivos de entrada e/ou saida (RTC, CIs PFC...)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C (128x64)
  display.clearDisplay();                    // limpa o buffer
  display.display();

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(45,7);                   // coluna, linha
  display.println("R&M");
  display.setTextSize(1);
  display.setCursor(42,27);
  display.println("Company");
  display.setCursor(15, 50);
  display.print("STOP: ");
  display.println(HW_status); 
  display.display();                         // mostra na tela a falha de hardware ocorrida
  while (1);                                 // PARA A EXECUCAO POR SEGURANCA e irá reiniciar pelo watchdog

}

// -------------------------------------- Recebe do APP as variaveis --------------------------------------

//int BotaoRESET;                               // BotaoRESET = Virtual do APP
BLYNK_WRITE(V39){
  BotaoRESET = param.asInt();                   // função de RESET que força a atuação do watchtdog
  }

int rearme;                                     // Comando remoto para rearme do quadro
BLYNK_WRITE(V55){                               // quando aperta = 1; solta = 0
      rearme = param.asInt();
    }

// *************************************************** //
//            Interações APP e Silo 1
// *************************************************** //
// recebe a string DiaSemPGM1. Ex.: [1,2,3] = seg, ter, quarta
BLYNK_WRITE(V40){                           
  HoraLigaPGM1    = param[0].asInt();
  HoraDESLigaPGM1 = param[1].asInt();
  DiaSemPGM1      = param[3].asStr();

//  converte a informação de HorLigaPGM em formato (H:M:S)
  hora_PGM_ON1 = int(HoraLigaPGM1 / 3600);
  min_PGM_ON1  = int((HoraLigaPGM1 - (hora_PGM_ON1 * 3600)) / 60);
  sec_PGM_ON1  = int(HoraLigaPGM1 % 60);
//  grava na memória = HORA LIGA PGM
  preferences.begin  ("my-app", false);
  preferences.putUInt("hora_PGM_ON1", hora_PGM_ON1);
  preferences.end(); 

  preferences.begin  ("my-app", false);
  preferences.putUInt("min_PGM_ON1", min_PGM_ON1);
  preferences.end();

  preferences.begin  ("my-app", false);
  preferences.putUInt("sec_PGM_ON1", sec_PGM_ON1);
  preferences.end();

 // converte a informação de HoraDESLigaPGM1 em formato (H:M:S)
  hora_PGM_OFF1 = int(HoraDESLigaPGM1 / 3600);
  min_PGM_OFF1  = int((HoraDESLigaPGM1 - (hora_PGM_OFF1 * 3600)) / 60);
  sec_PGM_OFF1  = int(HoraDESLigaPGM1 % 60);
 // grava na memória = HORA DESLIGA PGM
  preferences.begin  ("my-app", false);
  preferences.putUInt("hora_PGM_OFF1", hora_PGM_OFF1); 
  preferences.end();

  preferences.begin  ("my-app", false);
  preferences.putUInt("min_PGM_OFF1", min_PGM_OFF1);
  preferences.end();

  preferences.begin  ("my-app", false);
  preferences.putUInt("sec_PGM_OFF1", sec_PGM_OFF1);
  preferences.end();

//  grava na memória a informção DiaSemPGM1
  preferences.begin  ("my-app", false);            // inicia 
  preferences.putString("DiaSemPGM1", DiaSemPGM1); // grava na NVS
  preferences.end();
  }

int remoteLiga1;                                   // comando remoto força Liga = Virtual 41 do APP
BLYNK_WRITE(V41){                                  // quando aperta = 1; solta = 0
    remoteLiga1 = param.asInt();
    Serial.println("Silo 1 - Recebido comando Liga do APP...");
    }

int remoteDESLiga1;                                // Comando remoto força Desliga = Virtual 42 do APP
BLYNK_WRITE(V42){
    remoteDESLiga1 = param.asInt(); 
    Serial.println("Silo 1 - Recebido comando Desliga Silo 1 do APP...");
    }

int remoteManual1;                                 // Comando remoto de Manual(0)
BLYNK_WRITE(V27){
    remoteManual1 = param.asInt();}

int remoteAgendamento1;                            // Comando remoto de Agendamento(1)
BLYNK_WRITE(V28){
    remoteAgendamento1 = param.asInt();}

int remoteAuto1;                                   // Comando remoto de Automatico(2) conforme umidade
BLYNK_WRITE(V29){
    remoteAuto1 = param.asInt();}

BLYNK_WRITE(V8){                                   // sobe valor de setUmidade1
    setUmidade1 = setUmidade1 +1;
    preferences.begin  ("my-app", false);               // inicia 
    preferences.putUInt("setUmidade1", setUmidade1 );   // grava na NVS
    preferences.end();
  }

BLYNK_WRITE(V9){                                   // desce valor de setUmidade1
    setUmidade1 = setUmidade1 -1;
    preferences.begin  ("my-app", false);               // inicia 
    preferences.putUInt("setUmidade1", setUmidade1 );   // grava na NVS
    preferences.end();
  }

// *************************************************** //
//            Interações APP e Silo 2
// *************************************************** //
// recebe a string DiaSemPGM1. Ex.: [1,2,3] = seg, ter, quarta
BLYNK_WRITE(V69){                           
  HoraLigaPGM2    = param[0].asInt();
  HoraDESLigaPGM2 = param[1].asInt();
  DiaSemPGM2      = param[3].asStr();

//  converte a informação de HorLigaPGM em formato (H:M:S)
  hora_PGM_ON2 = int(HoraLigaPGM2 / 3600);
  min_PGM_ON2  = int((HoraLigaPGM2 - (hora_PGM_ON2 * 3600)) / 60);
  sec_PGM_ON2  = int(HoraLigaPGM2 % 60);
//  grava na memória = HORA LIGA PGM
  preferences.begin  ("my-app", false);
  preferences.putUInt("hora_PGM_ON2", hora_PGM_ON2);
  preferences.end(); 

  preferences.begin  ("my-app", false);
  preferences.putUInt("min_PGM_ON2", min_PGM_ON2);
  preferences.end();

  preferences.begin  ("my-app", false);
  preferences.putUInt("sec_PGM_ON2", sec_PGM_ON2);
  preferences.end();

 // converte a informação de HoraDESLigaPGM1 em formato (H:M:S)
  hora_PGM_OFF2 = int(HoraDESLigaPGM2 / 3600);
  min_PGM_OFF2  = int((HoraDESLigaPGM2 - (hora_PGM_OFF2 * 3600)) / 60);
  sec_PGM_OFF2  = int(HoraDESLigaPGM2 % 60);
 // grava na memória = HORA DESLIGA PGM
  preferences.begin  ("my-app", false);
  preferences.putUInt("hora_PGM_OFF2", hora_PGM_OFF2); 
  preferences.end();

  preferences.begin  ("my-app", false);
  preferences.putUInt("min_PGM_OFF2", min_PGM_OFF2);
  preferences.end();

  preferences.begin  ("my-app", false);
  preferences.putUInt("sec_PGM_OFF2", sec_PGM_OFF2);
  preferences.end();

//  grava na memória a informção DiaSemPGM1
  preferences.begin  ("my-app", false);            // inicia 
  preferences.putString("DiaSemPGM2", DiaSemPGM2); // grava na NVS
  preferences.end();
  }

int remoteLiga2;                                   // comando remoto força Liga = Virtual 41 do APP
BLYNK_WRITE(V70){                                  // quando aperta = 1; solta = 0
    remoteLiga2 = param.asInt();
    Serial.println("Silo 2 - Recebido comando Liga do APP...");
    }

int remoteDESLiga2;                                // Comando remoto força Desliga = Virtual 42 do APP
BLYNK_WRITE(V71){
    remoteDESLiga2 = param.asInt(); 
    Serial.println("Silo 2 - Recebido comando Desliga do APP...");
    }

int remoteManual2;                                 // Comando remoto de Manual(0)
BLYNK_WRITE(V61){
    remoteManual2 = param.asInt();}

int remoteAgendamento2;                            // Comando remoto de Agendamento(1)
BLYNK_WRITE(V62){
    remoteAgendamento2 = param.asInt();}

int remoteAuto2;                                   // Comando remoto de Automatico(2) conforme umidade
BLYNK_WRITE(V63){
    remoteAuto2 = param.asInt();}

BLYNK_WRITE(V67){                                   // sobe valor de setUmidade2
    setUmidade2 = setUmidade2 +1;
    preferences.begin  ("my-app", false);               // inicia 
    preferences.putUInt("setUmidade2", setUmidade2 );   // grava na NVS
    preferences.end();
  }

BLYNK_WRITE(V65){                                   // desce valor de setUmidade2
    setUmidade2 = setUmidade2 -1;
    preferences.begin  ("my-app", false);               // inicia 
    preferences.putUInt("setUmidade2", setUmidade2 );   // grava na NVS
    preferences.end();
  }


// *************************************************** //
//            Interações APP e Silo 3
// *************************************************** //
// recebe a string DiaSemPGM1. Ex.: [1,2,3] = seg, ter, quarta
BLYNK_WRITE(V89){                           
  HoraLigaPGM3    = param[0].asInt();
  HoraDESLigaPGM3 = param[1].asInt();
  DiaSemPGM3      = param[3].asStr();

//  converte a informação de HorLigaPGM em formato (H:M:S)
  hora_PGM_ON3 = int(HoraLigaPGM3 / 3600);
  min_PGM_ON3  = int((HoraLigaPGM3 - (hora_PGM_ON3 * 3600)) / 60);
  sec_PGM_ON3  = int(HoraLigaPGM3 % 60);
//  grava na memória = HORA LIGA PGM
  preferences.begin  ("my-app", false);
  preferences.putUInt("hora_PGM_ON3", hora_PGM_ON3);
  preferences.end(); 

  preferences.begin  ("my-app", false);
  preferences.putUInt("min_PGM_ON3", min_PGM_ON3);
  preferences.end();

  preferences.begin  ("my-app", false);
  preferences.putUInt("sec_PGM_ON3", sec_PGM_ON3);
  preferences.end();

 // converte a informação de HoraDESLigaPGM1 em formato (H:M:S)
  hora_PGM_OFF3 = int(HoraDESLigaPGM3 / 3600);
  min_PGM_OFF3  = int((HoraDESLigaPGM3 - (hora_PGM_OFF3 * 3600)) / 60);
  sec_PGM_OFF3  = int(HoraDESLigaPGM3 % 60);
 // grava na memória = HORA DESLIGA PGM
  preferences.begin  ("my-app", false);
  preferences.putUInt("hora_PGM_OFF3", hora_PGM_OFF3); 
  preferences.end();

  preferences.begin  ("my-app", false);
  preferences.putUInt("min_PGM_OFF3", min_PGM_OFF3);
  preferences.end();

  preferences.begin  ("my-app", false);
  preferences.putUInt("sec_PGM_OFF3", sec_PGM_OFF3);
  preferences.end();

//  grava na memória a informção DiaSemPGM3
  preferences.begin  ("my-app", false);            // inicia 
  preferences.putString("DiaSemPGM3", DiaSemPGM3); // grava na NVS
  preferences.end();
  }

int remoteLiga3;                                   // comando remoto força Liga = Virtual 41 do APP
BLYNK_WRITE(V90){                                  // quando aperta = 1; solta = 0
    remoteLiga3 = param.asInt();
    Serial.println("Silo 3 - Recebido comando Liga do APP...");
    }

int remoteDESLiga3;                                // Comando remoto força Desliga = Virtual 42 do APP
BLYNK_WRITE(V91){
    remoteDESLiga3 = param.asInt(); 
    Serial.println("Silo 3 - Recebido comando Desliga do APP...");
    }

int remoteManual3;                                 // Comando remoto de Manual(0)
BLYNK_WRITE(V81){
    remoteManual3 = param.asInt();}

int remoteAgendamento3;                            // Comando remoto de Agendamento(1)
BLYNK_WRITE(V82){
    remoteAgendamento3 = param.asInt();}

int remoteAuto3;                                   // Comando remoto de Automatico(2) conforme umidade
BLYNK_WRITE(V83){
    remoteAuto3 = param.asInt();}

BLYNK_WRITE(V87){                                   // sobe valor de setUmidade3
    setUmidade3 = setUmidade3 +1;
    preferences.begin  ("my-app", false);               // inicia 
    preferences.putUInt("setUmidade3", setUmidade3 );   // grava na NVS
    preferences.end();
  }

BLYNK_WRITE(V85){                                   // desce valor de setUmidade3
    setUmidade3 = setUmidade3 -1;
    preferences.begin  ("my-app", false);               // inicia 
    preferences.putUInt("setUmidade3", setUmidade3 );   // grava na NVS
    preferences.end();
  }

// *************************************************** //

void setup(){
  // configuração do watchdog
  rtc_wdt_protect_off();               //Disable RTC WDT write protection
  rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_RTC);
  rtc_wdt_set_time (RTC_WDT_STAGE0, WDT_TIMEOUT);
  rtc_wdt_enable();                    //Start the RTC WDT timer
  rtc_wdt_protect_on();                //Enable RTC WDT write protection

  pinMode(Heartbeat_PIN,OUTPUT);       // vinculo para setup do pino na biblioteca HeartBeat.h
  pinMode(rearme_PIN,OUTPUT);
  //dacWrite(CANAL_DAC0,   0);         // Saida DAC_01 da KC868-A6 - start ligado
  //dacWrite(CANAL_DAC1, 255);         // Saida DAC_02 da KC868-A6 - start desligado
  // DAC vai pulsar a cada reset ou atualização de software devido ao hardware (CI LM258)
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);        // ajusta os pinos do I2C

  Wire.beginTransmission(0x24);        // inicia a transmissao para o CI PCF8574 de saida endereço 0x24
  Wire.write(output_PLC);              // coloca todas saidas em HIGH, reles desligados (0b11111111)
  int errorCode_OUTPUT = Wire.endTransmission();
  //Serial.println(errorCode_OUTPUT);  // se errorCode_OUTPUT = 0, o dispositivo foi encontrado no endereço
  if (errorCode_OUTPUT != 0)  {
    Serial.println("Falha no dispositivo de saida PCF8574!");
    failMSG("FALHA OUPUT");
    } else { Serial.println("Saida I2C PCF8574     OK");}
  delay(50);

  Wire.beginTransmission(0x22);         // inicia a transmissao para o CI PCF8574 de entrada endereço 0x22
  int errorCode_INPUT = Wire.endTransmission();
  //Serial.println(errorCode_INPUT);    // se errorCode_OUTPUT = 0, o dispositivo foi encontrado no endereço
  if (errorCode_INPUT != 0)  {
    Serial.println("Falha no dispositivo de entrada PCF8574!");
    failMSG("FALHA INPUT");
    } else {Serial.println("Entrada I2C PCF8574   OK");}
  delay(50);

  // ------   Inicia e cria espaco na memoria NVS - namespace:my-app    ------
  preferences.begin("my-app", false);                  // Note: Namespace name is limited to 15 chars.
  //preferences.clear();                               // remove all preferences under the opened namespace
  //preferences.remove("counterRST");                  // or remove the counter key only
  counterRST = preferences.getUInt("counterRST", 0);   // lê da NVS, se counterRST nao existir retorna o valor 0
  counterRST++;                                        // incrementa a cada reset
  preferences.putUInt("counterRST", counterRST);       // grava o novo valor de counterRST em Preferences/My-app/counterRST

  setUmidade1 = preferences.getUInt("setUmidade1", 0);   // lê da NVS o valor de setUmidade1 para uso no modo Automático
  setUmidade2 = preferences.getUInt("setUmidade2", 0); 
  setUmidade3 = preferences.getUInt("setUmidade3", 0); 

  varModoOper1 = preferences.getUInt("varModoOper1", 0); // lê da NVS o último modo de operação
  varModoOper2 = preferences.getUInt("varModoOper2", 0);
  varModoOper3 = preferences.getUInt("varModoOper3", 0);

  bool MemMotorState1 = preferences.getBool("MemMotorState1", true); // lê da NVS MemMotorState (true = motor deslidado)
  bool MemMotorState2 = preferences.getBool("MemMotorState2", true);
  bool MemMotorState3 = preferences.getBool("MemMotorState3", true);
  
  preferences.end();                                   // finaliza o uso da memória NVS  
  delay(50);

  ResetReason();                                       // imprime na serial razao do ultimo reset e
                                                       // se iniciar por (1) POWER ON RESET coloca o app em modo manual
  Serial.printf("Quantidade de RESETs: %u\n", counterRST);

  Serial.printf("Umidade ajustada para o acionamento do Silo 1: %u\n", setUmidade1);
  Serial.printf("Umidade ajustada para o acionamento do Silo 2: %u\n", setUmidade2);
  Serial.printf("Umidade ajustada para o acionamento do Silo 3: %u\n", setUmidade3);

  Serial.printf("Último estado do motor do Silo 1 (1 = desligado): %u\n", MemMotorState1);
  Serial.printf("Último estado do motor do Silo 2 (1 = desligado): %u\n", MemMotorState2);
  Serial.printf("Último estado do motor do Silo 3 (1 = desligado): %u\n", MemMotorState3);

      //  Desabilidada a ativacao do motor pelo efeito memória (MemMotorState)
      /*
          if (MemMotorState == false){                               // verificar se necessario forcaLiga1 == 0 ?!?!            
          //digitalWrite(OUT1_PIN, HIGH);
          output_PLC = output_PLC & 0b1111110;    // faz AND, apenas bit 0 = 0
          Wire.beginTransmission(0x24);           // escreve na saida do PLC
          Wire.write(output_PLC);                 // 0 = rele ligado, 1 = desligado
          Wire.endTransmission();
          forcaLiga1 = 0;
          Serial.printf("Motor ativado pela memória! \n");
          delay (1000);
          //digitalWrite(OUT1_PIN, LOW);
          output_PLC = output_PLC | 0b00000001;   // faz OU, apenas bit 0 = 1
          Wire.beginTransmission(0x24);           // escreve na saida do PLC
          Wire.write(output_PLC);                 // 0 = rele ligado, 1 = desligado
          Wire.endTransmission();
          cicloOFF_1 = 0;
          } else if (MemMotorState = true){ 
                     //digitalWrite(OUT2_PIN, HIGH);
                     output_PLC = output_PLC & 0b1111101;    // faz AND, apenas bit 1 = 0
                     Wire.beginTransmission(0x24);           // escreve na saida do PLC
                     Wire.write(output_PLC);                 // 0 = rele ligado, 1 = desligado
                     Wire.endTransmission();
                     forcaDESLiga1 = 0;
                     Serial.printf("Motor desativado pela memória! \n");
                     delay (1000);
                     //digitalWrite(OUT2_PIN, LOW);
                     output_PLC = output_PLC | 0b00000010;   // faz OU, apenas bit 1 = 1
                     Wire.beginTransmission(0x24);           // escreve na saida do PLC
                     Wire.write(output_PLC);                 // 0 = rele ligado, 1 = desligado
                     Wire.endTransmission();
                     cicloON_1 = 0;                    
                     }
        */

  

  Serial1.begin(9600, SERIAL_8N1, 14, 27);   // porta RS-485 do hardware KC-868-A6
  ExtSensor.begin  (Slave_ID_EXT, Serial1);         // Slave address: 20H  Sensor 4x1
  //ExtSensorCWT.begin  ( 1, Serial1);         // Slave address: 01H  Sensor CWT-TH04

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C (128x64)
  timerStart();                              // rotina de logomarca temporizada (minimo 10 segundos para RTC start)
  /*
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C (128x64)
  display.clearDisplay();                    // limpa o buffer
  display.setTextSize(2);                    // tamanho do texto
  display.setTextColor(SSD1306_WHITE);       // cor do texto
  display.setCursor(5, 25);                  // coluna, linha 
  display.println("Iniciando!");             // informação
  display.display();                         // mostra na tela
  delay(10000);                               // delay necessário para o RTC DS1307 inicializar
  */

  if (! RTC.begin()) {
     Serial.println("Não foi possível encontrar o RTC!");
     failMSG("FALHA RTC");
     } else { delay(100);                     // Serial.println("RTC identificado com sucesso!")
              DateTime now = RTC.now();
              char currentTime[64];
              //Cria uma string formatada da estrutura "timeinfo"
              sprintf(currentTime, "%02d.%02d.%04d - %02d:%02d:%02d", now.day(), now.month(), now.year(), now.hour(), now.minute(),now.second());
              Serial.println("Leitura do RTC DS1307 OK");
              Serial.print  ("Leitura da Data e Hora do sistema: ");
              Serial.println(currentTime);
              delay(100);
              }

  //rtc.adjust(DateTime(__DATE__, __TIME__));                 // seta o RTC com os parametros da data e hora da compilação
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);   // inicia e busca as infos de data e hora (NTP)

  edgentTimer.setInterval(1000L, Main2);                      // rotina se repete a cada XXXXL (milisegundos)
  //edgentTimer.setInterval(5000L, NTPserverTime);
  edgentTimer.setInterval(5000L, timerButtonAPP);             // timer para receber os comandos do APP
  BlynkEdgent.begin();
  delay(100);
  Serial.println("--------------------------- SETUP Concluido ---------------------------");
}

// ----------------------------- temporizador para ler os botoes do app --------------------------------------
void timerButtonAPP(){                        // timer de botao pressionado no app

  if (rearme == 1){                           // recebido do V55 - botao no app de reset do quadro
      // coloca o app em modo AUTO
      varModoOper1 = 2;
      varModoOper2 = 2;
      varModoOper3 = 2;
      preferences.begin  ("my-app", false);                       // inicia 
      preferences.putUInt("varModoOper1", varModoOper1);          // grava na NVS
      preferences.putUInt("varModoOper2", varModoOper2);
      preferences.putUInt("varModoOper3", varModoOper3);
      preferences.end();
      delay(50);
      Serial.println("Recebido o comando rearme do quadro!");
      Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " Comando REARME do Quadro Geral");
      
      digitalWrite(rearme_PIN, HIGH);
      delay(2000);
      } else {
        digitalWrite(rearme_PIN, LOW);
        }      

 // *************************************************************************** //
 //            Interações do temporizador de botoes do APP e Silo 1
 // *************************************************************************** //
  if (remoteLiga1 == 1 && varModoOper1 == 0){                     // se apertar o botao ON e estiver em Manual
      forcaDESLiga1 = 0;
      forcaLiga1 = remoteLiga1;                                   // permite o pulso no relé ON
      Serial.println("Silo 1 - Comando remoto forca LIGAR executado!");
      }

  if (remoteDESLiga1 == 1 && varModoOper1 == 0){                  // se apertar o botao OFF e estiver em Manual
      forcaLiga1 = 0;
      forcaDESLiga1 = remoteDESLiga1;                             // permite o pulso no relé OFF
      Serial.println("Silo 1 - Comando remoto forca DESLIGAR excutado!");
      }

  if (remoteManual1 == 1){                                        // seta varModoOper1 em modo Manual (0)
      forcaLiga1    = 0;                                          // se for para o modo Manual habilita botões
      forcaDESLiga1 = 0;
      timer_Motor1  = 0;                                          // zera o temporizador
      varModoOper1  = 0; cicloON_1 = 0; cicloOFF_1 = 0; 
      preferences.begin  ("my-app", false);                       // inicia 
      preferences.putUInt("varModoOper1", varModoOper1);          // grava na NVS
      preferences.end();
      Serial.println("Silo 1 - Recebido set de comando para modo Remoto do APP...");
      }

  if (remoteAgendamento1 == 1){                                    // seta varModoOper1 em modo Agendamento (1)
      varModoOper1 = 1; cicloON_1 = 0; cicloOFF_1 = 0; 
      preferences.begin  ("my-app", false);                        // inicia 
      preferences.putUInt("varModoOper1", varModoOper1 );          // grava na NVS
      preferences.end();
      Serial.println("Silo 1 - Recebido set de comando para modo Agendamento do APP...");
      }

  if (remoteAuto1 == 1){                                           // seta varModoOper1 em modo Automático (2)
      varModoOper1 = 2; cicloON_1 = 0; cicloOFF_1 = 0; 
      preferences.begin  ("my-app", false);                        // inicia 
      preferences.putUInt("varModoOper1", varModoOper1 );          // grava na NVS
      preferences.end();
      Serial.println("Silo 1 - Recebido set de comando para modo Automatico do APP...");
      }

 // *************************************************************************** //
 //            Interações do temporizador de botoes do APP e Silo 2
 // *************************************************************************** //
 if (remoteLiga2 == 1 && varModoOper2 == 0){                     // se apertar o botao ON e estiver em Manual
     forcaDESLiga2 = 0;
     forcaLiga2 = remoteLiga2;
     Serial.println("Silo 2 - Comando remoto forca LIGAR executado!");
     }

 if (remoteDESLiga2 == 1 && varModoOper2 == 0){                  // se apertar o botao OFF e estiver em Manual
     forcaLiga2 = 0; 
     forcaDESLiga2 = remoteDESLiga2;
     Serial.println("Silo 2 - Comando remoto forca DESLIGAR excutado!");
     }

 if (remoteManual2 == 1){                                        // seta varModoOper2 em modo Manual (0)
     forcaLiga2    = 0;                                          // se for para o modo Manual habilita botões
     forcaDESLiga2 = 0;
     timer_Motor2  = 0;  
     varModoOper2  = 0; cicloON_2 = 0; cicloOFF_2 = 0; 
     preferences.begin  ("my-app", false);                       // inicia 
     preferences.putUInt("varModoOper2", varModoOper2);          // grava na NVS
     preferences.end();
     Serial.println("Silo 2 - Recebido set de comando para modo Remoto do APP...");
     }

if (remoteAgendamento2 == 1){                                    // seta varModoOper2 em modo Agendamento (1)
  varModoOper2 = 1; cicloON_2 = 0; cicloOFF_2 = 0; 
  preferences.begin  ("my-app", false);                          // inicia 
  preferences.putUInt("varModoOper2", varModoOper2 );            // grava na NVS
  preferences.end();
  Serial.println("Silo 2 - Recebido set de comando para modo Agendamento do APP...");
  }

if (remoteAuto2 == 1){                                           // seta varModoOper2 em modo Automático (2)
  varModoOper2 = 2; cicloON_2 = 0; cicloOFF_2 = 0; 
  preferences.begin  ("my-app", false);                          // inicia 
  preferences.putUInt("varModoOper2", varModoOper2 );            // grava na NVS
  preferences.end();
  Serial.println("Silo 2 - Recebido set de comando para modo Automatico do APP...");
  }

 // *************************************************************************** //
 //            Interações do temporizador de botoes do APP e Silo 3
 // *************************************************************************** //
 if (remoteLiga3 == 1 && varModoOper3 == 0){                     // se apertar o botao ON e estiver em Manual
     forcaDESLiga3 = 0;
     forcaLiga3 = remoteLiga3;
     Serial.println("Silo 3 - Comando remoto forca LIGAR executado!");
     }

 if (remoteDESLiga3 == 1 && varModoOper3 == 0){                  // se apertar o botao OFF e estiver em Manual
     forcaLiga3 = 0;
     forcaDESLiga3 = remoteDESLiga3;
     Serial.println("Silo 3 - Comando remoto forca DESLIGAR excutado!");
     }

 if (remoteManual3 == 1){                                        // seta varModoOper3 em modo Manual (0)
     forcaLiga3    = 0;                                          // se for para o modo Manual habilita botões
     forcaDESLiga3 = 0;
     timer_Motor3  = 0;  
     varModoOper3  = 0; cicloON_3 = 0; cicloOFF_3 = 0; 
     preferences.begin  ("my-app", false);                       // inicia 
     preferences.putUInt("varModoOper3", varModoOper3);          // grava na NVS
     preferences.end();
     Serial.println("Silo 3 - Recebido set de comando para modo Remoto do APP...");
     }

if (remoteAgendamento3 == 1){                                    // seta varModoOper3 em modo Agendamento (1)
  varModoOper3 = 1; cicloON_3 = 0; cicloOFF_3 = 0; 
  preferences.begin  ("my-app", false);                          // inicia 
  preferences.putUInt("varModoOper3", varModoOper3 );            // grava na NVS
  preferences.end();
  Serial.println("Silo 3 - Recebido set de comando para modo Agendamento do APP...");
  }

if (remoteAuto3 == 1){                                           // seta varModoOper3 em modo Automático (2)
  varModoOper3 = 2; cicloON_3 = 0; cicloOFF_3 = 0; 
  preferences.begin  ("my-app", false);                          // inicia 
  preferences.putUInt("varModoOper3", varModoOper3 );            // grava na NVS
  preferences.end();
  Serial.println("Silo 3 - Recebido set de comando para modo Automatico do APP...");
  }

 // *************************************************************************** //
}

void getDataHora(){
  // *************************************************************************** //
  //                    Leitura de dados da memória Silo 1                       //
  // *************************************************************************** //
  ////Serial.println("===============  SILO 1 ===============");
  ////Serial.println("Leitura da memória - Horário Start: ");
  preferences.begin("my-app", false);
  hora_PGM_ON1 = preferences.getUInt("hora_PGM_ON1", 0);              // lê da NVS
  min_PGM_ON1 = preferences.getUInt("min_PGM_ON1", 0);
  sec_PGM_ON1 = preferences.getUInt("sec_PGM_ON1", 0);
  preferences.end();

  ////Serial.print(hora_PGM_ON1);
  ////Serial.printf(": %u", min_PGM_ON1);
  ////Serial.printf(": %u", sec_PGM_ON1);
  
  HoraOn_PGMMem1 = ((hora_PGM_ON1*3600) + (min_PGM_ON1 * 60) + sec_PGM_ON1);   // calcula o segundo programado
  ////Serial.printf("    %u\n", HoraOn_PGMMem1);
  
  ////Serial.print("Leitura da memória - Horário Stop:  ");
  preferences.begin("my-app", false);
  hora_PGM_OFF1 = preferences.getUInt("hora_PGM_OFF1", 0);          // lê da NVS
  min_PGM_OFF1 = preferences.getUInt("min_PGM_OFF1", 0);
  sec_PGM_OFF1 = preferences.getUInt("sec_PGM_OFF1", 0);
  preferences.end();
  ////Serial.print(hora_PGM_OFF1);
  ////Serial.printf(": %u", min_PGM_OFF1);
  ////Serial.printf(": %u", sec_PGM_OFF1);

  HoraOff_PGMMem1 = ((hora_PGM_OFF1*3600) + (min_PGM_OFF1 * 60) + sec_PGM_OFF1);
  ////Serial.printf("    %u\n", HoraOff_PGMMem1);

  ////Serial.print("Leitura da memória - Manual(0), Agendamento(1), Auto(2): ");
  preferences.begin("my-app", false);
  varModoOper1 = preferences.getUInt("varModoOper1", 0);        // lê da NVS
  preferences.end();
  ////Serial.println(varModoOper1);

  ////Serial.print("Leitura da memória - Agenda ativada nos dias...   ");
  preferences.begin("my-app", false);
  DiaSemPGM1 = preferences.getString("DiaSemPGM1","NULL");      // lê da NVS
  preferences.end();
  ////Serial.println(DiaSemPGM1);

  // verifica se na string DiaSemPGM1 (originado no Blynk) tem o currentDay (recebido do RTC) 
  WdayON1 = DiaSemPGM1.indexOf(currentWday);  //minhaString.indexOf(string ou char a ser procurada) retorno -1 = nao liga

  ////Serial.print("Weekday atual:         ");   
  ////Serial.println(currentWday);
  
  ////Serial.print("WdayON1 >= 0 atua hoje: ");   
  ////Serial.println(WdayON1);
  //Serial.print(" ===========================");

  // *************************************************************************** //
  //                    Leitura de dados da memória Silo 2                       //
  // *************************************************************************** //
  ////Serial.println("===============  SILO 2 ===============");
  ////Serial.println("Leitura da memória - Horário Start: ");
  preferences.begin("my-app", false);
  hora_PGM_ON2 = preferences.getUInt("hora_PGM_ON2", 0);              // lê da NVS
  min_PGM_ON2  = preferences.getUInt("min_PGM_ON2", 0);
  sec_PGM_ON2  = preferences.getUInt("sec_PGM_ON2", 0);
  preferences.end();

  ////Serial.print(hora_PGM_ON2);
  ////Serial.printf(": %u", min_PGM_ON2);
  ////Serial.printf(": %u", sec_PGM_ON2);
  
  HoraOn_PGMMem2 = ((hora_PGM_ON2*3600) + (min_PGM_ON2 * 60) + sec_PGM_ON2);   // calcula o segundo programado
  ////Serial.printf("    %u\n", HoraOn_PGMMem2);
  
  ////Serial.print("Leitura da memória - Horário Stop:  ");
  preferences.begin("my-app", false);
  hora_PGM_OFF2 = preferences.getUInt("hora_PGM_OFF2", 0);          // lê da NVS
  min_PGM_OFF2  = preferences.getUInt("min_PGM_OFF2", 0);
  sec_PGM_OFF2  = preferences.getUInt("sec_PGM_OFF2", 0);
  preferences.end();
  ////Serial.print(hora_PGM_OFF2);
  ////Serial.printf(": %u", min_PGM_OFF2);
  ////Serial.printf(": %u", sec_PGM_OFF2);

  HoraOff_PGMMem2 = ((hora_PGM_OFF2*3600) + (min_PGM_OFF2 * 60) + sec_PGM_OFF2);
  ////Serial.printf("    %u\n", HoraOff_PGMMem2);

  ////Serial.print("Leitura da memória - Manual(0), Agendamento(1), Auto(2): ");
  preferences.begin("my-app", false);
  varModoOper2 = preferences.getUInt("varModoOper2", 0);        // lê da NVS
  preferences.end();
  ////Serial.println(varModoOper2);

  ////Serial.print("Leitura da memória - Agenda ativada nos dias...   ");
  preferences.begin("my-app", false);
  DiaSemPGM2 = preferences.getString("DiaSemPGM2","NULL");      // lê da NVS
  preferences.end();
  ////Serial.println(DiaSemPGM2);

  // verifica se na string DiaSemPGM1 (originado no Blynk) tem o currentDay (recebido do RTC) 
  WdayON2 = DiaSemPGM2.indexOf(currentWday);  //minhaString.indexOf(string ou char a ser procurada) retorno -1 = nao liga

  ////Serial.print("Weekday atual:         ");   
  ////Serial.println(currentWday);
  
  ////Serial.print("WdayON2 >= 0 atua hoje: ");   
  ////Serial.println(WdayON2);
  //Serial.print(" ===========================");


  // *************************************************************************** //
  //                    Leitura de dados da memória Silo 3                       //
  // *************************************************************************** //
  ////Serial.println("===============  SILO 3 ===============");
  ////Serial.println("Leitura da memória - Horário Start: ");
  preferences.begin("my-app", false);
  hora_PGM_ON3 = preferences.getUInt("hora_PGM_ON3", 0);              // lê da NVS
  min_PGM_ON3  = preferences.getUInt("min_PGM_ON3", 0);
  sec_PGM_ON3  = preferences.getUInt("sec_PGM_ON3", 0);
  preferences.end();

  ////Serial.print(hora_PGM_ON3);
  ////Serial.printf(": %u", min_PGM_ON3);
  ////Serial.printf(": %u", sec_PGM_ON3);
  
  HoraOn_PGMMem3 = ((hora_PGM_ON3*3600) + (min_PGM_ON3 * 60) + sec_PGM_ON3);   // calcula o segundo programado
  ////Serial.printf("    %u\n", HoraOn_PGMMem3);
  
  ////Serial.print("Leitura da memória - Horário Stop:  ");
  preferences.begin("my-app", false);
  hora_PGM_OFF3 = preferences.getUInt("hora_PGM_OFF3", 0);          // lê da NVS
  min_PGM_OFF3  = preferences.getUInt("min_PGM_OFF3", 0);
  sec_PGM_OFF3  = preferences.getUInt("sec_PGM_OFF3", 0);
  preferences.end();
  ////Serial.print(hora_PGM_OFF3);
  ////Serial.printf(": %u", min_PGM_OFF3);
  ////Serial.printf(": %u", sec_PGM_OFF3);

  HoraOff_PGMMem3 = ((hora_PGM_OFF3*3600) + (min_PGM_OFF3 * 60) + sec_PGM_OFF3);
  ////Serial.printf("    %u\n", HoraOff_PGMMem3);

  ////Serial.print("Leitura da memória - Manual(0), Agendamento(1), Auto(2): ");
  preferences.begin("my-app", false);
  varModoOper3 = preferences.getUInt("varModoOper3", 0);        // lê da NVS
  preferences.end();
  ////Serial.println(varModoOper3);

  ////Serial.print("Leitura da memória - Agenda ativada nos dias...   ");
  preferences.begin("my-app", false);
  DiaSemPGM3 = preferences.getString("DiaSemPGM3","NULL");      // lê da NVS
  preferences.end();
  ////Serial.println(DiaSemPGM3);

  // verifica se na string DiaSemPGM1 (originado no Blynk) tem o currentDay (recebido do RTC) 
  WdayON3 = DiaSemPGM3.indexOf(currentWday);  //minhaString.indexOf(string ou char a ser procurada) retorno -1 = nao liga

  ////Serial.print("Weekday atual:         ");   
  ////Serial.println(currentWday);
  
  ////Serial.print("WdayON3 >= 0 atua hoje: ");   
  ////Serial.println(WdayON3);
  //Serial.print(" ===========================");
}

void ComandoOutput() {
//Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " -", timer_Motor1," Timer Motores");
//Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " -", varModoOper2," MODO");

  // *************************************************************************** //
  //                          Controle de saidas Silo 1                          //
  // *************************************************************************** //
// testa o modo de operação do sistema
  if (timer_Motor1 > tempoAtivacao1){
   timer_Motor1 = 0;                       // envia uma vez a informação ao Blynk
   //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 1 - Falha status do Motor");
  }

  switch (varModoOper1){
   case 0:                                               // 0 = esta no modo remoto - controle manual no app 
    if (forcaLiga1==1 && statusMotor1){                 // se o botao do app foi apertado e motor esta desligado
       //Blynk.setProperty(V44, "color", "#EB4E45"); // laranja  "#F7CB46"
       Blynk.virtualWrite(V44, (timer_Motor1 % 2 == 0)); // pisca led V44 status do motor, terminar em número par!
       //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " -", timer_Motor1," Temporizando em Manual");
       
       timer_Motor1 = timer_Motor1+1;                    // contador incrementa a cada execução usado com timer dos motores
       if (timer_Motor1 >= tempoAtivacao1){              // se passou XX segundos
          //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 1 - Cmd ligar pelo app");
          // Liga Silo 1 = pulsa a saida 1
          output_PLC = output_PLC & 0b11111110;          // faz AND, onde apenas bits 0 = 0
          Wire.beginTransmission(0x24);                  // escreve na saida do PLC
          Wire.write(output_PLC);                        // 0 = rele ligado, 1 = desligado
          Wire.endTransmission();
          delay(300);
          output_PLC = output_PLC | 0b00000001;          // faz OU, apenas bit 0 = 1
          Wire.beginTransmission(0x24);                  // escreve na saida do PLC
          Wire.write(output_PLC);                        // 0 = rele ligado, 1 = desligado
          Wire.endTransmission();
     
          timer_Motor1  = 0;
          forcaLiga1    = 0;                                // força variavel a ficar em zero
          forcaDESLiga1 = 0;
          cicloOFF_1    = 0;
       }  
    }

    if (forcaDESLiga1 == 1){ 
          //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 1 - Cmd desligar pelo app"); 
          // Desliga Silo 1 = pulsa a saida 2
          output_PLC = output_PLC & 0b11111101;         // faz AND, onde apenas bits 0 = 0
          Wire.beginTransmission(0x24);                 // escreve na saida do PLC
          Wire.write(output_PLC);                       // 0 = rele ligado, 1 = desligado
          Wire.endTransmission();
          delay(300);
          output_PLC = output_PLC | 0b00000010;         // faz OU, apenas bit 0 = 1
          Wire.beginTransmission(0x24);                 // escreve na saida do PLC
          Wire.write(output_PLC);                       // 0 = rele ligado, 1 = desligado
          Wire.endTransmission();

          timer_Motor1  = 0;
          forcaLiga1    = 0;
          forcaDESLiga1 = 0;
          cicloON_1     = 0;                   
     }
   break;

   case 1:                                              // 1 = esta no modo de controle via agendamento
    if ((currentSecDAY >= HoraOn_PGMMem1) && (currentSecDAY <= HoraOff_PGMMem1) && WdayON1 >= 0) {
      for (cicloOFF_1; cicloOFF_1 < 1; cicloOFF_1++) {   
          //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 1 - Cmd desligar por Agenda");                
          // Desliga Silo 1 = pulsa a saida 2
          output_PLC = output_PLC & 0b11111101;         // faz AND, onde apenas bits 0 = 0
          Wire.beginTransmission(0x24);                 // escreve na saida do PLC
          Wire.write(output_PLC);                       // 0 = rele ligado, 1 = desligado
          Wire.endTransmission();
          delay(300);
          output_PLC = output_PLC | 0b00000010;         // faz OU, apenas bit 0 = 1
          Wire.beginTransmission(0x24);                 // escreve na saida do PLC
          Wire.write(output_PLC);                       // 0 = rele ligado, 1 = desligado
          Wire.endTransmission();

          Serial.print (cicloOFF_1); Serial.println(" - CMD DESLIGAR VIA AGENDA !!!"); 
          timer_Motor1  = 0;
          forcaLiga1    = 0;
          forcaDESLiga1 = 0;
          cicloON_1     = 0;                                // habilita executar uma vez o comando 
          }

    } else if (statusMotor1){                        // enquanto o motor estiver desligado executa
             Blynk.virtualWrite(V44, (timer_Motor1 % 2 == 0));     // pisca led V44 status do motor, terminar em número par!
             //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " -", timer_Motor1," Temporizando em Agenda");
             
             timer_Motor1 = timer_Motor1+1;             // contador incrementa a cada execução usado com timer dos motores
             if (timer_Motor1 >= tempoAtivacao1){       // se passou XX segundos
              for (cicloON_1; cicloON_1 < 1; cicloON_1++) {
              //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 1 - Cmd ligar por Agenda");
              // Liga Silo 1 = pulsa a saida 1
              output_PLC = output_PLC & 0b11111110;     // faz AND, onde apenas bits 0 = 0
              Wire.beginTransmission(0x24);             // escreve na saida do PLC
              Wire.write(output_PLC);                   // 0 = rele ligado, 1 = desligado
              Wire.endTransmission();
              delay(300);
              output_PLC = output_PLC | 0b00000001;     // faz OU, apenas bit 0 = 1
              Wire.beginTransmission(0x24);             // escreve na saida do PLC
              Wire.write(output_PLC);                   // 0 = rele ligado, 1 = desligado
              Wire.endTransmission();

              timer_Motor1  = 0;
              forcaLiga1    = 0;
              forcaDESLiga1 = 0;
              cicloOFF_1    = 0;
              }
            }
          }
   break;
    
   case 2: 
   if (UmiExt <= setUmidade1 && statusMotor1){       // enquanto umidade menor ou igual e motor off executa
    Blynk.virtualWrite(V44, (timer_Motor1 % 2 == 0));   // pisca led V44 status do motor, terminar em número par!
    //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " -", timer_Motor1," Temporizando em AUTO");
    
    timer_Motor1 = timer_Motor1+1;                      // contador incrementa a cada execução usado com timer dos motores
    if (timer_Motor1 >= tempoAtivacao1){                // se passou XX segundos
     for (cicloON_1; cicloON_1 < 1; cicloON_1++) {
      //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 1 - Cmd ligar Auto");
       // Liga Silo 1 = pulsa a saida 1
       output_PLC = output_PLC & 0b11111110;            // faz AND, onde apenas bits 0 = 0
       Wire.beginTransmission(0x24);                    // escreve na saida do PLC
       Wire.write(output_PLC);                          // 0 = rele ligado, 1 = desligado
       Wire.endTransmission();
       delay(300);
       output_PLC = output_PLC | 0b00000001;            // faz OU, apenas bit 0 = 1
       Wire.beginTransmission(0x24);                    // escreve na saida do PLC
       Wire.write(output_PLC);                          // 0 = rele ligado, 1 = desligado
       Wire.endTransmission();

       timer_Motor1  = 0;
       forcaLiga1    = 0;
       forcaDESLiga1 = 0;
       cicloOFF_1    = 0;
      }
     }
    } else if ((UmiExt -2) > setUmidade1) {             // enquanto umidade maior executa desliga
        for (cicloOFF_1; cicloOFF_1 < 1; cicloOFF_1++) {    
        //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 1 - Cmd desligar Auto");              
        // Desliga Silo 1 = pulsa a saida 2
        output_PLC = output_PLC & 0b11111101;           // faz AND, onde apenas bits 0 = 0
        Wire.beginTransmission(0x24);                   // escreve na saida do PLC
        Wire.write(output_PLC);                         // 0 = rele ligado, 1 = desligado
        Wire.endTransmission();
        delay(300);
        output_PLC = output_PLC | 0b00000010;           // faz OU, apenas bit 0 = 1
        Wire.beginTransmission(0x24);                   // escreve na saida do PLC
        Wire.write(output_PLC);                         // 0 = rele ligado, 1 = desligado
        Wire.endTransmission();

        Serial.print (cicloOFF_1); Serial.println(" - CMD DESLIGAR VIA AUTO !!!"); 
        timer_Motor1  = 0;
        forcaLiga1    = 0;
        forcaDESLiga1 = 0;
        cicloON_1     = 0;                               // habilita executar uma vez o comando 
        }
      }
    break;
  }

  // *************************************************************************** //
  //                          Controle de saidas Silo 2                          //
  // *************************************************************************** //
// testa o modo de operação do sistema
if (timer_Motor2 > tempoAtivacao2){
  timer_Motor2 = 0;                       // envia uma vez a informação ao Blynk
  //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 2 - Falha status do Motor");
 }

 switch (varModoOper2){
  case 0:                                               // 0 = esta no modo remoto - controle manual no app 
   if (forcaLiga2==1 && statusMotor2){      // se o botao do app foi apertado
      Blynk.virtualWrite(V72, (timer_Motor2 % 2 == 0)); // pisca led V44 status do motor, terminar em número par!
      //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " -", timer_Motor1," Temporizando em Manual");
      
      timer_Motor2 = timer_Motor2+1;                    // contador incrementa a cada execução usado com timer dos motores
      if (timer_Motor2 >= tempoAtivacao2){              // se passou XX segundos
         //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 2 - Cmd ligar pelo app");
         // Liga Silo 2 = pulsa a saida 3
         output_PLC = output_PLC & 0b11111011;          // faz AND, onde apenas bits 0 = 0
         Wire.beginTransmission(0x24);                  // escreve na saida do PLC
         Wire.write(output_PLC);                        // 0 = rele ligado, 1 = desligado
         Wire.endTransmission();
         delay(300);
         output_PLC = output_PLC | 0b00000100;          // faz OU, apenas bit 0 = 1
         Wire.beginTransmission(0x24);                  // escreve na saida do PLC
         Wire.write(output_PLC);                        // 0 = rele ligado, 1 = desligado
         Wire.endTransmission();
    
         timer_Motor2  = 0;
         forcaLiga2    = 0;                             // força variavel a ficar em zero
         forcaDESLiga2 = 0;
         cicloOFF_2    = 0;
      }  
   }

   if (forcaDESLiga2 == 1){ 
         //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 2 - Cmd desligar pelo app"); 
         // Desliga Silo 2 = pulsa a saida 4
         output_PLC = output_PLC & 0b11110111;         // faz AND, onde apenas bits 0 = 0
         Wire.beginTransmission(0x24);                 // escreve na saida do PLC
         Wire.write(output_PLC);                       // 0 = rele ligado, 1 = desligado
         Wire.endTransmission();
         delay(300);
         output_PLC = output_PLC | 0b00001000;         // faz OU, apenas bit 0 = 1
         Wire.beginTransmission(0x24);                 // escreve na saida do PLC
         Wire.write(output_PLC);                       // 0 = rele ligado, 1 = desligado
         Wire.endTransmission();

         timer_Motor2  = 0;
         forcaLiga2    = 0;                            // força variavel a ficar em zero
         forcaDESLiga2 = 0;
         cicloON_2     = 0;                   
    }
  break;

  case 1:                                              // 1 = esta no modo de controle via agendamento
   if ((currentSecDAY >= HoraOn_PGMMem2) && (currentSecDAY <= HoraOff_PGMMem2) && WdayON2 >= 0) {
     for (cicloOFF_2; cicloOFF_2 < 1; cicloOFF_2++) {   
         //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 2 - Cmd desligar por Agenda");                
         // Desliga Silo 2 = pulsa a saida 4
         output_PLC = output_PLC & 0b11110111;         // faz AND, onde apenas bits 0 = 0
         Wire.beginTransmission(0x24);                 // escreve na saida do PLC
         Wire.write(output_PLC);                       // 0 = rele ligado, 1 = desligado
         Wire.endTransmission();
         delay(300);
         output_PLC = output_PLC | 0b00001000;         // faz OU, apenas bit 0 = 1
         Wire.beginTransmission(0x24);                 // escreve na saida do PLC
         Wire.write(output_PLC);                       // 0 = rele ligado, 1 = desligado
         Wire.endTransmission();

         Serial.print (cicloOFF_2); Serial.println(" - CMD DESLIGAR VIA AGENDA !!!"); 
         timer_Motor2  = 0;
         forcaLiga2    = 0;                            // força variavel a ficar em zero
         forcaDESLiga2 = 0;
         cicloON_2     = 0;                            // habilita executar uma vez o comando 
         }

   } else if (oldStatusMotor2){                        // enquanto o motor estiver desligado executa
            Blynk.virtualWrite(V72, (timer_Motor2 % 2 == 0));     // pisca led V44 status do motor, terminar em número par!
            //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " -", timer_Motor1," Temporizando em Agenda");
            
            timer_Motor2 = timer_Motor2+1;             // contador incrementa a cada execução usado com timer dos motores
            if (timer_Motor2 >= tempoAtivacao2){       // se passou XX segundos
             for (cicloON_2; cicloON_2 < 1; cicloON_2++) {
             //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 2 - Cmd ligar por Agenda");
             // Liga Silo 2 = pulsa a saida 3
             output_PLC = output_PLC & 0b11111011;     // faz AND, onde apenas bits 0 = 0
             Wire.beginTransmission(0x24);             // escreve na saida do PLC
             Wire.write(output_PLC);                   // 0 = rele ligado, 1 = desligado
             Wire.endTransmission();
             delay(300);
             output_PLC = output_PLC | 0b00000100;     // faz OU, apenas bit 0 = 1
             Wire.beginTransmission(0x24);             // escreve na saida do PLC
             Wire.write(output_PLC);                   // 0 = rele ligado, 1 = desligado
             Wire.endTransmission();

             timer_Motor2  = 0;
             forcaLiga2    = 0;                        // força variavel a ficar em zero
             forcaDESLiga2 = 0;
             cicloOFF_2    = 0;
             }
           }
         }
  break;
   
  case 2: 
  if (UmiExt <= setUmidade2 && oldStatusMotor2){       // enquanto umidade menor ou igual e motor off executa
   Blynk.virtualWrite(V72, (timer_Motor2 % 2 == 0));   // pisca led V44 status do motor, terminar em número par!
   //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " -", timer_Motor1," Temporizando em AUTO");
   
   timer_Motor2 = timer_Motor2+1;                      // contador incrementa a cada execução usado com timer dos motores
   if (timer_Motor2 >= tempoAtivacao2){                // se passou XX segundos
    for (cicloON_2; cicloON_2 < 1; cicloON_2++) {
     //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 2 - Cmd ligar Auto");
      // Liga Silo 2 = pulsa a saida 3
      output_PLC = output_PLC & 0b11111011;            // faz AND, onde apenas bits 0 = 0
      Wire.beginTransmission(0x24);                    // escreve na saida do PLC
      Wire.write(output_PLC);                          // 0 = rele ligado, 1 = desligado
      Wire.endTransmission();
      delay(300);
      output_PLC = output_PLC | 0b00000100;            // faz OU, apenas bit 0 = 1
      Wire.beginTransmission(0x24);                    // escreve na saida do PLC
      Wire.write(output_PLC);                          // 0 = rele ligado, 1 = desligado
      Wire.endTransmission();

      timer_Motor2  = 0;
      forcaLiga2    = 0;                               // força variavel a ficar em zero
      forcaDESLiga2 = 0;
      cicloOFF_2    = 0;
     }
    }
   } else if ((UmiExt -2) > setUmidade2) {             // enquanto umidade maior executa desliga
       for (cicloOFF_2; cicloOFF_2 < 1; cicloOFF_2++) {    
       //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 2 - Cmd desligar Auto");              
       // Desliga Silo 2 = pulsa a saida 4
       output_PLC = output_PLC & 0b11110111;           // faz AND, onde apenas bits 0 = 0
       Wire.beginTransmission(0x24);                   // escreve na saida do PLC
       Wire.write(output_PLC);                         // 0 = rele ligado, 1 = desligado
       Wire.endTransmission();
       delay(300);
       output_PLC = output_PLC | 0b00001000;           // faz OU, apenas bit 0 = 1
       Wire.beginTransmission(0x24);                   // escreve na saida do PLC
       Wire.write(output_PLC);                         // 0 = rele ligado, 1 = desligado
       Wire.endTransmission();

       Serial.print (cicloOFF_2); Serial.println(" - CMD DESLIGAR VIA AUTO !!!"); 
       timer_Motor2  = 0;
       forcaLiga2    = 0;                             // força variavel a ficar em zero
       forcaDESLiga2 = 0;
       cicloON_2     = 0;                             // habilita executar uma vez o comando 
       }
     }
   break;
 }

  // *************************************************************************** //
  //                          Controle de saidas Silo 3                          //
  // *************************************************************************** //
// testa o modo de operação do sistema
if (timer_Motor3 > tempoAtivacao3){
  timer_Motor3 = 0;                       // envia uma vez a informação ao Blynk
  //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 3 - Falha status do Motor");
 }

 switch (varModoOper3){
  case 0:                                               // 0 = esta no modo remoto - controle manual no app 
   if (forcaLiga3==1 && statusMotor3){      // se o botao do app foi apertado
      Blynk.virtualWrite(V92, (timer_Motor3 % 2 == 0)); // pisca led V44 status do motor, terminar em número par!
      //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " -", timer_Motor1," Temporizando em Manual");
      
      timer_Motor3 = timer_Motor3+1;                    // contador incrementa a cada execução usado com timer dos motores
      if (timer_Motor3 >= tempoAtivacao3){              // se passou XX segundos
         //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 3 - Cmd ligar pelo app");
         // Liga Silo 1 = pulsa a saida 5
         output_PLC = output_PLC & 0b11101111;          // faz AND, onde apenas bits 0 = 0
         Wire.beginTransmission(0x24);                  // escreve na saida do PLC
         Wire.write(output_PLC);                        // 0 = rele ligado, 1 = desligado
         Wire.endTransmission();
         delay(300);
         output_PLC = output_PLC | 0b00010000;          // faz OU, apenas bit 0 = 1
         Wire.beginTransmission(0x24);                  // escreve na saida do PLC
         Wire.write(output_PLC);                        // 0 = rele ligado, 1 = desligado
         Wire.endTransmission();

         timer_Motor3  = 0;
         forcaLiga3    = 0;                             // força variavel a ficar em zero
         forcaDESLiga3 = 0;
         cicloOFF_3    = 0;
      }  
   }

   if (forcaDESLiga3 == 1){ 
         //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 3 - Cmd desligar pelo app"); 
         // Desliga Silo 6 = pulsa a saida 6
         output_PLC = output_PLC & 0b11011111;         // faz AND, onde apenas bits 0 = 0
         Wire.beginTransmission(0x24);                 // escreve na saida do PLC
         Wire.write(output_PLC);                       // 0 = rele ligado, 1 = desligado
         Wire.endTransmission();
         delay(300);
         output_PLC = output_PLC | 0b00100000;         // faz OU, apenas bit 0 = 1
         Wire.beginTransmission(0x24);                 // escreve na saida do PLC
         Wire.write(output_PLC);                       // 0 = rele ligado, 1 = desligado
         Wire.endTransmission();

         timer_Motor3  = 0;
         forcaLiga3    = 0;                            // força variavel a ficar em zero
         forcaDESLiga3 = 0;
         cicloON_3     = 0;                   
    }
  break;

  case 1:                                              // 1 = esta no modo de controle via agendamento
   if ((currentSecDAY >= HoraOn_PGMMem3) && (currentSecDAY <= HoraOff_PGMMem3) && WdayON3 >= 0) {
     for (cicloOFF_3; cicloOFF_3 < 1; cicloOFF_3++) {   
         //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 3 - Cmd desligar por Agenda");                
         // Desliga Silo 3 = pulsa a saida 6
         output_PLC = output_PLC & 0b11011111;         // faz AND, onde apenas bits 0 = 0
         Wire.beginTransmission(0x24);                 // escreve na saida do PLC
         Wire.write(output_PLC);                       // 0 = rele ligado, 1 = desligado
         Wire.endTransmission();
         delay(300);
         output_PLC = output_PLC | 0b00100000;         // faz OU, apenas bit 0 = 1
         Wire.beginTransmission(0x24);                 // escreve na saida do PLC
         Wire.write(output_PLC);                       // 0 = rele ligado, 1 = desligado
         Wire.endTransmission();

         Serial.print (cicloOFF_3); Serial.println(" - CMD DESLIGAR VIA AGENDA !!!"); 
         timer_Motor3  = 0;
         forcaLiga3    = 0;                            // força variavel a ficar em zero
         forcaDESLiga3 = 0;
         cicloON_3     = 0;                            // habilita executar uma vez o comando 
         }

   } else if (oldStatusMotor3){                        // enquanto o motor estiver desligado executa
            Blynk.virtualWrite(V92, (timer_Motor3 % 2 == 0));     // pisca led V44 status do motor, terminar em número par!
            //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " -", timer_Motor1," Temporizando em Agenda");
            
            timer_Motor3 = timer_Motor3+1;             // contador incrementa a cada execução usado com timer dos motores
            if (timer_Motor3 >= tempoAtivacao3){       // se passou XX segundos
             for (cicloON_3; cicloON_3 < 1; cicloON_3++) {
             //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 3 - Cmd ligar por Agenda");
             // Liga Silo 3 = pulsa a saida 5
             output_PLC = output_PLC & 0b11101111;     // faz AND, onde apenas bits 0 = 0
             Wire.beginTransmission(0x24);             // escreve na saida do PLC
             Wire.write(output_PLC);                   // 0 = rele ligado, 1 = desligado
             Wire.endTransmission();
             delay(300);
             output_PLC = output_PLC | 0b00010000;     // faz OU, apenas bit 0 = 1
             Wire.beginTransmission(0x24);             // escreve na saida do PLC
             Wire.write(output_PLC);                   // 0 = rele ligado, 1 = desligado
             Wire.endTransmission();

             timer_Motor3  = 0;
             forcaLiga3    = 0;                        // força variavel a ficar em zero
             forcaDESLiga3 = 0;
             cicloOFF_3    = 0;
             }
           }
         }
  break;
   
  case 2: 
  if (UmiExt <= setUmidade3 && oldStatusMotor3){       // enquanto umidade menor ou igual e motor off executa
   Blynk.virtualWrite(V92, (timer_Motor3 % 2 == 0));   // pisca led V44 status do motor, terminar em número par!
   //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " -", timer_Motor1," Temporizando em AUTO");
   
   timer_Motor3 = timer_Motor3+1;                      // contador incrementa a cada execução usado com timer dos motores
   if (timer_Motor3 >= tempoAtivacao3){                // se passou XX segundos
    for (cicloON_3; cicloON_3 < 1; cicloON_3++) {
     //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 3 - Cmd ligar Auto");
      // Liga Silo 3 = pulsa a saida 5
      output_PLC = output_PLC & 0b11101111;            // faz AND, onde apenas bits 0 = 0
      Wire.beginTransmission(0x24);                    // escreve na saida do PLC
      Wire.write(output_PLC);                          // 0 = rele ligado, 1 = desligado
      Wire.endTransmission();
      delay(300);
      output_PLC = output_PLC | 0b00010000;            // faz OU, apenas bit 0 = 1
      Wire.beginTransmission(0x24);                    // escreve na saida do PLC
      Wire.write(output_PLC);                          // 0 = rele ligado, 1 = desligado
      Wire.endTransmission();

      timer_Motor3  = 0;
      forcaLiga3    = 0;                               // força variavel a ficar em zero
      forcaDESLiga3 = 0;
      cicloOFF_3    = 0;
     }
    }
   } else if ((UmiExt -2) > setUmidade3) {             // enquanto umidade maior executa desliga
       for (cicloOFF_3; cicloOFF_3 < 1; cicloOFF_3++) {    
       //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin," Silo 3 - Cmd desligar Auto");              
       // Desliga Silo 3 = pulsa a saida 6
       output_PLC = output_PLC & 0b11011111;           // faz AND, onde apenas bits 0 = 0
       Wire.beginTransmission(0x24);                   // escreve na saida do PLC
       Wire.write(output_PLC);                         // 0 = rele ligado, 1 = desligado
       Wire.endTransmission();
       delay(300);
       output_PLC = output_PLC | 0b00100000;           // faz OU, apenas bit 0 = 1
       Wire.beginTransmission(0x24);                   // escreve na saida do PLC
       Wire.write(output_PLC);                         // 0 = rele ligado, 1 = desligado
       Wire.endTransmission();

       Serial.print (cicloOFF_3); Serial.println(" - CMD DESLIGAR VIA AUTO !!!"); 
       timer_Motor3  = 0;
       forcaLiga3    = 0;                              // força variavel a ficar em zero
       forcaDESLiga3 = 0;
       cicloON_3     = 0;                              // habilita executar uma vez o comando 
       }
     }
   break;
 }
}

//int timerON = 0;            // executa uma vez timer de X segundos configurados em tempoStart a cada reinicio
void timerStart() {
  if (timerON != 1){
  Serial.println("Temporizando início do sistema...  "); 
  for (tempoStart; tempoStart > -1; tempoStart--) {
       heartBeat();     delay(900);               // +/- 1000ms considerando o tempo de escrita no display e heartbeat
       //Serial.println(tempoStart);
       display.clearDisplay();                    // limpa o buffer do display
       display.setTextSize(2);
       display.setTextColor(SSD1306_WHITE);
       display.setCursor(45,10);
       display.println("R&M");
       display.setTextSize(1);
       display.setCursor(42,30);
       display.println("Company");
       display.setCursor(70, 55);
       display.print("FW:");                      //  versao BLYNK_FIRMWARE_VERSION
       display.println(BLYNK_FIRMWARE_VERSION);   //  F1 = falha do sensor
       display.setCursor(5, 55);
       display.print("RST: "); 
       display.println(counterRST);               // quantidade de reset's lido da memória NVS

       //Mostra timer de inicialização
       display.setTextSize(2);
       display.setCursor(96, 24);    // coluna, linha 
       display.print(tempoStart);    // informação
       display.display();            // mostra na tela
       }
    timerON = 1;
  }
}

void loop() {
 heartBeat();                        // rotina na biblioteca HeartBeat.h
 //timerStart();                     // rotina de logomarca temporizada
 BlynkEdgent.run();

 // testa se o botão USER foi pressionado OU 
 // se esta na hora de ajustar e vai para rotina de set RTC, tenta por 9 segundos
 if ((flagSetRTC == 1) ||
     /*(currentWday = 1) && */(currentHour == 2) && (currentMin == 10) && (currentSec < 10)) {
     setRTC();}

 if (flagIoTStatus == 1){
  /*
   display.setTextSize(1); 
   display.setCursor(68,57); 
   display.print(StrStateBlynk);
   display.display();               // mostra na tela
   */
   flagIoTStatus = 0;}
}