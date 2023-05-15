/*******************************************************************************
* Código para carga nos end-nodes Esp32 V2
* Leitura de Tensão feita pelo módulo ACS712  
* 
* Codigo usado para enviar a leitura de tensão feita pelo módulo ACS712 
* através da rede LoRa usando o protocolo LoRaWAN para a TTN utilziando a abordagem OTAA
* 
* Escrito por Wesley Nóbrega (01/04/2023).
* 
*******************************************************************************/
#include <Arduino.h>
#include <dummy.h>  // silenciar avisos do Arduino IDE
// GPS and Sensores
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include "DHTesp.h"
//#include <TheThingsNetwork.h>
#include <CayenneLPP.h>
#include <Arduino_JSON.h>
#include "EmonLib.h"


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <SSD1306.h>

// LED built in
#define LEDPIN 25

//valor de calibração (deve ser ajustado em paralelo com um multímetro medindo a corrente da carga)
#define CURRENT_CAL 18.40 

// Mapeamento de pinos OLED
#define OLED_I2C_ADDR 0x3C
#define OLED_RESET    16
#define OLED_SDA      4
#define OLED_SCL      15

// Display LCD
SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

/* Este EUI deve estar no formato little-endian, portanto, byte menos significativo
 primeiro. Ao copiar um EUI da saída ttnctl, isso significa reverter
 os bytes. Para EUIs emitidos por TTN, os últimos bytes devem ser 0xD5, 0xB3,
 0x70.
*/
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// Isso também deve estar no formato little endian, veja acima.
static const u1_t PROGMEM DEVEUI[8]={ 0x24, 0x6F, 0x28, 0x2B, 0x24, 0x98, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

/* Esta chave deve estar no formato big endian (ou, já que não é realmente um
 número, mas um bloco de memória, o endianness realmente não se aplica). Dentro
 prática, uma chave retirada de ttnctl pode ser copiada como está.
*/
static const u1_t PROGMEM APPKEY[16] = { 0x4E, 0xC9, 0x6E, 0xAF, 0x8E, 0xB1, 0xF2, 0x00, 0x11, 0x79, 0xB9, 0xAF, 0x13, 0x6F, 0x96, 0x9D };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// Endereço de configuração do dispositivo 
static const u4_t DEVADDR = 0x260DE173 /* TTN Device Address */ ; // <-- Change this address for every node!

//Esquema de pinagem para a placa Esp32 Heltec Wifi LoRa 32
const lmic_pinmap lmic_pins = {
    .nss = 18, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
//    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}  // Heltec ESP32 LoRa V1 / TTGO Lora32 with 3D metal antenna V1
  .dio = {/*dio0*/ 26, /*dio1*/ 35, /*dio2*/ 34} // Heltec ESP32 LoRa V2 / TTGO Lora32 with 3D metal antenna V2
};

//pino analógico em que o sensor está conectado
const int pinoSensor = 34; 

//ruído produzido na saída do sensor (deve ser ajustado com a carga desligada após carregamento do código no arduino)
float ruido = 0.08; 

// Tempo de intervalo de entre o envio dos pacotes 
const unsigned TX_INTERVAL = 60;

EnergyMonitor snitch; //cria uma instância para o monitoramento

// Aqui será definido o Payload
char payload[3];
static osjob_t sendjob;

DynamicJsonDocument doc(1024);

// Variáveis globais
char device[8];
unsigned int counter = 0;
char TTN_response[30];
String output;
//static osjob_t sendjob;

// GPS
// The TinyGPS++ object
TinyGPSPlus gps;
typedef union {
  float f[2];               // A atribuição de fVal.f também preencherá fVal.bytes;
  unsigned char bytes[8];   // Ambos fVal.f e fVal.bytes compartilham os mesmos 4 bytes de memória.
} floatArr2Val;
floatArr2Val latlong;
float latitude;
float longitude;
float _altitude;
String ligado;
char s[16];                 // usado para sprintf para display OLED
#define GPS_RX 22
#define GPS_TX 23
HardwareSerial GPSSerial(1);

// Conversão de dados para hexadecimal
void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

// Faz a leitura dos dados do sensor de tensão
String getTension() {
  ligado = "";
  snitch.calcvi(17,100); //função de cálculo (17 semiciclos / tempo limite para fazer a medição)
  double currentdraw = snitch.irms; //variável recebe o valor de corrente rms obtido
  currentdraw = currentdraw-ruido; //variável recebe o valor resultante da corrente rms menos o ruído
  
  if(currentdraw < 0){ //se o valor da variável for menor que 0, faz 
    ligado = "NÃO";
  } else {
    ligado = "SIM";
  }  
  return ligado;
}

void do_send(osjob_t* j){
   #ifdef DEBUG
      Serial.println(F("Enter do_send"));
    #endif
    
    // Verifica se não há um trabalho TX/RX atual em execução
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
      // Get_Data
        getTension().toCharArray(payload[0],800);
        // Prepara a transmissão de dados upstream no próximo momento possível.
        LMIC_setTxData2(1, (xref2u1_t)&payload, sizeof(payload)-1, 0);
        #ifdef DEBUG
          Serial.print(F("Packet queued: "));
          Serial.println(LMIC.freq);
          Serial.print(F("Sending uplink packet: "));
          Serial.println(counter);
        #endif
        
        digitalWrite(LEDPIN, HIGH);
        display.clear();
        display.drawString (0, 0, "Sending uplink packet...");
        display.drawString (0, 50, String (++counter));
        display.drawString (0, 25, device);
        display.display ();
    }
    // Next TX is scheduled after TX_COMPLETE event.
    #ifdef DEBUG
      Serial.println(F("Leave do_send"));
    #endif
}

// Função para tratamento de eventos ocorridos duante a leitura dos dados
void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            /* Desativar validação de verificação de link (ativado automaticamente
             durante a junção, mas porque as taxas de dados lentas alteram o TX máximo
             tamanho, não o usamos neste caso.
            */
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || Este evento é definido, mas não é usado no código. Não
        || ponto em desperdiçar espaço de código nele.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || Este evento é definido, mas não é usado no código. Não
        || ponto em desperdiçar espaço de código nele.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void setup() {
    delay(5000);
    while (!Serial);
     // inicializa a comunicação serial:
    Serial.begin(9600);
    pinMode(pinoSensor, INPUT); // Configuração para detecção de derivações LO
    snitch.current(pinoSensor, current_cal) // chamada da função de monitoramento com os parâmetros (pino analógio / valor de calibração)
    Serial.println(F("Starting"));
    
    // Prepara o DEVADDR para apresentação no display
    sprintf (device, "0x%08x", DEVADDR);

     // Use a cor azul para sinalizar a transmissão.
    pinMode(LEDPIN, OUTPUT);

    //Configurar e redefinir o OLED
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW);
    delay(50);
    digitalWrite(OLED_RESET, HIGH);

    display.init();
    //display.flipScreenVertically(); // Ajustar a orientação da tela
    display.setFont (ArialMT_Plain_10);
    display.setTextAlignment (TEXT_ALIGN_LEFT);
    display.drawString (0, 0, "Starting....");
    display.display();
    
    // 23:48:51.263 -> {"sensor":"gps","time":1351824120,"data":[48.75608,2.302038]}
    String input = "{\"sensor\":\"snitch_ACS712\",\"hora\":1351824120,\"dados\":[]}";
    deserializeJson(doc, input);
    JsonObject obj = doc.as<JsonObject>();
   // Pode-se usar uma String para obter um elemento de um JsonObject
   // Nenhuma duplicação é feita.
    long time = obj[String("time")];
   // Pode-se usar uma String para definir um elemento de um JsonObject
   // ATENÇÃO: o conteúdo da String será duplicado no JsonDocument.
   obj[String("time")] = time;

  // Pode-se obter uma String de um JsonObject ou JsonArray:
  // Nenhuma duplicação é feita, pelo menos não no JsonDocument.
   String sensor = obj["sensor"];

  // Infelizmente, o seguinte não funciona (problema #118):
  // sensor = obj["sensor"]; // <- erro "sobrecarga ambígua para 'operator='"
  // Como solução alternativa, substituir por:
  sensor = obj["sensor"].as<String>();

  // Pode-se definir uma String para um JsonObject ou JsonArray:
  // ATENÇÃO: o conteúdo da String será duplicado no JsonDocument.
  obj["sensor"] = "snitch_ACS712";

  // Funciona com serialized() também:
  obj["sensor"] = serialized(sensor);

  // Pode-se também pode concatenar strings
  // ATENÇÃO: o conteúdo da String será duplicado no JsonDocument.
  obj[String("sen") + "sor"] = String("gp") + "s";

  // Pode-se comparar o conteúdo de um JsonObject com um String
  if (obj["sensor"] == sensor) {
    // ...
  }

  // Por fim, pode-se imprimir o JSON resultante em uma String
  serializeJson(doc, output);
    #ifdef VCC_ENABLE
      // Para placas Pinoccio Scout
      pinMode(VCC_ENABLE, OUTPUT);
      digitalWrite(VCC_ENABLE, HIGH);
      delay(1000);
    #endif

    // LMIC init
    os_init();
    // Redefina o estado MAC. Sessão e transferências de dados pendentes serão descartadas.
    LMIC_reset();
    // Desative o modo de verificação de link e o ADR, porque o ADR tende a complicar a leitura.
    LMIC_setLinkCheckMode(0);
    // Seleciona o SF para o envio dos pacotes
    LMIC_setDrTxpow(DR_SF8,20);
    // Seleciona a subbanda que será utlizada, neste caso a subbanda 2
    LMIC_selectSubBand(1);
    // Iniciar envio (o envio também inicia o OTAA)
    do_send(&sendjob);
    LMIC_startJoining();
}

void loop() {
    os_runloop_once();
}
