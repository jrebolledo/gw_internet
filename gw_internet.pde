#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <SPI.h>
#include <avr/wdt.h> 
#include <Ethernet.h>
#include <avr/io.h>
#include <XBee.h>
#include "utils.h"
#include <SdFat.h>
#include <SdFatUtil.h>
#include <EthernetDHCP.h>
#include <EthernetDNS.h>

/************************************/
/* CONFIGURACION DE INTERFAZ DE RED */
/************************************/

byte mac[] = SERVER_MAC;
//byte ip[] = LOCAL_IP;
//byte gateway[] = GATEWAY_IP;
//byte subnet[] = MASK_NETWORK;
//byte server[] = { 205,234,146,134 }; // servidor jaime
//byte server[] = { 192,168,0,168 }; // servidor jaime
byte dnsServerIp[] = DNS_SERVER_GOOGLE_IP;
byte server[]= SERVER_RESOLVE_IP;
Client client(server, SERVER_PORT);

/****************************************************************************/
/*  LOOKUP TABLES CON EL ESTADO E INFORMACION DE CADA NODO CONECTADO ZIGBEE */
/****************************************************************************/


nodeDef nodeDefinition[MAX_NODES];


/******************************************************/
/* STATIC QUEUES WITH PENDANT REQUEST FROM ZB AND TCP */
/******************************************************/

ZBqueue zigbeeQueue[ZB_QUEUE_MAX]; // STORE MAXIMUM 10 pendant payloads (70 bytes each)
SRVqueue serverQueue[SRV_QUEUE_MAX]; // STORE MAXIMUM 5 pendant server packets (100 bytes each)
MNGqueue managerQueue; // contains indexes of producer pointer and consumer pointer of eash queue

/******************************************************/
/*        MEASUREMENT CAPTURED USED TO CONTROL        */
/******************************************************/
Meas_Buffer measurementsBuffer[MAX_MEASUREMENT_BUFFER];


/***********************************/
uint8_t PacketToSendToServer[PACKET_TO_SEND_TO_SERVER_MAX]={
  0};
  
  
int len_PacketToSendToServer;
int maxq_array_index;
long MAC1;
long MAC2;


/************************/
unsigned long waiting_response_from=0;
boolean server_connection_istimeout;
unsigned long last_reconnection_time= 0;
boolean isconnectedtoserver = false;
boolean isauthenticated = false;
boolean servernameresolved = false;
unsigned long last_time_sent_echo;
long int interval_beetwen_echos = INTERVAL_ECHOS;
parsedZBPacket ppzigbee;
parsedSRVPacket ppserver;

/*************************/
/* CONFIGURACIONS ZIGBEE */
/*************************/
uint8_t payload[PAYLOAD_MAX]={
  0};
  
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40485ced);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

/************************/
/*  MAXQ                 /
/************************/
MAXQ_BUFFER _MAXQBuffer;
unsigned long waiting_for_max_request_from = -160000;
boolean requesting_MAXQ_data = false;

/**********************/
/***** INTERRUPT *****/
/**********************/
boolean Iamdead;
int count;
boolean enabled;

int Ethernet_reset_pin = RESET_PIN_ETHERNET;


void setup() {
  MCUSR = 0;
  //wdt_disable();
  pinMode(Ethernet_reset_pin, OUTPUT); // reset manual para reset
  
  // inicializa las conexiones seriales  
  xbee.begin(9600);    // xbee
  //Ethernet.begin(mac, ip);
  Serial.begin(9600);  // ethernet
  Serial1.begin(9600); // consola
  Serial1.println("SETUP()...");
    
  initializeStructs(); // clean nodedefintions

  EthernetDNS.setDNSServer(dnsServerIp);

  //EthernetDHCP.begin(mac,1);
  //EthernetDHCP.begin(mac);

  
  // adjust timer to detect problems in loop()
  TCCR2B |= ((1 << CS22) | (1 << CS21) | (1 << CS20));
  //Timer2 Overflow Interrupt Enable
  TIMSK2 |= (1 << TOIE2);
  RESET_TIMER2;
  sei();
  enabled = true;

  randomSeed(analogRead(0));
  //wdt_enable(WDTO_8S);
}

void loop()
{   
  
  showNodesState();

  checkConnection();
  
  //checkMAXQTrigger();

  checkQueueandServe();
  
  Iamdead = false;
  
}


//Interrupcion del Timer para el reset
ISR(TIMER2_OVF_vect) {
  
  if(enabled){
    if(count>3000){		//Aprox 5 seg
      count=0;
      if(Iamdead){
        if (!restartEthernet()) {
          Serial1.println("\tSoftReset");
          enabled=false;
          asm volatile ("jmp 0x0000"); 
        }
      }
      Iamdead=true;
    }
    count++;
  }
};





























