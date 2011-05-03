#ifndef utils_h
#define utils_h
#include "WProgram.h"
#define DEBUG false

#define DHCP false

//Definiciones para la inicializacion del timer
#define INIT_TIMER_COUNT 0
#define RESET_TIMER2 TCNT2 = INIT_TIMER_COUNT

#define PARSER_SRV_PARAMS_MAX 200
#define PARSER_ZB_PARAMS_MAX 100
#define PAYLOAD_MAX 70
#define PARSER_ZB_PARAMS_MAX 100
#define PACKET_TO_SEND_TO_SERVER_MAX 200
#define ZB_QUEUE_MAX 10
#define SRV_QUEUE_MAX 5
#define INTERVAL_ECHOS 120000
#define MAX_MEASUREMENT_BUFFER 2 // maximum 2 asyncronous sensor with 6 measurement values each 
#define MAX_SIGNALS_MEASUREMENT_BUFFER 6 // each packet contains 6 signals

#define MAX_AGE_OF_A_MEASUREMENT_BEFORE_EXPIRE 1200000
#define MAX_NODES 10
#define ZB_TX_TRIES 5
#define SRV_TX_TRIES 5
#define IDS_NODES_MAX 3
#define SLOTS_NODES_MAX 5
#define MAXQ_TYPE 1
#define RELAY_TYPE 2
#define TEMP_TYPE 3
#define LIGHT_TYPE 4

/////// MAXQs//////////
#define ZIGBEE_WAITING_INTERVAL 1500
#define WAITING_INTERVAL_FOR_RESPONSE 1000
#define WAITING_INTERVAL_BEFORE_RECONNECTION 2000
#define INTERVAL_MAXQ_REQUEST 180000
#define MAX_MAXQ_VARS_ALLOWED 30

/* CONNECTION */
#define AUTH "jaime,jaime,MAC Local 50 - Gateway"
#define RESET_PIN_ETHERNET 47
#define SERVER_RESOLVE_IP { 0,0,0,0 }
#define SERVER_PORT 7081
#define SERVER_MAC { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }

#if (!DHCP)
  #define MY_LOCAL_IP {172,30,112,40} //{ 205,234,146,134 };
  //#define MY_LOCAL_IP {192,168,0,199} //{ 205,234,146,134 };
  #define GATEWAY_IP {172,30,112,1}
  //#define GATEWAY_IP {192,168,0,1}
  #define MASK_NETWORK {255,255,255,0}
  
  #define DNS_SERVER_GOOGLE_IP {172,20,1,100}
#else
  #define DNS_SERVER_GOOGLE_IP {172,20,1,100}
#endif


#define SERVER_HOSTNAME "salcobrand.no-ip.info"
//#define SERVER_LAN_IP {192,168,0,106}
/////////// METHODS ////////////
//MAXQ
#define MAXQ_GET_VARIABLE_METHOD 98 // get a single param and send it back to gateway, (used for calibration porpuses)
#define MAXQ_GET_NOW_SAMPLE_METHOD 99 // get all params available and send them to gateway
#define MAXQ_UPDATE_CALI_PARAMS_METHOD 100 // update calibration params and reload MAXQ
#define MAXQ_UPDATE_DATETIME_METHOD 97 // method to update datetime
#define MAXQ_END_STREAM 45 // attacha to third parameters when maxq stream is completed
#define MAXQ_INIT_EEPROM 232 // this can alter EEPROM energy accum and pointers to unsynced data
//RELES
#define RELE_REGLA_MANUAL_METHOD 23 
#define RELE_REGLA_CONTROL_METHOD 24 
#define RELE_REFRESH_METHOD 25
#define RELE_MEDICION_SENSORES_METHOD 26 
#define RELE_UPDATE_DATETIME_METHOD 27
#define RELE_CLEAN_EEPROM_METHOD 28 // reset eeprom
//ASYNC SENSORS
#define ASYNC_NODE_METHOD 15   // nodo-> gateway 
//TOD
#define TOD_METHOD 11 // node -> gateway
//SERVER
#define ECHO_METHOD 70 // gateway  --> server
#define IDENTIFY_METHOD 0 
#define RESET_SLAVES_METHOD 128 // this method will reset all slaves connected to TX

#define BUSY_METHOD 68 // send by gateway to server when QUEUE are full 
#define BUSY_METHOD 200

/* PROTOCOL Node -> GW  */

#define ASK_MEAS_METHOD 16// if relay is asking for measurements 


typedef struct parsedZBPacket_struct {
  byte method;
  byte params[PARSER_ZB_PARAMS_MAX];byte len_params;
  byte opID;
}parsedZBPacket;

typedef struct parsedSRVPacket_struct {
  byte method;
  byte params[PARSER_SRV_PARAMS_MAX];byte len_params;
  byte opID;
}parsedSRVPacket;

typedef struct MAXQ_buffer_struct { // store temporary MAXQ packets
  byte current_index; 
  byte last_part;
  boolean ready;
  byte signals[MAX_MAXQ_VARS_ALLOWED*4+10];
}MAXQ_BUFFER;

typedef struct nodeData_struct {
  byte id;
  long int MAC1;
  long int MAC2;
  boolean isonline;
  byte type;
  byte slotsnum;
  byte IDs[SLOTS_NODES_MAX][IDS_NODES_MAX][2];  //([slot][dev1][id,type], // type (1): MAXQ, (2): Rele, (3):SenTemp, (4):SenLighting
}nodeDef;

typedef struct ZBqueue_struct {
  byte nodeindex;
  parsedZBPacket ppzigbee;
  long int datetime;
}ZBqueue;

typedef struct SRVqueue_struct {
  byte nodeindex;
  parsedSRVPacket ppserver;
  long int datetime;
}SRVqueue;

typedef struct MNGqueue_struct {
  byte producer_ZB;
  byte consumer_ZB;
  boolean ZB_is_full;
  byte ZBwaiting;
  byte producer_SRV;
  byte consumer_SRV;
  boolean SRV_is_full;
  byte SRVwaiting;
}MNGqueue;

// measurement buffer from asynchronous sensor
typedef struct Meas_Buffer_struct {
  byte devid;    //dev id del sensor
  byte measurement[MAX_SIGNALS_MEASUREMENT_BUFFER]; // measurement
  unsigned long time; // time when measurements arrived
  boolean used; // flag if element has been initialized
}Meas_Buffer;

/* PROTO FUNCIONES */
const char* ip_to_str(const uint8_t* ipAddr);
void PROGMEMprint(const prog_uchar str[],boolean br);


void sendReset(); // envia msg de reset al servidor para comunicar que el gateway se ha reiniciado para DEBUG
void showNodesState(); // muestra los nodos que estan online
void checkConnection(); // chequea si esta conectado y reconecta si no esta
void softReset();
#if (DHCP)
boolean checkDHCP();
#endif
boolean restartEthernet();
boolean resolveServerIP();

parsedZBPacket parseZigbeePacket(uint8_t *dataToParse,int length); // devuelve una estructura con el paquete zigbee parseado en metodo, data (string) y el id
parsedSRVPacket parseServerPacket2(); // devuelve una estructura con el paquete tcp/ip parseado en metodo, data (string) y el id, devuelve cod de errores y guarda el dato directamente en GLOBAL response_from_server
void buildPacketToSendToServer(int method, byte* params, int len_params, char *purestring, boolean option, int opID); // dataToSend puntero a char que contiene los caracteres que seran enviados, ultimo caracter contiene NULL
void joinNode(long int MAC1, long int MAC2); // chequea si existe el nodo y actualiza su status, en caso que no se encuentre agrega uno nuevo a nodesData
void sendPacketToServer(byte method, byte* params, int len_params, int opID); // envia paquuete generico sin esperar respuesta al servidor 
void ZBTX(byte method, byte* params, int start_index, int len_params, int opID, long int MAC1, long int MAC2);
void printCHARArrayasDEC(char* cadena,int len);
void printuint8ArrayasDEC(byte* cadena,int len);

void printArrayasHEX(byte* cadena,int len);
/* REad anything from eprom */
template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
	  EEPROM.write(ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
	  *p++ = EEPROM.read(ee++);
    return i;
} 

void printPendantRequest(byte pendant_request_id,boolean type);
boolean sendPacketToServerandParseResponse(int opID, boolean wait_ack, byte method); // envia packete al servidor, implementa tiemout, y guarda datos en estructura ppserver, retorna false si falla
boolean sendPacketToZigbeeNodeandParseResponse(int opID,long int zigbeetimeout,long int MAC_MSB, long int MAC_LSB, byte tries, boolean wait_ack, byte method); // // Send join packet to new node and wait for confirmation, if success, True, otherwose False
byte searchDevIdBySlotandMAC(long int MAC1,long int MAC2, byte slot, byte typeofdev); // return an unique dev id corresponding to an specific MAC and slot
void printNodeData();
byte getnodeDefinitionFromMAC(long int MAC1,long int MAC2); 
byte getnodeDefinitionFromdevID(byte devID);
boolean fillPayloadWithParams(byte index_start_payload,byte index_start_params,byte* params, int len_params);
void changeNodeState(byte nodeindex, boolean state); // change state of selected node def 1:online, 0:offline
byte getAvailableIndexforNewNodeDefinition();
void initializeStructs();
void pushToQueue(byte queueid); // insert new element to queue (queueid:1 zigbee queue, queueid:2 server queue)
void checkQueueandServe(); // check if there are element not served in queues and process them one by one
void checkUnconnectedEthernetCable();

/* MACRO FUCNCIONES */
void processingTOD(long int MAC1,long int MAC2);
void processingRelayRefresh(long int MAC1,long int MAC2);
void processingMAXQSample(long int MAC1,long int MAC2);
void processingControlPacket(byte type);
//void maxqProcessingPart(byte part,byte opID, byte devID); // create a MAXQ packet to server
void processingASYNC(long int MAC1,long int MAC2, byte type); // send aync data to server, type:0 (temp), 1 (light) 
boolean checkNewZigbeeRequest();
boolean checkNewTCPRequest();
//void checkMAXQTrigger();
void processingMeasRequest(long int MAC1,long int MAC2); // get measument from RAM buffer and send it back to the relay for control porpuses
void processingTimeRequest(long int MAC1,long int MAC2);

void getMyZBMac(byte* mac); // using AT commands store ZIgbee MAC into mac
void passingPacketToNode(); // pass a packet directly to devid (params len restricted to 68)
void confirmRuleSaved(); // if a rules has been correctly saved in target, an ack packet will be generated and send it vack to server
#endif



