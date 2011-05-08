#include <stdlib.h>
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include "utils.h"
#include <avr/io.h>
#include <WProgram.h>
#include <Ethernet.h>
#include <string.h>
#include <XBee.h>
#if (DHCP)
  #include <EthernetDHCP.h>
#endif
#include <EthernetDNS.h>
//#include <avr/wdt.h> 
// declaración de variables externas extern
extern Client client;
extern int Ethernet_reset_pin;
extern byte server[4];
#if (!DHCP)
  extern byte ip[];
  extern byte gateway[];
  extern byte subnet[];
#endif
extern byte mac[];
extern byte dnsServerIp[];
extern nodeDef nodeDefinition[MAX_NODES];
extern Meas_Buffer measurementsBuffer[MAX_MEASUREMENT_BUFFER];
extern boolean isconnectedtoserver;
extern boolean isauthenticated;
extern boolean servernameresolved;
extern byte server_Ip[4]; // store the ip of the server once its name has been resolve
extern unsigned long last_reconnection_time;
extern unsigned long waiting_response_from;
extern boolean server_connection_istimeout;
extern XBee xbee;
extern ZBRxResponse rx;
extern XBeeResponse response;
extern ZBTxRequest zbTx;
extern ZBTxStatusResponse txStatus;
extern parsedZBPacket ppzigbee;
extern parsedSRVPacket ppserver;
extern ZBqueue zigbeeQueue[ZB_QUEUE_MAX]; // STORE MAXIMUM 10 pendant payloads (70 bytes each)
extern SRVqueue serverQueue[SRV_QUEUE_MAX];// STORE MAXIMUM 5 pendant server packets (100 bytes each)
extern MNGqueue managerQueue; // STORE POINTERS QUEUE
extern uint8_t PacketToSendToServer[PACKET_TO_SEND_TO_SERVER_MAX];
extern int len_PacketToSendToServer;
extern int maxq_array_index;
extern MAXQ_BUFFER _MAXQBuffer;
extern XBeeAddress64 addr64;
extern uint8_t payload[PAYLOAD_MAX];
extern unsigned long waiting_for_max_request_from;
extern boolean requesting_MAXQ_data;
extern unsigned long last_time_sent_echo;

extern boolean Iamdead;
extern int count;
extern boolean enabled;



/* MESSAGES */

const prog_uchar parserZB1[] PROGMEM  = {"\t(N)(PARSER ZIGBEE)Resultado de parseo Zigbee: "};   // "String 0" etc are strings to store - change to suit.
const prog_uchar parserZB2[] PROGMEM  = {"\t\tMethod: "};
const prog_uchar parserZB3[] PROGMEM  = {"\t\topID: "};
const prog_uchar parserZB4[] PROGMEM  = {"\t\tParams: "};

const prog_uchar parseServerPacket2_1[] PROGMEM  = {"---------\n\t(PARSER TCP)Nuevo paquete TCP ("};
const prog_uchar parseServerPacket2_2[] PROGMEM  = {" bytes) :"};
const prog_uchar parseServerPacket2_3[] PROGMEM  = {"TCP PARAMS#"};
const prog_uchar parseServerPacket2_4[] PROGMEM  = {"\n\t(N)(PARSER TCP)Resultado de parseo Server: "};
const prog_uchar parseServerPacket2_5[] PROGMEM  = {"\t\tMethod: "};
const prog_uchar parseServerPacket2_6[] PROGMEM  = {"\t\topID: "};
const prog_uchar parseServerPacket2_7[] PROGMEM  = {"\t\tParams: "};

const prog_uchar softreset_1[] PROGMEM  = {"\t\tParams: "};

const prog_uchar sendreset_1[] PROGMEM  = {"*"};
const prog_uchar sendreset_2[] PROGMEM  = {"\n\t(RESET)"};
const prog_uchar sendreset_3[] PROGMEM  = {"\t(RESET)Conectado al servidor"};
const prog_uchar sendreset_4[] PROGMEM  = {"\t(RESET)sendReset()"};
const prog_uchar sendreset_5[] PROGMEM  = {"\t(RESET)No pudo conectar al servidor"};
const prog_uchar sendreset_6[] PROGMEM  = {"\t(RESET)sendReset() failed"};

const prog_uchar showNodesState_1[] PROGMEM  = {"\t(NODE STATE)"};

const prog_uchar printArrayHEX_1[] PROGMEM  = {", "};


const char* ip_to_str(const uint8_t* ipAddr)
{
  static char buf[16];
  sprintf(buf, "%d.%d.%d.%d\0", ipAddr[0], ipAddr[1], ipAddr[2], ipAddr[3]);
  return buf;
}

void PROGMEMprint(const prog_uchar str[],boolean br)
{
  char c;
  if(!str) return;
  while((c = pgm_read_byte(str++)))
    Serial1.print(c,BYTE);
  
  if (br) {
    Serial1.println();
  }
}

void printArrayasDEC(byte* cadena,int len) {
  for (int j=0;j<len-1;j++) {
    Serial1.print(cadena[j],DEC);Serial1.print(", ");
  }
  Serial1.print(cadena[len-1],DEC);
  Serial1.println("");
}

parsedZBPacket parseZigbeePacket(uint8_t* dataToParse,int length) {
  //dataToParse in the format "methodo:param1,param2,param3,..paramn:id"
  parsedZBPacket pp;
  pp.method = dataToParse[0];
  for (int u=1;u<length-1;u++) {
    pp.params[u-1]=dataToParse[u];
  }
  pp.len_params = length-2;
  pp.opID = dataToParse[length-1];
    PROGMEMprint(parserZB1,true);    
    PROGMEMprint(parserZB2,false);Serial1.println(pp.method,DEC);
    PROGMEMprint(parserZB3,false);Serial1.println(pp.opID,DEC);
    PROGMEMprint(parserZB4,false);printArrayasDEC(pp.params,pp.len_params);
  return pp;
}

parsedSRVPacket parseServerPacket2() {
  //dataToParse in the format "methodo:param1,param2,param3,..paramn:id"
  parsedSRVPacket pp;

  int index_response = 0;
  int len_params;
  boolean packet_overflow = false;
  if (client.available()) {
    PROGMEMprint(parseServerPacket2_1,false);Serial1.print(client.available());PROGMEMprint(parseServerPacket2_2,false);
  }
  
  while (client.available()) { // if data collected is greater than maximum response_from_server buffer this could be adapted to reread again till all data is processed

    pp.method = client.read();
    pp.len_params = client.read();
    PROGMEMprint(parseServerPacket2_3,false);Serial1.println(pp.len_params,DEC);
    for (int po=0;po<pp.len_params;po++) {
      if (client.available() > 0) {
        if (pp.len_params <= PARSER_SRV_PARAMS_MAX) {
          pp.params[po] = client.read();
        } else {
          packet_overflow = true;
          pp.params[0] = client.read(); // cleaning the ethernet buffer, avoiding overflow in params
        }
      }
      else {
        packet_overflow = true;
        break;
      }
    }
    
    pp.opID = client.read();
    
    if (packet_overflow) {
      pp.method = 69;
      pp.opID = 1; // packet exceed maximum
      return pp; // exit
    }
    else {
      break; // this allows only take one packet from ethernet buffer
    }
  }
    

  PROGMEMprint(parseServerPacket2_4,true);
  PROGMEMprint(parseServerPacket2_5,false);Serial1.println(pp.method,DEC);
  PROGMEMprint(parseServerPacket2_6,false);Serial1.println(pp.opID,DEC);
  PROGMEMprint(parseServerPacket2_7,false);printArrayasDEC(pp.params,pp.len_params);
  
  return pp;
}

void softReset() {
  if(enabled){
    if(count>3000){		//Aprox 5 seg
      count=0;
      if(Iamdead){
        PROGMEMprint(softreset_1,true);
        enabled=false;
        asm volatile ("jmp 0x0000"); 
      }
      Iamdead=true;
    }
    count++;
  }
}
// envia notificacion al servidor de reseto DEBUG
void sendReset() {
  uint8_t resetpk[] = {69,0,0};
  for (int y=0;y<50;y++) {
      PROGMEMprint(sendreset_1,false);
  }
  PROGMEMprint(sendreset_2,true);

  
  if (client.connect()) {
    PROGMEMprint(sendreset_3,true);
    PROGMEMprint(sendreset_4,true);
    client.write(resetpk,3);
    isconnectedtoserver = true;
    last_reconnection_time = millis();
  }
  else {
    last_reconnection_time = millis();
    isconnectedtoserver = false;
    PROGMEMprint(sendreset_5,true);
    PROGMEMprint(sendreset_6,true);


  }
}

void showNodesState() { // deberia solo mostrar los alguna vez registrados desde que se encendio
  if (DEBUG) {
    PROGMEMprint(showNodesState_1,true);
    printNodeData();
  }
}

// Imprime por consola un array en formato HEX
void printArrayasHEX(byte* cadena,int len) {
  for (int j=0;j<len-1;j++) {
    Serial1.print(cadena[j],HEX);PROGMEMprint(printArrayHEX_1,false);
  }
  Serial1.print(cadena[len-1],HEX);
  Serial1.println();
}

void printCHARArrayasDEC(char* cadena,int len) {
  for (int j=0;j<len-1;j++) {
    Serial1.print(cadena[j],DEC);PROGMEMprint(printArrayHEX_1,false);
  }
  Serial1.print(cadena[len-1],DEC);
  Serial1.println();
}

void printuint8ArrayasDEC(byte* cadena,int len) {
  for (int j=0;j<len-1;j++) {
    Serial1.print(cadena[j],DEC);PROGMEMprint(printArrayHEX_1,false);
  }
  Serial1.print(cadena[len-1],DEC);
  Serial1.println();
}

boolean resolveServerIP() {
  char hostName[25] = SERVER_HOSTNAME;
  boolean dns_resolved = false;
  Serial1.print("\t(DNS)Resolving: ");
  Serial1.print(hostName);
  Serial1.println("...");
  
  DNSError err = EthernetDNS.sendDNSQuery(hostName);

  if (DNSSuccess == err) {
    do {
      err = EthernetDNS.pollDNSReply(server);	
      if (DNSTryLater == err) {
      }
    } while (DNSTryLater == err);
  }
  
  if (DNSSuccess == err) {
    servernameresolved = true;
    Serial1.print("\t(DNS)TheIPaddressIs ");
    Serial1.print(ip_to_str(server));
    Serial1.println(".");
    dns_resolved = true;
    isconnectedtoserver = true; // there is internet connection
    //isconnectedtoserver = false;
    return dns_resolved;
  } else if (DNSTimedOut == err) {
    Serial1.println("\t(DNS)Timedout");
  } else if (DNSNotFound == err) {
    Serial1.println("\t(DNS)Doesnotexist");
  } else {
    Serial1.print("\t(DNS)FailedCode ");
    Serial1.print((int)err, DEC);
    Serial1.println(".");
  }

  return dns_resolved;
}
#if (DHCP)
boolean checkDHCP() {
  static DhcpState prevState = DhcpStateNone;
  static unsigned long prevTime = 0;
  
  DhcpState state = EthernetDHCP.poll();
  
  if (prevState != state) {

    switch (state) {
      case DhcpStateDiscovering:
        Serial1.print("\t(DHCP)DiscoveringServers");
        break;
      case DhcpStateRequesting:
        Serial1.print("\t(DHCP)RequestingLease");
        break;
      case DhcpStateRenewing:
        Serial1.print("\t(DHCP)RenewingLease");
        break;
      case DhcpStateLeased: {
        Serial1.println("\t(DHCP)ObtainedLease!");

        // Since we're here, it means that we now have a DHCP lease, so we
        // print out some information.
        const byte* ipAddr = EthernetDHCP.ipAddress();
        const byte* gatewayAddr = EthernetDHCP.gatewayIpAddress();
        const byte* dnsAddr = EthernetDHCP.dnsIpAddress();

        Serial1.print("\t(DHCP)MyIPaddressIs ");
        Serial1.println(ip_to_str(ipAddr));
        
        Serial1.print("\t(DHCP)GatewayIPaddressIs ");
        Serial1.println(ip_to_str(gatewayAddr));

        Serial1.print("\t(DHCP)DNsIPaddressIs ");
        Serial1.println(ip_to_str(dnsAddr));
        
        Serial1.println();
        
        break;
      }
    }
    
  } else if (state != DhcpStateLeased && millis() - prevTime > 300) {
     prevTime = millis();
     Serial1.print('.'); 
  }
  prevState = state;
  
  if (state == DhcpStateLeased) {
    return true;
  }
  else {
    return false;
  }

}
#endif
void getMyZBMac(byte* mac) {
  uint8_t shCmd[] = {'S','H'};
  uint8_t slCmd[] = {'S','L'};
  
  AtCommandRequest atRequest = AtCommandRequest();
  AtCommandResponse atResponse = AtCommandResponse();
    
  for (byte t=0;t<2;t++) { // send to at commands requesting both serial LSB, MSB
    // send the command
    if (t==0) {
      atRequest.setCommand(shCmd);
    }
    else {
      atRequest.setCommand(slCmd);
    }
    
    xbee.send(atRequest);
  
    // wait up to 5 seconds for the status response
    if (xbee.readPacket(5000)) {
      // got a response!
  
      // should be an AT command response
      if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
        xbee.getResponse().getAtCommandResponse(atResponse);
  
        if (atResponse.isOk()) {
          if (atResponse.getValueLength() > 0) {
            
            for (int i = 0; i < atResponse.getValueLength(); i++) {
              mac[t*4+i] = atResponse.getValue()[i];
            }
          }
        } 

      }  
    }
  }
  
}

// chequea si esta conectado y reconecta si no

void checkConnection() {
  if (DEBUG) {
    Serial1.println("/**CHECKING CONNECTION**/");
  }
  int index_response = 0;
  parsedSRVPacket pp; 
  char params[1] = {0};
  byte zbmac[8];

  
  if (!isconnectedtoserver) {
    if (!restartEthernet()) {
      // se reinicia ethernet, se reconecta la red y se consulta el DNS server
      Serial1.println("\tNo conecta");
      return;
    }
    else {
      Serial1.println("\tSi conecta");
    }
  }
  
  if (!client.connected()) { // se intenta reconectar al servidor
    isconnectedtoserver = false;
    isauthenticated = false;
    client.flush();
    client.stop();
  }
  else {
    if (!isconnectedtoserver) {
      client.flush();
      servernameresolved = false;
      client.stop();
    }
  }
  
  
  // conecta y autentifica
  // overflow control
  if (last_reconnection_time > millis()) {
    last_reconnection_time = millis();
  }
  
  if (!(isconnectedtoserver && isauthenticated) && millis() - last_reconnection_time > WAITING_INTERVAL_BEFORE_RECONNECTION) {
    Serial1.println("\t...reconectando");
    if (client.connect()) {
      isconnectedtoserver = true;
      
      Serial1.println("\t(ConnCheck)ConnectedToServerOK.. Auth..");
      last_reconnection_time = millis();
      
      // get my zigbee mac
      getMyZBMac(zbmac);
      Serial1.print("\t(ZBMac):");printArrayasHEX(zbmac,8);
      
      buildPacketToSendToServer(IDENTIFY_METHOD, zbmac, 8, params, false, 78);
      if (sendPacketToServerandParseResponse(78,true,IDENTIFY_METHOD)) {
          isauthenticated = true;
          Serial1.println("\t(ConnCheck)AuthenticationOK");
      }
      else {
        //Serial1.println("Fallo autentificacion. Cerrando conexion y reintentando");
        isauthenticated = false;
        servernameresolved = false;
        isconnectedtoserver = false;
        delay(1000);
      }
    }
    else {
      isconnectedtoserver = false;
      isauthenticated = false;
      servernameresolved = false;
      Serial1.println("\t(ConnCheck)CanNotConnect");
      last_reconnection_time = millis();
    }
  }
  
  checkUnconnectedEthernetCable();
  
    // imprime variables de entorno
  if (DEBUG) {
    if (!(client.connected() && isconnectedtoserver && isauthenticated)) {
      Serial1.print("\tClient.connected():");Serial1.println((int)client.connected());
      Serial1.print("\tisconnectedtoserver:");Serial1.println((int)isconnectedtoserver);
      Serial1.print("\tisauthenticated:");Serial1.println((int)isauthenticated);  
    }
  }
}

void joinNode(long int MAC1, long int MAC2) { 
  // from MAC1, MAC2, and previously sent MAC packet to server and dev id available in ppserver from server response, join node and parse devids

  if (ppserver.params[0]==0) {
    Serial1.println("\t(JOIN)NodeIsRegistered");
    return;
  }
  byte nodeDefIndex = getnodeDefinitionFromMAC(MAC1,MAC2);

  if (nodeDefIndex != 254) {// if node is already registeres but previously offline
    nodeDefinition[nodeDefIndex].isonline = true;
  }
  else { // create a new node instance and attaching dev ids
    byte newNodeIndex = getAvailableIndexforNewNodeDefinition();
    if (newNodeIndex !=254) {
      if (DEBUG) {
        Serial1.println("\t(JOIN)NewSlotCreated");
      }
      nodeDefinition[newNodeIndex].MAC1=MAC1;
      nodeDefinition[newNodeIndex].MAC2=MAC2;
      nodeDefinition[newNodeIndex].isonline = true;
      nodeDefinition[newNodeIndex].slotsnum = ppserver.params[0];
      // parsing dev id data from server packet
      int param_index;
      int ids_num;
      param_index = 0;
      for (int s=1;s<=nodeDefinition[newNodeIndex].slotsnum;s++) { // precessing one at time
        param_index++;
        ids_num = ppserver.params[param_index];
          for (int id=0;id<ids_num;id++) { // processing one devID at time
            param_index++;
            nodeDefinition[newNodeIndex].IDs[s-1][id][0] = ppserver.params[param_index]; // saving id
            param_index++;
            nodeDefinition[newNodeIndex].IDs[s-1][id][1] = ppserver.params[param_index]; // saving type (1): MAXQ, (2): Rele, (3):SenTemp, (4):SenLighting
          }
      }
    }
    else {      
      Serial1.println("\t(JOIN)NoMoreNodesSlotsAreAllowed");
    }
    
  }
}

void sendPacketToServer(byte method, byte* params,byte len_params, int opID) {
  char purestring[1] = {0};
  buildPacketToSendToServer(method, params, len_params, purestring, false, opID);
  if (sendPacketToServerandParseResponse(opID,false,method)) {
    Serial1.println("\t(TXTCPPACKET)Packetsent");
  }
  else {
    Serial1.println("\t(TXTCPPACKET)ErrorTX");
  }
}

void ZBTX(byte method, byte* params, int start_index, int len_params, int opID, long int MAC1, long int MAC2) {
  payload[0]=method;
  for (int y=start_index;y<len_params+start_index;y++) {
    payload[1+y]=params[y]; // byte parte 2
  }
  payload[len_params+1]=(uint8_t)opID; 
  zbTx.setPayloadLength(2+len_params);
  // requesting packet 2
  if (sendPacketToZigbeeNodeandParseResponse(opID,ZIGBEE_WAITING_INTERVAL,MAC1,MAC2,1,false,method)) { //MAXQ_METHOD
    Serial1.println("\t(TXZBPACKET)Packetsent");
  }
  else {
    Serial1.println("\t(TXZBPACKET)ErrorTX");
  }
}

void buildPacketToSendToServer(int method, byte* params, int len_params, char* purestring, boolean option, int opID) {
  int sizepurestring = 0;
  int index_size =0;
  int array_index = 0;
  // cleaning PacketToSendToServer
  
  for (int y=0;y<PACKET_TO_SEND_TO_SERVER_MAX;y++) {
    PacketToSendToServer[y] = 0;
  }

  PacketToSendToServer[0] = (uint8_t)method;
  array_index++;
  if (option) {
    sizepurestring = strlen(purestring);
    for (int f=0;f<sizepurestring;f++) {
      PacketToSendToServer[array_index] = (uint8_t)purestring[f];
      array_index++;
    }
  }
  else {
    for (int s=0;s<len_params;s++) {
      PacketToSendToServer[array_index] = (uint8_t)params[s];
      array_index++;
    }
  }
  
  PacketToSendToServer[array_index] = (uint8_t)opID;
  array_index++;
  
  len_PacketToSendToServer = array_index;
  
  //Serial1.print("\t(TCP)ToSend:");printuint8ArrayasDEC(PacketToSendToServer,len_PacketToSendToServer);

}

boolean sendPacketToServerandParseResponse(int opID, boolean wait_ack, byte method) {
  byte buffer=0;
  boolean something_received = false;
  Serial1.print("\t(TCP)ToSend:");printuint8ArrayasDEC(PacketToSendToServer,len_PacketToSendToServer);
  client.write(PacketToSendToServer,len_PacketToSendToServer); // sending PacketToSendToServer, waiting for response an parse
  waiting_response_from = millis(); // waiting_response_from (tiempo de inicio de la solicitud, usado como ref para calculo de timeout)
  server_connection_istimeout = false;    // server_connection_istimeout (flag que indica que el proceso esta en timeout

  if (!wait_ack) {
    return true;
  }
  
  while (1) {
    if (buffer >= SRV_TX_TRIES) { // can support on the buffer only 5 packets different from the answer expected after that break the loop and show errors
      Serial1.print("\t\tNomore:");Serial1.println(buffer,DEC);
      ppserver.method = 69; // codigo de error
      ppserver.opID = 3; // opID no corresponse al enviado, codigo de error
      break;
    }
    else {
        buffer++;
        Serial1.print("\t\tTry:");Serial1.println(buffer,DEC);
    }
    waiting_response_from = millis();
    
    while (!server_connection_istimeout && !client.available()) {  // out of here is timeout exception is arised
      // overflow control
      if (waiting_response_from > millis()) {
        waiting_response_from = millis();
      }
      if (millis() - waiting_response_from > WAITING_INTERVAL_FOR_RESPONSE) {
        server_connection_istimeout = true;
      }
    }
    
    if (!server_connection_istimeout) { // if server data is present and no timeout occurs
      ppserver = parseServerPacket2();
      something_received = true;
      Serial1.println("\t(TCP)DatareceivedOK");
      if (ppserver.opID != opID || ppserver.method != method) {
          Serial1.println("\t(TCP)Newrequestpushedtoqueue");
          pushToQueue(2);
          continue;
          ppserver.method = 69;
          ppserver.opID = 3; // opID o metodo no corresponse al enviado
      }
      break;
    }
    else {
      ppserver.method = 69;
      ppserver.opID = 2; // set error code to timeout
      // connection variables are cleared to allow to connection manager to reconnect again
      isconnectedtoserver = false;
      resolveServerIP(); // check server ip again
      break;
    }
  }
  

  
  if (ppserver.method == 69) { // print error codes, if no error data is available in global struct ppserver
    switch (ppserver.opID) {
      case 1:
        Serial1.println("\t(TCP)Packettoobig");  
        break;
      case 2:
        Serial1.println("\t(TCP)Timeout");
        if (!something_received) {
          //resolveServerIP(); // check server ip again
        }
        break;
      case 3:
        Serial1.println("\t(TCP)opIDdontmatch");
      default:
        break;
    }
    return false;
  }
  else {
    return true;
  }
}

boolean sendPacketToZigbeeNodeandParseResponse(int opID,long int zigbeetimeout,long int MAC_MSB, long int MAC_LSB, byte tries, boolean wait_ack, byte method) {
  boolean RX_success = false;
  byte buffer=0;
  
  addr64.setMsb(MAC_MSB);
  addr64.setLsb(MAC_LSB);
  zbTx.setAddress64(addr64);
  for (byte w=0;w<tries;w++) {
    Serial1.print("\t(TXRXZB)Target: ");Serial1.print(MAC_MSB,DEC);Serial1.print(" - ");Serial1.println(MAC_LSB,DEC);
    Serial1.print("\t(TXRXZB)Try#:");Serial1.print(w,DEC);Serial1.print(" DATA:");printuint8ArrayasDEC(payload,(int)zbTx.getPayloadLength());
    
    // check input zb buffer
    if (xbee.readPacket(zigbeetimeout)) {
      Serial1.println("\t(TXRXZB)CheckZBBuffer");
      if (xbee.getResponse().isAvailable()) {
        if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
          xbee.getResponse().getZBRxResponse(rx); 
          // check if packet is from the target
          ppzigbee = parseZigbeePacket(rx.getData(), rx.getDataLength());

        }
      }
    }
    
    xbee.send(zbTx);  // sending payload to destination, waiting for response an parse
    
    // waiting for ack status
    if (xbee.readPacket(500)) {
      if (xbee.getResponse().isAvailable()) {
        // check if packet arrived ok to targer        
        if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
    	   xbee.getResponse().getZBTxStatusResponse(txStatus);
    		
    	   // get the delivery status, the fifth byte
           if (txStatus.getDeliveryStatus() == SUCCESS) {
            	// success.  time to celebrate
              Serial1.println("\t\t(TXRXZB)TXsuccess");
              RX_success = true;
           } else {
              byte nodeindex = getnodeDefinitionFromMAC(MAC_MSB,MAC_LSB);
              changeNodeState(nodeindex,0); // set the node offline
              Serial1.println("\t\t(TXRXZB)TXerror.Probablyoffline");
           }
        }
        
      }
      
    }
    else {
      // codigos de error
      ppzigbee.method = 69;
      ppzigbee.opID = 4;
    } // en waiting for ack status
    
    
    // waiting for response, cleaning ppzigbee first
    
    if (wait_ack && RX_success) {
      while (1) {
        if (buffer >= ZB_TX_TRIES) { // can support on the buffer only 5 packets different from the answer expected after that break the loop and show errors
          Serial1.println("\t\tNomore");
          ppzigbee.method = 69; // codigo de error
          ppzigbee.opID = 3; // opID no corresponse al enviado, codigo de error
          break;
        }
        else {
          buffer++;
          Serial1.print("\t\tTry:");Serial1.println(buffer,DEC);
        }
        if (xbee.readPacket(zigbeetimeout)) {
          if (xbee.getResponse().isAvailable()) {
            if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
              xbee.getResponse().getZBRxResponse(rx); 
              // check if packet is from the target
              if (rx.getRemoteAddress64().getMsb() == MAC_MSB && rx.getRemoteAddress64().getLsb() == MAC_LSB) {      
                ppzigbee = parseZigbeePacket(rx.getData(), rx.getDataLength());
                Serial1.print("ppzigbee.opID");Serial1.println(ppzigbee.opID,DEC);
                Serial1.print("ppzigbee.method");Serial1.println(ppzigbee.method,DEC);
                Serial1.print("opID");Serial1.println(opID,DEC);
                Serial1.print("method");Serial1.println(method,DEC);
                if (ppzigbee.opID != opID || ppzigbee.method != method) {
                  // attach pendant operation to a zigbeeRequestQueue[] from target
                  pushToQueue(1);
                  Serial1.print("\t(TXRXZB) ");Serial1.print("(");Serial1.print(rx.getRemoteAddress64().getMsb(),DEC);Serial1.print("-");Serial1.print(rx.getRemoteAddress64().getLsb(),DEC);Serial1.println(") New request pushet to queue");
                  continue;
                  ppzigbee.method = 69; // codigo de error
                  ppzigbee.opID = 3; // opID no corresponse al enviado, codigo de error
                }
                break;
              }
              else {
                Serial1.print("\t(TXRXZB) ");Serial1.print("(NOT ME ");Serial1.print(rx.getRemoteAddress64().getMsb(),DEC);Serial1.print("-");Serial1.print(rx.getRemoteAddress64().getLsb(),DEC);Serial1.println(") New request pushet to queue");
                continue;
              }
            }
          }
        } // end parse response
        else {
          // codigos de error
          ppzigbee.method = 69;
          ppzigbee.opID = 2; 
        } // en waiting for ack status
        
        
      }
      
      if (ppzigbee.method != 69) return true;
    }
    

      
    
  } // end tries
  
  if (ppzigbee.method == 69) { // print error codes, if no error data is available in global struct ppserver
    switch (ppzigbee.opID) {
      case 1:
        Serial1.println("\t(TXRXZB)Packettoobig");  
        break;
      case 2:
        Serial1.println("\t(TXRXZB)Timeout");
        break;
      case 3:
        Serial1.println("\t(TXRXZB)opIDdontmatch");
        break;
      case 4:
        Serial1.println("\t(TXRXZB)ZBstatusnotreceived");
      default:
        break;
    }
    return false;
  }
  else {
    return true;
  }
}

byte searchDevIdBySlotandMAC(long int MAC1,long int MAC2, byte slot, byte typeofdev) {
  // search MAC
  //printNodeData();
  for (byte g=0;g<MAX_NODES;g++) {
    if (nodeDefinition[g].MAC1 != 0 && nodeDefinition[g].MAC2 != 0) {
      if (nodeDefinition[g].MAC1 == MAC1 && nodeDefinition[g].MAC2 == MAC2) {
        for (int u=0;u<IDS_NODES_MAX;u++) {
          if (nodeDefinition[g].IDs[slot-1][u][0] !=0 && nodeDefinition[g].IDs[slot-1][u][1] == typeofdev) { // print online used dev ids
            return nodeDefinition[g].IDs[slot-1][u][0];
          }
        }
      }
    }
  }
  return 0;
}

void processingTOD(long int MAC1,long int MAC2) {
  byte params[8];
  
  if (!(isconnectedtoserver && isauthenticated)) {
    Serial1.println("\t(TOD)Noconnetion");
    return;
  }
  params[0] = (byte)(MAC1 >> 24);
  params[1] = (byte)(MAC1 >> 16);
  params[2] = (byte)(MAC1 >> 8);
  params[3] = (byte)MAC1;
  params[4] = (byte)(MAC2 >> 24);
  params[5] = (byte)(MAC2 >> 16);
  params[6] = (byte)(MAC2 >> 8);
  params[7] = (byte)MAC2;
            
  int len_param   = 8;
  int method  = TOD_METHOD;
  char purestring[1] = {0};
  boolean option = false;
  buildPacketToSendToServer(method, params, len_param, purestring, option, ppzigbee.opID);

  boolean wait_ack = true;
  if (sendPacketToServerandParseResponse(ppzigbee.opID,wait_ack,method)) { 
    // dev_ids data is available from now in ppserver
    
    // building zigbee packet to join new node to network
    payload[0]=TOD_METHOD;
    payload[1]=0;
    payload[2]=ppzigbee.opID;
    zbTx.setPayloadLength(3);
    
    // Send join packet to new node and wait for confirmation, if success, join node to network
    byte tries = 1; // how many times must the gateway try to send the packet to target and wait for the response
    
    //sendPacketToZigbeeNodeandParseResponse(ppzigbee.opID,ZIGBEE_WAITING_INTERVAL,MAC1,MAC2,tries,false,25);
    joinNode(MAC1,MAC2);
    Serial1.println("\t(TOD)TodOK");
    
    
    
  }
  else {
    Serial1.println("\tJoiningfailed");
  }
  printNodeData();
}

void processingRelayRefresh(long int MAC1,long int MAC2) {
  byte relay_temp1;
  boolean option;
  int len_param;
  int method;
  char purestring[1];
  boolean wait_ack;
  
  // extract dev id, from slot params[0]
  relay_temp1 = searchDevIdBySlotandMAC(MAC1,MAC2,ppzigbee.params[0],2); // saving type (1): MAXQ, (2): Rele, (3):SenTemp, (4):SenLighting
  if (relay_temp1==0) { // check if node is registered in gateway
    Serial1.println("\t(RELAY)dev not found. TODDING");
    processingTOD(MAC1,MAC2);
    relay_temp1 = searchDevIdBySlotandMAC(MAC1,MAC2,ppzigbee.params[0],2); // saving type (1): MAXQ, (2): Rele, (3):SenTemp, (4):SenLighting
  }
  
  if (relay_temp1!=0) { // check if node is registered in gateway
    ppzigbee.params[0] = relay_temp1; // override slot with first devid founded;
    // build server packet with relays state
    len_param   = 13;
    purestring[0] = 0;
    buildPacketToSendToServer(RELE_REFRESH_METHOD, ppzigbee.params, len_param, purestring, false, ppzigbee.opID);
    
    if (sendPacketToServerandParseResponse(ppzigbee.opID,false,RELE_REFRESH_METHOD)) { // send packet to server and wait for ack
      
      // building zigbee packet to confirm relay state
      payload[0]=RELE_REFRESH_METHOD;
      payload[1]=0;
      payload[2]=ppzigbee.opID;
      zbTx.setPayloadLength(3);

      // Send join packet to new node and wait for confirmation, if success, join node to network
      
      if (sendPacketToZigbeeNodeandParseResponse(ppzigbee.opID,ZIGBEE_WAITING_INTERVAL,MAC1,MAC2,1,false,RELE_REFRESH_METHOD)) { 
        // change state of node locally and process dev id from ppserver
        joinNode(MAC1,MAC2);
        Serial1.println("\t(RELAY)RelayOK"); 
      }
      else {
        Serial1.println("\t(RELAY)Noanswer");
      }            
    }
    else { 
      Serial1.println("\t(RELAY)Server timeout");
    }
    
  }
  else {
    Serial1.println("\t(RELAY)dev not found.Discard");
  }

}

boolean checkNewZigbeeRequest() {
  unsigned long MAC1, MAC2;
  byte s;
  xbee.readPacket(1000); // Ver si hay paquetes zigbee por atender
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      xbee.getResponse().getZBRxResponse(rx); 
      Serial1.print("(ZIGBEE)\t\t\n\tZigbee received (");Serial1.print(rx.getRemoteAddress64().getMsb(),DEC);Serial1.print("-");Serial1.println(rx.getRemoteAddress64().getLsb(),DEC);

      MAC1=rx.getRemoteAddress64().getMsb();
      MAC2=rx.getRemoteAddress64().getLsb();
      s = getnodeDefinitionFromMAC(MAC1,MAC2);
      
      if (s==254) {
        Serial1.println("\t(ZIGBEE) Node not registered in gateway, syncing");
        processingTOD(MAC1,MAC2);
      }
      else {
        Serial1.print("\t(ZIGBEE) Node Index:");Serial1.println(s,DEC);
      }
      return true;
     }
  }
  if (managerQueue.ZBwaiting > 0) {
    return true;
  }
  
  return false;
}

boolean checkNewTCPRequest() {
  if (client.available() || managerQueue.SRVwaiting > 0) {
    Serial1.println("(TCP/IP)\t\t\n\tTCP request available");
    return true;
  }
  else {
    return false;
  }
}

void processingMAXQSample(long int MAC1,long int MAC2) {
  Serial1.println("\t(MAXQ)Paquete Zigbee");
  byte index;
  char purestring[] = {0};
  boolean valid = false;
  byte devid;
  if (ppzigbee.params[0] == 1) {
    Serial1.println("\t(MAXQ)Procesando inicio");
    
    devid = searchDevIdBySlotandMAC(MAC1,MAC2, ppzigbee.params[0], MAXQ_TYPE);
    
    _MAXQBuffer.signals[0] = devid;
    _MAXQBuffer.current_index = 1;
    valid = true;
    _MAXQBuffer.ready = false;
  }
  else {
    if (ppzigbee.params[0] == _MAXQBuffer.last_part + 1) { // append new part to current MAXQ buffer
      valid = true;
    } 
    else { // not synced, miss maxq part waiting for the first one again
      Serial1.println("\t(MAXQ)Miss part");
      _MAXQBuffer.ready = false;
    }
  }
  
  if (valid) {
     _MAXQBuffer.last_part = ppzigbee.params[0];
    
    byte y;
    
    for (y=0;y<ppzigbee.len_params-2;y++) {
      _MAXQBuffer.signals[_MAXQBuffer.current_index+y] = ppzigbee.params[y+2];
      Serial1.print(_MAXQBuffer.signals[_MAXQBuffer.current_index+y],DEC);Serial1.print(",");
    }
    
    _MAXQBuffer.current_index = _MAXQBuffer.current_index + y;
    

    //printuint8ArrayasDEC(_MAXQBuffer.signals,_MAXQBuffer.current_index);
    
    if (ppzigbee.params[1] == MAXQ_END_STREAM) {
      _MAXQBuffer.ready = true;

      // enviar al servidor no funciona
      buildPacketToSendToServer(ppzigbee.method,_MAXQBuffer.signals, _MAXQBuffer.current_index, purestring, false, ppzigbee.opID);
      if (sendPacketToServerandParseResponse(ppzigbee.opID,true,ppzigbee.method)) {
        Serial1.println("\t(MAXQ) Data synced");
        // confirmar al nodo maxq para que borre el registro
        ppzigbee.params[0] = 0;
        ppzigbee.params[1] = 1;
        ZBTX(ppzigbee.method,ppzigbee.params,0,2,ppzigbee.opID,MAC1,MAC2);
      }
      else {
        // always confirm maxq packet reception reception 
        ppzigbee.params[0] = 0;
        ppzigbee.params[1] = 1;
        ZBTX(ppzigbee.method,ppzigbee.params,0,2,ppzigbee.opID,MAC1,MAC2);
      }
      
    }
    
  }

    
      
}


void printNodeData() {
    Serial1.println("\t(NODE DEFINITION)");
    for (int g=0;g<MAX_NODES;g++) {
      if (nodeDefinition[g].MAC1 != 0 && nodeDefinition[g].MAC2 != 0) {
        Serial1.print("\t\tMAC: ");Serial1.print(nodeDefinition[g].MAC1);Serial1.print(" - ");Serial1.println(nodeDefinition[g].MAC2);
        Serial1.print("\t\tSlots: ");Serial1.println(nodeDefinition[g].slotsnum,DEC);
        Serial1.print("\t\tisOnline: ");Serial1.println((int)nodeDefinition[g].isonline,DEC);
        for (int h=0;h<nodeDefinition[g].slotsnum;h++) {
          Serial1.print("\t\t\tslot: ");Serial1.println(h+1,DEC);
          for (int u=0;u<IDS_NODES_MAX;u++) {
            if (nodeDefinition[g].IDs[h][u][0] !=0) { // print online used dev ids
              Serial1.print("\t\t\t\tid :");Serial1.print(nodeDefinition[g].IDs[h][u][0],DEC);Serial1.print("\ttype :");Serial1.println(nodeDefinition[g].IDs[h][u][1],DEC);
            }
          }
        }
      }
    }

}

void initializeStructs() {
  // clean nodes
  for (int f=0;f<MAX_NODES;f++) {
    nodeDefinition[f].id = 0;
    nodeDefinition[f].MAC1 = 0;
    nodeDefinition[f].MAC2 = 0;
    nodeDefinition[f].isonline = false;
    nodeDefinition[f].slotsnum = 0;
    for(int s = 0; s < SLOTS_NODES_MAX; s++) {
      for(int i = 0; i < IDS_NODES_MAX; i++) {
        for(int j = 0; j < 2; j++) nodeDefinition[f].IDs[s][i][j] = 0; // cleaning dev id and type
      }
    }
  }
  // clean queue manager
  
  managerQueue.producer_ZB = 0;
  managerQueue.consumer_ZB = 0;
  managerQueue.producer_SRV = 0;
  managerQueue.consumer_SRV = 0;

  managerQueue.ZB_is_full = false;
  managerQueue.SRV_is_full = false;
  managerQueue.ZBwaiting = 0;
  managerQueue.SRVwaiting = 0;
  
  // clean asynchronous measurements buffer
  for (byte r=0;r<MAX_MEASUREMENT_BUFFER;r++){ 
    measurementsBuffer[r].used = false;
  }
}

byte getnodeDefinitionFromMAC(long int MAC1,long int MAC2) {
  for (int f=0;f<MAX_NODES;f++) {
    if (MAC1 == nodeDefinition[f].MAC1 && nodeDefinition[f].MAC2 == MAC2) {
      return f;
    }
  }
  return 254; // error code, node not found
}

byte getnodeDefinitionFromdevID(byte devID) {
  for (int f=0;f<MAX_NODES;f++) {
    if (0 != nodeDefinition[f].MAC1 && nodeDefinition[f].MAC2 != 0) {
      for (int k=0;k<nodeDefinition[f].slotsnum;k++) {
        for (int u=0;u<IDS_NODES_MAX;u++) {
          if (nodeDefinition[f].IDs[k][u][0] == devID) {
            return f;
          }
        }
      }
    }
  }
  return 255; // error code for device not found
}

byte getAvailableIndexforNewNodeDefinition() {
  for (int a=0;a<MAX_NODES;a++) {
    if (0 == nodeDefinition[a].MAC1 && nodeDefinition[a].MAC2 == 0) {
      return a;
    }
  }
  return 254; // error code, array of node is full
}

void changeNodeState(byte nodeindex, boolean state) {
  if (nodeDefinition[nodeindex].MAC1 != 0 && nodeDefinition[nodeindex].MAC2 != 0) {
    nodeDefinition[nodeindex].isonline = state;    
  }
  else {
    Serial1.print("\tNode is not registered");
  }
  printNodeData();
}

void checkUnconnectedEthernetCable() {
  if (last_time_sent_echo > millis()) {
    last_time_sent_echo = millis();
  }
  if (millis() - last_time_sent_echo > INTERVAL_ECHOS) {
    // send echo
    byte len_param   = 1; // at least a blank param
    byte params[] = {0};
    byte method  = ECHO_METHOD; // echo
    char purestring[] = {0};
    boolean option = false; // send bytes no strings from purestring
    boolean wait_ack = true;
    byte opID = 77;
    buildPacketToSendToServer(method, params, len_param, purestring, option, opID);
    
    if (sendPacketToServerandParseResponse(opID,wait_ack,method)) { // send packet to server and wait for ack
      Serial1.println("\t(CONN CHECK)cable plugged");
    }
    else {
      Serial1.println("\t(CONN CHECK)cable unplugged");
    }
 
    last_time_sent_echo = millis();

    Serial1.print("\t(Running from:");Serial1.print(last_time_sent_echo/1000,DEC);Serial1.println(" seg)");
  }
}


void processingASYNC(long int MAC1,long int MAC2, byte type) {
    int method  = ASYNC_NODE_METHOD;
    char purestring[1] = {0};
    byte devid;
    byte typeofdevice;
    byte shift=0;
    boolean inv;
    // type_dict = {'RLS8':2,'MAXQ':1,'TMP':3,'LGH':4}
    if (type == 1) { // slot 1 has TMP
      typeofdevice = TEMP_TYPE;
      inv = false;
      shift = 2;
    }else { // slot 2 has LGH
      typeofdevice = LIGHT_TYPE;      
      inv = true;
      shift = 0;
    }
    
    for (byte t=0;t<MAX_SIGNALS_MEASUREMENT_BUFFER;t++) {
      if (inv) {
        if ((ppzigbee.params[1+t] == 255) || (ppzigbee.params[1+t] == 0)) {
          return;
        }
        else {
          ppzigbee.params[1+t] = 255-(ppzigbee.params[1+t]>>shift); //
        }

      }
      else {
        ppzigbee.params[1+t] = (ppzigbee.params[1+t]>>shift); //         
      }
    }
        
    devid = searchDevIdBySlotandMAC(MAC1,MAC2, ppzigbee.params[0], typeofdevice);
    if (devid == 0) { // not found, has to server for its definition
      Serial1.println("\t(ASYNC)AskNodeDefinitionToServer");
      processingTOD(MAC1,MAC2);
      devid = searchDevIdBySlotandMAC(MAC1,MAC2, ppzigbee.params[0], typeofdevice);
      if (devid == 0) {
        Serial1.println("\t(ASYNC)NodeNotRegistered");
        return;
      }
    }
    
    ppzigbee.params[0] = devid;
    // store measurement locally in RAM for control porpuses
    boolean founded = false; // if there measurement buffer slot has been found
    byte index_available = MAX_MEASUREMENT_BUFFER + 1;
    
    for (byte y=0;y<MAX_MEASUREMENT_BUFFER;y++) {
      if (measurementsBuffer[y].devid == devid && measurementsBuffer[y].used) {
        for (byte i=0;i<MAX_SIGNALS_MEASUREMENT_BUFFER;i++) {
          measurementsBuffer[y].measurement[i] = ppzigbee.params[1+i]; // 
        }
        measurementsBuffer[y].time = millis();
        founded = true;
        break;
      }
      if (!measurementsBuffer[y].used) {index_available = y;}
    }

    if (!(founded)) {
      if (index_available != MAX_MEASUREMENT_BUFFER + 1) {
        for (byte p=0;p<MAX_SIGNALS_MEASUREMENT_BUFFER;p++) {
          measurementsBuffer[index_available].measurement[p] = ppzigbee.params[1+p]; // 
        }
        measurementsBuffer[index_available].used = true;
        measurementsBuffer[index_available].devid = devid;
        measurementsBuffer[index_available].time = millis();
      }
      else {
        Serial1.println("t(SYNC)NoBuffer");
      }
    }
    
    for (byte f=0;f<MAX_MEASUREMENT_BUFFER;f++) {
      if (measurementsBuffer[f].used) {
        Serial1.print("\t(SYNCBUFF)devid:");Serial1.print(measurementsBuffer[f].devid,DEC);Serial1.print(" time:");Serial1.print(measurementsBuffer[f].time,DEC);Serial1.print(" data:");printuint8ArrayasDEC(measurementsBuffer[f].measurement,MAX_SIGNALS_MEASUREMENT_BUFFER);
      }
    }
    
    buildPacketToSendToServer(method, ppzigbee.params, ppzigbee.len_params, purestring, false, ppzigbee.opID);
    
    if (sendPacketToServerandParseResponse(ppzigbee.opID,false,method)) { // send packet to server and wait for ack
      Serial1.println("\t(ASYNC)SyncedWithServerOK");
    }
    else {
      Serial1.println("\t(ASYNC)NotSynced");
    }
    
    Serial1.println("\t(ASYNC)SendtoRelays");
    for (byte g=0;g<MAX_NODES;g++) {
      for (int slot=0;slot<nodeDefinition[g].slotsnum;slot++) {
        for (int u=0;u<IDS_NODES_MAX;u++) {
          if (nodeDefinition[g].IDs[slot][u][0] !=0 && nodeDefinition[g].IDs[slot][u][1] == 2) { // print online used dev ids
            Serial1.print("\t(ASYNC)Relay devid:");Serial1.println(nodeDefinition[g].IDs[slot][u][0],DEC);
            ZBTX(RELE_MEDICION_SENSORES_METHOD,ppzigbee.params,0,ppzigbee.len_params,ppzigbee.opID,nodeDefinition[g].MAC1,nodeDefinition[g].MAC2);
          }
        }
      }
    }   
}

boolean fillPayloadWithParams(byte index_start_payload,byte index_start_params,byte* params, int len_params) {
  for (byte index=index_start_payload;index<len_params+index_start_payload;index++) {
    if (index == PAYLOAD_MAX) {
      return false;
    }
    payload[index] = params[index+index_start_params-index_start_payload];
  }
  return true;
}

void sendErrorToServer() {
  byte params[1] = {0};
  char purestring[1] = {0};
  byte method_to_check_ack;
  buildPacketToSendToServer(69, params, 1, purestring, false, 99);
  if (sendPacketToServerandParseResponse(99,false,69)) {
    Serial1.println("\t(TCP)Error packet sent");
  }
  else {
    Serial1.println("\t(TCP)TX error");
  }
}

void processingControlPacket(byte type) {
  // find device
  byte method_to_check_ack;
  byte nodeindex = getnodeDefinitionFromdevID(ppserver.params[0]);
  if (nodeindex == 255) {
    Serial1.println("\t(NEW CONTROL PACKET)Device is not present");
    return;
  }


  // prepare payload
  switch (type) {
    case 1:
      Serial1.println("\t(NEW CONTROL PACKET) MANUAL CONTROL");
      payload[0]=ppserver.method;
      method_to_check_ack = ppserver.method;
      break;
    case 2:
      Serial1.println("\t(NEW CONTROL PACKET) CONTROL BY RULE");
      payload[0]=ppserver.method;
      method_to_check_ack = ppserver.method;
      break;
  }
  
  
  // if ppserver.params is higher than aMAX payload, send error to server
  if (!(fillPayloadWithParams(1,1,ppserver.params,ppserver.len_params-1))) {
    Serial1.println("\t(NEW CONTROL PACKET) Payload overflow");
    sendErrorToServer();
    return;
  }
  
  payload[ppserver.len_params] = ppserver.opID;
  zbTx.setPayloadLength(ppserver.len_params+1);

  // send packet to target and wait for ack
  if (sendPacketToZigbeeNodeandParseResponse(ppserver.opID,ZIGBEE_WAITING_INTERVAL,nodeDefinition[nodeindex].MAC1,nodeDefinition[nodeindex].MAC2,1,true,method_to_check_ack)) { 
    Serial1.println("\t(NEW CONTROL PACKET) Rule saved on target");
    // send confirmation to server
    byte params[1] = {0};
    char purestring[1] = {0};
    buildPacketToSendToServer(5, params, 1, purestring, false, ppserver.opID);
    if (sendPacketToServerandParseResponse(ppserver.opID,false,5)) {
      Serial1.println("\t(NEW CONTROL PACKET)Updated on target OK");
    }
    else {
      Serial1.println("\t(NEW CONTROL PACKET)Can not send ack to server");
    }
  }
  else {
    Serial1.println("\t(NEW CONTROL PACKET)Can not send rule to target");     
  }

}

void processingMeasRequest(long int MAC1,long int MAC2) {
  // extract dev
  boolean founded = false;
  byte params[2];
  byte signal_requested;
  byte index_founded;
   char purestring[1]={0};
  for (byte r=0;r<MAX_MEASUREMENT_BUFFER;r++) {
    if (ppzigbee.params[0] == measurementsBuffer[r].devid && measurementsBuffer[r].used) {
      index_founded = r;
      break;
    }
  }
  if (founded) {
    if (measurementsBuffer[index_founded].time > millis()) {
      measurementsBuffer[index_founded].time = millis();
    }
    if (millis() - measurementsBuffer[index_founded].time < MAX_AGE_OF_A_MEASUREMENT_BEFORE_EXPIRE) {
      signal_requested = measurementsBuffer[index_founded].measurement[ppzigbee.params[1]];
      Serial1.print("\t(CONTROL MEAS)Data found, sending:");Serial1.println(signal_requested,DEC);
      params[0] = signal_requested;
      ZBTX(ppzigbee.method,params,0,1,ppzigbee.opID,MAC1,MAC2);
      

    }
    else {  // measurement founded in RAM is too old, maybe the connection is lost with node, send notification to relay
      
      Serial1.println("\t(CONTROL MEAS)Data old, maybe sensor is lost");
      params[0] = 255;
      params[1] = 255;
      ZBTX(ppzigbee.method,params,0,2,ppzigbee.opID,MAC1,MAC2);
    }
  }
  else {
    params[0] = ppzigbee.params[0];
    params[1] = ppzigbee.params[1];

    buildPacketToSendToServer(ppzigbee.method, params, 2, purestring, false, ppzigbee.opID);
    // ask data to the server (this could happen if gateway loss data after a reset
    if (sendPacketToServerandParseResponse(ppzigbee.opID,true,ppzigbee.method)) { // sending to server
      signal_requested = ppserver.params[0];
      Serial1.print("\t(CONTROL MEAS)Data found in server, sending:");Serial1.println(signal_requested,DEC);
      params[0] = signal_requested;
      ZBTX(ppzigbee.method,params,0,1,ppzigbee.opID,MAC1,MAC2);
    }
    else {
      Serial1.println("\t(CONTROL MEAS)Data not found in server");
      params[0] = 255;
      params[1] = 255;
      ZBTX(ppzigbee.method,params,0,2,ppzigbee.opID,MAC1,MAC2);
    }
  }
}

boolean restartEthernet() {
  #if (DHCP) 
    boolean dhcp_ready = false;  
  #else
    boolean dhcp_ready = true;  
  #endif
  
  boolean server_resolve = false;
  
  Serial1.println("\t(FORCING ETHERNET RESET)");
  digitalWrite(Ethernet_reset_pin,LOW); // put reset pin to low ==> reset the ethernet shield
  delay(200);
  digitalWrite(Ethernet_reset_pin,HIGH); // set it back to high
  client.stop();
  #if (DHCP)
    EthernetDHCP.begin(mac,1);
  #else
    Ethernet.begin(mac,ip,gateway,subnet);
  #endif
  
  for (int r=0;r<10;r++) {
    Serial1.println(r,DEC);
    #if (DHCP)
      dhcp_ready = checkDHCP();
    #endif

    if (dhcp_ready) {
      break;
    }
  }
  
  if (dhcp_ready) {
    server_resolve = resolveServerIP();
  }
  
  if (server_resolve && dhcp_ready) {
    return true;
  }
  else {
    return false;
  }
}

void processingTimeRequest(long int MAC1,long int MAC2) {
  byte params[6]; // slot (rputing byte + date (3 byte) + time (2 byte)
  char purestring[1];
  params[0] = 0;
  buildPacketToSendToServer(ppzigbee.method, params, 1, purestring, false, ppzigbee.opID);
  // ask data to the server (this could happen if gateway loss data after a reset
  if (sendPacketToServerandParseResponse(ppzigbee.opID,true,ppzigbee.method)) { // sending to server
    Serial1.print("\t(SYNC DATETIME)Time, sending:");printArrayasDEC(ppserver.params,5);
    //params[0] = ppzigbee.params[0]; // slot to route packet bac to node
    for (byte y=0;y<ppserver.len_params;y++) {
      params[y] = ppserver.params[y];
    }
    ZBTX(ppzigbee.method,params,0,5,ppserver.opID,MAC1,MAC2);
  }
}

void printPendantRequest(byte pendant_request_id,boolean type) {
  if (type) { // ZB request
    Serial1.print("\t\tMethod:");Serial1.println(zigbeeQueue[pendant_request_id].ppzigbee.method,DEC);
    Serial1.print("\t\tParams:");printArrayasDEC(zigbeeQueue[pendant_request_id].ppzigbee.params,zigbeeQueue[pendant_request_id].ppzigbee.len_params);
    Serial1.print("\t\tOpID:");Serial1.println(zigbeeQueue[pendant_request_id].ppzigbee.opID,DEC);
  }
  else { // SRV request
    Serial1.print("\t\tMethod:");Serial1.println(serverQueue[pendant_request_id].ppserver.method,DEC);
    Serial1.print("\t\tParams:");printArrayasDEC(serverQueue[pendant_request_id].ppserver.params,serverQueue[pendant_request_id].ppserver.len_params);
    Serial1.print("\t\tOpID:");Serial1.println(serverQueue[pendant_request_id].ppserver.opID,DEC);
  }

}

void pushToQueue(byte queueid) { // add new element to queue
  // check if there are slots available
  byte params[1] = {0};
  switch (queueid) {
    case 1: // ZB queue
      if (managerQueue.ZB_is_full) {
        Serial1.print("(QUEUE ZB) No more request are available.. wait");
        //sendPacketToZigbee(BUSY_METHOD,params,1,ppzigbee.opID);
        return;
      }
      zigbeeQueue[managerQueue.producer_ZB].ppzigbee = ppzigbee;
      if (managerQueue.producer_ZB == ZB_QUEUE_MAX - 1) {
        managerQueue.producer_ZB = 0;
      }
      else {
        managerQueue.producer_ZB++;        
      }
      managerQueue.ZBwaiting++;
      Serial1.print("managerQueue.ZBwaiting:");Serial1.println(managerQueue.ZBwaiting,DEC);
      Serial1.print("ZB Produces/Consumer:");Serial1.print(managerQueue.producer_ZB,DEC);Serial1.print("/");Serial1.println(managerQueue.consumer_ZB,DEC);
      // check if it's full
      if (managerQueue.ZBwaiting == ZB_QUEUE_MAX) {
        managerQueue.ZB_is_full = true;
      }
      //if (managerQueue.ZBwaiting + 1 == ZB_QUEUE_MAX) managerQueue.ZB_is_full = true;   
      break;
    case 2: // SRV queue
      if (managerQueue.SRV_is_full) {
        Serial1.println("\t(QUEUE SERVER) No more request are available.. wait");
        //sendPacketToServer(BUSY_METHOD,params,1,ppserver.opID); 
        return;
      }
      serverQueue[managerQueue.producer_SRV].ppserver = ppserver;
      if (managerQueue.producer_SRV == SRV_QUEUE_MAX - 1) {
        managerQueue.producer_SRV = 0;
      }
      else {
        managerQueue.producer_SRV++;        
      }
      managerQueue.SRVwaiting++;
      Serial1.print("managerQueue.SRVwaiting:");Serial1.println(managerQueue.SRVwaiting,DEC);
      Serial1.print("SRV Produces/Consumer:");Serial1.print(managerQueue.producer_SRV,DEC);Serial1.print("/");Serial1.println(managerQueue.consumer_SRV,DEC);
      // check if it's full
      //if (managerQueue.SRVwaiting + 1 == SRV_QUEUE_MAX) managerQueue.SRV_is_full = true;       
      if (managerQueue.SRVwaiting == SRV_QUEUE_MAX) {
        managerQueue.SRV_is_full = true;
      }
      
      break;
  }
}

void processingEECmd() {
  byte node = getnodeDefinitionFromdevID(ppserver.params[0]);
  if (node == 255) {// no esta conectado 
    return;
  }
  else {
    ZBTX(ppserver.method,ppserver.params,0,ppserver.len_params,ppserver.opID,nodeDefinition[node].MAC1,nodeDefinition[node].MAC2);  
  }
}



void passingPacketToNode() {
  byte node = getnodeDefinitionFromdevID(ppserver.params[0]);
  if (node == 255) {// no esta conectado 
    Serial1.println("\t node not linked");
    return;
  }
  else {
    if (ppserver.len_params-1 > 69) {
      Serial1.println("\t(PassPckExceedMax");
      return;
    }
    ZBTX(ppserver.method,ppserver.params,1,ppserver.len_params-1,ppserver.opID,nodeDefinition[node].MAC1,nodeDefinition[node].MAC2);  
  }
}

void confirmRuleSaved() {
  char purestring[1];
  buildPacketToSendToServer(ppzigbee.method, ppzigbee.params, ppzigbee.len_params, purestring, false, ppzigbee.opID);
  // ask data to the server (this could happen if gateway loss data after a reset
  if (sendPacketToServerandParseResponse(ppzigbee.opID,false,ppzigbee.method)) { // sending to server
    Serial1.print("\t(AckRuleSent)");
  }  
}


void checkQueueandServe() {
  long int MAC1;
  long int MAC2;
  // preload ppzigbee and ppserver if if something pendant
  if (managerQueue.SRVwaiting>0 || managerQueue.ZBwaiting>0) {
    if (managerQueue.SRVwaiting > 0) {
      Serial1.print("\t(SERVING SRV QUEUE) ");Serial1.print(managerQueue.SRVwaiting,DEC);Serial1.println(" TCP request pendant");
      ppserver = serverQueue[managerQueue.consumer_SRV].ppserver;
    }
    if (managerQueue.ZBwaiting > 0) {
      Serial1.print("\t(SERVING ZB QUEUE) ");Serial1.print(managerQueue.ZBwaiting,DEC);Serial1.println(" ZIGBEE request pendant");
      // refresh pointers
      ppzigbee = zigbeeQueue[managerQueue.consumer_ZB].ppzigbee;
      Serial1.println("\t(SERVING ZB QUEUE) Pendant Payload:");printPendantRequest(managerQueue.consumer_ZB,true);
      MAC1 = nodeDefinition[zigbeeQueue[managerQueue.consumer_ZB].nodeindex].MAC1;
      MAC2 = nodeDefinition[zigbeeQueue[managerQueue.consumer_ZB].nodeindex].MAC2;
    }
  }

  
  if (checkNewZigbeeRequest()) { // TOD, Relay Request, Asynchronous Measurements, MAXQ, pendant request
    // id from sender

    // parsea dato zigbee y extre ppzigbee.method, ppzigbee.opID, ppzigbee.params
    if (managerQueue.ZBwaiting == 0) { // prioritize pendant queue
      ppzigbee = parseZigbeePacket(rx.getData(), rx.getDataLength()); 
      MAC1=rx.getRemoteAddress64().getMsb();
      MAC2=rx.getRemoteAddress64().getLsb();
    } else { // once the pendant payload has been charge, refresh consumer pointer
      if (managerQueue.consumer_ZB == ZB_QUEUE_MAX - 1) {
        managerQueue.consumer_ZB = 0;
      }
      else {
        managerQueue.consumer_ZB++;        
      }
      managerQueue.ZBwaiting--;
      
      if (managerQueue.ZBwaiting < ZB_QUEUE_MAX) {
        managerQueue.ZB_is_full = false;
      }
    }
    

    switch (ppzigbee.method) {
      case TOD_METHOD: // process new node trying to join to zigbee network, if valid, server will send opIDs
//        Serial1.println("\t(TOD)");
//        // build TOD request to server, attaching mac, Server must return devs_id packet
//        processingTOD(MAC1,MAC2);
//        break;
      case MAXQ_GET_NOW_SAMPLE_METHOD: // Max Q Nodes
        Serial1.println("\t(MAXQ)");
        processingMAXQSample(MAC1,MAC2);
        break;
      case RELE_REFRESH_METHOD: // Node will send this method if one or more Relays change its state
        Serial1.println("\t(RELAYS)");
        // check if sender is registered as online
        processingRelayRefresh(MAC1,MAC2);
        break;
      case RELE_REGLA_MANUAL_METHOD:
      case RELE_REGLA_CONTROL_METHOD:
        confirmRuleSaved();
        break;
      case MAXQ_UPDATE_DATETIME_METHOD: // if a node ask for time
        Serial1.println("\t(ASKING TIME)");
        processingTimeRequest(MAC1,MAC2);
        break;
      case RELE_UPDATE_DATETIME_METHOD: // if a node ask for time
        Serial1.println("\t(ASKING TIME)");
        processingTimeRequest(MAC1,MAC2);
        break;
      case ASYNC_NODE_METHOD:
        if (ppzigbee.params[0] == 1) {
          Serial1.println("\t(TEMP)");
          processingASYNC(MAC1,MAC2,1); // 1: temp , then broadcast to relays
        }
        if (ppzigbee.params[0] == 2) {
          Serial1.println("\t(LIGHT)");
          processingASYNC(MAC1,MAC2,2); // 2: light, then broadcast to relays
        }
        break;
      default:
        Serial1.println("\tZigbee Method not recognized:");
        break;
    }

  }
  
  
  /*********************************************************/
  /************ PRECESA NUEVOS PAQUETES TCP ****************/
  /*********************************************************/
  
  if (checkNewTCPRequest()) { // check if there is a pendant request or a request in ethernet buffer
    if (managerQueue.SRVwaiting == 0) { // prioritize pendant queue
      ppserver = parseServerPacket2();
    } else {
      // refresh pointers once the pendant request has been load in context
      if (managerQueue.consumer_SRV == SRV_QUEUE_MAX - 1) {
        managerQueue.consumer_SRV = 0;
      }
      else {
        managerQueue.consumer_SRV++;        
      }
      managerQueue.SRVwaiting--;
      
      if (managerQueue.SRVwaiting < SRV_QUEUE_MAX) {
        managerQueue.SRV_is_full = false;
      }
    }
    
    switch (ppserver.method) {
      case ASYNC_NODE_METHOD:
        Serial1.println("\t(ASYNCSyncAck)");
        break;
      case RELE_REFRESH_METHOD:
        Serial1.println("\t(RELEsynced)");
        break;
      case RELE_REGLA_MANUAL_METHOD: // manual control
        Serial1.println("\t(NEWMANUAL)");
        processingControlPacket(1); 
        break;
      case RELE_REGLA_CONTROL_METHOD: // control rule
        Serial1.println("\t(NEWRULE)");
        processingControlPacket(2);
        break;
      case RELE_CLEAN_EEPROM_METHOD: // clear eeprom
        Serial1.println("\t(RELE_EE)");  
        passingPacketToNode();
        break;
      case MAXQ_INIT_EEPROM: // send packet directly to maxq, no wait 
        Serial1.println("\t(MAXQ_EE)");
        passingPacketToNode();
        break;
      case MAXQ_UPDATE_CALI_PARAMS_METHOD:
        Serial1.println("\t(MAXQ_CALI)");
        passingPacketToNode();
        break;
      case RESET_SLAVES_METHOD:
        Serial1.println("\t(RESET SLAVES)");
        passingPacketToNode();
        break;
      default:
        Serial1.println("\tTCP Method not recognized:");
        break;
    }
  }
  
  
}




