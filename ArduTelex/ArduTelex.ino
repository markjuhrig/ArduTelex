#include <Arduino.h>
#include "ArduTelex.h"
#include "baudot.h"
#include "lookup.h"
#include <Time.h>
#include <TimeLib.h>
#include <EthernetUdp.h>

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = { MYMACADRESS };

//Set the static IP address to use if the DHCP fails to assign
IPAddress ip(MYIPFALLBACK);
// Initialize the Ethernet client library
// with the IP address and port of the server
#ifndef CENTRALEX
  EthernetServer server(MYPORT);
#endif
EthernetClient client, purgeclient;
byte currentSocknum;


// These define's must be placed at the beginning before #include "TimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0

#define USE_TIMER_1     true
#if ( defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)  || \
        defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_MINI) ||    defined(ARDUINO_AVR_ETHERNET) || \
        defined(ARDUINO_AVR_FIO) || defined(ARDUINO_AVR_BT)   || defined(ARDUINO_AVR_LILYPAD) || defined(ARDUINO_AVR_PRO)      || \
        defined(ARDUINO_AVR_NG) || defined(ARDUINO_AVR_UNO_WIFI_DEV_ED) || defined(ARDUINO_AVR_DUEMILANOVE) || defined(ARDUINO_AVR_FEATHER328P) || \
        defined(ARDUINO_AVR_METRO) || defined(ARDUINO_AVR_PROTRINKET5) || defined(ARDUINO_AVR_PROTRINKET3) || defined(ARDUINO_AVR_PROTRINKET5FTDI) || \
        defined(ARDUINO_AVR_PROTRINKET3FTDI) )
  #define USE_TIMER_2     true
  #warning Using Timer1
#else          
  #define USE_TIMER_3     true
  #warning Using Timer3
#endif
#include "TimerInterrupt.h"

#define START_TIMER1  {TCCR1B=9;}
#define STOP_TIMER1   {TCCR1B=0;}
#define TIMER1ZERO    {TCNT1L=0;TCNT1H=0;}

volatile byte    bit_no;
volatile byte    tx_byte;
byte state;       // state of state machine

byte currentDigit;
byte currentNumberPos;
#define NUMBER_BUFFER_LENGTH 20
char number[NUMBER_BUFFER_LENGTH];
String Nummer = " ";

#define TCPIP_DATA_BUFFER_LENGTH 50
//volatile 
byte TCPIPDataBuffer[TCPIP_DATA_BUFFER_LENGTH];
#define BAUDOT_DATA_BUFFER_LENGTH 50
volatile byte BaudotBuffer[BAUDOT_DATA_BUFFER_LENGTH];
volatile byte BDData;
volatile byte BDBufferRDPTR = 0;
volatile byte BDBufferWRPTR = 0;
volatile byte RecVersion = 0;
volatile boolean new_char_recieved=false;
volatile boolean new_TCPIP_data_recieved=false;
volatile boolean IamServer = false;
volatile int TCPData;
volatile int NoIPBytes;

volatile byte CharSent  = 0;    // Characters sent to TTY

unsigned int Input;
unsigned int tmp;

volatile byte recieve_buf=0; // recieve buffer
volatile byte recieved_char; // recieved baudot character

unsigned long st_timestamp, con_timestamp, OneSecTimer;

volatile unsigned int bit_pos=0; // aktuelles bit in rcve, startbit=1
volatile boolean recieving=false;
volatile unsigned long last_timestamp;

enum CONNECTION_STATE{CON_NOT_ESTABLISHED = 0 , CON_ESTABLISHED = 1, CON_ESTABLISHED_TIME_SENT = 2 };
volatile byte ConnectionState = CON_NOT_ESTABLISHED;

// variables for Time
//unsigned long interval=30*60UL*1000;
//unsigned long interval_min;
// NTP Servers:
//IPAddress timeServer(178,63,61,67); // 
IPAddress timeServer(132,163,96,3);
//const int timeZone = 2;      // Central European Time Sommerzeit
const int timeZone = 1;     // Central European Time
EthernetUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

#ifdef CENTRALEX
  boolean cmdMode = false;
  unsigned long RS_Heartbeat_timestamp;
  unsigned long RS_Client_timestamp;
  //byte RS_ConCnt;
  byte RS_state;                    // for remote server state machine
#endif

byte putbuffer (byte item) 
{
  if (((BDBufferWRPTR + 1) % BAUDOT_DATA_BUFFER_LENGTH) == BDBufferRDPTR)
  {
     // buffer is full, avoid overflow
     return 0;
  }
  BaudotBuffer[BDBufferWRPTR] = item;
  BDBufferWRPTR = (BDBufferWRPTR + 1) % BAUDOT_DATA_BUFFER_LENGTH;
  #ifdef _DEBUG_RXTX
      PgmPrint("PR:");
      Serial.print(BDBufferRDPTR,DEC);
      PgmPrint("W:");
      Serial.print(BDBufferWRPTR,DEC);
      PgmPrint("D:");
      Serial.print(item,DEC);      
    #endif  
  return 1;
}

byte getbuffer (byte * value) 
{
  if (BDBufferRDPTR == BDBufferWRPTR)
  {
     // buffer is empty
     return 0;
  }
  *value = BaudotBuffer[BDBufferRDPTR];
  BDBufferRDPTR = (BDBufferRDPTR + 1) % BAUDOT_DATA_BUFFER_LENGTH;
  #ifdef _DEBUG_RXTX
      PgmPrint("GR:");
      Serial.print(BDBufferRDPTR,DEC);
      PgmPrint("W:");
      Serial.print(BDBufferWRPTR,DEC);
      PgmPrint("D:");
      Serial.print(*value,DEC);
    #endif
  return 1;
}

void byte_send_no_IRQ(byte _byte) {       // the no-IRQ TX-Routine may be used to print diagnostic messages on the TTY                
  if (recieving){                         // using the no-IRQ routing stops the main loop() such that only the message is printed
    recieving=false;
  }
  digitalWrite(SEND_PIN, DATA_LINE_CURRENT_OFF);         // start bit = space = line current off
  delay(STARTBIT_DURATION);
  for (int _bit = 4; _bit >= 0; _bit--) {   // reversed iTelex bit order!
    //    for (int _bit = 0; _bit <= 4; _bit++) {   // correct bit order
    #ifndef SOLID_STATE_TW39
      digitalWrite(SEND_PIN, !bitRead(_byte, _bit));      // 1 = Mark = Low
    #else
      digitalWrite(SEND_PIN, bitRead(_byte, _bit));      // 1 = Mark = High (solid state interface)
    #endif
      delay(DATABIT_DURATION);
  }
  digitalWrite(SEND_PIN, DATA_LINE_CURRENT_ON);         // stop bit = mark = line current on
  delay(STOPBIT_DURATION);
  recieving=true;

} // end byte_send_no_IRQ()

void sendAsciiAsBaudot(char ascii, bool IRQ) {        // IRQ = false ==> use byte_send_not_IRQ()
  byte modeswtichcode;
  byte baudot = asciiToBaudot(ascii,&modeswtichcode);

  if(baudot!=BAUDOT_CHAR_UNKNOWN){
    if(modeswtichcode!=BAUDOT_MODE_UNDEFINED){
      if (IRQ){
        byte_send(modeswtichcode);
      }
      else {
        byte_send_no_IRQ(modeswtichcode);
      }
    }
    if (IRQ){
      byte_send(baudot);
    }
    else {
      byte_send_no_IRQ(baudot);
    }
  }
} // sendAsciiAsBaudot()

void sendAsciiAsBDnTCPIP(char ascii, bool IRQ) {        // IRQ = false ==> use byte_send_not_IRQ()
  byte modeswtichcode;
  byte baudot = asciiToBaudot(ascii,&modeswtichcode);

  if(baudot!=BAUDOT_CHAR_UNKNOWN){
    if(modeswtichcode!=BAUDOT_MODE_UNDEFINED){
      if (IRQ){
        byte_send(modeswtichcode);
      }
      else {
        byte_send_no_IRQ(modeswtichcode);
      }
    sendBaudot(modeswtichcode);
    }
    if (IRQ){
      byte_send(baudot);
    }
    else {
      byte_send_no_IRQ(baudot);
    }
    sendBaudot(baudot);
  }
} // sendAsciiAsBDnTCPIP

void sendAsciitoTCPIP(char ascii) {  
  byte modeswtichcode;
  byte baudot = asciiToBaudot(ascii,&modeswtichcode);

  if(baudot!=BAUDOT_CHAR_UNKNOWN){
    PgmPrint("BD:");
    if(modeswtichcode!=BAUDOT_MODE_UNDEFINED){
      sendBaudot(modeswtichcode);
      Serial.print(modeswtichcode, DEC);
    }
    sendBaudot(baudot);
    Serial.print(baudot, DEC);
    PgmPrint(" ");
  }
} // sendAsciitoTCPIP

/**
   %Print a string in flash memory to the serial port.

   \param[in] str Pointer to string stored in flash memory.
*/
//static NOINLINE
#define PgmPrintBD(x) SerialPrint_BD(PSTR(x))
void SerialPrint_BD(PGM_P str) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) {
    sendAsciiAsBaudot((byte)c, false);
  }
}

void sendCMD2TCPIP(byte CMD, byte data) {
  byte buf[3];
  buf[0] = CMD;
  buf[1] = 1;     // lenght = 1
  buf[2] = data;
  client.write(buf, 3);
}

void sendHeartbeat(void) {
  byte buf[3];
  buf[0] = Heartbeat;
  buf[1] = 0;     // lenght = 0
  client.write(buf, 2);
}

void sendVersion(void) {
  byte buf[3];
  buf[0]=iTVersion;
  buf[1] = 1;     // lenght = 1
  buf[2] = 1;     // Version = 1
  client.write(buf, 3);
}

void sendACK(void) {
  byte buf[3];
  buf[0] = Acknowledge;
  buf[1] = 1;     // lenght = 1
  buf[2] = CharSent;
  client.write(buf, 3);
  #ifdef _DEBUG_RXTX
    PgmPrint("ACK:");
    Serial.print(CharSent,DEC);
    PgmPrint(" ");
  #endif 
}

void sendDD(byte DDNumber) {
  byte buf[3];
  buf[0] = DirectDial;
  buf[1] = 1;     // lenght = 1
  buf[2] = DDNumber;
  client.write(buf, 3);
}

void sendBaudot(byte BDdata) {
      byte buf[3];
      buf[0] = BaudotData;
      buf[1] = 1;     // lenght = 1
      buf[2] = BDdata;
      client.write(buf, 3);
}

void iTelexCommandParser (bool IamServ){
  switch (TCPIPDataBuffer[0]){
    case Heartbeat:
      // do nothing  
    break;
    case DirectDial: 
      #ifdef _DEBUG
      PgmPrintln("Direct Dial Command Received");
      #endif   
      //sendACK(); 
      ConnectionState = CON_ESTABLISHED;
    break;
    case BaudotData:
      // copy Baudot data into input buffer
      for (tmp = 0; tmp<TCPIPDataBuffer[1];tmp++ ){     // TCPIPDataBuffer[1] = length              
        putbuffer(TCPIPDataBuffer[2 + tmp]); 
      }
    #ifdef _DEBUG_RXTX
        PgmPrint("L:");
        Serial.print(TCPIPDataBuffer[1],DEC);
        PgmPrint("D:");
        Serial.print(TCPIPDataBuffer[2],DEC);
      #endif
    break;               
    case End:
    #ifdef _DEBUG
      PgmPrintln("End Command received, disconnecting");
      PgmPrintln("==>STATE_BUSY");
    #endif
      state=STATE_BUSY;
    break;       
    case Reject:  
    #ifdef _DEBUG
      PgmPrintln("End Reject received, disconnecting");
      PgmPrintln("==>STATE_BUSY");
    #endif
      state=STATE_BUSY;
    break;
    case Acknowledge:
      // do nothing  
    break;
    case iTVersion:   
      RecVersion = TCPIPDataBuffer[2];
    #ifdef _DEBUG
      PgmPrintln("Version Command Received");
    #endif 
    if (IamServ){
      sendVersion();
      #ifdef _DEBUG
        PgmPrintln("Send Version 1");
      #endif      
    }
    break;              
    case Loopback:   
      // do nothing  
    break;
    default:
    break;
  }
} // iTelexCommandParser

#ifdef USEIRQ
void onlinepinchange(){
    if(recieving && bit_pos==0){
        last_timestamp=millis();
        bit_pos++;
    }
 }
#endif

void TimerHandler1(void)  // Timer 1 interrupt handler for sending at 50Baud
{
  if (bit_no == 0){
    recieving=true;       // recieving allowed again
    STOP_TIMER1 ;
  }  
  if ((bit_no <= 7) && (bit_no >= 3)){                   // bit_no: 7...3 data bits, 2/1 stop bit, 0 end
    #ifndef SOLID_STATE_TW39
      digitalWrite(SEND_PIN, !bitRead(tx_byte, bit_no-3)); // 1 = Mark = Low
    #else
      digitalWrite(SEND_PIN, bitRead(tx_byte, bit_no-3));  // 1 = Mark = High (solid state interface)
    #endif
  }
  if (bit_no == 2){
    digitalWrite(SEND_PIN, DATA_LINE_CURRENT_ON);         // stop bit = mark = line current on
  }
  bit_no--;
} // End: Timer 1 interrupt handler for sending at 50Baud

void storeMainSocket(){
  for (int i = 0; i < MAX_SOCK_NUM; i++) {
    uint8_t s = W5100.readSnSR(i);
    if(s == 0x17) {
      currentSocknum=i;
    }
  }
}

#ifndef CENTRALEX
  void handleClientsWhileConnected(){ //check for new clients which try to connect while connection is already in progress
    for (int i = 0; i < MAX_SOCK_NUM; i++) {
      if(i!=currentSocknum){ //only the other ports
        uint8_t s = W5100.readSnSR(i);
        if(s == 0x17) { //connected -> disconnect
          purgeclient = server.available();
          if (purgeclient) {
              purgeclient.stop();
              #ifdef _DEBUG
                PgmPrintln("another TCPIP client had connected and was stopped!");
              #endif            
          }
        }
      }
    }
  }
#endif

void byte_send(byte _byte) {
  if (recieving){
    recieving=false;
  }
  tx_byte = _byte;
  bit_no = 7;                                             // 7..3 bits, 2/1 stop bit, 0 end iTelex bit order
  //  bit_no = 0;                                         // 0..4 bits, 5/6 stop bit, 7 end, correct bit order
  digitalWrite(SEND_PIN, DATA_LINE_CURRENT_OFF);          // start bit = space = line current off
  START_TIMER1
} // end byte_send()

byte ReceiveTCPIPCommand(void){
  TCPIPDataBuffer[0] = NoCommand; 
  if(client.connected()){
    int NoIPBytes = client.available();
    if (NoIPBytes > 0) {         // data has been received through TCPIP 
      if (NoIPBytes > TCPIP_DATA_BUFFER_LENGTH) {
        NoIPBytes = TCPIP_DATA_BUFFER_LENGTH;
      }
      #ifdef _DEBUG_RXTX
        PgmPrint("NoIPBytes: ");        
        Serial.print(NoIPBytes,DEC); 
        PgmPrint(" TCPIP-Data: ");        
      #endif
      for (byte temp = 0; temp<NoIPBytes;temp++ ){
        TCPIPDataBuffer[temp] = client.read();
        #ifdef _DEBUG_RXTX
        Serial.print(TCPIPDataBuffer[temp],HEX); 
          PgmPrint(" ");
        #endif
      }
      #ifdef _DEBUG_RXTX
        PgmPrintln(" ");
      #endif
      if ((NoIPBytes-2)>=TCPIPDataBuffer[1]){
        if (TCPIPDataBuffer[0] <= MaxCMD) {
          #ifdef _DEBUG_RXTX
            PgmPrint("C: ");        
            Serial.print(TCPIPDataBuffer[0],HEX);       
            PgmPrint(" ");        
          #endif
          return TCPIPDataBuffer[0];            // return command code
        }        
      } else {
        #ifdef _DEBUG
          PgmPrint("NoB:");
          Serial.print(NoIPBytes,DEC);
          PgmPrint(" iTl:");
          Serial.println(TCPIPDataBuffer[1],DEC);
        #endif
      }
      if ((NoIPBytes-2)<TCPIPDataBuffer[1]){
        #ifdef _DEBUG
          PgmPrint("NoB:");
          Serial.print(NoIPBytes,DEC);
          PgmPrint(" iTl:");
          Serial.println(TCPIPDataBuffer[1],DEC);
        #endif          
      }
    } else {
      return NoCommand;   // no command recieved
    }
  } else {
    return NotConnected; 
  }
}// end: ReceiveTCPIPCommand(void)

#ifdef CENTRALEX
  void CheckHeartbeat(void) {
    if (millis()>(RS_Heartbeat_timestamp+RSHeartbeat)){
      #ifdef _DEBUG_RXTX
        PgmPrint("Timestamp: ");
        Serial.print(RS_Heartbeat_timestamp, DEC);
        PgmPrint(" millis: ");
        Serial.print(millis(), DEC);
        PgmPrint(" RS_state: ");
        Serial.print(RS_state, DEC);
        PgmPrint(" TC[]: ");
        Serial.print(TCPIPDataBuffer[0], HEX);        
        PgmPrintln(" Heartbeat sent");
      #endif          
      client.write((uint8_t)Heartbeat);     // command; Connect
      client.write((uint8_t)0);             // length of data = 0
      RS_Heartbeat_timestamp = millis();       // save time of connect for next heartbeat 
    }
  } //end: CheckHeartbeat()

  void Parse_RS_Command(byte RxCMD){
    if (RxCMD != NoCommand ){
      #ifdef _DEBUG_RXTX
        PgmPrint("RC: ");
        //Serial.print(TCPIPDataBuffer[0], HEX);
        Serial.print(RxCMD, HEX);
        PgmPrint(" ");
      #endif 
      switch (RxCMD) {
        case RemoteConfirm:
          PgmPrintln("Connect confirmation recieved ==>RS_STATE_RemoteCall");
          RS_state = RS_STATE_RemoteCall;             
          RS_Client_timestamp = millis();       // save time of connect for connection timeout 
        break;

        case RemoteCall:
          PgmPrintln("Incomming Call ==>RS_STATE_AcceptCall");
          RS_state = RS_STATE_AcceptCall;             
          RS_Client_timestamp = millis(); // save time of connect for connection timeout 
        break;

        case Heartbeat:
            #ifdef _DEBUG_RXTX
              PgmPrintln("Heartbeat recieved, keep waiting for confirmation");
            #endif
            RS_Client_timestamp = millis(); // save time of connect for connection timeout
        break;

        case Acknowledge:
            #ifdef _DEBUG_RXTX
              PgmPrintln("Acknowledge recieved, keep waiting for confirmation");
            #endif
            RS_Client_timestamp = millis(); // save time of connect for connection timeout
        break;

        case NotConnected:
          PgmPrintln("Client no longer connected! ==>RS_STATE_ClientConnect");
          RS_state = RS_STATE_ClientConnect;
        break;

        case End:
          PgmPrintln("End Command ==>RS_STATE_ClientConnect");
          RS_state = RS_STATE_ClientConnect;
        break;

        case Reject:
          PgmPrintln("Reject Command ==>RS_STATE_ClientConnect");
          RS_state = RS_STATE_ClientConnect;
        break;
          
        default:
          if (millis() > (RS_Client_timestamp+RSClientRetryTimeout)){
            RS_state = RS_STATE_ConnectRemote;         // resend connect command after timeout
            PgmPrintln("RC Confirmation timeout! ==>RS_STATE_ConnectRemote");
          }          
        break;
      } 
    } else {
      if (millis() > (RS_Client_timestamp+RSClientRetryTimeout)){
        RS_state = RS_STATE_ClientConnect;         // reconnect client
        PgmPrintln("Command Timeout ==>RS_STATE_ClientConnect");
      } 
    }
    CheckHeartbeat();
  } //end: Parse_RS_Command
#endif

// routines for Time and NTP-Time

void PrintBD_DateTime(void){
  char timestr[22] = {"\r\nDD.MM.YYYY  HH:MM\r\n\000"};
  //String timestr = "\r\nDD.MM.YYYY  HH:MM\r\n";
  if (timeStatus() != timeNotSet) {
    // to Do MJ !!!!!
    sprintf(timestr,"\r\n%02u.%02u.%04u  %02d:%02d\r\n\000",day(),month(),year(),hour(),minute());
  }
  else
  {
    timestr[0] = {"\r\nDD.MM.YYYY  HH:MM\r\n\000"};
  }
  for (byte count=0; timestr[count]; count++) {
    sendAsciiAsBDnTCPIP((byte)timestr[count], false);
    Serial.print(timestr[count]);
  }
  PgmPrintln(" End");
} // PrintBD_DateTime

void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year()); 
  Serial.println(); 
} // digitalClockDisplay

void printDigits(int digits){
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
} // printDigits

/*-------- NTP code ----------*/
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
} // getNtpTime

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
} // sendNTPpacket

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }  

  // timer 1 + interrupt for 50 baud TX interrupt routine

  // Timer0 is used for micros(), millis(), delay(), etc and can't be used
  // Select Timer 1-2 for UNO, 1-5 for MEGA, 1,3,4 for 16u4/32u4
  // Timer 2 is 8-bit timer, only for higher frequency
  // Timer 4 of 16u4 and 32u4 is 8/10-bit timer, only for higher frequency

  ITimer1.init();

  // Using ATmega328 used in UNO => 16MHz CPU clock ,
  // For 16-bit timer 1, 3, 4 and 5, set frequency from 0.2385 to some KHz
  // For 8-bit timer 2 (prescaler up to 1024, set frequency from 61.5Hz to some KHz

  if (ITimer1.attachInterruptInterval(DATABIT_DURATION, TimerHandler1))     // interrupt will the defines pin every 20 ms
  {
    #if (TIMER_INTERRUPT_DEBUG > 1)    
      Serial.println(F("Timer1 attached"));
    #endif    
  } else {
    Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
  }
  STOP_TIMER1
  TIMER1ZERO
  Serial.println(F("Timer stopped"));

  pinMode(RECIEVE_PIN, INPUT_PULLUP);
  pinMode(SEND_PIN, OUTPUT);
  pinMode(COMMUTATE_PIN, OUTPUT);

  digitalWrite(SEND_PIN, DIAL_LINE_CURRENT_ON);
  digitalWrite(COMMUTATE_PIN, DIAL_MODE);

  pinMode(W5100_SS_PIN, OUTPUT);                       // set the W5100 SS-pin as an output (necessary!)
  digitalWrite(W5100_SS_PIN, W5100_OFF);               // but turn off the W5100 chip!

  if (!SD.begin(4)) {
      PgmPrintln("SD initialization failed!");
  } else {
    PgmPrintln("Success SD init.");
  }

  if (GetOwnDataFromSD()){
    PgmPrintln("Telex-Number and Server PIN have been read from SD-Card");
  } else {
    PgmPrintln("Telex-Number and Server PIN not on SD-Card, using hard-coded data");
  }

  PgmPrintln("...starting TCPIP-Network");

  if (Ethernet.begin(mac) == 0) {
    PgmPrintln("Failed to configure Ethernet using DHCP");
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip);
  }
  PgmPrintln("Ethernet has been configured using DHCP");

  // give the Ethernet shield a second to initialize:
  delay(OneSecond);
  PgmPrint("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    PgmPrint(".");
  }

  //interval_min=interval/60/1000;
  //Serial.print ("\nIntervlall in Minuten :" );
  //Serial.println (interval_min);
  //Serial.println("NTP-TimeNTP:");
  //Serial.print("IP number is ");
  //Serial.println(Ethernet.localIP());
  //digitalClockDisplay();
  Udp.begin(localPort);
  Serial.println("waiting for NTP sync");
  setSyncProvider(getNtpTime);
  Serial.print("Time and Date: ");
  digitalClockDisplay();

  #ifndef CENTRALEX
    server.begin(); // Server is only needed if another tty can directly call this tty (via public IP-address)
  #endif

  #ifdef CENTRALEX
    Serial.print("connecting to centralex server...");
    if(client.connect(CENTRALEXHOST,CENTRALEXPORT)){
      Serial.println("success!");
    }else{
      Serial.println("failed!");
    }
  #else
    server.begin();
  #endif
  
  PgmPrintln("Welcome to ArduTelex!");
  PgmPrint("Free RAM: ");
  Serial.println(FreeRam());
  state=STATE_RUHEZUSTAND;
  #ifdef CENTRALEX
    RS_state=RS_STATE_ClientConnect;
    RS_Client_timestamp=millis();
    RS_Heartbeat_timestamp=millis();
  #endif
  bit_pos=0;
  last_timestamp=millis();
  st_timestamp=millis();
  new_char_recieved=false;
  currentDigit=0;
  currentNumberPos=0;
  #ifdef USEIRQ
    attachInterrupt(digitalPinToInterrupt(RECIEVE_PIN), onlinepinchange, RISING);//CHANGE);
  #else
    PgmPrint("no IRQ Version");
  #endif
}

void loop() {
  #ifdef CENTRALEX
    switch  (RS_state) {
      case RS_STATE_ClientConnect:
        if(!client.connected()){    // client is not connected
          client.stop();
          if((millis()>(RS_Client_timestamp+ConnectTimeout))){
            if(client.connect(CENTRALEXHOST,CENTRALEXPORT)){
              PgmPrintln("centralex client not running\nreconnecting(1)...\nsuccess! ==>RS_STATE_ConnectRemote");
              delay(OneSecond);
              RS_state = RS_STATE_ConnectRemote;
              RS_Client_timestamp=millis();
              ReceiveTCPIPCommand();            // purge any data 
            }else{
              PgmPrintln("centralex client not running\nreconnecting(1)...\nclient connection failed! ==>RS_STATE_ConnectRetry1 (in 5 sec.)");
              RS_state = RS_STATE_ClientConnect;
              RS_Client_timestamp=millis();
            }
          }
        }else{                    // client is already connected
          RS_state = RS_STATE_ConnectRemote;
          RS_Client_timestamp=millis();
          ReceiveTCPIPCommand();            // purge any data 
        }
      break;

      case RS_STATE_ConnectRemote:
          PgmPrintln("logging into remote server");
          for (byte counter=0; counter < 8; counter++){
            TCPIPDataBuffer[counter]=0;
          }
          *((byte *)(TCPIPDataBuffer+0)) = (byte)ConnectRemote; // command code
          *((byte *)(TCPIPDataBuffer+1)) = (byte)6;             // lenght = 6 bytes
          *((long *)(TCPIPDataBuffer+2)) = TelexNumber;  // 32 bit TELEX-Number
          *((long *)(TCPIPDataBuffer+6)) = ServerPIN;    // 16 bit Pin
          for (byte counter=0; counter < 8; counter++){
            client.write(TCPIPDataBuffer[counter]);
          }
          RS_Client_timestamp = millis(); // save time of connect for connection timeout
          RS_Heartbeat_timestamp = millis();       // save time of connect for next heartbeat
          PgmPrintln("Connect to Remote Server ==>RS_STATE_RemoteConfirm");
          delay(OneSecond);
          RS_state = RS_STATE_RemoteConfirm;
      break;

      case RS_STATE_RemoteConfirm:
        Parse_RS_Command(ReceiveTCPIPCommand());
      break;

      case RS_STATE_RemoteCall:
        Parse_RS_Command(ReceiveTCPIPCommand());
      break;

      case RS_STATE_AcceptCall:
        PgmPrintln("Accepting Remote Call, ==>STATE_ONLINE");
        IamServer = true;                         // for command parser
        client.write(AcceptCall);               // command Connect
        client.write((uint8_t)0);               // length of data = 0
        RS_Client_timestamp = millis();         // save time of connect for connection timeout 
        RS_state = RS_STATE_InCall;
        CharSent  = 0;
        digitalWrite(COMMUTATE_PIN, DATA_MODE);
        digitalWrite(SEND_PIN, DATA_LINE_CURRENT_ON);  
        state = STATE_ONLINE;
        delay(OneSecond);
      break;

      case RS_STATE_InCall:                     // if (RS_state == RS_STATE_InCall){} will be used to check if Remote Server call is active
        CheckHeartbeat();  
      break;

      case RS_STATE_PAUSE:
        // do nothing
      break;
      default:
      break;
    }
  #endif // endif Centralex

  switch (state){     // state machine

    case STATE_RUHEZUSTAND:
      baudot_recieving_mode=BAUDOT_MODE_UNDEFINED;
      baudot_sending_mode=BAUDOT_MODE_UNDEFINED;
      ConnectionState = CON_NOT_ESTABLISHED;
      digitalWrite(SEND_PIN, DIAL_LINE_CURRENT_ON);
      digitalWrite(COMMUTATE_PIN, DIAL_MODE);
      if(digitalRead(RECIEVE_PIN)==RXD_ANFANGSSIGNAL){ // AT
        delay(5);
        if(digitalRead(RECIEVE_PIN)==RXD_ANFANGSSIGNAL){// immernoch AT
          delay(5);
          if(digitalRead(RECIEVE_PIN)==RXD_ANFANGSSIGNAL){// immernoch AT
            last_timestamp=millis();
            state=STATE_ANFANGSSIGNAL;
            #ifdef _DEBUG
              PgmPrintln("==>STATE_ANFANGSSIGNAL");
            #endif
          }
        }
      }
      #ifndef CENTRALEX
          client = server.available();			// this server is used for incomming TCPIP calls 
          if (client) {			// not needed if Centralex server is used 
            #ifdef _DEBUG
              PgmPrintln("TCPIP client connecting!");
            #endif
            IamServer = true;
            CharSent  = 0;
            storeMainSocket();

            state=STATE_ONLINE;
            #ifdef _DEBUG
              PgmPrintln("==>STATE_ONLINE");
            #endif
            digitalWrite(COMMUTATE_PIN, DATA_MODE);
            digitalWrite(SEND_PIN, DATA_LINE_CURRENT_ON);  
            delay(OneSecond);
          }//end client connecting
      #endif
    break;

    case STATE_ANFANGSSIGNAL:
      digitalWrite(SEND_PIN, DIAL_LINE_CURRENT_ON);  // MJ-Solid-State-TW39-HW needs a High  on TXD to enable loop current in dial mode
      digitalWrite(COMMUTATE_PIN, DIAL_MODE);
      if(millis()>(last_timestamp+OneSecond)){
          state=STATE_WAHLAUFFORDERUNGSSIGNAL;
          last_timestamp=millis();
            #ifdef _DEBUG
              PgmPrintln("==>STATE_WAHLAUFFORDERUNGSSIGNAL");
            #endif
      }
    break;

    case STATE_WAHLAUFFORDERUNGSSIGNAL:
      digitalWrite(SEND_PIN, DIAL_LINE_CURRENT_OFF);
      digitalWrite(COMMUTATE_PIN, DIAL_MODE);
      currentNumberPos=0;
      #ifdef CENRTRALEX
        RS_state = RS_STATE_PAUSE;
      #endif
      if(millis()>(last_timestamp+BEGIN_CALL_PULSE)){
            digitalWrite(SEND_PIN, DIAL_LINE_CURRENT_ON);
            delay(10);
            last_timestamp=millis();
            state=STATE_WARTE_AUF_ZIFFER;
            #ifdef _DEBUG
              PgmPrint("==>STATE_WARTE_AUF_ZIFFER: 0, Nummer: ");
            #endif
      }
    break;

    case STATE_WARTE_AUF_ZIFFER:
      currentDigit=0;
      digitalWrite(SEND_PIN, DIAL_LINE_CURRENT_ON);
      digitalWrite(COMMUTATE_PIN, DIAL_MODE);
      if(digitalRead(RECIEVE_PIN)==1){// wahlimpuls
            last_timestamp=millis();
            state=STATE_WAHLIMPULS;
      }
      //todo check number
      if((currentNumberPos>0) && (millis()>(last_timestamp+DIAL_TIMEOUT))){
        last_timestamp=millis();
        state=STATE_WAHL_ENDE;
        #ifdef _DEBUG
          PgmPrintln("==>STATE_WAHL_ENDE");
        #endif
      }
    break;

    case STATE_WAHLIMPULS:
      #ifdef _DEBUG
        //PgmPrintln("STATE_WAHLIMPULS");
      #endif
      digitalWrite(SEND_PIN, DIAL_LINE_CURRENT_ON);
      digitalWrite(COMMUTATE_PIN, DIAL_MODE);
      if(digitalRead(RECIEVE_PIN)==0){// wahlimpuls_ende
            currentDigit++;
            last_timestamp=millis();
            state=STATE_WARTE_AUF_NAECHSTEN_WAHLIMPULS;
      }
      if(millis()>(last_timestamp+DIAL_PULSE_TIMEOUT)){
            last_timestamp=millis();
            state=STATE_DISCONNECT;
            #ifdef _DEBUG
              PgmPrintln("Timeout during dialimpulse");
              PgmPrintln("==>STATE_DISCONNECT");
            #endif
      }
    break;

    case STATE_WARTE_AUF_NAECHSTEN_WAHLIMPULS:
      digitalWrite(SEND_PIN, DIAL_LINE_CURRENT_ON);
      digitalWrite(COMMUTATE_PIN, DIAL_MODE);
      if(digitalRead(RECIEVE_PIN)==1){// wahlimpuls
            last_timestamp=millis();
            state=STATE_WAHLIMPULS;
      }
      if(millis()>(last_timestamp+DIAL_BETW_NUMBERS)){
        last_timestamp=millis();
        state=STATE_WARTE_AUF_ZIFFER;
        if(currentDigit==0){
        #ifdef _DEBUG
          PgmPrintln("kein impuls");
          PgmPrintln("==>STATE_DISCONNECT");
        #endif
          state=STATE_DISCONNECT;
        }
        if(currentDigit==10){
          currentDigit=0;
        }
        number[currentNumberPos++]='0'+currentDigit;
        if(currentNumberPos>=NUMBER_BUFFER_LENGTH){
          #ifdef _DEBUG
            PgmPrintln("number to long!");
          #endif
          currentNumberPos=NUMBER_BUFFER_LENGTH-1;
        }
        #ifdef _DEBUG
          Serial.print(currentDigit);
        #endif
      }
    break;

    case STATE_WAHL_ENDE:
      #ifdef _DEBUG
        PgmPrintln("STATE_WAHL_ENDE");
      #endif
      digitalWrite(SEND_PIN, DIAL_LINE_CURRENT_ON);
      digitalWrite(COMMUTATE_PIN, DIAL_MODE);
      number[currentNumberPos]=0;
      #ifdef _DEBUG
        PgmPrint("\ngewählte Nummer: ");
        Serial.println(number);
      #endif
      state=STATE_LOOKUP;

      recieving=true;
    break;

    case STATE_LOOKUP:
      #ifdef _DEBUG
        PgmPrintln("STATE_LOOKUP");
      #endif
        if(lookupNumber(number)){                 // TLN-Server lookup is done with a different socket (EthernetClient tln_client;)
          #ifdef _DEBUG
            PgmPrint("lookup ok: ");
            Serial.print(lookup_host);
            PgmPrint(":");
            Serial.print((unsigned int)lookup_port);
            PgmPrint("*");
            Serial.print(lookup_durchwahl);
            PgmPrint("t");
            Serial.println(lookup_type);
            PgmPrintln("STATE_CONNECTING");
          #endif
          switch (lookup_type){
            case 1:
              #ifdef _DEBUG
              PgmPrintln("Peer Type: 1, Baudot + Hostname");
              #endif
              state=STATE_CONNECTING;
            break;
            case 2:
              #ifdef _DEBUG
              PgmPrintln("Peer Type: 2, Baudot + IP-Address");
              #endif
              state=STATE_CONNECTING;
            break;
            case 5:
              #ifdef _DEBUG
              PgmPrintln("Peer Type: 5, Baudot + Dyn-IP");
              #endif
              state=STATE_CONNECTING;
            break;
            default:
              #ifdef _DEBUG
              PgmPrintln("Wrong Peer Type!");
              PgmPrintln("==>STATE_BUSY");
              #endif
              state=STATE_BUSY;
            break;
          }
      } else {
        #ifdef _DEBUG
          PgmPrintln("nummer nicht gefunden");
          PgmPrintln("==>STATE_BUSY");
        #endif
        state=STATE_BUSY;
      }
    break;

    case STATE_CONNECTING:
      #ifdef _DEBUG
        PgmPrintln("STATE_CONNECTING");
        PgmPrintln("connecting to remote");
      #endif
      #ifdef CENTRALEX
          client.stop();
          PgmPrintln("centralex server connection terminated");
          delay(OneSecond);
      #endif
        digitalWrite(SEND_PIN, DATA_LINE_CURRENT_ON);   
        digitalWrite(COMMUTATE_PIN, DATA_MODE);

        if (client.connect(lookup_host.c_str(), lookup_port)) {     // stop the Remote-Server-Client before connecting to the Server
      #ifdef _DEBUG
          PgmPrintln("connected to server");
          delay(OneSecond);
      #endif
          storeMainSocket();
          state = CS_START;
        }else{
          #ifdef _DEBUG
              PgmPrintln("connecting to remote failed");
              PgmPrintln("==>STATE_BUSY");
          #endif
          state=STATE_BUSY;
        }
    break;

    // **********************  iTelex Protocoll stuff ********************************
    case CS_START:
      state = CS_SEND_VER;          
      tmp = 0;
      RecVersion = 0;
      break;
    case CS_SEND_VER:
      if (tmp > 3){
            state = CS_ABORT;
      #ifdef _DEBUG
        PgmPrintln("Version negotiation did not work!");
      #endif         
      }
      sendVersion();            // send version = 1
      state = CS_REC_VER;
      st_timestamp=millis();
      #ifdef _DEBUG
        PgmPrint("Send Version via TCPIP: ");
        Serial.print(tmp,DEC);
        PgmPrintln("");        
      #endif
      break; 
    case CS_REC_VER:
      if ((RecVersion>0)||(millis()>(st_timestamp+TCPIP_TIMEOUT))){
          if (millis()>(st_timestamp+TCPIP_TIMEOUT)){
            state = CS_ABORT;
            #ifdef _DEBUG
              PgmPrintln("TCPIP Timeout");
            #endif            
          }else {
            if (RecVersion != 1){
              state = CS_SEND_VER;
              tmp++;
              #ifdef _DEBUG
                PgmPrintln("Wrong Version received, try again");
              #endif
            }
            else {
              state = CS_SEND_DD;
              #ifdef _DEBUG
                PgmPrintln("Correct Version received");
              #endif            
            }
          }
      }          
      break;
    case CS_CON_VER:
      sendVersion();            
      state = CS_SEND_DD;
      #ifdef _DEBUG
        PgmPrintln("Confrim Version");
      #endif
      break;
    case CS_SEND_DD:
      sendDD(lookup_durchwahl);            // send DirectDial No (Durchwahl)
      state = CS_END;
      #ifdef _DEBUG
        PgmPrint("ITx-CMD Direct Dial: ");
        Serial.print(lookup_durchwahl,DEC);
        PgmPrintln("");
      #endif
      break;
    case CS_ABORT:
      state=STATE_BUSY;             
      #ifdef _DEBUG
        PgmPrintln("Abort!");
      #endif
      break;
    case CS_END:
      state=STATE_ONLINE;
      IamServer = false;
      CharSent  = 0;
      recieving=true;
      digitalWrite(COMMUTATE_PIN, DATA_MODE);
      digitalWrite(SEND_PIN, DATA_LINE_CURRENT_ON);  
      delay(OneSecond);
      #ifdef _DEBUG
        PgmPrintln("==>STATE_ONLINE");
      #endif
      break;
    // **********************  iTelex Protocoll stuff (end) **************************

    case STATE_BUSY: //ump=1, SSR=1, delay 2000 -> 0
      PgmPrintBD("(discon.)");
      digitalWrite(COMMUTATE_PIN, DATA_MODE);
      digitalWrite(SEND_PIN, DATA_LINE_CURRENT_ON);  
      delay(2000);
      #ifdef _DEBUG
        PgmPrintln("STATE_BUSY");
        PgmPrintln("==>STATE_DISCONNECT 1");
      #endif
      state=STATE_DISCONNECT;
    break;

    case STATE_LOCALMODE:
      if(!digitalRead(RECIEVE_PIN)){ // check for disconnect condition
        st_timestamp=millis();
      }else if(millis()>(st_timestamp+ST_DURATION)){
        #ifdef _DEBUG
          PgmPrintln("We want to disconnect in localmode!");
          PgmPrintln("==>STATE_DISCONNECT 2");
        #endif
        state=STATE_DISCONNECT;
      }
    break;

    case STATE_DISCONNECT:
      #ifdef _DEBUG
        PgmPrintln("STATE_DISCONNECT");
        PgmPrintln("TCPIP Disconnecting!");
        PgmPrintln("==>STATE_RUHEZUSTAND");
      #endif
      client.stop();
      IamServer = false;
      state=STATE_RUHEZUSTAND; 
      #ifdef CENTRALEX
        RS_Client_timestamp=millis();
        RS_Heartbeat_timestamp=millis();
        RS_state = RS_STATE_ClientConnect;
        PgmPrintln("==>RS_STATE_ClientConnect");      
      #endif
      digitalWrite(SEND_PIN, DIAL_LINE_CURRENT_ON);
      digitalWrite(COMMUTATE_PIN, DIAL_MODE);
      delay(OneSecond);
    break;

    case STATE_ONLINE:
      digitalWrite(COMMUTATE_PIN, DATA_MODE);
      #ifndef CENTRALEX
        handleClientsWhileConnected(); //check for new clients which try to connect while connection is already in progress
      #endif
      if(!client.connected()){ // did the client disconnect?
        client.stop();
        #ifdef _DEBUG
          PgmPrintln("Client disconnected!");
        #endif
        state=STATE_DISCONNECT;
      }
      if(!digitalRead(RECIEVE_PIN)){ // check for disconnect condition
        st_timestamp=millis();
      }else if(millis()>st_timestamp+(ST_DURATION * 2)){
      #ifdef _DEBUG
        PgmPrintln("We want to disconnect!");
        PgmPrintln("==>STATE_DISCONNECT 3");
      #endif
        state=STATE_DISCONNECT;
      }
    break;
  }//END of state machine (state) switch

  #ifdef _DEBUGSOCKETS
    for (int i = 0; i < MAX_SOCK_NUM; i++) {
        uint8_t s = W5100.readSnSR(i);
        Serial.print(s,HEX);
        Serial.print("\t");
    }
    Serial.print(currentSocknum);
    Serial.println();
  #endif

  if((state==STATE_ONLINE) && (millis()>(OneSecTimer+OneSecond))) {   // one sec timer
    OneSecTimer = millis();   // reset one sec timer
    sendACK();                // send acknowledge iTx-command       
  }

  if(state==STATE_ONLINE || state==STATE_LOCALMODE){ // send and recieve tw-39
    if(bit_pos==0){//wait for startbit
      #ifdef USEIRQ 
          //do nothing (done in pinchangeinterrupt)
      #else
        if(recieving){
          if( digitalRead(RECIEVE_PIN)){ //startbit
            last_timestamp=millis();
            bit_pos++;
          }
        }
      #endif
    } else {                                // we had the startbit and are now waiting for further bits
      unsigned long timestamp=millis();
      long diff=timestamp-last_timestamp;
      if((bit_pos<6) && (diff>= (STARTBIT_DURATION+(DATABIT_DURATION*(bit_pos-1))+(DATABIT_DURATION*SAMPLEPOS/120)))){
        #ifdef _DEBUGTIMINGS
          Serial.print(bit_pos,DEC);
          PgmPrint("\t");
          Serial.print(last_timestamp,DEC);
          PgmPrint("\t");
          Serial.print(timestamp,DEC);
          PgmPrint("\t");
          Serial.print(diff,DEC);
          PgmPrint("\t");
          Serial.print((STARTBIT_DURATION+(DATABIT_DURATION*(bit_pos-1))+(DATABIT_DURATION*SAMPLEPOS/120)));
          PgmPrint("\n");
        #endif

        recieve_buf=recieve_buf<<1;           // shits up, for reverse iTelex bit order
        //      recieve_buf=recieve_buf>>1;   // shif down, for correct bit order 
        bit_pos++;
        if(digitalRead(RECIEVE_PIN)){
        }else{
          recieve_buf=recieve_buf+1;       // set bit 0, for reverse iTelex bit order
          // recieve_buf=recieve_buf+16;   // set bit 4, for correct bit order 
        }
        if(bit_pos==6){
            new_char_recieved=true;
            recieved_char=recieve_buf;
            recieve_buf=0;
        }
      }
      if(bit_pos==6 && (diff>= (STARTBIT_DURATION+DATABIT_DURATION*6))){   //1/3 stopbit to early to not miss anything
        #ifdef _DEBUGTIMINGS
          Serial.print(bit_pos,DEC);
          PgmPrint("\t");
          Serial.print(last_timestamp,DEC);
          PgmPrint("\t");
          Serial.print(timestamp,DEC);
          PgmPrint("\t");
          Serial.print(diff,DEC);
          PgmPrint("\t");
          Serial.print((STARTBIT_DURATION+(DATABIT_DURATION*(bit_pos-1))+(DATABIT_DURATION*SAMPLEPOS/120)));
          PgmPrint("\n");
          Serial.println(recieved_char,BIN);
        #endif
        bit_pos=0;
      }
    }
  }//send and recieve TW-39

  if (new_char_recieved) {         // new character received from 50 baud teletype receiver
    char c=baudotToAscii(recieved_char);
    if(c!=BAUDOT_CHAR_UNKNOWN){
      if(client.connected()){
        sendBaudot(recieved_char);
        #ifdef _DEBUG_RXTX
        PgmPrint("(");
        Serial.print(recieved_char,DEC);
        PgmPrint("){");
        #endif
        if(c!=BAUDOT_CHAR_UNKNOWN && c!='\016' && c!='\017' &&c!=BAUDOT_MODE_BU && c!=BAUDOT_MODE_ZI && c!=0){
          Serial.print(c);
        }
        #ifdef _DEBUG_RXTX
        PgmPrint("}");
        #endif          
      }
    }
    new_char_recieved=false;
  } // end new character received

  if (Serial.available() > 0) {    // serial data received (from serial monitor)
    if(state==STATE_ONLINE||state==STATE_LOCALMODE){
      sendAsciiAsBaudot(Serial.read(), true);  
    }else{
      #ifdef _DEBUG
      PgmPrintln("==>STATE_LOCALMODE");
      #endif
      digitalWrite(COMMUTATE_PIN, DATA_MODE);
      digitalWrite(SEND_PIN, DATA_LINE_CURRENT_ON);  
      delay(OneSecond);
      state=STATE_LOCALMODE;
      recieving=true;
    }
  } // End: serial data received (monitor)

  #ifdef CENTRALEX
    if ((RS_state == RS_STATE_InCall)||(RS_state==RS_STATE_PAUSE)){ 
      if (ReceiveTCPIPCommand() != NoCommand){
        iTelexCommandParser(IamServer);
        if (ConnectionState == CON_ESTABLISHED){
          recieving=true; 
          PrintBD_DateTime();
          ConnectionState = CON_ESTABLISHED_TIME_SENT;
          sendACK(); 
          OneSecTimer = millis();      // initialize one sec. timer
        }
      }
    }
  #else   
    if (ReceiveTCPIPCommand() != NoCommand){
      iTelexCommandParser(IamServer);
      if (ConnectionState == CON_ESTABLISHED){
        recieving=true; 
        PrintBD_DateTime();
        ConnectionState = CON_ESTABLISHED_TIME_SENT;
        sendACK(); 
        OneSecTimer = millis();      // initialize one sec. timer
      }
    }
  #endif
  if ((state==STATE_ONLINE)&&(recieving)) {         // Baudot Data in Buffer to be sent to TTY
    if (getbuffer(&BDData)){
        byte_send(BDData);
        CharSent++;
        char c = baudotToAscii(BDData);
        #ifdef _DEBUG_RXTX
        PgmPrint("[");
        Serial.print(BDData,DEC);
        PgmPrint("]{");
        #endif
        if(c!=BAUDOT_CHAR_UNKNOWN && c!='\016' && c!='\017' &&c!=BAUDOT_MODE_BU && c!=BAUDOT_MODE_ZI && c!=0){
          Serial.print(c);
        }else{
          #ifdef _DEBUG_RXTX
            PgmPrint("§");
            if(BDData==BAUDOT_MODE_BU){
              PgmPrint("<BU>");
            }
            if(BDData==BAUDOT_MODE_ZI){
              PgmPrint("<ZI>");
            }
          #endif  
        }
        #ifdef _DEBUG_RXTX
        PgmPrint("}");
        #endif        
    }
  } // End: Baudot Data in Buffer to be sent to TTY
} // end: loop()
