#ifndef __ETELEX_H__
  #define __ETELEX_H__

  //#define CENTRALEX
  //#define _DEBUG_RXTX
  #define _DEBUG
  //#define _DEBUGTIMINGS

  #define SOLID_STATE_TW39  // if Solid state TW39 interface with TLP627-4 is used

  #define MYPORT 134
  #define MYIPFALLBACK 192, 168, 1, 64
  #define MYMACADRESS 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED //MPIR D, Wiznet based ethernet shield 
  //#define TLN_SERVER "sonnibs.no-ip.org"
  #define TLN_SERVER "tlnserv.teleprinter.net"
  //#define TLN_SERVER "telexgateway.de"
  #define TLN_SERVER_PORT 11811
  #define MYSSID "xxx"         // not needed with Arduino as it is ETH based
  #define MYWIFIPASSWORD "xxx"  // not needed with Arduino as it is ETH based
  #define MYHOSTNAME "etelex"

  #define TELEX_NUMBER  (unsigned long)123456
  #define SERVER_PIN  (word)1234

  // States
  #define STATE_RUHEZUSTAND  0                      //4ma  ump=0, ssr=0
  #define STATE_ANFANGSSIGNAL  10                   //4ma -> 40 ma -> opto==1 ca. 1000ms delay
  #define STATE_WAHLAUFFORDERUNGSSIGNAL 20          //brk=1, 25ms delay
  #define STATE_WARTE_AUF_ZIFFER  30                //brk=0, /warte  5000ms auf opto==0 -> 31 sonst -> 34 auf opto==0
  #define STATE_WAHLIMPULS  31                      //opto==0, warte auf opto==1 -> 32
  #define STATE_WARTE_AUF_NAECHSTEN_WAHLIMPULS  32  //opto==1, warte auf 200ms auf opto==0 -> 31 sonst -> 33
  #define STATE_WAHL_ENDE  34                       // nummer zu kurz -> 90 sonst 40
  #define STATE_LOOKUP  40                          //nummer nicht gefunden -> 90 sonst 50
  #define STATE_CONNECTING  50                      // verbindungs timeout -> 90 sonst 100
  #define CS_START      	  60                      // Start of Version 
  #define CS_SEND_VER       70                      // Send Version 07 01 01
  #define CS_REC_VER        80                      // Wait for Version from client, Client version should be 07 01 01, goto Send version if not
  #define CS_CON_VER        90                      // confirm version
  #define CS_SEND_DD       100                      // send Direct Dial = Durchwahl
  #define CS_ABORT         110                      // if Version Negitiation was not successful
  #define CS_END           120                      // finished, set state = STATE_ONLINE
  #define STATE_BUSY       130                      //ump=1, SSR=1, delay 2000 -> 0
  #define STATE_ONLINE     140                      //ump=1, SSR=1, disconnect -> 0, opto==0 <2000s -> 110 sonst -> 0, recieve data via tcp ->120
  #define STATE_LOCALMODE  150
  #define STATE_DISCONNECT 200

  //iTelex Commands, see https://wiki.telexforum.de/index.php?title=Communication_Specification_(i-Telex)#Packet_Types
  #define Heartbeat   0x00  // heartbeat, length = 0, Ex: 00 00
  #define DirectDial  0x01  // Durchwahl, length = 1, Ex: 01 01 00 = no DW
  #define BaudotData  0x02  // Baudot data, Ex: 02 07 1F 01 10 14 01 02 08 = "test" + CR + LF 
  #define End         0x03  // End
  #define Reject      0x04  // Reject, Ex: 04 02 6E 61 (na = not allowed)
  #define Acknowledge 0x06  // Number of received and processed characters, Ex: 06 01 05 
  #define iTVersion   0x07  // Verion, Ex: 07 05 01 35 32 34 00 (version 1, + "524" = SVN-Rev)
  #define Loopback    0x08  // Loopback Test, Ex: 08 02 45 6B
  #define ACSIIData   0x0B  // ASCII data, Ex: 00B 07 48 65 6C 6C 70 20 21 = "Hello !"  NOT YET SUPPORTED
  #define MaxCMD      0x85  // no more command from 0x85 and higher

  //remote server commands
  #define ConnectRemote   0x81  // TLX>RemSrv 81 06 Own Number (long) PIN (word)
  #define RemoteConfirm   0x82  // RemSrv>TLX 82 00
  #define RemoteCall      0x83  // RemSrv>TLX 83 00
  #define AcceptCall      0x84  // TLX>RemSrv 84 00
  #define NotConnected    0xFE
  #define NoCommand       0xFF

  #define OneSecond       1000  // one second = 1000 ms

  #define COMMUTATE_PIN   2 // reverse polarity (umpol) pin for the relais/H-bridge
  #define RECIEVE_PIN     3 // recieve Pin
  #define SEND_PIN        5 // send pin for the Mosfet

  #define W5100_SS_PIN   10 // S-Chip-Select Pin of W5100 Ethernet Chip
  #define W5100_OFF    HIGH // Chip select is low active ==> HIGH = off

  #ifdef SOLID_STATE_TW39       // solid state TW39 circuitry
    #define DIAL_MODE HIGH
    #define DATA_MODE LOW
    #define DIAL_LINE_CURRENT_ON LOW
    #define DIAL_LINE_CURRENT_OFF HIGH
    #define DATA_LINE_CURRENT_ON HIGH
    #define DATA_LINE_CURRENT_OFF LOW
    #define RXD_ANFANGSSIGNAL 0
  #else
    #define DIAL_MODE LOW        // normal Arduino TW39 circuitry with relays
    #define DATA_MODE HIGH
    #define DIAL_LINE_CURRENT_ON LOW
    #define DIAL_LINE_CURRENT_OFF HIGH
    #define DATA_LINE_CURRENT_ON LOW
    #define DATA_LINE_CURRENT_OFF HIGH
    #define RXD_ANFANGSSIGNAL 0
  #endif

  //Defines for software serial interface and the related timer1 + Interrup t
  #define DATABIT_DURATION    20 // milliseconds
  #define STARTBIT_DURATION   20 // milliseconds
  #define STOPBIT_DURATION    40 // milliseconds
  #define SAMPLEPOS           60 // 60 of 120

  #ifdef CENTRALEX
    // Remote Server States
    #define RS_STATE_ClientConnect        0   //
    #define RS_STATE_ConnectRemote       10   //
    #define RS_STATE_RemoteConfirm       20   //
    #define RS_STATE_RemoteCall          30   //
    #define RS_STATE_AcceptCall          40   //
    #define RS_STATE_InCall              50   //
    #define RS_STATE_PAUSE               60   // 

    #define CENTRALEXHOST "telexgateway.de"

    #define CENTRALEXPORT         49491
    #define RSHeartbeat           10000
    #define RSClientRetryTimeout  30000   // retry after 30 seconds
    #define ConnectTimeout         5000   // retry after five seconds
  #endif

  #ifdef CENTRALEX
    #undef _DEBUGPINGS
  #endif

  #define _DEBUG
  #undef _DEBUGTIMINGS
  #undef _DEBUGSOCKETS

  #define ITELEX
  #define USEIRQ
  #define USETIMERIRQ

  #define ST_DURATION 1000 // How long to hold down the ST key
  #define TCPIP_TIMEOUT 3000

  #define DIAL_PULSE_TIMEOUT  10000
  #define BEGIN_CALL_PULSE    25
  #define DIAL_TIMEOUT        5000
  #define DIAL_BETW_NUMBERS   200

  #define ICACHE_RAM_ATTR

  #include <Ethernet.h>
  #include <utility/w5100.h>

  #include <SD.h>
  #include <SPI.h>

  extern  void SerialPrint_P(PGM_P str);
  extern  void SerialPrintln_P(PGM_P str);

#endif
