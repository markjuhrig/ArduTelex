#include "lookup.h"
#include "ArduTelex.h"

char sd_c;
EthernetClient tln_client;

String lookup_host;
unsigned int lookup_port;
byte lookup_durchwahl;
byte lookup_type;
String linebuffer;

unsigned long TelexNumber=TELEX_NUMBER;
unsigned long TempNumber;
word ServerPIN=SERVER_PIN;

bool GetOwnDataFromSD(void){
#ifdef _DEBUG
  PgmPrintln("Get own data from SD-Card (Telex number and PIN)");
#endif
  pinMode(W5100_SS_PIN, OUTPUT);                       // set the SS pin as an output (necessary!)
  digitalWrite(W5100_SS_PIN, W5100_OFF);                    // but turn off the W5100 chip!
  String filename="owndata/owndata.txt";
  File myFile = SD.open(filename);
  if (myFile) {
    int pos=0; 
    linebuffer="";
     // read from the file until there's nothing else in it:
     while (myFile.available()) {
        sd_c=myFile.read();
        if(sd_c=='#'){
          while (myFile.available()) {
            sd_c=myFile.read();
            if(sd_c=='\n'){
              break;
            }
          }
        }
        if(sd_c!='\r' && sd_c!='\n'){
          linebuffer+=sd_c;
          //Serial.write(sd_c);          
        } 
        if(sd_c=='\n'){
          linebuffer.trim();
          if(linebuffer.length()>0){
            if(pos==0){
#ifdef _DEBUG
              PgmPrint("Status: ");
              Serial.print(linebuffer);
#endif
              if(linebuffer!="Data:"){
#ifdef _DEBUG
                PgmPrintln("file header is not OKAY!");
#endif
                myFile.close(); 
                return false;
              }
            }
            if(pos==1){
#ifdef _DEBUG
              PgmPrint("\nTelex Nummer: ");
              Serial.println(linebuffer);
#endif
              TempNumber = linebuffer.toInt();
              if ((TempNumber>0) && (TempNumber < 999999)){
                TelexNumber = TempNumber;   
              } else {
#ifdef _DEBUG
                PgmPrintln("Telex number is out of range");
#endif
                myFile.close(); 
                return false;
              }
            }
            if(pos==2){
#ifdef _DEBUG
              PgmPrint("Server PIN: ");
              Serial.println(linebuffer);
#endif
              TempNumber = linebuffer.toInt();
              if ((TempNumber>=0) && (TempNumber <= 65535)){
                ServerPIN = TempNumber;  
                myFile.close(); 
                return true;
              } else {
#ifdef _DEBUG
                PgmPrintln("Server PIN is out of Range");
#endif
                myFile.close(); 
                return false;
              }
            }
            if(linebuffer=="+++"){
              pos=0;
            }else{
              pos++;                          
            }
            linebuffer="";
          }
        }
     }
     // close the file:
    myFile.close();
   } else {
     // if the file didn't open, print an error:
#ifdef _DEBUG
      PgmPrintln("error opening file");
#endif      
   }
  return false;
}  // end: GetOwnDataFromSD()

bool lookupSD(String number){
#ifdef _DEBUG
  PgmPrintln("lookup SD-Card");
#endif
  pinMode(W5100_SS_PIN, OUTPUT);                       // set the SS pin as an output (necessary!)
  digitalWrite(W5100_SS_PIN, W5100_OFF);                    // but turn off the W5100 chip!
  String filename="pb/"+number+".txt";
  File myFile = SD.open(filename);
  if (myFile) {
    int pos=0; 
    linebuffer="";
     // read from the file until there's nothing else in it:
     while (myFile.available()) {
        sd_c=myFile.read();
        if(sd_c=='#'){
          while (myFile.available()) {
            sd_c=myFile.read();
            if(sd_c=='\n'){
              break;
            }
          }
        }
        if(sd_c!='\r' && sd_c!='\n'){
          linebuffer+=sd_c;
          //Serial.write(sd_c);          
        } 
        if(sd_c=='\n'){
          linebuffer.trim();
          if(linebuffer.length()>0){
            if(pos==0){
#ifdef _DEBUG
              PgmPrint("Status: ");
              Serial.println(linebuffer);
#endif

              if(linebuffer!="ok"){
#ifdef _DEBUG
                PgmPrintln("Status not ok");
#endif
                myFile.close(); 
                return false;
              }
            }
            if(pos==1){
#ifdef _DEBUG
              PgmPrint("Nummer: ");
              Serial.println(linebuffer);
#endif              
              if(linebuffer!=number){
#ifdef _DEBUG
                PgmPrintln("number not the same");
#endif
                myFile.close(); 
                return false;
              }

            }
            if(pos==2){
#ifdef _DEBUG
              PgmPrint("Name: ");
              Serial.println(linebuffer);
#endif
            }
            if(pos==3){
#ifdef _DEBUG
              PgmPrint("Typ: ");
              Serial.println(linebuffer);
#endif
              lookup_type=linebuffer.toInt();
            }
            if(pos==4){
#ifdef _DEBUG              
              PgmPrint("Host: ");
              Serial.println(linebuffer);
#endif
              lookup_host=linebuffer;
            }
            if(pos==5){
#ifdef _DEBUG
              PgmPrint("Port: ");
              Serial.println(linebuffer);
#endif
              lookup_port=linebuffer.toInt();
            }
            if(pos==6){
#ifdef _DEBUG              
              PgmPrint("Durchwahl: ");
              Serial.println(linebuffer);
#endif
              lookup_durchwahl=linebuffer.toInt();
              myFile.close(); 
              return true;
              
            }
            
            if(linebuffer=="+++"){
              pos=0;
            }else{
              pos++;                          
            }
            linebuffer="";
          }
        }
     }
     // close the file:
    myFile.close();
   } else {
     // if the file didn't open, print an error:
#ifdef _DEBUG
      PgmPrintln("error opening file");
#endif      
   }
  return false;
}

/*
#Zum Typ:
#1 ist eine echte i-Telex-Station mit einem Hostnamen
#2 oder 5 ist eine echte i-Telex-Station mit einer IP-Adresse
#3 ist eine Station, die nur Ascii-Daten verarbeiten kann, so wie Frank's Wetterdienst-Server (mit Hostnamen)
#4 ist eine Station, die nur Ascii-Daten verarbeiten kann, mit IP-Adresse
#6 Email-Adresse (für dich nicht Sinnvoll und auch kaum verwendet)
*/

bool lookupTlnSrv(String number){
#ifdef _DEBUG
  PgmPrintln("lookupTlnSrv");
#endif
    String IP="";
    if (tln_client.connect(TLN_SERVER, TLN_SERVER_PORT)) {
#ifdef _DEBUG
      PgmPrintln("connected to tln Server");
#endif
      String query="q";
      query+=number;
      tln_client.println(query);
#ifdef _DEBUG
              PgmPrint("Query: ");
              Serial.println(query);
#endif
      while(!tln_client.available()){;}
      
    int pos=0; 
    linebuffer="";
     while(tln_client.available()){
        sd_c=tln_client.read();
        if(sd_c!='\r' && sd_c!='\n'){
          linebuffer+=sd_c;    
        } 
        if(sd_c=='\n'){
          linebuffer.trim();
          if(linebuffer.length()>0){
            if(pos==0){
#ifdef _DEBUG
              PgmPrint("Status: ");
              Serial.println(linebuffer);
#endif
              if(linebuffer!="ok"){
#ifdef _DEBUG
                PgmPrintln("Status not ok");
#endif
                tln_client.stop();
                return false;
              }
            }
            if(pos==1){
#ifdef _DEBUG
              PgmPrint("Nummer: ");
              Serial.print(linebuffer);
              PgmPrintln("CR");
#endif
              if(linebuffer!=number){
#ifdef _DEBUG
                PgmPrintln("number not the same");
                //PgmPrint("Nummer: ");
                //Serial.print(linebuffer);
                //PgmPrintln("CR");
#endif
                tln_client.stop();
                return false;
              }

            }
            if(pos==2){
#ifdef _DEBUG
              PgmPrint("Name: ");
              Serial.println(linebuffer);
#endif
            }
            if(pos==3){
#ifdef _DEBUG
              PgmPrint("Typ: ");
              Serial.println(linebuffer);
#endif
              lookup_type=linebuffer.toInt();
            }
            if(pos==4){
#ifdef _DEBUG
              PgmPrint("Host: ");
              Serial.println(linebuffer);
#endif
              lookup_host=linebuffer;
            }
            if(pos==5){
#ifdef _DEBUG
              PgmPrint("Port: ");
              Serial.println(linebuffer);
#endif
              lookup_port=linebuffer.toInt();
            }
            if(pos==6){
#ifdef _DEBUG
              PgmPrint("Durchwahl: ");
              Serial.println(linebuffer);
#endif
              lookup_durchwahl=linebuffer.toInt();
              tln_client.stop();
              return true;
            }   
            if(linebuffer=="+++"){
              pos=0;
            }else{
              pos++;                          
            }
            linebuffer="";
          }
        }
     }       
     tln_client.stop();
    }
#ifdef _DEBUG
PgmPrintln("");
#endif
  return(false);
}
bool lookupHardcoded(String number){
#ifdef _DEBUG
  PgmPrintln("lookupHardcoded");
#endif
  if(number=="0"){
    lookup_host="192.168.1.16";
    lookup_port=134;
    lookup_durchwahl=0;
    lookup_type=4;
    return true;
  }
  if(number=="1"){
    lookup_host="www.glsys.de";
    lookup_port=134;
    lookup_durchwahl=0;
    lookup_type=3;
    return true;
  }
  return false;
}

bool lookupNumber(String number){
#ifdef _DEBUG
  Serial.println();
  PgmPrint("lookupNumber: ");
  Serial.println(number);
#endif
  if(lookupSD(number)) return true;
  return lookupTlnSrv(number);
}

