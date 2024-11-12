#ifndef __LOOKUP_H__
#define __LOOKUP_H__
#include <Arduino.h>

extern String lookup_host;
extern unsigned int lookup_port;
extern byte lookup_durchwahl;
extern byte lookup_type;
extern char tlnserver[];
extern bool lookupNumber(String number);
extern bool GetOwnDataFromSD(void);
extern unsigned long TelexNumber;
extern word ServerPIN;

#endif //ndef __LOOKUP_H__
