#ifndef __BAUDOT_H__
#define __BAUDOT_H__
#include <Arduino.h>
#include "ArduTelex.h"

#define BAUDOT_MODE_UNDEFINED 0
#define BAUDOT_MODE_BU 31
#define BAUDOT_MODE_ZI 27

// for reverse baudot RX/TX bitorder
const char baudot_bu[] = "#t\ro hnm\nlrgipcvezdbsyfxawj\017uqk\016";
const char baudot_zi[] = "#5\r9 #,.\n)4#80:=3+@?'6#/-2%\01771(\016";
#define BAUDOT_CHAR_UNKNOWN 0

// for correct baudot RX/TX bitorder
//const char baudot_bu[] = "#e\na siu\rdrjnfcktzlwhypqobg\017mxv\016";
//const char baudot_zi[] = "#3\n- #87\r@4%,#:(5+)2#6019?#\017./=\016";

extern byte baudot_recieving_mode;
extern byte baudot_sending_mode;
extern char baudotToAscii(byte baudot);
extern byte asciiToBaudot(char ascii,byte *modeswtichcode);

#endif //ndef __BAUDOT_H__
