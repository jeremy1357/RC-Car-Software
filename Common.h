// Jeremy Bowler

#ifndef COMMON_H_
#define COMMON_H_

#define F_CPU    8000000L
#define BAUD     9600
#define MyBaud   (F_CPU/BAUD/16) - 1
#define EEPROM1W 0b10100010
#define EEPROM1R 0b10100011
#define EEPROM2W 0b10100100
#define EEPROM2R 0b10100101 

#endif /* COMMON_H_ */