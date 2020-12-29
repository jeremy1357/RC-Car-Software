// Jeremy T. Bowler
#include "Common.h"

static const char asctab[] = "0123456789ABCDEF";
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint8_t overfillsT0 = 0;
volatile uint8_t triggers = 0;
volatile int internEEPROMIndex = 0;
uint8_t boardMode = 2;

const float Kp = 150.0f;
const float Ki = 15.0f;
const float Kd = 1.5f;
const int MAX_SIZE = 2048;

int externEEPROMIndex1 = 0;
int externEEPROMIndex2 = 0;

void setDutyCycle(int percentage) {
	double mapped = round(2.55 * percentage);
	OCR1A = (uint8_t)mapped;
}

void convertHexToASCII(unsigned char inc, char *c1, char *c2) {
	(*c1) = asctab[(inc >> 4) & 0x0f];
	(*c2) = asctab[inc & 0x0f];
}

unsigned char TWIReadNACK(void){
	TWCR = (1 << TWINT) | (1 << TWEN);
	while((TWCR & (1<<TWINT)) == 0) {
		;
	}
	return TWDR;
}

void TWIStart(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while((TWCR & (1<<TWINT)) == 0) {
		;
	}
}

void TWIStop(void){
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

unsigned char TWIGetStatus(void){
	return (TWSR & 0xF8);
}

void TWIWrite(uint8_t u8data){
	TWDR = u8data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0) {
		;
	}
}

void selectExternalEEPROM(int device, int write) {
	if (device == 1) {
		if (write == 1) {
			TWIWrite(EEPROM1W);
		}
		else {
			TWIWrite(EEPROM1R);
		}
	}
	else if (device == 2) {
		if (write == 1) {
			TWIWrite(EEPROM2W);
		}
		else {
			TWIWrite(EEPROM2R);
		}
	}
	else {
		TWIWrite(EEPROM1W);		
	}
}

void writeExternalEEPROM(uint16_t u16address, uint8_t u8data, int device) {
	TWIStart();
	selectExternalEEPROM(device, 1);
	while(TWIGetStatus() != 0x18){
		TWIStart();
		while(TWIGetStatus() != 0x10) {
			;
		}
		selectExternalEEPROM(device, 1);
	}
	TWIWrite((uint8_t)(u16address >> 8));
	while(TWIGetStatus() != 0x28) {
		;
	}
	TWIWrite((uint8_t)(u16address));
	while(TWIGetStatus() != 0x28) {
		;
	}
	TWIWrite(u8data);
	while(TWIGetStatus() != 0x28) {
		;
	}
	_delay_us(100);
	TWIStop();
}

uint8_t readExternalEEPROM(uint16_t u16address, int device){
	TWIStart();
	while (TWIGetStatus() != 0x08) {
		;
	}
	selectExternalEEPROM(device, 1);
	while(TWIGetStatus() != 0x18) {
		TWIStart();
		while(TWIGetStatus() != 0x10);
		selectExternalEEPROM(device, 1);
	}
	TWIWrite((uint8_t)(u16address >> 8));
	while(TWIGetStatus() != 0x28) {
		;
	}
	TWIWrite((uint8_t)(u16address));
	while(TWIGetStatus() != 0x28) {
		;
	}
	_delay_us(1000);
	TWIStart();
	selectExternalEEPROM(device, 0);
	while(TWIGetStatus() != 0x40){
		TWIStart();
		while(TWIGetStatus() != 0x10) {
			;
		}
		selectExternalEEPROM(device, 0);
	}
	uint8_t dataRead = TWIReadNACK();
	_delay_us(100);
	TWIStop();
	return dataRead;
}

void intialize(unsigned int baudRate) {
	// Set pins as INPUT or OUTPUT
	DDRB |= (1 << PB4);   // MISO
	DDRB |= (1 << PB2);   // Relay enable
	DDRB |= (1 << PB1);   // PWM pin
	DDRB |= (1 << PB0);   // LED
	DDRC |= (1 << PC5) | (1 << PC4); // Outputs for SCL and SDA
	DDRC &= ~(1 << PC0);  // Set PC0 (ADC0) as an input
	DDRC &= ~(1 << PC1);  // Set PC1 (ADC1) as an input
	DDRC &= ~(1 << PC2);  // Set PC2 (ADC2) as an input
	DDRD &= ~(1 << PD7);  // Switch 5
	DDRD &= ~(1 << PD6);  // Switch 4
	DDRD |= (1 << PD5);   // CPE mode enable
	DDRD &= ~(1 << PD2);  // Int 0
	DDRD &= ~(1 << PD0);  // Rx
	DDRD |= (1 << PD1);   // Tx
	writeHexToUSART(DDRB);
	writeHexToUSART(DDRC);
	writeHexToUSART(DDRD);
	// ADC
	ADCSRA |= (1 << ADEN) | (1 << ADPS1);
	// Timer 0 (8-bit)
	TCCR0 = (1 << CS02); // 512 prescale
	TIMSK = (1 << TOIE0); // Timer 0 local interrupt enable
	// Timer 1 (16-bit)
	TCCR1A = (1 << WGM10);   // Phase correcting PWM mode 8-bit
	TCCR1A |= (1 << COM1A1); // Non-inverting mode
	TCCR1B = (1 << CS10);    // No prescale
	//setDutyCycle(0);
	// External Interrupt 0
	MCUCR |= (1 << ISC01) | (1 << ISC00);
	GICR  |= (1 << INT0);
	// USART
	UBRRH = (unsigned char)(baudRate >> 8);
	UBRRL = (unsigned char)(baudRate);
	UCSRB = (1 << RXEN)  | (1 << TXEN);
	UCSRC = (1 << URSEL) | (1 << USBS) | (3 << UCSZ0);
}

void blinkLED() {
	PORTB = ~(1 << PB0) & PORTB;
	_delay_ms(100);
	PORTB = (1 << PB0) | PORTB;
	_delay_ms(100);
}

uint8_t convertAsciiToHEX(unsigned char ch) {
	unsigned char temp;
	if (ch < 'A'){
		temp = ch - '0';
		} else {
		temp = ch - '7';
	}
	return temp;
}

void writeByteUSART(uint8_t data) {
	while (!(UCSRA & (1 << UDRE))) {
		;
	}
	UDR = data;
}

uint8_t readByteUSART(void) {
	while(!(UCSRA & (1<<RXC))) {
		;
	}
	return UDR;
}

void writeStringUSART(const char *msg) {
	int k = 0;
	while (msg[k] != '\0') {
		writeByteUSART(msg[k++]);
	}
}

void newlineUSART(void) {
	writeStringUSART("\n\r");
}

uint16_t buildUINT16(void) {
	uint16_t returnUINT16;
	uint8_t ch;
	ch = readByteUSART();
	writeByteUSART(ch);
	returnUINT16 = convertAsciiToHEX(ch) << 12;
	ch = readByteUSART();
	writeByteUSART(ch);
	returnUINT16 += convertAsciiToHEX(ch) << 8;
	ch = readByteUSART();
	writeByteUSART(ch);
	returnUINT16 += convertAsciiToHEX(ch) << 4;
	ch = readByteUSART();
	writeByteUSART(ch);
	returnUINT16 += convertAsciiToHEX(ch);
	return returnUINT16;
}

uint8_t buildUINT8(void) {
	uint8_t returnUINT8 = 0x00;
	uint8_t ch;
	ch = readByteUSART();
	writeByteUSART(ch);
	returnUINT8 += convertAsciiToHEX(ch) << 4; // Shift the four bits
	ch = readByteUSART();
	writeByteUSART(ch);
	returnUINT8 += convertAsciiToHEX(ch);
	return returnUINT8;
}

void writeInternalEEPROM(uint16_t address, uint8_t data) {
	while(EECR & (1 << EEWE)) {
		;
	}
	EEAR = address;
	EEDR = data;
	EECR |= (1 << EEMWE);
	EECR |= (1 << EEWE);
}

uint8_t readInternalEEPROM(uint16_t addr) {
	while(EECR & (1 << EEWE)) {
		;
	}
	EEAR = addr;
	EECR |= (1 << EERE);
	return EEDR;
}

// 0 = ADC0 = Battery Sense
// 1 = ADC1 = CPE Feedback
// 2 = ADC2 = Potentiometer Output
uint8_t readADC(uint8_t channel) {
	// Select the proper channel
	ADMUX = (ADMUX & 0xF0) | (0x0F & channel);
	ADCSRA |= (1 << ADSC);
	while (!(ADCSRA & (1 << ADIF))) {
		;
	}
	ADMUX |= (1 << ADLAR);
	return ADCH;
}

void enable_relay(void) {
	PORTB |= (1 << PB2);
}

void idleMode(void) {
	blinkLED();
	blinkLED();
	blinkLED();
	int shouldRun = 1;
	enable_relay();

	while (shouldRun == 1) 
	{
		writeStringUSART("0]Blink LED on PB0");
		newlineUSART();
		writeStringUSART("1]Sample and print ADC values");
		newlineUSART();
		writeStringUSART("2]Test read/write internal EEPROM");
		newlineUSART();
		writeStringUSART("3]Test read/write external EEPROM");
		newlineUSART();
		writeStringUSART("4]Print timer values");
		newlineUSART();
		writeStringUSART("5]Print interrupt count");
		newlineUSART();
		writeStringUSART("6]Export internal EEPROM values");
		newlineUSART();
		writeStringUSART("7]Export external EEPROM values");
		newlineUSART();
		writeStringUSART("8]Quit CPE Mode");
		newlineUSART();
		writeStringUSART("[CPE MODE] Enter Command: ");

		char command = readByteUSART();
		writeByteUSART(command);
		newlineUSART();
		switch (command) {
			case '0': {
				blinkLED();
				break;
			}
			case '1': {
				writeStringUSART("Enter number of samples to collect: ");
				unsigned int sampleCount = buildUINT16();
				newlineUSART();
				writeStringUSART("0] Battery Sense 1] CPE Feedback 2] Potentiometer Out");
				newlineUSART();
				writeStringUSART("Enter channel to sample: ");
				uint8_t channel = buildUINT8();
				newlineUSART();
				sampleADCToUSART((uint16_t)sampleCount, channel);
				break;
			}
			case '2': {
				writeStringUSART("Enter internal EEPROM address: ");
				unsigned int address = buildUINT16();
				newlineUSART();
				writeStringUSART("Enter data (byte): 0x");
				uint8_t data = buildUINT8();
				newlineUSART();
				writeInternalEEPROM(address, data);
				writeStringUSART("Reading same address: 0x");
				data = readInternalEEPROM(address);
				writeHexToUSART(data);
				newlineUSART();
				break;
			}
			case '3': {
				writeStringUSART("Enter external EEPROM address: ");
				unsigned int address = buildUINT16();
				newlineUSART();
				writeStringUSART("Enter data (byte): ");
				unsigned char data = buildUINT8();
				newlineUSART();
				writeExternalEEPROM(address, data, 1);
				writeStringUSART("Reading same address: 0x");
				data = readExternalEEPROM(address, 1);
				writeHexToUSART(data);
				break;
			}
			case '4': {
				writeStringUSART("Timer0 Value: 0x");
				writeHexToUSART(overfillsT0);
				break;
			}
			case '5': {
				writeStringUSART("External Interrupt Trigger Count: 0x");
				writeHexToUSART(triggers);
				break;
			}
			case '6': {
				exportInternEEPROM();
				break;
			}
			case '7': {
				exportPIDdata();
				break;
			}
			case '8': {
				writeStringUSART("Exiting CPE Mode!");
				newlineUSART();
				shouldRun = 0;
				break;
			}
			default:
			writeStringUSART("Invalid Command");
		}
		newlineUSART();
	}
}

void EE_PID(void) {
	enable_relay();
	PORTD &= ~(1 << PD5); // Disable CPE mode (assuming 0 = off)
}

void CPE_PID(void) {
	enable_relay();
	PORTD |= (1 << PD5); // Enable CPE mode (assuming 1 = on)

}

void exportInternEEPROM(void) {
	for (int i = 0; i < 512; i++) {
		uint8_t data = readInternalEEPROM((uint16_t)i);
		char c1, c2;
		convertHexToASCII(data, &c1, &c2);
		writeByteUSART(c1);
		writeByteUSART(c2);
		writeByteUSART(',');
	}	
}


void exportPIDdata(void) 
{
	// Desired
	for (int i = 0; i < MAX_SIZE; i++) {
		uint8_t reading = readExternalEEPROM((uint16_t)i, 1);
		char c1, c2;
		convertHexToASCII(reading, &c1, &c2);
		writeByteUSART(c1);
		writeByteUSART(c2);
		writeByteUSART(',');
	}
	newlineUSART();
	newlineUSART();
	newlineUSART();
	newlineUSART();	
	
	// Current
	for (int i = MAX_SIZE; i < MAX_SIZE * 2; i++) {
		uint8_t reading = readExternalEEPROM((uint16_t)i, 1);
		char c1, c2;
		convertHexToASCII(reading, &c1, &c2);
		writeByteUSART(c1);
		writeByteUSART(c2);
		writeByteUSART(',');
	}
}

void sampleADCToUSART(uint16_t numSamples, uint8_t channel) {
	for (uint16_t i = 0; i < numSamples; i++) {
		uint8_t ADCreading = readADC(channel);
		char c1, c2;
		convertHexToASCII(ADCreading, &c1, &c2);
		writeByteUSART(c1);
		writeByteUSART(c2);
		writeByteUSART(',');
	}
	newlineUSART();
}

void writeHexToUSART(uint8_t hexValue) {
	char c1, c2;
	convertHexToASCII(hexValue, &c1, &c2);
	writeByteUSART(c1);
	writeByteUSART(c2);
	newlineUSART();
}

// Timer 0 Overfill
ISR(TIMER0_OVF_vect) {
	if (overfillsT0 <= 0xFE) {
		overfillsT0++;
	}
	// CPE Mode
	if (boardMode == 1) {
		// Desired value
		uint8_t potOut   = readADC((uint8_t)2);
		// Current value
		uint8_t feedback = readADC((uint8_t)1);
		writeExternalEEPROM(externEEPROMIndex1++, potOut, 1);
		writeExternalEEPROM(externEEPROMIndex2++, feedback, 1);
		if (externEEPROMIndex1 == MAX_SIZE) {
			externEEPROMIndex1 = 0;
		}
		if (externEEPROMIndex2 == MAX_SIZE * 2) {
			externEEPROMIndex2 = MAX_SIZE;
		}

		calculateSpeed(potOut, feedback);
	}
	// EE Mode
	else if (boardMode == 0) {
		setDutyCycle(0); // Just to be safe
	}
	else {
		overfillsT0 = 0;
	}
}

// External Interrupt 0
ISR(INT0_vect) {
	triggers++; // Only kept for debug m6ode
	if (boardMode == 0) {
		writeByteUSART('a');
		writeInternalEEPROM(internEEPROMIndex++, overfillsT0);
	}
	overfillsT0 = 0;
	if (internEEPROMIndex == 512 && boardMode == 0) {
		internEEPROMIndex = 0;
	}
}

void calculateSpeed(uint8_t desiredData, uint8_t currentData) {
	//Integration Error
	static float ei = 0.0f;
	//New Error Value, Output, and Error Difference
	float error_val_n, out, ed;
	//Past Error Value
	static float error_val_p = 0.0;
	//Direct Constant
	float kp = Kp;
	//Integration Constant
	float ki = Ki;
	//Difference Constant
	float kd = Kd;
	
	error_val_n = (float)desiredData - (float)currentData;

	//Then take Difference between Error Values
	ed  = error_val_n - error_val_p;

	//Compound the error every speed check
	ei += error_val_n;

	// was 205, changed . Changed to allow ei*ki to be twice the range of "out"
	// this will allow for some integral wind up but not too much.  Also allows integral error
	// on its own to make the duty cycle 0% or 100%.
	if(ei > (500/ki)) 
	{
		ei = 500/ki;
	}
	else if(ei < -(500/ki)) 
	{
		ei = -500/ki;
	}
	//The output of the Speed Control Algorithm
	out = floor(error_val_n * kp + ei * ki + ed * kd);
	if (out > 255.0) 
	{
		out = 255.0;
	}
	if (currentData >= desiredData) 
	{
		out = desiredData;
	}
	else if (out < 0.0) 
	{
		out = 0.0;
	}
	uint8_t percentage = round(2.55 * out);
	OCR1A = percentage;
	error_val_p = error_val_n;	
}

int main(void) {
	blinkLED();
	intialize(MyBaud);
	sei();
	externEEPROMIndex2 = MAX_SIZE;
	boardMode = ((PIND & (1 << PD7)) >> 6) | ((PIND & (1 << PD6)) >> 6);
	writeStringUSART("Board Mode: ");
	writeHexToUSART(boardMode);
	while (1) {
		boardMode = ((PIND & (1 << PD7)) >> 6) | ((PIND & (1 << PD6)) >> 6);

		switch (boardMode) {
			case 0:
			EE_PID();
			writeByteUSART('q');
			break;
			case 1:
			CPE_PID();
			break;
			default:
			idleMode();
		}
	}
	return 0;
}
