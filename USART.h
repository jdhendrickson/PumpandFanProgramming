/* Functions to send and receive over USART */

#ifndef BAUD                          /* if not defined in Makefile... */
#define BAUD  9600                    /* set a safe default baud rate */
#endif

#define   USART_HAS_DATA   bit_is_set(UCSR0A, RXC0)
#define   USART_READY      bit_is_set(UCSR0A, UDRE0)

/* Blocking transmit and receive functions.
   When you call receiveByte() your program will hang until
   data comes through. */
void transmitByte(uint8_t data);
uint8_t receiveByte(void);

void printString(const char myString[]);
/* Utility function to transmit an entire string from RAM */

void readString(char myString[], uint8_t maxLength);
/* Define a string variable, pass it to this function
   The string will contain whatever you typed over serial */

/* Prints a byte out as its 3-digit ascii equivalent */
void printByte(uint8_t byte);

/* Prints a word (16-bits) out as its 5-digit ascii equivalent */
void printWord(uint16_t word);

/* Prints a byte out in 1s and 0s */
void printBinaryByte(uint8_t byte);

char nibbleToHex(uint8_t nibble);
char nibbleToHexCharacter(uint8_t nibble);

/* Prints a byte out in hexadecimal */
void printHexByte(uint8_t byte);

/* Takes in up to three ascii digits,
   converts them to a byte when press enter */
uint8_t getNumber(void);