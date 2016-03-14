/*
        Chip control functions
*/

#ifndef rf24_ctrl
#define rf24_ctrl

#include <avr035.h>
#include <delay.h>

                       // PORTB REG (The Port B Data Register)
#define __MOSI_PRT     PORTB, PORTB3
#define __MISO_PRT     PORTB, PORTB4
#define __SCK_PRT      PORTB, PORTB5
#define __SS_PRT       PORTB, PORTB2
#define __CE_PRT       PORTB, PORTB1
                       // DDRB REG (The Port B Data Direction)
#define __MOSI_DDR     DDRB,  DDB3
#define __MISO_DDR     DDRB,  DDB4
#define __SCK_DDR      DDRB,  DDB5
#define __SS_DDR       DDRB,  DDB2
#define __CE_DDR       DDRB,  DDB1
                       // PINB REG (The Port B Input Pins Address)
#define __MOSI_IO      PINB,  PINB3
#define __MISO_IO      PINB,  PINB4
#define __SCK_IO       PINB,  PINB5
#define __SS_IO        PINB,  PINB2
#define __CE_IO        PINB,  PINB1

                       // INT pin (INT1, PD3)   we'll use FALLING front
#define __INT_PRT      PORTD, PORTD3
#define __INT_DDR      DDRD,  DDD3
#define __INT_PIN      PIND,  PIND3

#define __INT_SET      { EICRA |= (1<<ISC11); EIMSK |= (1<<INT1); EIFR = (1<<INTF1); }
#define __INT_RESET    { EIMSK &= ~(1<<INT1); EICRA &= ~(1<<ISC11); }

#define RF24_SPI_START { SPSR &= ~(1<<SPI2X); SPCR = (0<<SPR1)|(0<<SPR0)|(0<<DORD)|(1<<MSTR)|(1<<SPE); }  // fosc4 4 MHz @16 MHz MCU
#define RF24_SPI_END   { SPCR &= ~(1<<SPE); }

#define _TX_PULSE_DELAY      10     // us, CE pin pulse;  datashit assumes min 10 us

#define __CE_PULSE  { C_SETBIT(__CE_PRT); delay_us(_TX_PULSE_DELAY); C_CLEARBIT(__CE_PRT); }
#define __CE_LOW    C_CLEARBIT(__CE_PRT)
#define __CE_HIGH   C_SETBIT(__CE_PRT)
#define IS_CE_HIGH  C_CHECKBIT(__CE_IO)

unsigned char rf24_spi_cmd(unsigned char cmd, unsigned char *buf, unsigned char len);
void rf24_pins_init(void);




void rf24_pins_init(void) {
    C_SETBIT(__MOSI_DDR);     // MOSI as OUTPUT
    C_SETBIT(__SCK_DDR);      // SCK as OUTPUT
    C_SETBIT(__SS_DDR);       // SS as OUTPUT
    C_SETBIT(__SS_PRT);       // and HIGH
    C_CLEARBIT(__MISO_DDR);   // MISO as INPUT
    C_SETBIT(__MISO_PRT);     // and PULLUP
    C_CLEARBIT(__INT_DDR);    // INT pin as OUTPUT
    C_SETBIT(__INT_PRT);      // and PULLUP
    
    __INT_SET;                // INT pin interrupt (FALLING EDGE) handler set
}


unsigned char rf24_spi_cmd(unsigned char cmd, unsigned char *buf, unsigned char len) {

unsigned char status;
unsigned char *p;

    C_CLEARBIT(__SS_PRT);                       // SS -> LOW  i.e select slave
    SPDR = cmd;
    while (!(SPSR & _BV(SPIF)));
    status = SPDR;

    if (len) {
        for (p = buf+len-1; p >= buf; p--) {    // LSB first
            SPDR = *p;
            while (!(SPSR & _BV(SPIF)));
            *p = SPDR;
        };
    };

    C_SETBIT(__SS_PRT);                           // SS -> HIGH  i.e. release slave
    return status;
}


#endif
