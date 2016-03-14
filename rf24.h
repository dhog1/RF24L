/*
                    RF24L means RF24 Light
        nRF24L01 and compatible (nRF24L01P, SE8R01) functions
        
        Mars, 2016  by Dmitry aka dhog1
*/

#ifndef RF24_H
#define RF24_H

#define _RF24_DEBUG

#include <rf24_ctrl.h>    // chip control functions
#include <delay.h>        // standart delay functions: delay_us() and delay_ms()

        // SPI commands
                    // Every new command must be started by a high to low transition on  CSN (SS) pin.
                    // The STATUS register is serially shifted out on the  MISO  pin simultaneously to the SPI command
                    // word shifting to the  MOSI  pin.
                    // The serial shifting SPI commands is in the following format:
                    // <Command word: MSBit to LSBit (one byte)>
                    // <Data bytes: LSByte to MSByte, MSBit in each byte first>
#define R_REGISTER          0x00    // 000AAAAA   1 to 5 LSByte first  Read command and status registers.
                                    // AAAAA = 5 bit Register Map Address
#define W_REGISTER          0x20    // 001AAAAA   1 to 5 LSByte first Write command and status registers.
                                    // AAAAA = 5 bit Register Map Address
                                    // Executable in power down or standby modes only.
#define R_RX_PAYLOAD        0x61    // 1 to 32 bytes LSByte first Read RX-payload: 1 – 32 bytes.
                                    // A read operation always starts at byte 0. Payload is deleted from
                                    // FIFO after it is read. Used in RX mode.
#define W_TX_PAYLOAD        0xA0    // 1 to 32 bytes LSByte first Write TX-payload: 1 – 32 bytes. A write operation
                                    // always starts at byte 0 used in TX payload.
#define FLUSH_TX            0xE1    // Flush TX FIFO, used in TX mode
#define FLUSH_RX            0xE2    // Flush RX FIFO, used in RX mode
                                    // Should not be executed during transmission of acknowledge, that is,
                                    // acknowledge package will not be completed.
#define REUSE_TX_PL         0xE3    // Used for a PTX device. Reuse last transmitted payload. TX payload reuse is
                                    // active until W_TX_PAYLOAD or FLUSH TX is executed. TX payload reuse must not be
                                    // activated or deactivated during package transmission.
#define R_RX_PL_WID         0x60    // The bits in the FEATURE register have to be set.
                                    // 1 data byte. Read RX payload width for the top R_RX_PAYLOAD  in the RX FIFO.
                                    // Note: Flush RX FIFO if the read value is larger than 32 bytes.
#define W_ACK_PAYLOAD       0xA8    // 10101PPP  PPP - data pipe nbr. 1 to 32 bytes LSByte first.
                                    // The bits in the FEATURE register have to be set.
                                    // Used in RX mode. Write Payload to be transmitted together with ACK packet on
                                    // PIPE PPP. (PPP valid in the range from 000 to 101). Maximum three ACK
                                    // packet payloads can be pending. Payloads with same PPP are handled using
                                    // first in - first out principle. Write payload: 1 – 32 bytes. A write
                                    // operation always starts at byte 0.
#define W_TX_PAYLOAD_NOACK  0xB0    // The bits in the FEATURE register have to be set.
                                    // 1 to 32 bytes LSByte first. Used in TX mode. Disables  AUTOACK  on this
                                    // specific packet.
#define NOP                 0xFF    // No Operation. Might be used to read the  STATUS  register

                        // The  W_REGISTER  and  R_REGISTER  commands operate on single or multi-byte registers.
                        // When accessing multi-byte registers read or write to the MSBit of LSByte first.
                        // You can terminate the writing before all bytes in a multi-byte register are written,
                        // leaving the unwritten MSByte(s) unchanged. For example, the LSByte of  RX_ADDR_P0
                        // can be modified by writing only one byte to the  RX_ADDR_P0  register. The content of the
                        // status register is always read to  MISO  after a high to low transition on  CSN .

        // Register map table
#define REG_CONFIG  0x00
    #define BIT_MASK_RX_DR  6   // (Data Received) 0 enable INT; 1 disable INT; default 0
    #define BIT_MASK_TX_DS  5   // (Data Sent) 0 enable INT; 1 disable INT; default 0
    #define BIT_MASK_MAX_RT 4   // (MAX RE-TRY) 0 enable INT; 1 disable INT; default 0
    #define BIT_EN_CRC      3   // Enable CRC. Forced high if one of the bits in the EN_AA is high; default 1
    #define BIT_CRCO        2   // CRC encoding scheme '0' - 1 byte '1' – 2 bytes; default 0
    #define BIT_PWR_UP      1   // 1: POWER UP, 0:POWER DOWN; default 0
    #define BIT_PRIM_RX     0   // RX/TX control   1: PRX, 0: PTX; default 0

#define REG_EN_AA   0x01        // Enhanced ShockBurst; Enable ‘Auto Acknowledgment’ Function Disable
                                // this functionality to be compatible with nRF2401
    #define BIT_ENAA_P5 5       // Enable auto acknowledgement data pipe 5; default 1
    #define BIT_ENAA_P4 4       // Enable auto acknowledgement data pipe 5; default 1
    #define BIT_ENAA_P3 3       // Enable auto acknowledgement data pipe 5; default 1
    #define BIT_ENAA_P2 2       // Enable auto acknowledgement data pipe 5; default 1
    #define BIT_ENAA_P1 1       // Enable auto acknowledgement data pipe 5; default 1
    #define BIT_ENAA_P0 0       // Enable auto acknowledgement data pipe 5; default 1

#define REG_EN_RXADDR   0x02    // Enabled RX Addresses
    #define BIT_ERX_P5 5        // Enable data pipe 5. default 0 (disable)
    #define BIT_ERX_P4 4        // Enable data pipe 4. default 0 (disable)
    #define BIT_ERX_P3 3        // Enable data pipe 3. default 0 (disable)
    #define BIT_ERX_P2 2        // Enable data pipe 2. default 0 (disable)
    #define BIT_ERX_P1 1        // Enable data pipe 1. default 1 (enable)
    #define BIT_ERX_P0 0        // Enable data pipe 0. default 1 (enable)

#define REG_SETUP_AW    0x03    // Setup of RX/TX Address Widths (common for all data pipes)
#define _SETUP_AW_MASK  0x03    // '00' - Illegal; '01' - 3 bytes; '10' - 4 bytes; '11' – 5 bytes (default)
                                // LSByte is used if address width is below 5 bytes

#define REG_SETUP_RETR  0x04    // Setup of Automatic Retransmission
                    //  Please take care when setting this parameter. If the ACK payload is more than 15 bytes in 2Mbps
                    // mode the ARD must be 500µS or more, if the ACK payload is more than 5 bytes in 1Mbps mode
                    // the ARD must be 500µS or more. In 250kbps mode (even when the payload is not in ACK) the ARD
                    // must be 500µS or more.
                    //
                    //  This is the time the PTX is waiting for an ACK packet before a retransmit is made.
                    // The PTX is in RX mode for 250µS (500µS in 250kbps mode) to wait for address match. If the
                    // address match is detected, it stays in RX mode to the end of the packet, unless ARD elapses.
                    // Then it goes to standby-II mode for the rest of the specified ARD. After the ARD
                    // it goes to TX mode and then retransmits the packet.
    #define _SETUP_RETR_ARD_MASK  0xF0  // Auto Retransmit Delay
                                        // ‘0000’ – Wait 250µS (default)
                                        // ‘0001’ – Wait 500µS
                                        // ‘0010’ – Wait 750µS
                                        // ……..
                                        // ‘1111’ – Wait 4000µS

    #define _SETUP_RETR_ARC_MASK  0x0F  // Auto Retransmit Count
                                        // ‘0000’ – Re-Transmit disabled (default '0011' = 3)
                                        // ‘0001’ – Up to 1 Re-Transmit on fail of AA
                                        // ……
                                        // ‘1111’ – Up to 15 Re-Transmit on fail of AA

#define REG_RF_CH     0x05      // RF Channel number (0 - 125; 2 - default)

#define REG_RF_SETUP  0x06
    #define BIT_CONT_WAVE    7  // Enables continuous carrier transmit when high
    #define BIT_RF_DR_LOW    5  // Set RF Data Rate to 250kbps
    #define BIT_RF_DR_HIGH   3  // Select between the high speed data rates. This bit is don’t care if RF_DR_LOW is set.
                                // Encoding: [RF_DR_LOW, RF_DR_HIGH]: ‘00’ – 1Mbps; ‘01’ – 2Mbps; ‘10’ – 250kbps
    #define _RF_SETUP_POWER_MASK 0x06  //  Set RF output power in TX mode
                                      //'00' -18dBm; '01' -12dBm; '10' -6dBm; '11' 0dBm   (default = 11 - 0dBm)

#define REG_STATUS    0x07      // In parallel to the SPI command word applied on the  MOSI  pin,
                                // the STATUS register is shifted serially out on the  MISO  pin
                                //
                                // The RX_DR IRQ is asserted by a new packet arrival event. The procedure for handling
                                // this interrupt should be:
                                // 1) read payload through SPI
                                // 2) clear RX_DR IRQ
                                // 3) read FIFO_STATUS to check if there are more payloads available in RX FIFO
                                // 4) if there are more data in RX FIFO, repeat from step 1).
    #define BIT_RX_DR          6    // Data Ready RX FIFO interrupt. Asserted when new data arrives RX FIFOc.
                                    // Write 1 to clear bit.
    #define BIT_TX_DS          5    // Data Sent TX FIFO interrupt. Asserted when packet transmitted on TX.
                                    // If  AUTO_ACK  is activated, this bit is set high only when ACK is received.
                                    // Write 1 to clear bit.
    #define BIT_MAX_RT         4    // Maximum number of TX retransmits interrupt. Write 1 to clear bit. If  MAX_RT
                                    //  is asserted it must be cleared to enable further communication.
    #define BIT_RX_P_NBR_MASK  0x0E // Data pipe number for the payload available for reading from  RX_FIFO
                                    // 000-101: Data Pipe Number
                                    // 110: Not Used
                                    // 111: RX FIFO Empty  (default)
    #define BIT_TX_FULL0       0    // TX FIFO full flag. 1: TX FIFO full. 0: Available locations in TX FIFO.

#define REG_OBSERVE_TX      0x08    // Transmit observe register
    #define _PLOS_CNT_MASK  0xF0    // Count lost packets. The counter is overflow protected to 15, and discontinues at
                                    // max until reset. The counter is reset by writing to  RF_CH .
    #define _ARC_CNT_MASK   0x0F    // Count retransmitted packets. The counter is reset when transmission
                                    // of a new packet starts.

#define REG_RPD             0x09
    #define BIT_RPD            0    // Received Power Detector. This register is called CD (Carrier Detect) in the
                                    // nRF24L01. The name is different in nRF24L01+ due to the different input
                                    // power level threshold for this bit.

#define REG_RX_ADDR_P0      0x0A    // Receive address data pipe 0. 5 Bytes maximum length. LSByte is written first.
                                    // Write the number of bytes defined by SETUP_AW. Default = 0xE7E7E7E7E7
#define REG_RX_ADDR_P1      0x0B    // Receive address data pipe 1. 5 Bytes maximum length. LSByte is written first.
                                    // Write the number of bytes defined by SETUP_AW. Default = 0xC2C2C2C2C2
#define REG_RX_ADDR_P2      0x0C    // Receive address data pipe 2. Only LSB. MSBytes are equal to RX_ADDR_P1[39:8]
                                    // Default 0xC3
#define REG_RX_ADDR_P3      0x0D    // Default 0xC4
#define REG_RX_ADDR_P4      0x0E    // Default 0xC5
#define REG_RX_ADDR_P5      0x0F    // Default 0xC6


#define REG_TX_ADDR         0x10    // Transmit address. Used for a PTX device only. (LSByte is written first)
                                    // Set RX_ADDR_P0 equal to this address to handle automatic acknowledge
                                    // if this is a PTX device with Enhanced ShockBurst enabled.
                                    // Default 0xE7E7E7E7E7

#define REG_RX_PW_P0        0x11    // Number of bytes in RX payload in data pipe 0 (1 to 32 bytes).
                                    // 0 Pipe not used; 1 = 1 byte; ...; 32 = 32 bytes
    #define _RX_PW_P_MASK   0x3F
#define REG_RX_PW_P1        0x12    // Number of bytes in RX payload in data pipe 1
#define REG_RX_PW_P2        0x13    // Number of bytes in RX payload in data pipe 2
#define REG_RX_PW_P3        0x14    // Number of bytes in RX payload in data pipe 3
#define REG_RX_PW_P4        0x15    // Number of bytes in RX payload in data pipe 4
#define REG_RX_PW_P5        0x16    // Number of bytes in RX payload in data pipe 5

#define REG_FIFO_STATUS     0x17    // FIFO Status Register. Read only.
    #define BIT_TX_REUSE    6       // Used for a PTX device. Pulse the CE pin high for at least 10µs to Reuse last
                                    // transmitted payload. TX payload reuse is active until  W_TX_PAYLOAD  or
                                    // FLUSH TX  is executed. TX_REUSE  is set by the SPI command REUSE_TX_PL ,
                                    // and is reset by the SPI commands W_TX_PAYLOAD  or  FLUSH TX
    #define BIT_TX_FULL     5       // TX FIFO full flag. 1: TX FIFO full. 0: Available locations in TX FIFO
    #define BIT_TX_EMPTY    4       // TX FIFO empty flag.1: TX FIFO empty.0: Data in TX FIFO
    #define BIT_RX_FULL     1       // RX FIFO full flag. 1: RX FIFO full. 0: Available locations in RX FIFO
    #define BIT_RX_EMPTY    0       // RX FIFO empty flag.1: RX FIFO empty.0: Data in RX FIFO

                            // ACK_PLD  32 bytes Write only
                            // Written by separate SPI command ACK packet payload to data pipe number PPP
                            // given in SPI command. Used in RX mode only. Maximum three ACK packet payloads
                            // can be pending. Payloads with same PPP are handled first in first out.

                            // TX_PLD  32 bytes Write only
                            // Written by separate SPI command TX data payload register 1 - 32 bytes.
                            // This register is implemented as a FIFO with three levels. Used in TX mode only.

                            // RX_PLD  32 bytes Read only
                            // Read by separate SPI command. RX data payload register. 1 - 32 bytes.
                            // This register is implemented as a FIFO with three levels.
                            // All RX channels share the same FIFO.

#define REG_DYNPD           0x1C    // Enable dynamic payload length. Default 0x00
    #define BIT_DPL_P5  5           // Enable dynamic payload length data pipe 5.(Requires  EN_DPL  and  ENAA_P5 )
    #define BIT_DPL_P4  4           // Enable dynamic payload length data pipe 4.(Requires  EN_DPL  and  ENAA_P5 )
    #define BIT_DPL_P3  3           // Enable dynamic payload length data pipe 3.(Requires  EN_DPL  and  ENAA_P5 )
    #define BIT_DPL_P2  2           // Enable dynamic payload length data pipe 2.(Requires  EN_DPL  and  ENAA_P5 )
    #define BIT_DPL_P1  1           // Enable dynamic payload length data pipe 1.(Requires  EN_DPL  and  ENAA_P5 )
    #define BIT_DPL_P0  0           // Enable dynamic payload length data pipe 0.(Requires  EN_DPL  and  ENAA_P5 )

#define REG_FEATURE         0x1D
    #define BIT_EN_DPL      2       // Enables Dynamic Payload Length. Default = 0
                            // If ACK packet payload is activated, ACK packets have dynamic payload lengths
                            // and the Dynamic Payload Length feature should be enabled for pipe 0 on the PTX and PRX.
                            // This is to ensure that they receive the ACK packets with payloads.
                            // If the ACK payload is more than 15 byte in 2Mbps mode the ARD must be 500µS or more,
                            // and if the ACK payload is more than 5 byte in 1Mbps mode the ARD must be 500µS or
                            // more. In 250kbps mode (even when the payload is not in ACK) the ARD must be 500µS or more.
    #define BIT_EN_ACK_PAY  1       // Enables Payload with ACK. Default = 0
    #define BIT_EN_DYN_ACK  0       // Enables the  W_TX_PAYLOAD_NOACK  command. Default = 0

//  Operational modes configuration
//
//  Mode        PWR_UP REG  PRIM_RX REG     CE pin          FIFO state
//  RX mode         1           1             1                -
//  TX mode         1           0             1             Data in TX FIFOs. Will empty all levels in TX FIFOs (a)
//  TX mode         1           0         min 10us pulse    Data in TX FIFOs.Will empty one level in TX FIFOs   (b)
//  Standby II      1           0             1             TX FIFO empty.
//  Standby I       1           -             0             No ongoing packet transmission.
//  Power Down      0           -             -                -
//
// (a) If CE is held high all TX FIFOs are emptied and all necessary ACK and possible retransmits are car-
// ried out. The transmission continues as long as the TX FIFO is refilled. If the TX FIFO is empty when
// the CE is still high, nRF24L01+ enters standby-II mode. In this mode the transmission of a packet is
// started as soon as the CSN is set high after an upload (UL) of a packet to TX FIFO.
// (b) This operating mode pulses the CE high for at least 10µs. This allows one packet to be transmitted.
// This is the normal operating mode. After the packet is transmitted, the nRF24L01+ enters standby-I mode.


#define RF24_REG       GPIOR2

            // RF24_REG (GPIOR2) layout
#define INT_BIT        7
#define RX_DR_BIT      6
#define TX_DS_BIT      5
#define MAX_RT_BIT     4
#define PWR_BIT        3
#define ROL_BIT        2
#define NEW_DATA_BIT   1
#define B_BIT          0

            // DELAY
#define _PLL_DELAY_VALUE           132    // us, need to/from standby state transition; datashit assumes min 130 us
#define _PWR_ON_DELAY_VALUE        5      // ms; datashit assumes 4.5 ms min for external crystal and Ls=90 mH
#define _CE_DELAY_VALUE            5      // us, Delay from CE positive edge to CSN low; datashit assumes 4 us
#define _SOME_DELAY_VALUE          4      // us, a little delay

#define _PLL_DELAY           delay_us(_PLL_DELAY_VALUE)
#define _PWR_ON_DELAY        delay_ms(_PWR_ON_DELAY_VALUE)
#define _CE_CSN_DELAY        delay_us(_CE_DELAY_VALUE)
#define _SOME_DELAY          delay_us(_SOME_DELAY_VALUE)

#define MAX_CHANNEL         126     // from 0 to 125 i.e. from 2.400GHz to 2.525GHz
#define ADDR_LENGTH          5      // default nRF24L01P address length

typedef enum {pwr_down=0, standby_1, standby_2, rx_mode, tx_mode} op_mode_enum;
// typedef enum {_18dBm=0, _12dBm, _6dBm, _0dBm} PA_value_enum;   // from min to max
typedef enum {PTX=0, PRX} PRole;  // Primery Role (PTX or PRX)
typedef enum {w_ack=0, w_fast, w_nack} write_mode_enum;

#ifdef _RF24_DEBUG
typedef struct {unsigned char status;
                unsigned char config;
                unsigned char role;
                unsigned char ce;
                unsigned char tx_fifo;
                unsigned char rx_fifo;
                unsigned char rx_addr [5];
                unsigned char rx_addrs [6];
                unsigned char tx_addr;
                unsigned char crc_len;
                unsigned char crc_bits;
                unsigned char pa_level;
                unsigned char speed;
                unsigned char rf_channel;
                unsigned char losses;
                unsigned char fifo;
                unsigned char open_pipe_nbr;
                } rf24_info_struct;
#endif
                                                    // 10 bytes;  default addresses
const unsigned char BASE_ADDR[] = {0xB9, 0xC3, 0xD8, 0x68};
const unsigned char LAST_ADDR[] = {0xC3, 0x3C, 0xE3, 0xC7, 0xE6, 0x79};

unsigned char rf24_init(PRole role, unsigned char pipe);

unsigned char rf24_set_ack_pld(unsigned char pipe_nbr, unsigned char *data, unsigned char len);
unsigned char rf24_write(unsigned char *data, unsigned char *len, unsigned char mode);
unsigned char rf24_read(unsigned char *data, unsigned char *len);

void rf24_stop(void);
void rf24_start(void);

void rf24_set_channel(unsigned char ch_nbr);
void rf24_pa_level(unsigned char level);
void rf24_set_retx(unsigned char ard, unsigned char arc);
void rf24_close(unsigned char pipes);
void rf24_flush(void);


#ifdef _RF24_DEBUG

void rf24_info(rf24_info_struct *p);
unsigned char rf24_read_reg(unsigned char reg);
unsigned char rf24_write_reg(unsigned char reg, unsigned char value);

#endif


// *************************************************************************************************************

interrupt [EXT_INT1] void ext_int1_isr(void) {              // External Interrupt 1 service routine

    RF24_SPI_START;
    RF24_REG |= ((rf24_spi_cmd(NOP, 0, 0) & 0x70)) | (1<<INT_BIT);
    RF24_SPI_END;
}


unsigned char rf24_init(PRole role, unsigned char pipe) {

unsigned char temp;
unsigned char addr[5];

    RF24_REG = 0;
    __CE_LOW;
    _PWR_ON_DELAY;
    RF24_SPI_START;

    temp = (1<<BIT_EN_CRC) | (1<<BIT_CRCO) | (1<<BIT_PWR_UP);   // CRC 2 bytes
    if (role) {
        temp |= (1<<BIT_PRIM_RX);       // PRX
        RF24_REG |= (1<<ROL_BIT);
    };
    rf24_spi_cmd(W_REGISTER | REG_CONFIG, &temp, 1);

    temp = (1<<BIT_ENAA_P5)|(1<<BIT_ENAA_P4)|(1<<BIT_ENAA_P3)|(1<<BIT_ENAA_P2)|(1<<BIT_ENAA_P1)|(1<<BIT_ENAA_P0);
    rf24_spi_cmd(W_REGISTER | REG_EN_AA, &temp, 1);

    if (role) temp = (1<<BIT_ERX_P0)|(1<<BIT_ERX_P1)|(1<<BIT_ERX_P2)|(1<<BIT_ERX_P3)|(1<<BIT_ERX_P4)|(1<<BIT_ERX_P5);
    else temp = (1<<BIT_ERX_P0);         // PTX
    rf24_spi_cmd(W_REGISTER | REG_EN_RXADDR, &temp, 1);

    temp = 0x03;    // address is 5 bytes length
    rf24_spi_cmd(W_REGISTER | REG_SETUP_AW, &temp, 1);

    temp = 0x40;     //  ARD  wait 1250us
    temp |= 0x03;    //  ARC  times retransmit
    rf24_spi_cmd(W_REGISTER | REG_SETUP_RETR, &temp, 1);

    temp = 84;      // = WiFi ch #14 and not used in Russia
    rf24_spi_cmd(W_REGISTER | REG_RF_CH, &temp, 1);

    temp |= 0x02;   //  1Mbps and  -12dBm --> ideal for testing
    rf24_spi_cmd(W_REGISTER | REG_RF_SETUP, &temp, 1);

    for (temp=0; temp < 4; temp++) addr[temp] = BASE_ADDR[temp];     // set RX and TX addresses
    addr[4] = LAST_ADDR[0];
    rf24_spi_cmd(W_REGISTER | REG_RX_ADDR_P0, addr, 5);

    for (temp=0; temp < 4; temp++) addr[temp] = BASE_ADDR[temp];
    addr[4] = LAST_ADDR[0];
    rf24_spi_cmd(W_REGISTER | REG_TX_ADDR, addr, 5);

    for (temp=0; temp < 4; temp++) addr[temp] = BASE_ADDR[temp];
    if (role) {         // PRX
        addr[4] = LAST_ADDR[1];
        rf24_spi_cmd(W_REGISTER | REG_RX_ADDR_P1, addr, 5);         // 5 bytes to pipe1 !!
            for (temp = 2; temp < 6; temp++) {
                addr[4] = LAST_ADDR[temp];
                rf24_spi_cmd(W_REGISTER | (REG_RX_ADDR_P0 + temp), &addr[4], 1);
        };
    } else {            // PTX
        addr[4] = LAST_ADDR[pipe];
        rf24_spi_cmd(W_REGISTER | REG_TX_ADDR, &addr[4], 1);
        addr[4] = LAST_ADDR[pipe];
        rf24_spi_cmd(W_REGISTER | REG_RX_ADDR_P0, &addr[4], 1);
    };

    temp = (1<<BIT_DPL_P5)|(1<<BIT_DPL_P4)|(1<<BIT_DPL_P3)|(1<<BIT_DPL_P2)|(1<<BIT_DPL_P1)|(1<<BIT_DPL_P0);
    rf24_spi_cmd(W_REGISTER | REG_DYNPD, &temp, 1);

    temp = (1<<BIT_EN_DPL)|(1<<BIT_EN_ACK_PAY)|(1<<BIT_EN_DYN_ACK);
    rf24_spi_cmd(W_REGISTER | REG_FEATURE, &temp, 1);

    RF24_SPI_END;
    
    if (role) RF24_REG |= (1<<ROL_BIT);
    RF24_REG |= (1<<PWR_BIT);

    return 0;
}

                                                           // mode = 0 no ack; = 1 start write; = 2 wait for ack
unsigned char rf24_write(unsigned char *data, unsigned char *len, unsigned char mode) {

unsigned char temp;

    if (RF24_REG & (1<<ROL_BIT)) return 0;   // PRX
    RF24_SPI_START;
    
    temp = rf24_spi_cmd(NOP, 0, 0);
    if (temp & (1<<BIT_TX_FULL0)) { RF24_SPI_END; return 0; };  // no room
    if (temp & 0x70) { RF24_SPI_END; return 0; };              // there are ints

    switch (mode) {
        case 0:
            rf24_spi_cmd(W_TX_PAYLOAD_NOACK, data, *len);
            __CE_PULSE;
            while (!(RF24_REG & (1<<TX_DS_BIT)));
            RF24_REG &= ~((1<<TX_DS_BIT)|(1<<INT_BIT));
            rf24_spi_cmd(W_REGISTER | REG_STATUS, &temp, 1);    // reset interrupt
        break;
        case 1:
            RF24_REG &= ~(1<<INT_BIT);
            rf24_spi_cmd(W_TX_PAYLOAD, data, *len);
            __CE_PULSE;
        break;
        case 2:
            RF24_REG &= ~((1<<INT_BIT)|(1<<NEW_DATA_BIT));
            rf24_spi_cmd(W_TX_PAYLOAD, data, *len);
            __CE_PULSE;
            while (!(RF24_REG & (1<<INT_BIT)));
            rf24_spi_cmd(W_REGISTER | REG_STATUS, &temp, 1);    // reset interrupt

            if (RF24_REG & (1<<MAX_RT_BIT)) {             // fail to tx
                RF24_REG &= ~((1<<MAX_RT_BIT)|(1<<INT_BIT));
                rf24_spi_cmd(FLUSH_TX, 0, 0);             // remove data from TX FIFO
                RF24_SPI_END; 
                return 0;
            } else {
                if (RF24_REG & (1<<RX_DR_BIT)) {         // ack payload received
                    RF24_REG &= ~(1<<RX_DR_BIT);
                    rf24_spi_cmd(R_RX_PL_WID, &temp, 1);    // pld length
                    if (temp > 32) { rf24_spi_cmd(FLUSH_RX, 0, 0); RF24_SPI_END; return 0;};
                    if (!temp) break;
                    *len = temp;
                    rf24_spi_cmd(R_RX_PAYLOAD, data, *len);     // read pld
                    RF24_REG |= (1<<NEW_DATA_BIT);
                };
            };
            RF24_REG &= ~((1<<INT_BIT)|(1<<TX_DS_BIT));            
        break;
    };
    
    RF24_SPI_END;
    return *len;      // if ack pld data-> data received and *len - these data length
}

                                                      // return pipe nbr; return data_len in len; data - rx buffer 
unsigned char rf24_read(unsigned char *data, unsigned char *len) {

unsigned char temp, pipe;

    if (!(RF24_REG & (1<<INT_BIT))) return 0x07;   // no event, no data received
    if (!(RF24_REG & (1<<ROL_BIT))) return 0x07;   // PTX
    
    RF24_SPI_START;

    __CE_LOW;
    temp = rf24_spi_cmd(NOP, 0, 0);
    pipe = (temp & 0x0F) >> 1;
    rf24_spi_cmd(W_REGISTER | REG_STATUS, &temp, 1);    // reset interrupts

    rf24_spi_cmd(R_RX_PL_WID, &temp, 1);    // pld length
    if (temp > 32) { rf24_spi_cmd(FLUSH_RX, 0, 0); RF24_SPI_END; return 0x07;};
    if (temp) {
        *len = temp;
        rf24_spi_cmd(R_RX_PAYLOAD, data, *len);     // read pld
        RF24_REG |= (1<<NEW_DATA_BIT);
        RF24_REG &= ~((1<<RX_DR_BIT)|(1<<TX_DS_BIT));
    };

    rf24_spi_cmd(R_REGISTER | REG_FIFO_STATUS, &temp, 1);
    if (temp & (1<<BIT_RX_EMPTY)) {
        __CE_HIGH;
        _PLL_DELAY;
        RF24_REG &= ~(1<<INT_BIT);
    };

    RF24_SPI_END;
    return pipe;
}


unsigned char rf24_set_ack_pld(unsigned char pipe_nbr, unsigned char *data, unsigned char len) {

unsigned char temp;
    
    if (!(RF24_REG & (1<<ROL_BIT))) return 0;   // PTX
    RF24_SPI_START;
    temp = rf24_spi_cmd(NOP, 0, 0);
    if (temp & (1<<BIT_TX_FULL0)) rf24_spi_cmd(FLUSH_TX, 0, 0);
    rf24_spi_cmd(W_ACK_PAYLOAD | (pipe_nbr &0x07), data, len);
    RF24_SPI_END;
    return len;
}


void rf24_set_channel(unsigned char ch_nbr) {

unsigned char temp;

    if (IS_CE_HIGH) __CE_LOW;
    
    RF24_SPI_START;
    
    temp = ch_nbr & 0x7F;
    rf24_spi_cmd(W_REGISTER | REG_RF_CH, &temp, 1);
    
    RF24_SPI_END;
    
    if (RF24_REG & (1<<ROL_BIT)) {       // PRX
        __CE_HIGH;
        _PLL_DELAY;
    };
}


void rf24_pa_level(unsigned char level) {        // level = 0 (-18dbm) = 1 (-12dbm) = 2 (-6dbm) = 3 (0dbm - max)

unsigned char temp;

    if (IS_CE_HIGH) __CE_LOW;
    
    RF24_SPI_START;

    rf24_spi_cmd(R_REGISTER | REG_RF_SETUP, &temp, 1);
    temp &= ~(_RF_SETUP_POWER_MASK);
    temp |= ((level & 0x03) << 1);
    rf24_spi_cmd(W_REGISTER | REG_RF_SETUP, &temp, 1);
    
    RF24_SPI_END;
    
    if (RF24_REG & (1<<ROL_BIT)) {       // PRX
        __CE_HIGH;
        _PLL_DELAY;
    };
}


void rf24_set_retx(unsigned char ard, unsigned char arc) {

unsigned char temp;

    if (IS_CE_HIGH) __CE_LOW;
    
    RF24_SPI_START;

    temp = ((ard & 0x0F) << 4) | (arc & 0x0F);
    rf24_spi_cmd(W_REGISTER | REG_SETUP_RETR, &temp, 1);
    
    RF24_SPI_END;
    
    if (RF24_REG & (1<<ROL_BIT)) {       // PRX
        __CE_HIGH;
        _PLL_DELAY;
    };
}


void rf24_flush(void) {

unsigned char temp;
    
    RF24_SPI_START;
    
    rf24_spi_cmd(R_REGISTER | REG_FIFO_STATUS, &temp, 1);
    if (RF24_REG & (1<<ROL_BIT)) {    // PRX
        if (temp & (1<<BIT_RX_FULL)) rf24_spi_cmd(FLUSH_RX, 0, 0);
    } else {            // PTX
        if (temp & (1<<BIT_TX_FULL)) rf24_spi_cmd(FLUSH_TX, 0, 0);
    };
    
    RF24_SPI_END;
}


void rf24_close(unsigned char pipes) {

unsigned char temp;

    if (!(RF24_REG & (1<<ROL_BIT))) return;    // PTX

    if (IS_CE_HIGH) __CE_LOW;
    
    RF24_SPI_START;
    
    temp = ~pipes & 0x3F;
    rf24_spi_cmd(W_REGISTER | REG_EN_RXADDR, &temp, 1);
    
    RF24_SPI_END;
    
    if (RF24_REG & (1<<ROL_BIT)) {       // PRX
        __CE_HIGH;
        _PLL_DELAY;
    };
}


void rf24_stop(void) {

unsigned char temp;

    __CE_LOW;
    
    RF24_SPI_START;
    
    rf24_spi_cmd(R_REGISTER | REG_CONFIG, &temp, 1);
    temp &= ~(1<<BIT_PWR_UP);
    rf24_spi_cmd(W_REGISTER | REG_CONFIG, &temp, 1);
    RF24_REG &= ~(1<<PWR_BIT);
    
    RF24_SPI_END;
}


void rf24_start(void) {

unsigned char temp;

    
    RF24_SPI_START;
    
    rf24_spi_cmd(R_REGISTER | REG_CONFIG, &temp, 1);
    temp |= (1<<BIT_PWR_UP);
    rf24_spi_cmd(W_REGISTER | REG_CONFIG, &temp, 1);
    RF24_REG |= (1<<PWR_BIT);
    
    RF24_SPI_END;
    
    _PWR_ON_DELAY;
    if (RF24_REG & (1<<ROL_BIT)) {      // PRX
        __CE_HIGH;
        _PLL_DELAY;
    };
}


#ifdef _RF24_DEBUG

void rf24_info(rf24_info_struct *p) {

unsigned char i, temp;
unsigned char buf[5];

    RF24_SPI_START;

    p->status = rf24_spi_cmd(NOP, 0, 0);
    rf24_spi_cmd(R_REGISTER | REG_CONFIG, &temp, 1);
    p->config = temp;
    p->crc_len = temp & (1<<BIT_CRCO);
    if (temp & (1<<BIT_PRIM_RX)) p->role = 1; else p->role = 0;
    if (IS_CE_HIGH) p->ce = 1; else p->ce = 0;
    rf24_spi_cmd(R_REGISTER | REG_FIFO_STATUS, &temp, 1);
    p->fifo = temp & 0x3F;
    rf24_spi_cmd(R_REGISTER | REG_TX_ADDR, &temp, 1);
    p->tx_addr = temp;      
    rf24_spi_cmd(R_REGISTER | REG_RX_ADDR_P0, buf, 5);
    for (i = 0; i < 5; i++) p->rx_addr[4-i] = buf[i];
    p->rx_addrs[0] = buf[0];
    for (i = 0; i < 5; i++) {
        rf24_spi_cmd(R_REGISTER | REG_RX_ADDR_P1 + i, &temp, 1);
        p->rx_addrs[i+1] = temp;
    };
    rf24_spi_cmd(R_REGISTER | REG_RF_SETUP, &temp, 1);
    p->pa_level = (temp & _RF_SETUP_POWER_MASK) >> 1;
    p->speed = ((temp & (1<<BIT_RF_DR_LOW)) >> 4) | ((temp & (1<<BIT_RF_DR_HIGH)) >> 3);
    rf24_spi_cmd(R_REGISTER | REG_RF_CH, &temp, 1);
    p->rf_channel = temp;
    rf24_spi_cmd(R_REGISTER | REG_OBSERVE_TX, &temp, 1);
    p->losses = temp;
    rf24_spi_cmd(R_REGISTER | REG_EN_RXADDR, &temp, 1);
    p->open_pipe_nbr = temp;
    RF24_SPI_END;
}


void rf24_print_info(rf24_info_struct *p) {
}


unsigned char rf24_read_reg(unsigned char reg) {

unsigned char temp;

    RF24_SPI_START;
    rf24_spi_cmd(R_REGISTER | reg, &temp, 1);
    RF24_SPI_END;
    return temp;
}


unsigned char rf24_write_reg(unsigned char reg, unsigned char value) {

unsigned char temp, old_CE;

    if (IS_CE_HIGH) {old_CE = 1; __CE_LOW; } else old_CE = 0;
    RF24_SPI_START;
    temp = value;
    rf24_spi_cmd(W_REGISTER | reg, &temp, 1);
    _SOME_DELAY;
    rf24_spi_cmd(R_REGISTER | reg, &temp, 1);
    RF24_SPI_END;
    if (old_CE) { __CE_HIGH; _PLL_DELAY; };
    return temp;
}

#endif

#endif
