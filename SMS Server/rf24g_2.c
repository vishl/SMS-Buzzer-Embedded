//////////////////////////////////////////////////////////////////////////////// 
//                        Laipac RF-24G / TXRX24G 
//                   2.4GHz Wireless Transceiver Driver 
// 
// Original by micro222@yahoo.com 
//  Edited by Vishal Parikh 9/19/2010
// 
// Filename     : RF-24G_6-byte.c 
// Programmer   : Steven Cholewiak, www.semifluid.com 
// Version      : Version 1.01 - 01/30/2006 
// Remarks      : Datasheets for RF-24G / TXRX24G are available from: 
//                http://www.sparkfun.com/datasheets/RF/RF-24G_datasheet.pdf 
//                http://www.sparkfun.com/datasheets/RF/RF-24G.pdf 
//                http://www.sparkfun.com/datasheets/RF/nRF2401rev1_1.pdf 
//                http://store.qkits.com/moreinfo.cfm/txrx24g.pdf 
// 
//                This code is modified from the original to support more 
//                configuration options with more information provided about 
//                each configuration byte. 
// 
// History      : Version 1.01 - 01/30/2006 
//                - RF_24G_PAYLOADSIZE is now used to describe the packet payload and 
//                  adjusts DATA1_W, DATA2_W, and BUF_MAX accordingly 
// 
//                Version 1.00 - 01/21/2006 
//                - Initial release 
//////////////////////////////////////////////////////////////////////////////// 

#include <stdint.h>
#include  "msp430x20x2.h"
#include "binary.h"
#include "rf24g_2.h"

#define BIT_TEST(port, bit) ((port) & (bit))
#define BIT_SET(port, bit) port |= (bit)
#define BIT_CLEAR(port, bit) port &= ~(bit)


//Pin Configuration 
//    Pin   Name  Pin funtion Description 
//    1     GND   Power       Gound (0V) 
//    2     CE    Input       Chip Enable activates RX or TX mode 
//    3     CLK2  I/O         Clock output/input for RX data channel 2 
//    4     CS    Input       Chip Select activates Configuration mode 
//    5     CLK1  I/O         Clock Input(TX)&I/O(RX) for data channel 1 3-wire interface 
//    6     DATA  I/O         RX data channel 1/TX data input /3-wire interface 
//    7     DR1   Output      RX data ready at data channel 1 (ShockBurst only) 
//    8     DOUT2 Output      RX data channel 2 
//    9     DR2   Output      RX data ready at data channel 2 (ShockBurst only) 
//    10    VCC   Power       Power Supply (+3V DC) 

//PORTS
//NB: if you change these, make sure to update the dir/select registers in init()
#define RF_24G_CE_PORT          P2OUT
#define RF_24G_DATA_IN_PORT     P1IN
#define RF_24G_DATA_OUT_PORT    P1OUT
#define RF_24G_CLK1_PORT        P1OUT
#define RF_24G_DR1_PORT         P1IN
#define RF_24G_CS_PORT          P2OUT

#define RF_24G_CE_DIR           P2DIR
#define RF_24G_DATA_DIR         P1DIR
#define RF_24G_CLK1_DIR         P1DIR
#define RF_24G_DR1_DIR          P1DIR
#define RF_24G_CS_DIR           P2DIR

#define RF_24G_CE_BIT           BIT6
#define RF_24G_DATA_BIT         BIT4
#define RF_24G_CLK1_BIT         BIT5
#define RF_24G_DR1_BIT          BIT7
#define RF_24G_CS_BIT           BIT7



//Configuration Bytes 
// 
//Bytes 14-02: Shockburst Configuration 
//Bytes 01-00: General Device Configuration 

//Byte 14: Length of data payload section RX channel 2 in bits 
//    The total number of bits in a ShockBurst RF package may not exceed 256! 
//    Maximum length of payload section is hence given by: 
//    DATAx_W(bits) = 256 - ADDR_W - CRC 
#define DATA2_W            RF_24G_PAYLOADSIZE * 8 

//Byte 13: Length of data payload section RX channel 1 in bits 
#define DATA1_W            RF_24G_PAYLOADSIZE * 8 

//Byte 12-08: Channel 2 Address 
#define ADDR2_4            0x00 
#define ADDR2_3            0x00 
#define ADDR2_2            0x00 
#define ADDR2_1            0x42 
#define ADDR2_0            0x42 

//Byte 07-03: Channel 1 Address 
#define ADDR1_4            0x00 
#define ADDR1_3            0x00 
#define ADDR1_2            0x00 
#define ADDR1_1            0x42 
#define ADDR1_0            0x42 

//Byte 02 
//    Bit 07-02: ADDR_W    - Number of address bits (both RX channels) 
//                           Maximum number of address bits is 40 (5 bytes) 
//    Bit    01: CRC_L     - 8 or 16 bits CRC 
//    Bit    00: CRC_EN    - Enable on-chip CRC generation/checking 
//    Combine (via |) together constants from each group 
//                         b76543210    //NOTE bxxxxxxxx is a #define set in binary.h
#define ADDR_W_5_BYTE      b10100000 
#define ADDR_W_4_BYTE      b10000000 
#define ADDR_W_3_BYTE      b01100000 
#define ADDR_W_2_BYTE      b01000000 
#define ADDR_W_1_BYTE      b00100000 

#define CRC_L_8_BIT        b00000000 
#define CRC_L_16_BIT       b00000010 

#define CRC_EN_DISABLE     b00000000 
#define CRC_EN_ENABLE      b00000001 

//Byte 01 
//    Bit    07: RX2_EN    - Enable two channel receive mode 
//    Bit    06: CM        - Communication mode ( Direct or ShockBurst) 
//    Bit    05: RFDR_SB   - RF data rate (1Mbps requires 16MHz crystal) 
//    Bit 04-02: XO_F      - Crystal frequency (Factory default 16MHz crystal mounted) 
//    Bit 01-00: RF_PWR    - RF output power 
//    Combine (via |) together constants from each group 
//                         b76543210 
#define RX2_EN_DISABLE     b00000000 
#define RX2_EN_ENABLE      b10000000 

#define CM_DIRECT          b00000000 
#define CM_SHOCKBURST      b01000000 

#define RFDR_SB_250_KBPS   b00000000 
#define RFDR_SB_1_MBPS     b00100000 

#define XO_F_4MHZ          b00000000 
#define XO_F_8MHZ          b00000100 
#define XO_F_12MHZ         b00001100 
#define XO_F_16MHZ         b00001100 
#define XO_F_20MHZ         b00010000 

#define RF_PWR_N20DB       b00000000  // -20db 
#define RF_PWR_N10DB       b00000001  // -10db 
#define RF_PWR_N5DB        b00000010  // -5db 
#define RF_PWR_0DB         b00000011  // 0db (Full Power) 

//Byte 01 
//    Bit 07-01: RF_CH#    - Frequency channel (2400MHz + RF_CH# * 1.0MHz) 
//    Bit    00: RXEN      - RX or TX operation 
//    Combine (via |) together constants from each group 
//                         b76543210 
#define RF_CH              b10000000 // 64 - 2464GHz 

#define RXEN_TX            b00000000 
#define RXEN_RX            b00000001 

#define BUF_MAX            RF_24G_PAYLOADSIZE 
uint8_t RF_24G_Buffer[BUF_MAX]; 
//TODO do we need this delay business?
#define CLKDELAY()         /*delay_us(1) */
#define CSDELAY()          /*delay_us(10)*/ 
#define PWUPDELAY()        /*delay_ms(3) */

void RF_24G_init() 
{ 
    BIT_SET(RF_24G_DATA_DIR, RF_24G_DATA_BIT);    //output
    BIT_SET(RF_24G_CLK1_DIR, RF_24G_CLK1_BIT);    //output
    BIT_CLEAR(RF_24G_DR1_DIR, RF_24G_DR1_BIT);   //input
    BIT_CLEAR(P2SEL, RF_24G_CE_BIT);    //Use as gpio
    BIT_CLEAR(P2SEL, RF_24G_CS_BIT);    //Use as gpio
    BIT_SET(RF_24G_CE_DIR, RF_24G_CE_BIT);    //output
    BIT_SET(RF_24G_CS_DIR, RF_24G_CS_BIT);    //output
} 

void setOutput()
{
    BIT_SET(RF_24G_DATA_DIR, RF_24G_DATA_BIT);    //output
}

void setInput()
{
    BIT_CLEAR(RF_24G_DATA_DIR, RF_24G_DATA_BIT);    //input
}

void putByte( uint8_t b ) 
{  
    //MSB first 
    int8_t i; 
    for(i=0 ; i < 8 ; i++) { 
        BIT_CLEAR(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
        if( BIT_TEST(b,BIT7) ) { 
            BIT_SET(RF_24G_DATA_OUT_PORT, RF_24G_DATA_BIT); 
        }else{ 
            BIT_CLEAR(RF_24G_DATA_OUT_PORT, RF_24G_DATA_BIT); 
        } 
        b<<=1;
        CLKDELAY(); 
        BIT_SET(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT);  // Clock out on rising edge 
        CLKDELAY(); 
    } 
} 

uint8_t getByte() 
{  
    //MSB first 
    int8_t i, b = 0; 
    for(i=0 ; i < 8 ; i++) { 
        BIT_CLEAR(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
        CLKDELAY(); 
        BIT_SET(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
        CLKDELAY();           // Read before falling edge 
        if( BIT_TEST(RF_24G_DATA_IN_PORT, RF_24G_DATA_BIT) ) { 
            b|=1;
        } 
        if(i!=7)
            b<<=1;
    } 
    return b; 
} 

void RF_24G_Config() 
{ 
    BIT_CLEAR(RF_24G_CE_PORT, RF_24G_CE_BIT); 
    BIT_CLEAR(RF_24G_CS_PORT, RF_24G_CS_BIT); 
    BIT_CLEAR(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
    BIT_CLEAR(RF_24G_DATA_OUT_PORT, RF_24G_DATA_BIT); 
    PWUPDELAY(); 
    BIT_SET(RF_24G_CS_PORT, RF_24G_CS_BIT); 
    CSDELAY(); 

    //MSB byte first 
    putByte(DATA2_W); 
    putByte(DATA1_W); 
    putByte(ADDR2_4); 
    putByte(ADDR2_3); 
    putByte(ADDR2_2); 
    putByte(ADDR2_1); 
    putByte(ADDR2_0); 
    putByte(ADDR1_4); 
    putByte(ADDR1_3); 
    putByte(ADDR1_2); 
    putByte(ADDR1_1); 
    putByte(ADDR1_0); 
    putByte(ADDR_W_2_BYTE | CRC_L_16_BIT | CRC_EN_ENABLE); 
    //putByte(ADDR_W_2_BYTE | CRC_L_8_BIT | CRC_EN_ENABLE); 
    putByte(RX2_EN_DISABLE | CM_SHOCKBURST | RFDR_SB_1_MBPS | XO_F_16MHZ | RF_PWR_0DB); 
    //putByte(RF_CH | RXEN_RX); 
    putByte(RF_CH | RXEN_TX); 

    //OUTPUT_FLOAT(RF_24G_DATA); 
    BIT_CLEAR(RF_24G_CE_PORT, RF_24G_CE_BIT); 
    BIT_CLEAR(RF_24G_CS_PORT, RF_24G_CS_BIT); 
    BIT_CLEAR(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
} 

// Once the wanted protocol, modus and RF channel are set, 
// only one bit (RXEN) is shifted in to switch between RX and TX. 
void RF_24G_SetTx() 
{ 
    setOutput();
    BIT_CLEAR(RF_24G_CE_PORT, RF_24G_CE_BIT); 
    BIT_SET(RF_24G_CS_PORT, RF_24G_CS_BIT); 
    CSDELAY(); 
    BIT_CLEAR(RF_24G_DATA_OUT_PORT, RF_24G_DATA_BIT); 
    BIT_SET(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
    CLKDELAY(); 
    BIT_CLEAR(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
    CLKDELAY(); 
    BIT_CLEAR(RF_24G_CS_PORT, RF_24G_CS_BIT); 
    BIT_CLEAR(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
} 

// Once the wanted protocol, modus and RF channel are set, 
// only one bit (RXEN) is shifted in to switch between RX and TX. 
void RF_24G_SetRx() 
{ 
    setOutput();
    BIT_CLEAR(RF_24G_CE_PORT, RF_24G_CE_BIT); 
    BIT_SET(RF_24G_CS_PORT, RF_24G_CS_BIT); 
    CSDELAY(); 
    BIT_SET(RF_24G_DATA_OUT_PORT, RF_24G_DATA_BIT); 
    BIT_SET(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
    CLKDELAY(); 
    BIT_CLEAR(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
    CLKDELAY(); 
    BIT_CLEAR(RF_24G_CS_PORT, RF_24G_CS_BIT); 
    //OUTPUT_FLOAT(RF_24G_DATA); 
    BIT_CLEAR(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
    BIT_SET(RF_24G_CE_PORT, RF_24G_CE_BIT); 
    setInput();
} 

void putBuffer() 
{ 
    int8_t i; 
    BIT_SET(RF_24G_CE_PORT, RF_24G_CE_BIT); 
    CSDELAY(); 

    putByte(ADDR1_1); 
    putByte(ADDR1_0); 

    for( i=0; i<BUF_MAX ; i++) { 
        putByte(RF_24G_Buffer[i]); 
    } 
    BIT_CLEAR(RF_24G_CE_PORT, RF_24G_CE_BIT); 
    BIT_CLEAR(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
} 

int hasData()
{
    if(BIT_TEST(RF_24G_DR1_PORT, RF_24G_DR1_BIT)){ 
        return 1;
    }else{
        return 0;
    }
}

void getBuffer() 
{ 
    int8_t i; 
    for( i=0; i<BUF_MAX ; i++) { 
        RF_24G_Buffer[i] = getByte(); 
    } 
    BIT_CLEAR(RF_24G_CLK1_PORT, RF_24G_CLK1_BIT); 
    //wait for DR1 to go low
    while(hasData());
    BIT_SET(RF_24G_CE_PORT, RF_24G_CE_BIT); 
} 

//////////////////////////////////////////////////////////////////////////////// 
//Example Setup: 
// 
//    RF_24G_initPorts(); 
//    RF_24G_Config(); 
//    RF_24G_SetRx();    // Switch to receive 
//    RF_24G_Buffer[0] = 'A'; 
//    RF_24G_Buffer[1] = 'B';  // Not used 
//    RF_24G_Buffer[2] = 'C';  // Not used 
//    RF_24G_Buffer[3] = 'D';  // Not used 
// 
//    while(1) { 
//       if(BIT_TEST(RF_24G_DR1_PORT, RF_24G_DR1_BIT)){ 
//          getBuffer();   // Get packet 
//          putc(RF_24G_Buffer[0]); 
//       } 
// 
// 
//       // Transmit RF 
//       RF_24G_SetTx();     // switch to transmit 
//       delay_ms(1); 
//       putBuffer();      // send packet (RF_24G_Buffer) 
//       delay_ms(1);      // won't go back to recieve without this 
//       RF_24G_SetRx();     // switch back to receive 
//       delay_ms(1); 
//    } 


