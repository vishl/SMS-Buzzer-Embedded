/*******************************************************************************
 * RF24G wireless serial driver
 *
 * Sets up the RF-24G for 256 bit payload including address and CRC
 *
23: 0 Payloads have an 8 bit address
22: 0
21: 1
20: 0
19: 0
18: 0
17: 1 16-Bit CRC
16: 1 CRC Enabled

15: 0 One channel receive
14: 1 ShockBurst Mode
13: 1 1Mbps Transmission Rate
12: 0
11: 1
10: 1
9: 1 RF Output Power
8: 0 RF Output Power

7: 0 Channel select (channel 2)
6: 0
5: 0
4: 0
3: 0
2: 1
1: 0
0: 0 Transmit mode
 * 
 ******************************************************************************/
#define BIT_TEST(port, bit) ((port) & (bit))
#define BIT_SET(port, bit) port |= (bit)
#define BIT_CLEAR(port, bit) port &= ~(bit)

#define TX_CE_PORT
#define TX_CS_PORT
#define TX_CLK1_PORT
#define TX_DATA_PORT
#define TX_CE_BIT
#define TX_CS_BIT
#define TX_CLK1_BIT
#define TX_DATA_BIT

#define RX_CE_PORT
#define RX_CS_PORT
#define RX_CLK1_PORT
#define RX_DATA_PORT
#define RX_DR_PORT
#define RX_CE_BIT
#define RX_CS_BIT
#define RX_CLK1_BIT
#define RX_DATA_BIT
#define RX_DR_BIT


#define PACKETSIZE 29
#define NODE_ADDRESS 17

uint8_t data_array[PACKETSIZE];
uint8_t counter;
uint8_t packet_number;

void RF24G_configureReceiver(void);
void RF24G_configureTransmitter(void);
void RF24G_transmitData(void);
void RF24G_receiveData(void);

//This will clock out the current payload into the data_array
void RF24G_receiveData(void)
{
    uint8_t i, j, temp;

    BIT_CLEAR(RX_CE_PORT, RX_CE_BIT);//Power down RF Front end

    //Erase the current data array so that we know we are looking at actual received data
    for(i = 0 ; i < PACKETSIZE ; i++)
        data_array[i] = 0x00;

    //Clock out the data
    for(i = 0 ; i < PACKETSIZE ; i++) 
    {
        for(j = 0 ; j < 8 ; j++) //8 bits each
        {
            temp <<= 1;
            if(BIT_TEST(RX_PORT, RX_BIT))
                temp |= 1;

            BIT_SET(RX_CLK1_PORT, RX_CLK1_BIT);
            BIT_CLEAR(RX_CLK1_PORT, RX_CLK1_BIT);

        }

        data_array[i] = temp; //Store this byte
    }

    //if(RX_DR == 0) //Once the data is clocked completely, the receiver should make DR go low
    //    printf("DR went low\n\r", 0);

    //printf("Received packet %d\n\r", data_array[0]);
    //This prints out the entire array - very large!
    /*
       printf("\n\rData Received:\n\r", 0);
       for(i = 0 ; i < 29 ; i++)
       {
       printf("[%d]", i);
       printf(" : %h - ", data_array[i]);
       }
       */


    BIT_SET(RX_CE_PORT, RX_CE_BIT);  //Power up RF Front End
}

//This sends out the data stored in the data_array
//data_array must be setup before calling this function
void RF24G_transmitData(void)
{
    uint8_t i, j, temp;

    BIT_SET(TX_CE_PORT, TX_CE_BIT);

    //Clock in address
    temp = NODE_ADDRESS;
    for(i = 0 ; i < 8 ; i++)
    {
        if(BIT_TEST(temp, BIT7)){
            BIT_SET(TX_DATA_PORT, TX_DATA_BIT);
        }else{
            BIT_CLEAR(TX_DATA_PORT, TX_DATA_BIT);
        }
        BIT_SET(TX_CLK1_PORT, TX_CLK1_BIT);
        BIT_CLEAR(TX_CLK1_PORT, TX_CLK1_BIT);

        temp <<= 1;
    }

    //Clock in the data_array
    for(i = 0 ; i < PACKETSIZE ; i++) 
    {
        temp = data_array[i];

        for(j = 0 ; j < 8 ; j++) //One bit at a time
        {
            if(BIT_TEST(temp, BIT7)){
                BIT_SET(TX_DATA_PORT, TX_DATA_BIT);
            }else{
                BIT_CLEAR(TX_DATA_PORT, TX_DATA_BIT);
            }
            BIT_SET(TX_CLK1_PORT, TX_CLK1_BIT);
            BIT_CLEAR(TX_CLK1_PORT, TX_CLK1_BIT);

            temp <<= 1;
        }
    }

    BIT_CLEAR(TX_CE_PORT, TX_CE_BIT); //Start transmission
}

//2.4G Configuration - Receiver
//This setups up a RF-24G for receiving at 1mbps
void RF24G_configureReceiver(void)
{
    uint8_t i, j, temp;
    uint8_t config_setup[14];

    //During configuration of the receiver, we need RX_DATA as an output
    //TODO!!!!

    //Config Mode
    BIT_CLEAR(RX_CE_PORT, RX_CE_BIT);
    BIT_SET(RX_CS_PORT, RX_CS_BIT);

    //Setup configuration array
    //===================================================================
    //Data bits 111-104 Max data width on channel 1 (excluding CRC and adrs) is 232
    config_setup[0] = PACKETSIZE*8;

    //Data bits 103-64 Channel 2 address - we don't care, set it to 200 
    config_setup[1] = 0;
    config_setup[2] = 0;
    config_setup[3] = 0;
    config_setup[4] = 0;
    config_setup[5] = 200;

    //Data bits 63-24 Channel 1 address - set it to 17
    config_setup[6] = 0;
    config_setup[7] = 0;
    config_setup[8] = 0;
    config_setup[9] = 0;
    config_setup[10] = NODE_ADDRESS;

    //Data bits 23-16 Address width(8) and CRC(16 bit, enabled)
    config_setup[11] = 0x23; //0b.0010.0011;

    //Data bits 15-8 
    config_setup[12] = 0x4D; //0b.0100.1101;

    //Data bits 7-0
    config_setup[13] = 0x0D; //0b.0000.1101;
    //===================================================================

    //Clock in configuration data
    for(i = 0 ; i < 14 ; i++)
    {
        temp = config_setup[i];

        for(j = 0 ; j < 8 ; j++)
        {   
            if(BIT_TEST(temp, BIT7)){
                BIT_SET(RX_DATA_PORT, RX_DATA_BIT);
            }else{
                BIT_CLEAR(RX_DATA_PORT, RX_DATA_BIT);
            }
            BIT_SET(RX_CLK1_PORT, RX_CLK1_BIT);
            BIT_CLEAR(RX_CLK1_PORT, RX_CLK1_BIT);

            temp <<= 1;
        }
    }

    //Configuration is actived on falling edge of CS (page 10)
    BIT_CLEAR(RX_CE_PORT, RX_CE_BIT);
    BIT_CLEAR(RX_CS_PORT, RX_CS_BIT);

    //After configuration of the receiver, we need RX_DATA as an input
    //TODO

    //Start monitoring the air
    BIT_SET(RX_CE_PORT, RX_CE_BIT);
    BIT_CLEAR(RX_CS_PORT, RX_CS_BIT);

    //printf("RX Configuration finished...\n\r", 0);

}    

//2.4G Configuration - Transmitter
//This sets up one RF-24G for shockburst transmission
void RF24G_configureTransmitter(void)
{
    uint8_t i, j, temp;
    uint8_t config_setup[14];

    //Config Mode
    BIT_CLEAR(TX_CE_PORT, TX_CE_BIT);
    BIT_SET(TX_CS_PORT, TX_CS_BIT);

    //Setup configuration array
    //===================================================================
    //Data bits 111-104 Max data width on channel 1 (excluding CRC and adrs) is 232
    config_setup[0] = PACKETSIZE*8;

    //Data bits 103-64 Channel 2 address - we don't care, set it to 200 
    config_setup[1] = 0;
    config_setup[2] = 0;
    config_setup[3] = 0;
    config_setup[4] = 0;
    config_setup[5] = 200;

    //Data bits 63-24 Channel 1 address - set it to 17
    config_setup[6] = 0;
    config_setup[7] = 0;
    config_setup[8] = 0;
    config_setup[9] = 0;
    config_setup[10] = NODE_ADDRESS;

    //Data bits 23-16 Address width and CRC
    config_setup[11] = 0x23; //0b.0010.0011;

    //Data bits 15-8 
    config_setup[12] = 0x4D; //0b.0100.1101;

    //Data bits 7-0
    config_setup[13] = 0x0C; //0b.0000.1100;
    //===================================================================

    //Clock in configuration data
    for(i = 0 ; i < 14 ; i++)
    {
        temp = config_setup[i];

        for(j = 0 ; j < 8 ; j++)
        {   
            if(BIT_TEST(temp, BIT7)){
                BIT_SET(TX_DATA_PORT, TX_DATA_BIT);
            }else{
                BIT_CLEAR(TX_DATA_PORT, TX_DATA_BIT);
            }
            BIT_SET(TX_CLK1_PORT, TX_CLK1_BIT);
            BIT_CLEAR(TX_CLK1_PORT, TX_CLK1_BIT);


            temp <<= 1;
        }
    }

    //Configuration is actived on falling edge of CS (page 10)
    BIT_CLEAR(TX_CE_PORT, TX_CE_BIT);
    BIT_CLEAR(TX_CS_PORT, TX_CS_BIT);

    //printf("TX Configuration finished...\n\r", 0);
}


