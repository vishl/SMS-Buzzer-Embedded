/******************************************************************************
 *                  MSP-EXP430G2-LaunchPad User Experience Application
 * 
 * 1. input interrupt on UART (RXD) -> enqueues command in buffer
 * 2. input interrupt on receive ready ->enqueues data in buffer
 * 3. main
 *      monitor command buffer
 *      if a "trigger" command is received, queue a message to the rf
 *      monitor rf message buffer
 *      when the response is received, send a message to cpu over uart
 *      if timeout, send message to cpu resend message to rf
 ******************************************************************************/

#include  "msp430x20x2.h"
#include "../SMS Server/rf24g_2.h"

#define OPEN_THRESH (24*1)

#define     false               0
#define     true                1
#define     LED0                BIT0
#define     LED1                BIT6
#define     LED_DIR             P1DIR
#define     LED_OUT             P1OUT


#define     BUTTON              BIT3
#define     BUTTON_OUT          P1OUT
#define     BUTTON_DIR          P1DIR
#define     BUTTON_IN           P1IN
#define     BUTTON_IE           P1IE
#define     BUTTON_IES          P1IES
#define     BUTTON_IFG          P1IFG
#define     BUTTON_REN          P1REN

#define     TXD                 BIT1                      // TXD on P1.1
#define     RXD                 BIT2                      // RXD on P1.2

//   Conditions for 9600/4=2400 Baud SW UART, SMCLK = 1MHz
#define     Bitime_5              0x05*4                      // ~ 0.5 bit length + small adjustment
#define     Bitime                13*4//0x0D    

#define     TEST 0b01010101

//public functions
void InitializeLeds(void);
void InitializeButton(void);
void InitializeSerial(void); 
void InitializeClocks(void);
void puts(const char * s);
void putc(const char c);
void putc_i(const char c);
int getc(char *c);
void EchoTest();
void RFTest();
void openDoor();
void closeDoor();
void mainLoop();

//private globals and functions
unsigned int TxData;
unsigned int RxData;
char BitCnt;
volatile int recvFlag=0;

void TX_Byte(void);
void RX_Ready(void);


/*******************************************************************************
 * Main
 ******************************************************************************/
void main(void)
{
    unsigned char i;
    WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer
    InitializeClocks();
    InitializeButton();
    InitializeLeds();
    InitializeSerial();
    RF_24G_init();
    RF_24G_Config();
    __enable_interrupt();                     

    puts("Client.\r\n");

    //EchoTest(); //doesn't return
    //RFTest();//doesn't return
    while(1){
        mainLoop();
    }
}

void EchoTest()
{
    char counter='0';
    char inchar;
    while(1){
        //test
        if(BUTTON_IN & BUTTON){
            LED_OUT&=~(LED0|LED1);
        }else{
            LED_OUT|=(LED0|LED1);
            counter++;
            if(counter>'9')
                counter='0';
            putc(counter);
        }
        if(getc(&inchar)){
            putc(inchar);   //echo
            puts(": ");
        	putc_i(inchar);
            puts("\r\n");
        }
    }
}

void RFTest()
{
    uint8_t i;
    RF_24G_SetRx() ;
    for(i=0; i<RF_24G_PAYLOADSIZE; i++){
        RF_24G_Buffer[i]=0;
    }

    while(1){
        puts("Waiting");
        while(!hasData());
        puts("Data");
        getBuffer();
        for(i=0; i<RF_24G_PAYLOADSIZE; i++){
            putc_i(RF_24G_Buffer[i]);
        }
    }
}

void mainLoop()
{
    static int now;
    static int lastOpenTime;
    static int open;
    static int count;
    int i;
    int correct=1;
    RF_24G_SetRx() ;
    puts("Waiting");
    while(!hasData()){
        if(open){
            puts("Open      ");
            count++;
            if(count>OPEN_THRESH){
                closeDoor();
                open=0;
            }
        }
    }
    puts("Signal");
    getBuffer();
    for(i=0; i<RF_24G_PAYLOADSIZE; i++){
        putc_i(RF_24G_Buffer[i]);
        if(RF_24G_Buffer[i]!=RF_24G_PAYLOADSIZE-i){
            correct=0;
        }
    }
    if(correct){
        openDoor();
        open=1;
        count=0;
        lastOpenTime=now;
        puts("Correct");
    }
}

void openDoor()
{
    LED_OUT|=(LED0|LED1);
}

void closeDoor()
{
    LED_OUT&=~(LED0|LED1);
}
void InitializeClocks(void)
{

    BCSCTL1 = CALBC1_1MHZ;                     // Set range
    DCOCTL = CALDCO_1MHZ;
    BCSCTL2 &= ~(DIVS_3);                      // SMCLK = DCO / 8 = 1MHz  
}

void InitializeButton(void)              
{
    BUTTON_DIR &= ~BUTTON;
    BUTTON_OUT |= BUTTON;
    BUTTON_REN |= BUTTON;
    BUTTON_IES |= BUTTON;
    BUTTON_IFG &= ~BUTTON;
    //BUTTON_IE |= BUTTON;
}


void InitializeLeds(void)
{
    LED_DIR |= LED0 + LED1;                          
    LED_OUT &= ~(LED0 + LED1);  
}

void InitializeSerial(void)
{
    CCTL0 = OUT;                               // TXD Idle as Mark
    //TACTL = TASSEL_1 + MC_2;                 // ACLK, continuous mode --from example file
    TACTL = TASSEL_2 + MC_2 + ID_3;            // SMCLK/8, continuous mode --from example project that shiped w/ board
    //P1SEL |= TXD + RXD;                      //
    P1SEL |= TXD ;                        
    P1DIR |= TXD;                              // TXD is output
    P1DIR &= ~RXD;                             // RXD is input
    P1IES=RXD;  //Falling edge

    recvFlag=0;
    RX_Ready();
}

void puts(const char * s)
{
    while(*s!=0){
        putc(*s);
        s++;
    }
}

void putc(const char c)
{
    TxData = c;
    TX_Byte();  //blocks till completion
    RX_Ready();
}

/*******************************************************************************
 * print a character as an integer
 ******************************************************************************/
void putc_i(const char c)
{
    if(c>=100){
        putc(c/100 + '0');
    }
    if(c>=10){
        putc((c%100)/10 + '0');
    }
    putc((c%10)+'0');
}

//returns true if a character was received
int getc(char *c)
{
    if(recvFlag){
        *c = RxData;
        recvFlag=false;
        return true;
    }else{
        return false;
    }
}

// Function Transmits Character from TxData Buffer
void TX_Byte (void)
{
    BitCnt = 0xA;                             // Load Bit counter, 8data + ST/SP
    while (CCR0 != TAR)                       // Prevent async capture
        CCR0 = TAR;                           // Current state of TA counter
    CCR0 += Bitime;                           // Some time till first bit
    TxData |= 0x100;                          // Add mark stop bit 
    TxData = TxData << 1;                     // Add space start bit
    CCTL0 =  CCIS0 + OUTMOD0 + CCIE;          // TXD = mark = idle
    while ( CCTL0 & CCIE );                   // Wait for TX completion
}


// Function Readies UART to Receive Character into RxData Buffer
void RX_Ready (void)
{
    BitCnt = 0;                             // Load Bit counter
    //CCTL0 = SCS + OUTMOD0 + CM1 + CAP + CCIE;   // Sync, Neg Edge, Cap
    P1IE = RXD;
    P1IFG &= ~RXD;  //setting IE may trigger IFG, so clear it
}

//called from interrupt
void RecvMode(void)
{
    //CCTL0 &= ~ CAP;                         // Switch from capture to compare mode
    CCTL0 = OUTMOD0 + CCIE;   // Sync, Neg Edge, Cap
    CCR0 = Bitime+TAR;
}

// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
    if(P1IFG & RXD){
        //got a start bit
        //RxData is now corrupted, so clear flag
        recvFlag=0;
        P1IE &= ~RXD;                           //Disable interrupt
        P1IFG &= ~RXD;                          // P1.4 IFG cleared
        //enable timer interrupt to receive remaining bits
        CCTL0 = OUTMOD0 + CCIE;   // Sync, Neg Edge, Cap
        //Sample the next bit at TAR + Bittime
        CCR0 = Bitime+TAR;
    }
}

// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
    CCR0 += Bitime;                                 // Add Offset to CCR0

    // TX
    if (CCTL0 & CCIS0)                              // TX on CCI0B?
    {
        if ( BitCnt == 0)
            CCTL0 &= ~ CCIE;                        // All bits TXed, disable interrupt
        else
        {
            CCTL0 |=  OUTMOD2;                      // TX Space
            if (TxData & 0x01)
                CCTL0 &= ~ OUTMOD2;                 // TX Mark
            TxData = TxData >> 1;
            BitCnt --;
        }
    }
    // RX
    else
    {
        if( CCTL0 & CAP )                           // Capture mode = start bit edge
        {
            CCTL0 &= ~ CAP;                         // Switch from capture to compare mode
            CCR0 += Bitime_5;                       //we do this at the top too..wtf?
        }
        else
        {
            if(BitCnt<8){                               
                RxData = RxData >> 1;
                if(P1IN & RXD){                     // Get bit waiting in receive latch
                    RxData |= 0x80;                 // input data
                }
            }
            if(BitCnt>=8)
            {
                CCTL0 &= ~ CCIE;                    //All bits RXed, disable interrupt
                //_BIC_SR_IRQ(LPM3_bits);           //Clear LPM3 bits from 0(SR)
                P1IFG &= ~RXD;                      //for some reason the interrupt flag is set at the end, so clear it
                recvFlag=1;
            }
            BitCnt++;                               
        }
    }
}

