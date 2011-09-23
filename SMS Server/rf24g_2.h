//Payload size
typedef unsigned char uint8_t;
#define RF_24G_PAYLOADSIZE      6 
extern uint8_t RF_24G_Buffer[RF_24G_PAYLOADSIZE]; 

void RF_24G_init() ;
void RF_24G_Config() ;
void RF_24G_SetTx() ;
void RF_24G_SetRx() ;
void putBuffer() ;
void getBuffer() ;
int hasData();
