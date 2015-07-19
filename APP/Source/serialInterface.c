#include <string.h>
#include "hal_uart.h"
#include "serialInterface.h"
#include "hal_led.h"
#include "BLEPeripheral.h"
#include "bcomdef.h"
#include "peripheral.h"


uint8 rxbuffer[256];
uint16 rxbuffersize = 0;
extern gaprole_States_t gapProfileState;
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

#define PairMACPage                     250
#define PairMACAddr                     PairMACPage*512



static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg );

static uint8 serialInterface_TaskID; 
BLEPacket_t  rxSerialPkt;
BLEPacket_t  txSerialPkt;

void SerialInterface_Init( uint8 task_id )
{
  serialInterface_TaskID = task_id;   
  NPI_InitTransport(cSerialPacketParser);
  NPI_WriteTransport("hellwin",7);

}

uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
    
    if ( (pMsg = osal_msg_receive( serialInterface_TaskID )) != NULL )
    {
      SerialInterface_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      
      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  if ( events & SI_CMD_RX )
  {
    parseCmd();
    return ( events ^ SI_CMD_RX);
  }
  
  if ( events & SI_EVT_TX)
  { 
    
    return ( events ^ SI_EVT_TX);
  }
  
  // Discard unknown events
  return 0;
}

static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  default:
    // do nothing
    break;
  }
}

void delay(unsigned int i)
{
  unsigned int x,y;
  for(x = 0;x < 110; x++)
    for(y = 0;y < i; y++);
}

void cSerialPacketParser( uint8 port, uint8 events )
{
  (void)port;
  npi_serial_parse_state_t  pktState = NPI_SERIAL_STATE_ID;
  uint8           done = FALSE;
  uint16          numBytes = 0;
  uint8    cmd_identifier = 0;
  uint8    cmd_opcode = 0;
  uint8   cmd_len = 0;
  uint8   buffer[50];
  
  if (events & HAL_UART_RX_TIMEOUT)
  {
    delay(50);
    // get the number of available bytes to process
    numBytes = NPI_RxBufLen();
    if(gapProfileState == GAPROLE_CONNECTED)
    {
      (void)NPI_ReadTransport(buffer, numBytes);
      VOID memcpy( &rxbuffer[rxbuffersize], buffer, numBytes );
      rxbuffersize += numBytes;
    }
    if(rxbuffersize>=256)
    {
      rxbuffersize = 0;
    }
    else
    {
        // check if there's any serial port data to process
        while ( (numBytes > 0) && (!done) )
        {
          // process serial port bytes to build the command or data packet
          switch( pktState )
          {
            
            case NPI_SERIAL_STATE_ID:   //…Ë±∏ID
              {
                (void)NPI_ReadTransport((uint8 *)&cmd_identifier, 1);
                // decrement the number of available bytes
                numBytes -= 1;
                
                if(cmd_identifier != SERIAL_IDENTIFIER)
                {
                  // illegal packet type
                  return;
                }
                rxSerialPkt.header.identifier = cmd_identifier;
                pktState = NPI_SERIAL_STATE_OPCODE;
                break;
              }
              
            case NPI_SERIAL_STATE_OPCODE:
              {
                // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
                (void)NPI_ReadTransport((uint8 *)&cmd_opcode, 1);     
                // decrement the number of available bytes
                numBytes -= 1;
                
                // set next state based on the type of packet
                switch( cmd_opcode )
                {
                  rxSerialPkt.header.opCode = cmd_opcode;
                  pktState = NPI_SERIAL_STATE_STATUS;
                  break;
                default:
                  pktState = NPI_SERIAL_STATE_ID;
                  // illegal packet type
                  return;
                }
                break;
              }
            case NPI_SERIAL_STATE_STATUS:
              {
                // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
                (void)NPI_ReadTransport((uint8 *)&cmd_opcode, 1);     
                // decrement the number of available bytes
                numBytes -= 1;
                rxSerialPkt.header.status = cmd_opcode;
                pktState = NPI_SERIAL_STATE_LEN;
              }
              
            case NPI_SERIAL_STATE_LEN: // command length
              {
                // read the length
                // Note: Assumes we'll get the data indicated by Hal_UART_RxBufLen.
                (void)NPI_ReadTransport((uint8 *)&cmd_len, 1);
                rxSerialPkt.length = cmd_len;
                // decrement the number of available bytes
                numBytes -= 1;
                if (numBytes == 0)
                {
                  // not enough data to progress, so leave it in driver buffer
                  osal_set_event( serialInterface_TaskID, SI_CMD_RX );
                  pktState = NPI_SERIAL_STATE_ID;
                  done = TRUE;
                  break;
                }
                pktState = NPI_SERIAL_STATE_DATA;
                break;
              }
              
            case NPI_SERIAL_STATE_DATA:       // command payload
              {
                // check if there is enough serial port data to finish reading the payload
                if ( numBytes < cmd_len )
                {
                  // not enough data to progress, so leave it in driver buffer
                  pktState = NPI_SERIAL_STATE_ID;
                  done = TRUE;
                  break;
                }
                (void) NPI_ReadTransport((uint8 *)rxSerialPkt.data, cmd_len);
                 pktState = NPI_SERIAL_STATE_ID;
                 done = TRUE;
                 
                // Note. using OSAL messaging instead is more effective
                osal_set_event( serialInterface_TaskID, SI_CMD_RX );
              }
            default:
              {
                (void)NPI_ReadTransport((uint8 *)&cmd_identifier, 1);
                // decrement the number of available bytes
                numBytes -= 1;
                if(numBytes == 0)
                {
                   done = TRUE;
                }
                pktState = NPI_SERIAL_STATE_ID;
              }
          }
      }
    }
  }
  else 
  {
    return;
  }
  
}

void parseCmd(void){
  
  uint8 opCode =  rxSerialPkt.header.opCode;

    switch (opCode) {
    
    break;
    } 
}

void sendSerialEvt(void){
  
 static  uint8 opCode;
 opCode =  txSerialPkt.header.opCode;
  switch (opCode) {
  HalUARTWrite(NPI_UART_PORT, (uint8*)&txSerialPkt, sizeof(txSerialPkt.header)+ txSerialPkt.length + 1);
  break;
    
  case APP_EVT_CONNECT:
    // Send connect event to external device
    break;
    
  case APP_EVT_DISCONNECT:
    // Send disconnect event to external device
    break;
    
  }
}


void SendPCCommand(uint8 command)
{
  txSerialPkt.header.identifier = SERIAL_IDENTIFIER;
  txSerialPkt.header.opCode = command;
  txSerialPkt.header.status = 0x00;
  txSerialPkt.length = 0;
  sendSerialEvt();
}
