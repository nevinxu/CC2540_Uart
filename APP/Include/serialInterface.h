#ifndef _SERIALINTERFACE_
#define _SERIALINTERFACE_
#include "hal_uart.h"
#include "OSAL.h"
#include "npi.h"

#define SI_CMD_RX                                 0x0001
#define SI_EVT_TX                                 0x0002

#define MAX_PKT_SIZE 128
#define CMD_RESPONSE 2

#define SERIAL_IDENTIFIER       0x77


#define APP_CMD_SERIALCONNECT   0x01
#define APP_CMD_SERIALCONNECTACK   0x02
#define APP_CMD_SERIALDISCONNECT   0x0C
#define APP_CMD_SERIALDISCONNECTACK   0x0D
#define APP_CMD_ADVERTISE       0x03
#define APP_CMD_ADVERTISEACK       0x04
#define APP_CMD_ADVERTISEOVER       0x2E
#define APP_CMD_ADVERTISEBEGIN       0x0E
#define APP_CMD_CONNECTBLE       0x05
#define APP_CMD_CONNECTBLEACK       0x06
#define APP_CMD_DISCONNECTBLE      0x0A 
#define APP_CMD_DISCONNECTBLEACK      0x0B 
#define APP_CMD_NOTIFYBLE      0x07 
#define APP_CMD_NOTIFYBLEACK      0x08 
#define APP_CMD_DATASEND                0x09
#define APP_CMD_IDRECEIVEACK                0x0F
#define APP_CMD_IDRECEIVE                0x10
#define APP_CMD_AUTOCONNECT                0x11
#define APP_CMD_AUTOCONNECTACK                0x12
#define APP_CMD_RSSIVALUE                0x13
#define APP_CMD_RECEIVEECGDATA                0x14
#define APP_CMD_RECEIVEECGDATAACK                0x15
#define APP_CMD_STOPRECEIVEECGDATA                0x16
#define APP_CMD_STOPRECEIVEECGDATAACK                0x17
#define APP_CMD_ECGPATCHID                      0x18
#define APP_CMD_ECGPATCHIDACK                0x19
#define APP_CMD_SETECGPATCHID                      0x34
#define APP_CMD_SETECGPATCHIDACK                0x35
#define APP_CMD_AUTOCONNECTSTATUS                0x1A
#define APP_CMD_AUTOCONNECTSTATUSACK                0x1B
#define APP_CMD_CentralMAC               0x1C
#define APP_CMD_CentralMACACK               0x1D
#define APP_CMD_ECGPatchMAC               0x1E
#define APP_CMD_ECGPatchMACACK               0x1F
#define APP_CMD_PairingSTART               0x20
#define APP_CMD_PairingSTARTACK               0x21
#define APP_CMD_PairingStatus               0x22
#define APP_CMD_PairingStatusACK               0x23
#define APP_CMD_SETRUMMODE               0x24
#define APP_CMD_SETRUMMODEACK               0x25
#define APP_CMD_SET1MVVALUE               0x28
#define APP_CMD_SET1MVVALUEACK               0x29
#define APP_CMD_SET0MVVALUE               0x2C
#define APP_CMD_SET0MVVALUEACK               0x2D
#define APP_CMD_GET1MVVALUE               0x26
#define APP_CMD_GET1MVVALUEACK               0x27
#define APP_CMD_GET0MVVALUE               0x2A
#define APP_CMD_GET0MVVALUEACK               0x2B
#define APP_CMD_RECEIVEPACKAGENUMVALUE              0x36

#define APP_CMD_ECGDATAOKREQ                    0x37    //心电数据接收回应

#define APP_CMD_ECGPATCHHARDVERSIONACK          	0x30
#define APP_CMD_ECGPATCHHARDVERSIONREQ        		0x31
#define APP_CMD_ECGPATCHSOFTVERSIONACK          	0x32
#define APP_CMD_ECGPATCHSOFTVERSIONREQ        		0x33

// Serial Event Mnemonic        Opcode
#define	APP_EVT_CMD_RESPONSE	0x81
#define APP_EVT_CONNECT	        0x82 // Not used
#define	APP_EVT_DISCONNECT	0x83 // Not used


typedef struct _BLEPacketHeader
{
uint8 identifier;	
uint8 opCode;
uint8 status;
} BLEPacketHeader_t;


typedef struct _BLEPacket
{
BLEPacketHeader_t header;
// payload
uint8 length;  // opCode specific, 
uint8 data[32];  // command or event specific data, 10 Bytes currently
} BLEPacket_t;

typedef struct _SerialEventPacket
{
BLEPacketHeader_t header;
// payload
uint8 length;  // Length 
uint8 cmdCode; // Command Code
uint8 status;  // Status
} SerialEventPacket_t;


/* States for CRC parser */
typedef enum {
  NPI_SERIAL_STATE_ID,
  NPI_SERIAL_STATE_OPCODE,
  NPI_SERIAL_STATE_STATUS,
  NPI_SERIAL_STATE_LEN,
  NPI_SERIAL_STATE_DATA,
} npi_serial_parse_state_t;

extern void cSerialPacketParser( uint8 port, uint8 events );
extern void parseCmd(void);
extern void sendSerialEvt(void);
extern void SendPCCommand(uint8 command);


/*******************************************************************************
 * MACROS
 */


/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void SerialInterface_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events );

#endif
