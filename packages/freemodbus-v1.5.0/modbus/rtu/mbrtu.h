#ifndef __MBRTU__H
#define __MBRTU__H
#include "mbconfig.h"
#include "mbdatatype.h"
/* ----------------------- Defines ------------------------------------------*/
#define MB_SER_PDU_SIZE_MIN     4       /*!< Minimum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_MAX     256+1   /*!< Maximum size of a Modbus RTU frame. */
#define MB_SER_PDU_SIZE_CRC     2       /*!< Size of CRC field in PDU. */
#define MB_SER_PDU_ADDR_OFF     0       /*!< Offset of slave address in Ser-PDU. */
#define MB_SER_PDU_PDU_OFF      1       /*!< Offset of Modbus-PDU in Ser-PDU. */

/* ----------------------- Type definitions ---------------------------------*/
/*! \ingroup modbus
 * \brief Parity used for characters in serial mode.
 *
 * The parity which should be applied to the characters sent over the serial
 * link. Please note that this values are actually passed to the porting
 * layer and therefore not all parity modes might be available.
 */
typedef enum
{
    MB_PAR_NONE,                /*!< No parity. */
    MB_PAR_ODD,                 /*!< Odd parity. */
    MB_PAR_EVEN                 /*!< Even parity. */
} eMBParity;
typedef enum
{
    STATE_RX_INIT,              /*!< Receiver is in initial state. */
    STATE_RX_IDLE,              /*!< Receiver is in idle state. */
    STATE_RX_RCV,               /*!< Frame is beeing received. */
    STATE_RX_ERROR              /*!< If the frame is invalid. */
} eMBRcvState;
typedef enum
{
    STATE_TX_IDLE,              /*!< Transmitter is in idle state. */
    STATE_TX_XMIT               /*!< Transmitter is in transfer state. */
} eMBSndState;

typedef struct {
	void (*pvMBPortSerialInit)(ULONG ulBaudRate, eMBParity eParity);
	void (*pvMBPortTimersInit)( USHORT usTim1Timerout50us );
	void (*pvMBPortSerialGetByte)(UCHAR *pucByte);
	void (*pvMBPortSerialPutByte)(UCHAR ucByte);
	void (*pvMBPortTimersEnable)(USHORT Timerout50us);
	void (*pvMBPortTimersDisable)(void);
	void (*pvMBPortSerialEnable)(BOOL xRxEnable,BOOL xTxEnable);
	void (*pvMBEnterCriticalSection)(void);
	void (*pvMBExitCriticalSection)(void);	
}MB_RTU_PORT;

typedef struct _MB_RTU_DCB{
	UCHAR    			ucMBAddress;
	eMBRcvState			eRcvState;
	eMBSndState			eSndState;
	eMBEventType 		eQueuedEvent;
	BOOL     			xEventInQueue;
	USHORT 				ucBufferCount;	
	volatile UCHAR  	ucBuffer[MB_SER_PDU_SIZE_MAX];
	volatile UCHAR 		*pucBuffer;
	const MB_RTU_PORT *ucPort;
}MB_RTU_DCB;

void eMBRTUPoll(MB_RTU_DCB *const pMBRTU);

void eMBRTUInit(MB_RTU_DCB *const pMBRTU,const MB_RTU_PORT *pMBRTUPort,\
                UCHAR ucSlaveAddress,ULONG ulBaudRate, eMBParity eParity);

void xMBRTUTimerT35Expired(MB_RTU_DCB *const pMBRTU);

void eMBRTUStart( MB_RTU_DCB *const pMBRTU );

eMBErrorCode
eMBRTUSend(MB_RTU_DCB *const pMBRTU,const UCHAR * pucFrame, USHORT usLength );

eMBErrorCode 
eMBRTUReceive(MB_RTU_DCB *const pMBRTU, UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength );

void xMBRTUReceiveFSM(MB_RTU_DCB *const pMBRTU);
void xMBRTUTransmitFSM(MB_RTU_DCB *const pMBRTU);
#endif
