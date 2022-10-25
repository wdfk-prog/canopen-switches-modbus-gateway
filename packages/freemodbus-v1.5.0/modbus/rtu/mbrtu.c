/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : 
  * @brief          : 
  * @date           :
  ******************************************************************************
  * @attention  mbconfig.h 需要配置
                为了使用多从机。已改，仅可使用RTU模式
  * @author 从网上下载多从机版本。再进行修改
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "mb.h"

/* Private includes ----------------------------------------------------------*/
#include "mbrtu.h"
#include "mbevent.h"

#include "assert.h"
/* Private define ------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
ULONG           usTimerT35_50us;
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  MB RTU 初始化
  * @param  None
  * @retval None
  * @note   
*/
void eMBRTUInit(MB_RTU_DCB *const pMBRTU,const MB_RTU_PORT *pMBRTUPort,\
                UCHAR ucSlaveAddress,ULONG ulBaudRate, eMBParity eParity)
{
	pMBRTU->ucPort = pMBRTUPort;
	pMBRTU->ucMBAddress = ucSlaveAddress;
  
  pMBRTU->ucPort->pvMBEnterCriticalSection();

	pMBRTU->ucPort->pvMBPortSerialInit(ulBaudRate,eParity);
  /* If baudrate > 19200 then we should use the fixed timer values
  * t35 = 1750us. Otherwise t35 must be 3.5 times the character time.
  */
  if( ulBaudRate > 19200 )
  {
    usTimerT35_50us = 35;       /* 1800us. */
  }
  else
  {
    /* The timer reload value for a character is given by:
    *
    * ChTimeValue = Ticks_per_1s / ( Baudrate / 11 )
    *             = 11 * Ticks_per_1s / Baudrate
    *             = 220000 / Baudrate
    * The reload for t3.5 is 1.5 times this value and similary
    * for t3.5.
    */
    usTimerT35_50us = ( 7UL * 220000UL ) / ( 2UL * ulBaudRate );
  }
  
	pMBRTU->ucPort->pvMBPortTimersInit( ( USHORT ) usTimerT35_50us );
  
	xMBPortEventInit(pMBRTU);
	
  pMBRTU->ucPort->pvMBExitCriticalSection();
}
/**
  * @brief  MB RTU 启动
  * @param  None
  * @retval None
  * @note   
*/
void eMBRTUStart( MB_RTU_DCB *const pMBRTU )
{
    pMBRTU->ucPort->pvMBEnterCriticalSection();

    pMBRTU->eRcvState = STATE_RX_INIT;
 
    pMBRTU->ucPort->pvMBPortSerialEnable(TRUE,FALSE);
  
    pMBRTU->ucPort->pvMBPortTimersEnable( ( USHORT ) usTimerT35_50us );

    pMBRTU->ucPort->pvMBExitCriticalSection();
}
/**
  * @brief  MB RTU 循环
  * @param  None
  * @retval None
  * @note   放入while 或者线程
*/
void eMBRTUPoll(MB_RTU_DCB *const pMBRTU)
{
    int             i;//自己设置变量
    USHORT   usLength;
    UCHAR   *ucMBFrame;
    UCHAR    ucRcvAddress;
    UCHAR    ucFunctionCode;
    eMBEventType    eEvent;
    eMBErrorCode    eStatus = MB_ENOERR;
    eMBException eException;
    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */
    if( xMBPortEventGet(pMBRTU,&eEvent) == TRUE )
    {
        switch (eEvent)
        {
          case EV_READY:
              break;

          case EV_FRAME_RECEIVED:
              eStatus = eMBRTUReceive(pMBRTU, &ucRcvAddress, &ucMBFrame, &usLength );
              if( eStatus == MB_ENOERR )//不需要循环，完成上述步骤直接跳转下一步
              {
                  /* Check if the frame is for us. If not ignore the frame. */
                  if( ( ucRcvAddress == pMBRTU->ucMBAddress ) || ( ucRcvAddress == MB_ADDRESS_BROADCAST ) )
                  {
                      xMBPortEventPost( pMBRTU,EV_EXECUTE );
                  }
                  else
                  {/*接收到的不是自己的地址，接收使能，切换到EV_READY状态*/
                    xMBPortEventPost( pMBRTU,EV_READY );
                    pMBRTU->ucPort->pvMBPortSerialEnable( TRUE, FALSE );
                  }
              }
              else//错误退出
              {
                break;
              }
          case EV_EXECUTE:
              eException = MB_EX_ILLEGAL_FUNCTION;
              /*因为是局部变量，所以必须重新计算，否则变量值未知*/
              ucRcvAddress = pMBRTU->ucBuffer[MB_SER_PDU_ADDR_OFF];
              usLength = ( USHORT )( pMBRTU->ucBufferCount - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );
              ucMBFrame = ( UCHAR * ) & pMBRTU->ucBuffer[MB_SER_PDU_PDU_OFF];		
              ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
        
              for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
              {
                  /* No more function handlers registered. Abort. */
                  if( xFuncHandlers[i].ucFunctionCode == 0 )
                  {
                      break;
                  }
                  else if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                  {
                      eException = xFuncHandlers[i].pxHandler( ucMBFrame, &usLength );
                      break;
                  }
              }

              /* If the request was not sent to the broadcast address we
               * return a reply. */
              if( ucRcvAddress != MB_ADDRESS_BROADCAST )
              {
                  if( eException != MB_EX_NONE )
                  {
                      /* An exception occured. Build an error frame. */
                      usLength = 0;
                      ucMBFrame[usLength++] = ( UCHAR )( ucFunctionCode | MB_FUNC_ERROR );
                      ucMBFrame[usLength++] = eException;
                  }  
                  eStatus = eMBRTUSend( pMBRTU,ucMBFrame, usLength );
              }
              break;

          case EV_FRAME_SENT:
              break;
        }
    }
}
/**
  * @brief  MB RTU 循环
  * @param  None
  * @retval None
  * @note   放入while 或者线程
*/
eMBErrorCode 
eMBRTUReceive(MB_RTU_DCB *const pMBRTU, UCHAR * pucRcvAddress, UCHAR ** pucFrame, USHORT * pusLength )
{
    eMBErrorCode    eStatus= MB_ENOERR;
    pMBRTU->ucPort->pvMBEnterCriticalSection();
  
    assert( pMBRTU->ucBufferCount < MB_SER_PDU_SIZE_MAX );
    /* Length and CRC check */
    if( ( pMBRTU->ucBufferCount >= MB_SER_PDU_SIZE_MIN )
        && ( usMBCRC16( (UCHAR *)pMBRTU->ucBuffer, pMBRTU->ucBufferCount ) == 0 ) )
    {
        /* Save the address field. All frames are passed to the upper layed
         * and the decision if a frame is used is done there.
         */
        *pucRcvAddress = pMBRTU->ucBuffer[MB_SER_PDU_ADDR_OFF];

         /* Total length of Modbus-PDU is Modbus-Serial-Line-PDU minus
          * size of address field and CRC checksum.
          */
         *pusLength = ( USHORT )( pMBRTU->ucBufferCount - MB_SER_PDU_PDU_OFF - MB_SER_PDU_SIZE_CRC );

         /* Return the start of the Modbus PDU to the caller. */
         *pucFrame = ( UCHAR * ) & pMBRTU->ucBuffer[MB_SER_PDU_PDU_OFF];
    }
    else
    {
        eStatus = MB_EIO;
    }

    pMBRTU->ucPort->pvMBExitCriticalSection();
    return eStatus;
}
/**
  * @brief  MB RTU 接收FSM
  * @param  None
  * @retval None
  * @note   放入接收中断
*/
void xMBRTUReceiveFSM(MB_RTU_DCB *const pMBRTU)
{
    UCHAR           ucByte;
    
    /* Always read the character. */
    pMBRTU->ucPort->pvMBPortSerialGetByte( ( UCHAR * ) &ucByte );

    switch ( pMBRTU->eRcvState )
    {
        /* If we have received a character in the init state we have to
         * wait until the frame is finished.
         */
    case STATE_RX_INIT:
        pMBRTU->ucPort->pvMBPortTimersEnable( ( USHORT ) usTimerT35_50us );
        break;

        /* In the error state we wait until all characters in the
         * damaged frame are transmitted.
         */
    case STATE_RX_ERROR:
        pMBRTU->ucPort->pvMBPortTimersEnable( ( USHORT ) usTimerT35_50us );
        break;

        /* In the idle state we wait for a new character. If a character
         * is received the t1.5 and t3.5 timers are started and the
         * receiver is in the state STATE_RX_RECEIVCE.
         */
    case STATE_RX_IDLE:
        pMBRTU->ucBufferCount = 0;
        pMBRTU->ucBuffer[pMBRTU->ucBufferCount++] = ucByte;
        pMBRTU->eRcvState = STATE_RX_RCV;

        /* Enable t3.5 timers. */
        pMBRTU->ucPort->pvMBPortTimersEnable( ( USHORT ) usTimerT35_50us );
        break;

        /* We are currently receiving a frame. Reset the timer after
         * every character received. If more than the maximum possible
         * number of bytes in a modbus frame is received the frame is
         * ignored.
         */
    case STATE_RX_RCV:
        if( pMBRTU->ucBufferCount < MB_SER_PDU_SIZE_MAX )
        {
            pMBRTU->ucBuffer[pMBRTU->ucBufferCount++] = ucByte;
        }
        else
        {
            pMBRTU->eRcvState = STATE_RX_ERROR;
        }
        pMBRTU->ucPort->pvMBPortTimersEnable( ( USHORT ) usTimerT35_50us );
        
        break;
    }
}
/**
  * @brief  MB RTU 发送
  * @param  None
  * @retval None
  * @note   
*/
eMBErrorCode
eMBRTUSend(MB_RTU_DCB *const pMBRTU,const UCHAR * pucFrame, USHORT usLength )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          usCRC16;

    pMBRTU->ucPort->pvMBEnterCriticalSection();

    /* Check if the receiver is still in idle state. If not we where to
     * slow with processing the received frame and the master sent another
     * frame on the network. We have to abort sending the frame.
     */
    if( pMBRTU->eRcvState == STATE_RX_IDLE )
    {
        /* First byte before the Modbus-PDU is the slave address. */
        pMBRTU->pucBuffer = ( UCHAR * ) pucFrame - 1;
        pMBRTU->ucBufferCount = 1;

        /* Now copy the Modbus-PDU into the Modbus-Serial-Line-PDU. */
        pMBRTU->pucBuffer[MB_SER_PDU_ADDR_OFF] = pMBRTU->ucMBAddress;
        pMBRTU->ucBufferCount += usLength;

        /* Calculate CRC16 checksum for Modbus-Serial-Line-PDU. */
        usCRC16 = usMBCRC16( ( UCHAR * ) pMBRTU->pucBuffer, pMBRTU->ucBufferCount );
        pMBRTU->ucBuffer[pMBRTU->ucBufferCount++] = ( UCHAR )( usCRC16 & 0xFF );
        pMBRTU->ucBuffer[pMBRTU->ucBufferCount++] = ( UCHAR )( usCRC16 >> 8 );

        /* Activate the transmitter. */
        pMBRTU->eSndState = STATE_TX_XMIT;
        pMBRTU->ucPort->pvMBPortSerialEnable( FALSE, TRUE );
        xMBRTUTransmitFSM(pMBRTU);
    }
    else
    {
        eStatus = MB_EIO;
    }
    pMBRTU->ucPort->pvMBExitCriticalSection();
    return eStatus;
}
/**
  * @brief  MB RTU 发送FSM
  * @param  None
  * @retval None
  * @note   放入发送中断
*/
void xMBRTUTransmitFSM(MB_RTU_DCB *const pMBRTU)
{
    switch ( pMBRTU->eSndState )
    {
        /* We should not get a transmitter event if the transmitter is in
         * idle state.  */
    case STATE_TX_IDLE:
        /* enable receiver/disable transmitter. */
        pMBRTU->ucPort->pvMBPortSerialEnable( TRUE, FALSE );
        break;

    case STATE_TX_XMIT:
        /* check if we are finished. */
        if( pMBRTU->ucBufferCount != 0 )
        {
           pMBRTU->ucPort->pvMBPortSerialPutByte( ( CHAR )*pMBRTU->pucBuffer );
//           pMBRTU->pucBuffer++;  /* next byte in sendbuffer. */
//           pMBRTU->ucBufferCount--;
           pMBRTU->ucBufferCount = 0;
        }
        else
        {
            xMBPortEventPost(pMBRTU,EV_FRAME_SENT );
            /* Disable transmitter. This prevents another transmit buffer
             * empty interrupt. */
            pMBRTU->ucPort->pvMBPortSerialEnable( TRUE, FALSE );
            pMBRTU->eSndState = STATE_TX_IDLE;
        }
        break;
    }
}
/**
  * @brief  MB RTU 定时器调用函数
  * @param  None
  * @retval None
  * @note   放入定时完成回调
*/
void xMBRTUTimerT35Expired(MB_RTU_DCB *const pMBRTU)
{
    switch ( pMBRTU->eRcvState )
    {
        /* Timer t35 expired. Startup phase is finished. */
    case STATE_RX_INIT:
        xMBPortEventPost(pMBRTU,EV_READY);
        break;

        /* A frame was received and t35 expired. Notify the listener that
         * a new frame was received. */
    case STATE_RX_RCV:
        xMBPortEventPost(pMBRTU,EV_FRAME_RECEIVED);
        //pMBRTU->ucPort->pvMBPortSerialEnable( FALSE, FALSE );//使用这行代码，无法使用AT指令
        break;

        /* An error occured while receiving the frame. */
    case STATE_RX_ERROR:
        break;

        /* Function called in an illegal state. */
    default:
    assert( (  pMBRTU->eRcvState  == STATE_RX_INIT ) ||
                (  pMBRTU->eRcvState  == STATE_RX_RCV ) || (  pMBRTU->eRcvState  == STATE_RX_ERROR ) );
    }
    pMBRTU->ucPort->pvMBPortTimersDisable(  );
    pMBRTU->eRcvState = STATE_RX_IDLE;
}

