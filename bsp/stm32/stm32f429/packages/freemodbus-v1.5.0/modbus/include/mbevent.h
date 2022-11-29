#ifndef __MBEVENT__H
#define __MBEVENT__H
void xMBPortEventInit(MB_RTU_DCB *const pMBRTU);
void xMBPortEventPost(MB_RTU_DCB *const pMBRTU,eMBEventType eEvent );
BOOL xMBPortEventGet(MB_RTU_DCB  *const pMBRTU,eMBEventType * eEvent);
#endif
