#include "mb.h"
#include "mbrtu.h"
void xMBPortEventInit(MB_RTU_DCB *const pMBRTU)
{
    pMBRTU->xEventInQueue = FALSE;
}

void xMBPortEventPost(MB_RTU_DCB *const pMBRTU,eMBEventType eEvent )
{
    pMBRTU->xEventInQueue = TRUE;
    pMBRTU->eQueuedEvent = eEvent;
}

BOOL xMBPortEventGet(MB_RTU_DCB *const pMBRTU,eMBEventType * eEvent)
{
    if(pMBRTU->xEventInQueue){
		*eEvent = pMBRTU->eQueuedEvent;
        pMBRTU->xEventInQueue = FALSE;
		return TRUE;
    }
    else
		return FALSE;
}
