#include "freemodbus_tcp.h"

#include "log.h"
#ifdef __cplusplus
extern "C"
{
#endif

  USHORT usRegInputStart = REG_INPUT_START;
  USHORT usRegInputBuf[REG_INPUT_NREGS];

  USHORT usRegHoldingStart = REG_HOLDING_START;
  USHORT usRegHoldingBuf[REG_HOLDING_NREGS] = {0};

  /* function codes */
  eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress,
                             USHORT usNRegs)
  {
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    if ((usAddress >= REG_INPUT_START) &&
        (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
      iRegIndex = (int)(usAddress - usRegInputStart);
      while (usNRegs > 0)
      {
        *pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] >> 8);
        *pucRegBuffer++ = (unsigned char)(usRegInputBuf[iRegIndex] & 0xFF);
        iRegIndex++;
        usNRegs--;
      }
    }
    else
    {
      eStatus = MB_ENOREG;
    }

    return eStatus;
  }

  eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress,
                               USHORT usNRegs, eMBRegisterMode eMode)
  {
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    if ((usAddress >= REG_HOLDING_START) &&
        (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
    {
      iRegIndex = (int)(usAddress - usRegHoldingStart - 1);
      switch (eMode)
      {
      /* Pass current register values to the protocol stack. */
      case MB_REG_READ:
        while (usNRegs > 0)
        {
          *pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] >> 8);
          *pucRegBuffer++ = (UCHAR)(usRegHoldingBuf[iRegIndex] & 0xFF);
          iRegIndex++;
          usNRegs--;
        }
        break;

      /* Update current register values with new values from the protocol stack.
       */
      case MB_REG_WRITE:
        while (usNRegs > 0)
        {
          usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
          usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
          iRegIndex++;
          usNRegs--;
        }
        break;
      }
    }
    else
    {
      eStatus = MB_ENOREG;
    }
    return eStatus;
  }

  eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress,
                             USHORT usNCoils, eMBRegisterMode eMode)
  {
    return MB_ENOREG;
  }

  eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress,
                                USHORT usNDiscrete)
  {
    return MB_ENOREG;
  }

#ifdef __cplusplus
}
#endif
