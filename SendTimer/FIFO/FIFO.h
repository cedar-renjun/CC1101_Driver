//******************************************************************************************
//!
//! \file   FIFO.h
//! \brief  Genernal FIFO Model Interface.
//!         You can use uniform FIFO Model to manager Any type of data element.
//! \author cedar
//! \date   2013-12-16
//! \email  xuesong5825718@gmail.com
//!
//! \license
//!
//! Copyright (c) 2013 Cedar MIT License
//!
//! Permission is hereby granted, free of charge, to any person obtaining a copy
//! of this software and associated documentation files (the "Software"), to deal
//! in the Software without restriction, including without limitation the rights to
//! use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
//! the Software, and to permit persons to whom the Software is furnished to do so,
//! subject to the following conditions:
//!
//! The above copyright notice and this permission notice shall be included in all
//! copies or substantial portions of the Software.
//!
//! THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//! IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//! FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//! AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//! LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//! OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//! IN THE SOFTWARE.
///
//******************************************************************************************

#ifndef __FIFO_H__
#define __FIFO_H__

#ifdef __cplusplus
extern "C"
{
#endif

//******************************************************************************************
//!                           CONFIGURE MACRO
//******************************************************************************************

#define USE_SYS_INTXX                      //!< Use stdint standard datatype
#define USE_DYNAMIC_MEMORY                 //!< Use system malloc/free function


#include <stdio.h>

#ifdef USE_SYS_INTXX
#include <stdint.h>
#else
typedef unsigned char uint8_t;
typedef unsigned int  uint32_t;
#endif

#ifdef USE_DYNAMIC_MEMORY
#include <stdlib.h>
#endif

//******************************************************************************************
//!                           PUBLIC TYPE
//******************************************************************************************

//! FIFO Memory Model
typedef struct FIFO_t
{
    uint8_t* pStartAddr;                   //!< FIFO Memory Pool Start Address
    uint8_t* pEndAddr;                     //!< FIFO Memory Pool End Address
    uint32_t Free;                         //!< The capacity of FIFO
    uint32_t Used;                         //!< The number of elements in FIFO
    uint8_t  UnitSize;                     //!< FIFO Element Size(Unit: Byte)
    uint8_t* pReadIndex;                   //!< FIFO Data Read Index Pointer
    uint8_t* pWriteIndex;                  //!< FIFO Data Write Index Pointer
}FIFO_t;

//******************************************************************************************
//!                           PUBLIC API
//******************************************************************************************

#ifdef USE_DYNAMIC_MEMORY
//******************************************************************************************
//
//! \brief  Create An New FIFO Instance.
//! This function allocate enought room for N blocks fifo elements, then return the pointer
//! of FIFO.
//!
//! \param  [in] UnitSize is fifo element size.
//! \param  [in] UnitCnt is count of fifo elements.
//! \retval The Pointer of FIFO instance, return NULL is failure to allocate memory.
//!
//! \note   -# You must enable USE_MEMORY_ALLOC macro and ensure your system have <stdlib.h>
//!            Header file before use this function.
//! \note   -# Functions FIFO_Create and FIFO_Destory must be used in pairs.
//!
//******************************************************************************************
extern FIFO_t* FIFO_Create(uint8_t UnitSize, uint32_t UnitCnt);

//******************************************************************************************
//
//! \brief  Destory FIFO Instance.
//!  This function release memory, then reinit the FIFO struct.
//!
//! \param  [in] pFIFO is the pointer of FIFO instance
//! \retval None.
//!
//! \note   -# You must enable USE_MEMORY_ALLOC macro and ensure your system have <stdlib.h>
//!            Header file before use this function.
//
//******************************************************************************************
extern void FIFO_Destory(FIFO_t* pFIFO);
#endif // USE_DYNAMIC_MEMORY

//******************************************************************************************
//
//! \brief  Initialize an static FIFO struct.
//!
//! \param  [in] pFIFO is the pointer of valid FIFO instance.
//! \param  [in] pBaseAddr is the base address of pre-allocate memory, such as array.
//! \param  [in] UnitSize is fifo element size.
//! \param  [in] UnitCnt is count of fifo elements.
//! \retval 0 if initialize successfully, otherwise return -1.
//
//******************************************************************************************
extern int FIFO_Init(FIFO_t* pFIFO, void* pBaseAddr, uint8_t UnitSize, uint32_t UnitCnt);

//******************************************************************************************
//
//! \brief  Put an element into FIFO.
//!
//! \param  [in]  pFIFO is the pointer of valid FIFO.
//! \param  [in]  pElement is the address of element you want to put
//!
//! \retval 0 if operate successfully, otherwise return -1.
//
//******************************************************************************************
extern int FIFO_Put(FIFO_t* pFIFO, void* pElement);

//******************************************************************************************
//
//! \brief  Get an element from FIFO.
//!
//! \param  [in]  pFIFO is the pointer of valid FIFO.
//! \param  [out] pElement is the address of element you want to get
//!
//! \retval 0 if operate successfully, otherwise return -1.
//
//******************************************************************************************
extern int FIFO_Get(FIFO_t* pFIFO, void* pElement);

//******************************************************************************************
//
//! \brief  Pre-Read an element from FIFO.
//!
//! \param  [in]  pFIFO is the pointer of valid FIFO.
//! \param  [in]  Offset is the offset from current pointer.
//! \param  [out] pElement is the address of element you want to get
//!
//! \retval 0 if operate successfully, otherwise return -1.
//
//******************************************************************************************
extern int FIFO_PreRead(FIFO_t* pFIFO, uint8_t Offset, void* pElement);

//******************************************************************************************
//
//! \brief  FIFO is empty ?
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval - None-zero(true) if empty.
//!         - Zero(false) if not empty.
//
//******************************************************************************************
extern int FIFO_IsEmpty(FIFO_t* pFIFO);

//******************************************************************************************
//
//! \brief  FIFO is full ?
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval - None-zero(true) if full.
//!         - Zero(false) if not full.
//
//******************************************************************************************
extern int FIFO_IsFull(FIFO_t* pFIFO);

//******************************************************************************************
//
//! \brief  Get FIFO the number of elements?
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval The number of elements in FIFO.
//
//******************************************************************************************
extern int FIFO_CountUsed(FIFO_t* pFIFO);

//******************************************************************************************
//
//! \brief  Get FIFO the number of elements?
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval The number of elements in FIFO.
//
//******************************************************************************************
extern int FIFO_CountFree(FIFO_t* pFIFO);

//******************************************************************************************
//
//! \brief  Flush the content of FIFO.
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval 0 if success, -1 if failure.
//
//******************************************************************************************
extern int FIFO_Flush(FIFO_t* pFIFO);

//******************************************************************************************
//!                           Example
//******************************************************************************************

#ifdef __cplusplus
}
#endif

#endif // __FIFO_H__
