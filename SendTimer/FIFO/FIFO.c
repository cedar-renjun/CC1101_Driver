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
#include "FIFO.h"

//******************************************************************************************
//!                     ASSERT MACRO
//******************************************************************************************
#ifndef ASSERT

#ifdef  NDEBUG
#define ASSERT(x)
#else
#define ASSERT(x) do {while(!(x));} while(0)
#endif

#endif  // ASSERT

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
FIFO_t* FIFO_Create(uint8_t UnitSize, uint32_t UnitCnt)
{
    //! Check input parameters.
    ASSERT(0 != UnitSize);
    ASSERT(0 != UnitCnt);

    FIFO_t*   pFIFO     = NULL;         //!< FIFO Pointer
    uint8_t*  pBaseAddr = NULL;         //!< Memory Base Address

    //! Allocate Memory for pointer of new FIFO Control Block.
    pFIFO = (FIFO_t*) malloc(sizeof(FIFO_t));
    if(NULL == pFIFO)
    {
        //! Allocate Failure, exit now.
        return (NULL);
    }

    //! Allocate memory for FIFO.
    pBaseAddr = malloc(UnitSize*UnitCnt);
    if(NULL == pBaseAddr)
    {
        //! Allocate Failure, exit now.
        return (NULL);
    }

    //! Initialize General FIFO Module.
    FIFO_Init(pFIFO, pBaseAddr, UnitSize, UnitCnt);

    return (pFIFO);
}

//******************************************************************************************
//
//! \brief  Destory FIFO
//!  This function first release memory section, then reinit the fifo pointer parameter.
//!
//! \param  [in] pFIFO is the pointer of valid fifo.
//! \retval None.
//!
//! \note   -# You must enable USE_MEMORY_ALLOC macro and ensure your system have <stdlib.h>
//!            Header file before use this function.
//
//******************************************************************************************
void FIFO_Destory(FIFO_t* pFIFO)
{
    //! Check input parameters.
    ASSERT(NULL != pFIFO);
    ASSERT(NULL != pFIFO->pStartAddr);

    //! Free FIFO memory
    free(pFIFO->pStartAddr);

#if 0
    //! Reset FIFO parameters
    pFIFO->pStartAddr  = NULL;
    pFIFO->pEndAddr    = NULL;
    pFIFO->pReadIndex  = NULL;
    pFIFO->pWriteIndex = NULL;
    pFIFO->UnitSize    = 0;
    pFIFO->Count       = 0;
#endif

    //! Free FIFO Control Block memory.
    free(pFIFO);

    return;     //!< Success
}
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
int FIFO_Init(FIFO_t* pFIFO, void* pBaseAddr, uint8_t UnitSize, uint32_t UnitCnt)
{
    //! Check input parameters.
    ASSERT(NULL != pFIFO);
    ASSERT(NULL != pBaseAddr);
    ASSERT(0    != UnitSize);
    ASSERT(0    != UnitCnt);

    //! Initialize FIFO Control Block.
    pFIFO->pStartAddr  = (uint8_t*) pBaseAddr;
    pFIFO->pEndAddr    = (uint8_t*) pBaseAddr + UnitSize * UnitCnt;
    pFIFO->Free        = UnitCnt;
    pFIFO->Used        = 0;
    pFIFO->UnitSize    = UnitSize;
    pFIFO->pReadIndex  = (uint8_t*) pBaseAddr;
    pFIFO->pWriteIndex = (uint8_t*) pBaseAddr;

    return (0);
}

//******************************************************************************************
//
//! \brief  Put an element into FIFO.
//!
//! \param  [in]  pFIFO is the pointer of valid FIFO.
//! \param  [out] pElement is the address of memory that store the pop element.
//!
//! \retval 0 if operate successfully, otherwise return -1.
//
//******************************************************************************************
int FIFO_Put(FIFO_t* pFIFO, void* pElement)
{
    //! Check input parameters.
    ASSERT(NULL != pFIFO);
    ASSERT(NULL != pElement);

    uint8_t  i = 0;
    uint8_t*  _pElement = (uint8_t*)pElement;

    // Full ?
    if(0 == pFIFO->Free)
    {
        //! Error, FIFO is full!
        return (-1);
    }

    //! Copy Data
    for(i = 0; i < pFIFO->UnitSize; i++)
    {
        if(pFIFO->pWriteIndex > pFIFO->pEndAddr)
        {
            pFIFO->pWriteIndex = pFIFO->pStartAddr;
        }

        *(pFIFO->pWriteIndex) = *_pElement++;
        pFIFO->pWriteIndex++;
    }

    //! Update information
    pFIFO->Free--;
    pFIFO->Used++;

    return (0);
}

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
int FIFO_Get(FIFO_t* pFIFO, void* pElement)
{
    //! Check input parameters.
    ASSERT(NULL != pFIFO);
    ASSERT(NULL != pElement);

    uint8_t  i = 0;
    uint8_t*  _pElement = (uint8_t*)pElement;

    // Empty ?
    if(0 == pFIFO->Used)
    {
        //! Error, FIFO is Empty!
        return (-1);
    }

    //! Copy Data
    for(i = 0; i < pFIFO->UnitSize; i++)
    {
        if(pFIFO->pReadIndex > pFIFO->pEndAddr)
        {
            pFIFO->pReadIndex = pFIFO->pStartAddr;
        }

        *_pElement++ = *(pFIFO->pReadIndex);
        pFIFO->pReadIndex++;
    }

    //! Update information
    pFIFO->Free++;
    pFIFO->Used--;

    return (0);
}

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
int FIFO_PreRead(FIFO_t* pFIFO, uint8_t Offset, void* pElement)
{

    uint8_t  i = 0;
    uint8_t*  _pElement = (uint8_t*)pElement;
    uint8_t* _PreReadIndex = (void*)0;

    //! Check input parameters.
    ASSERT(NULL != pFIFO);
    ASSERT(NULL != pElement);

    // OverFlow ?
    if(Offset >= pFIFO->Used)
    {        
        return (-1);
    }

    // Move Read Pointer to right position   
    _PreReadIndex = pFIFO->pReadIndex;
    for(i = 0; i < pFIFO->UnitSize * Offset; i++)
    {
        if(_PreReadIndex > pFIFO->pEndAddr)
        {
            _PreReadIndex = pFIFO->pStartAddr;
        }
        
        _PreReadIndex++;
    }

    //! Copy Data
    for(i = 0; i < pFIFO->UnitSize; i++)
    {
        if(_PreReadIndex > pFIFO->pEndAddr)
        {
            _PreReadIndex = pFIFO->pStartAddr;
        }

        *_pElement++ = *(_PreReadIndex);
        _PreReadIndex++;
    }
    
    return (0);
}

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
int FIFO_IsEmpty(FIFO_t* pFIFO)
{
    //! Check input parameter.
    ASSERT(NULL != pFIFO);

    return (0 == pFIFO->Used);
}

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
int FIFO_IsFull(FIFO_t* pFIFO)
{
    //! Check input parameter.
    ASSERT(NULL != pFIFO);

    return (0 == pFIFO->Free);
}

//******************************************************************************************
//
//! \brief  Counter the number of free elements in FIFO.
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval The number of used elements in FIFO. (uint: element)
//
//******************************************************************************************
int FIFO_CountUsed(FIFO_t* pFIFO)
{
    //! Check input parameter.
    ASSERT(NULL != pFIFO);

    return (pFIFO->Used);
}

//******************************************************************************************
//
//! \brief  Counter the number of reserved element in FIFO.
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval The number of free elements in FIFO. (unit: element)
//
//******************************************************************************************
int FIFO_CountFree(FIFO_t* pFIFO)
{
    //! Check input parameter.
    ASSERT(NULL != pFIFO);

    return (pFIFO->Free);
}

//******************************************************************************************
//
//! \brief  Flush FIFO.
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval 0 if success, -1 if failure.
//
//******************************************************************************************
int FIFO_Flush(FIFO_t* pFIFO)
{
    //! Check input parameters.
    ASSERT(NULL != pFIFO);

    //! Initialize FIFO Control Block.
    pFIFO->Free        = (pFIFO->pEndAddr - pFIFO->pStartAddr)/(pFIFO->UnitSize);
    pFIFO->Used        = 0;
    pFIFO->pReadIndex  = pFIFO->pStartAddr;
    pFIFO->pWriteIndex = pFIFO->pStartAddr;

    return (0);
}

