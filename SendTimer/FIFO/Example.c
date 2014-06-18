
#include "FIFO.h"

typedef struct T
{
    uint8_t  name[20];
    uint8_t  sex;
    uint8_t  addr;
    uint32_t tel;
}T;

#define ELE_CNT 2

T FIFO_Pool[ELE_CNT];

static void TestCase_DynamicMemory(void)
{
    FIFO_t* MyFifo = FIFO_Create(sizeof(T), ELE_CNT);
    FIFO_Destory(MyFifo);
}

static void TestCase_StaticMemory(void)
{
    FIFO_t MyFifo;
    int ret = 0;
    T MyT_A, MyT_B, MyT_C;
    T MyT;

    MyT.sex  = 0;
    MyT.addr = 0;
    MyT.tel  = 0;

    MyT_A.sex  = 1;
    MyT_A.addr = 1;
    MyT_A.tel  = 1;

    MyT_B.sex  = 2;
    MyT_B.addr = 2;
    MyT_B.tel  = 2;

    MyT_C.sex  = 3;
    MyT_C.addr = 3;
    MyT_C.tel  = 3;

    FIFO_Init(&MyFifo, FIFO_Pool, sizeof(T), ELE_CNT);

    ret = FIFO_IsEmpty(&MyFifo);
    ret = FIFO_IsFull(&MyFifo);
    ret = FIFO_CountFree(&MyFifo);
    ret = FIFO_CountUsed(&MyFifo);

    FIFO_Put(&MyFifo, &MyT_A);
    FIFO_Put(&MyFifo, &MyT_B);
    FIFO_Put(&MyFifo, &MyT_C);

    ret = FIFO_IsEmpty(&MyFifo);
    ret = FIFO_IsFull(&MyFifo);
    ret = FIFO_CountFree(&MyFifo);
    ret = FIFO_CountUsed(&MyFifo);

    FIFO_Get(&MyFifo, &MyT);
    FIFO_Get(&MyFifo, &MyT);
    FIFO_Get(&MyFifo, &MyT);

    ret = FIFO_IsEmpty(&MyFifo);
    ret = FIFO_IsFull(&MyFifo);
    ret = FIFO_CountFree(&MyFifo);
    ret = FIFO_CountUsed(&MyFifo);

    FIFO_Put(&MyFifo, &MyT_A);
    FIFO_Put(&MyFifo, &MyT_B);
    FIFO_Put(&MyFifo, &MyT_C);

    ret = FIFO_Flush(&MyFifo);

    ret = FIFO_IsEmpty(&MyFifo);
    ret = FIFO_IsFull(&MyFifo);
    ret = FIFO_CountFree(&MyFifo);
    ret = FIFO_CountUsed(&MyFifo);
}

int main(void)
{
    TestCase_DynamicMemory();
    TestCase_StaticMemory();

    return (0);
}
