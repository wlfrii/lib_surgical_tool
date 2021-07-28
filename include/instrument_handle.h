#ifndef INSTRUMENT_HANDLE_H
#define INSTRUMENT_HANDLE_H
#include <stdio.h>
class InstrumentHandle
{
public:
    InstrumentHandle();

    void libtest(){
        printf("Call the lib_instrument successfully!\n");
    }
};

#endif // INSTRUMENT_HANDLE_H