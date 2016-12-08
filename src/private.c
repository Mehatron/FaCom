#include "private.h"

#include <string.h>

#include "error.h"

void FACOM_intToString(int number, char *str)
{
    size_t i = 0;
    while(number >  0)
    {
        *(str++) = digToChar(number % 10);
        number /= 10;
    }
    *str = '\0';
}

void FACOM_intToHexString(unsigned char number, char *str)
{
    unsigned char i;

    unsigned char hexL = 0;
    unsigned char hexH = 0;
    for(i = 1; i <= 0x08; i *= 2)
    {
        int t = i;
        if(number & i)
            hexL += i;
        if(number & (i << 4))
            hexH += i;
    }

    *(str++) = hexH < 10 ? digToChar(hexH) : hexDigToChar(hexH);
    *(str++) = hexL < 10 ? digToChar(hexL) : hexDigToChar(hexL);
    *str = '\0';
}

