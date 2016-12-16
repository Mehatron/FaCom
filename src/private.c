#include "private.h"

#include <string.h>

#include "error.h"
#include "facom.h"

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

/*
 * Count number of digits in number (integer)
 */
int FACOM_numberOfDigits(int number)
{
    int numberOfDigits;
    for(numberOfDigits = 0; number > 0; numberOfDigits++, number /= 10);
    return numberOfDigits;
}

/*
 * Check for error while set commands
 */
int FACOM_checkForErrors(void)
{
    char recived[10];
    int bytesRecived = FACOM_read(recived, 9);
    if(bytesRecived < 9)
        return ERROR_RECIVEING_DATA;

    char err = recived[5];
    if(err > '0')
        return err == 'A' ?
            ERROR_ILLEGAL_ADDRESS : err + ERROR_FREE;

    return SUCCESS;
}

