#include "private.h"

#include <string.h>

#include "error.h"
#include "facom.h"

void FACOM_intToString(int number, char *str)
{
    size_t i = 0;
    int numOfDigits = FACOM_numberOfDigits(number);
    while(number >  0)
    {
        str[numOfDigits - ++i] = digToChar(number % 10);
        number /= 10;
    }
    str[numOfDigits] = '\0';
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
            ERROR_ILLEGAL_ADDRESS : ERROR_FREE - err + '0';

    return SUCCESS;
}

/*
 * Get discrete address (for sending to Fate PLC)
 */
int FACOM_getDiscreteAddress(unsigned char discreteType,
                             int discreteNumber,
                             char *address)
{
    if(discreteNumber < 0 || discreteNumber > 9999 ||
       address == NULL)
        return ERROR_WRONG_PARAMETERS;

    char type;
    switch(discreteType)
    {
        case DISCRETE_X:
            type = 'X';
            break;
        case DISCRETE_Y:
            type = 'Y';
            break;
        case DISCRETE_M:
            type = 'M';
            break;
        case DISCRETE_S:
            type = 'S';
            break;
        case DISCRETE_T:
            type = 'T';
            break;
        case DISCRETE_C:
            type = 'C';
            break;
        default:
            return ERROR_WRONG_PARAMETERS;
            break;
    }
    address[0] = type;

    int numOfDigits = FACOM_numberOfDigits(discreteNumber);
    size_t i;
    for(i = 0; i < 4 - numOfDigits; i++)
        address[i + 1] = '0';
    FACOM_intToString(discreteNumber, &address[i + 1]);

    return SUCCESS;
}

