#include "facom.h"

#include <unistd.h>     // UNIX standard functions
#include <fcntl.h>      // File controle definitions
#include <termios.h>    // Terminal input/output stream
#include <string.h>     // Work with strings
#include <stdlib.h>     // Standard functions (memorry managment)

#include <stdio.h>      // Only for debug propose

#include "error.h"
#include "private.h"

/*
 * Global variables
 */
int fd;

/*
 * Open connection to PLC
 */
int FACOM_open(const char *port)
{
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd < 0)
        return ERROR_OPEN_PORT;

    fcntl(fd, F_SETFL, FNDELAY);    // Disable wainig when read from port

    /*
     * Set default port parameters
     */
    struct termios portOptions;
    if(tcgetattr(fd, &portOptions) < 0)
        return ERROR_GET_PORT_OPTIONS;
    portOptions.c_cflag |= (CLOCAL | CREAD); // Default parameters (must be set)
    portOptions.c_cflag &= ~(CRTSCTS);
    portOptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    portOptions.c_iflag |=  (INPCK | ISTRIP);
    portOptions.c_iflag &= ~(IXON | IXOFF | IXANY);
    portOptions.c_oflag &= ~(OPOST);
    if(tcsetattr(fd, TCSANOW, &portOptions) < 0)
        return ERROR_SET_PORT_OPTIONS;

    if(FACOM_setDataBits(DATA_BITS_8) < 0)
        return ERROR_SET_DATA_BITS;
    if(FACOM_setStopBits(STOP_BITS_1) < 0)
        return ERROR_SET_STOP_BITS;
    if(FACOM_setParity(PARITY_EVEN) < 0)
        return ERROR_SET_PARITY;
    if(FACOM_setBaudRate(BAUD_9600) < 0)
        return ERROR_SET_BAUD_RATE;

    return SUCCESS;
}

/*
 * Close connection to PLC
 */
int FACOM_close(void)
{
    if(close(fd) < 0)
        return ERROR_CLOSE_PORT;

    return SUCCESS;
}

/*
 * Set data bits for connection
 */
int FACOM_setDataBits(int dataBits)
{
    int port;
    switch(dataBits)
    {
        case DATA_BITS_5:
            port = CS5;
            break;
        case DATA_BITS_6:
            port = CS6;
            break;
        case DATA_BITS_7:
            port = CS7;
            break;
        case DATA_BITS_8:
            port = CS8;
            break;
        default:
            return ERROR_WRONG_PARAMETERS;
            break;
    }

    struct termios portOptions;
    if(tcgetattr(fd, &portOptions) < 0)
        return ERROR_GET_PORT_OPTIONS;

    portOptions.c_cflag &= ~CSIZE;
    portOptions.c_cflag |= port;

    if(tcsetattr(fd, TCSANOW, &portOptions) < 0)
        return ERROR_SET_PORT_OPTIONS;

    return SUCCESS;
}

/*
 * Set parity for connection
 */
int FACOM_setParity(int parity)
{
    struct termios portOptions;
    if(tcgetattr(fd, &portOptions) < 0)
        return ERROR_GET_PORT_OPTIONS;

    portOptions.c_cflag &= ~PARENB;
    portOptions.c_cflag &= ~PARODD;

    switch(parity)
    {
        case PARITY_OFF:
            break;
        case PARITY_EVEN:
            portOptions.c_cflag |= PARENB;
            break;
        case PARITY_ODD:
            portOptions.c_cflag |= (PARENB | PARODD);
            break;
        default:
            return ERROR_WRONG_PARAMETERS;
            break;
    }
    
    if(tcsetattr(fd, TCSANOW, &portOptions) < 0)
        return ERROR_SET_PORT_OPTIONS;

    return SUCCESS;
}

/*
 * Set stop bits for connection
 */
int FACOM_setStopBits(int stopBits)
{
    if(stopBits != STOP_BITS_1 &&
       stopBits != STOP_BITS_2)
        return ERROR_WRONG_PARAMETERS;

    struct termios portOptions;
    if(tcgetattr(fd, &portOptions) < 0)
        return ERROR_GET_PORT_OPTIONS;

    if(stopBits == STOP_BITS_2)
        portOptions.c_cflag |= CSTOPB;
    else
        portOptions.c_cflag &= ~CSTOPB;

    if(tcsetattr(fd, TCSANOW, &portOptions) < 0)
        return ERROR_SET_PORT_OPTIONS;

    return SUCCESS;
}

/*
 * Set baud rate for connection
 */
int FACOM_setBaudRate(int baudRate)
{
    struct termios portOptions;
    if(tcgetattr(fd, &portOptions) < 0)
        return ERROR_GET_PORT_OPTIONS;

    int baud;
    switch(baudRate)
    {
        case BAUD_0:
            baud = B0;
            break;
        case BAUD_9600:
            baud = B9600;
            break;
        case BAUD_19200:
            baud = B19200;
            break;
        default:
            return ERROR_WRONG_PARAMETERS;
            break;
    }

    cfsetispeed(&portOptions, baud);
    cfsetospeed(&portOptions, baud);

    if(tcsetattr(fd, TCSANOW, &portOptions) < 0)
        return ERROR_SET_PORT_OPTIONS;

    return SUCCESS;
}

/*
 * Checksum of message
 */
unsigned char FACOM_checksum(const char *message, int msgLength)
{
    unsigned char sum = 0;
    size_t i;

    for(i = 0; i < msgLength; i++)
        sum = (sum + message[i]) & 0xFF;

    return sum;
}

/*
 * Send data to Fatek PLC
 */
int FACOM_write(const char *data)
{
    int error = SUCCESS;

    int count = strlen(data);
    char *msg = malloc(count + 4);
    if(msg == NULL)
        return ERROR_MEMORRY_ALLOCATION;

    msg[0] = STX;
    size_t i;
    for(i = 0; i < count; i++)
        msg[i + 1] = data[i];

    unsigned char checksum = FACOM_checksum(msg, count + 1);
    FACOM_intToHexString(checksum, &msg[count + 1]);
    msg[count + 3] = ETX;

    if(write(fd, msg, count + 3) < 0)
        error = ERROR_SENDING_DATA;

    msg[count + 3] = '\0';
    free(msg);

    return error;
}

/*
 * Read data from Fatek PLC
 */
int FACOM_read(char *data, unsigned int bufferSize)
{
    unsigned int i = 0;
    while(1)
    {
        char ch;
        if(read(fd, &ch, 1) < 0)
            continue;

        data[i++] = ch;
        if(ch == ETX || i >= bufferSize)
            break;
    }

    return i;
}

