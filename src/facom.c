#include "facom.h"

#include <unistd.h>     // UNIX standard functions
#include <fcntl.h>      // File controle definitions
#include <termios.h>    // Terminal input/output stream
#include <string.h>     // Work with strings

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
 * Set discrete
 */
int FACOM_setDiscrete(const char *address, int action)
{
    if(strlen(address) != 5 ||
       action < 0 || action > 4)
        return ERROR_WRONG_PARAMETERS;

    char msg[15];

    msg[0] = STX;   // Start of text
    msg[1] = '0';   // Station
    msg[2] = '1';   // number
    msg[3] = '4';   // Command
    msg[4] = '2';   // code
    msg[5] = digToChar(action);

    size_t i;
    for(i = 6; *address != '\0'; address++, i++)
        msg[i] = *address;
    msg[i] = '\0';

    unsigned char checksum = FACOM_checksum(msg, strlen(msg));
    FACOM_intToHexString(checksum, &msg[i++]);

    msg[13] = ETX;  // End of message
    msg[14] = '\0';

    if(write(fd, msg, 15) < 0)
        return ERROR_SENDING_DATA;

    printf("%s\n", msg);
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
    sum = (((sum ^ 0xFF) + 1) & 0xFF);

    return sum;
}

