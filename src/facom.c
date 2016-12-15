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
char station[2];

/*
 * Open connection to PLC
 */
int FACOM_open(const char *port, unsigned char stationNumber)
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
    portOptions.c_cc[VMIN] = 0;     // Read don't block
    portOptions.c_cc[VTIME] = 5;    // Timeour for read 0.5s
    if(tcsetattr(fd, TCSANOW, &portOptions) < 0)
        return ERROR_SET_PORT_OPTIONS;

    if(FACOM_setDataBits(DATA_BITS_7) < 0)
        return ERROR_SET_DATA_BITS;
    if(FACOM_setStopBits(STOP_BITS_1) < 0)
        return ERROR_SET_STOP_BITS;
    if(FACOM_setParity(PARITY_EVEN) < 0)
        return ERROR_SET_PARITY;
    if(FACOM_setBaudRate(BAUD_9600) < 0)
        return ERROR_SET_BAUD_RATE;

    FACOM_intToHexString(stationNumber, station);

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
    char *msg = malloc(count + 6);
    if(msg == NULL)
        return ERROR_MEMORRY_ALLOCATION;

    msg[0] = STX;
    msg[1] = station[0];
    msg[2] = station[1];
    size_t i;
    for(i = 0; i < count; i++)
        msg[i + 3] = data[i];

    unsigned char checksum = FACOM_checksum(msg, count + 3);
    FACOM_intToHexString(checksum, &msg[count + 3]);
    msg[count + 5] = ETX;

    if(write(fd, msg, count + 6) < count + 6)
        error = ERROR_SENDING_DATA;

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
        usleep(20000);
        char ch;
        if(read(fd, &ch, 1) < 0)
            continue;

        data[i++] = ch;
        if(ch == ETX || i >= bufferSize)
            break;
    }

    return i;
}

/*
 * Set mode of Facom PLC (on/off)
 */
int FACOM_run(unsigned char run)
{
    char command[3] = "41";
    command[2] = run > 0 ? '1' : '0';

    int error = FACOM_write(command);
    if(error < 0)
        return error;

    char recived[10];
    error = FACOM_read(recived, 9);
    if(error < 9)
        return error;

    char err = recived[5];
    if(err > '0')
    {
        return err == 'A' ?
            ERROR_ILLEGAL_ADDRESS: err + ERROR_FREE;
    }

    return SUCCESS;
}

/*
 * Start Fatek PLC (set run mode on)
 */
int FACOM_start(void)
{
    return FACOM_run(ACTION_ON);
}

/*
 * Stop Fatek PLC (set run mode off)
 */
int FACOM_stop(void)
{
    return FACOM_run(ACTION_OFF);
}

