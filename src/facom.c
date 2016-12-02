#include "facom.h"

#include <unistd.h>     // UNIX standard functions
#include <fcntl.h>      // File controle definitions
#include <termios.h>    // Terminal input/output stream

#include "error.h"

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
        return ERROR;

    return SUCCESS;
}

/*
 * Close connection to PLC
 */
void FACOM_close(void)
{
}

