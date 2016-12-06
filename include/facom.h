#ifndef _FACOM_H_
#define _FACOM_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define DATA_BITS_5     5
#define DATA_BITS_6     6
#define DATA_BITS_7     7
#define DATA_BITS_8     8

#define PARITY_OFF      0
#define PARITY_EVEN     1
#define PARITY_ODD      2

#define STOP_BITS_1     1
#define STOP_BITS_2     2

#define BAUD_0          0
#define BAUD_9600       9600
#define BAUD_19200      19200

/*
 * Open connection to PLC
 */
extern int FACOM_open(const char *port);

/*
 * Close connection to PLC
 */
extern int FACOM_close(void);

/*
 * Set data bits for connection
 */
extern int FACOM_setDataBits(int dataBits);

/*
 * Set parity for connection
 */
extern int FACOM_setParity(int parity);

/*
 * Set stop bits for connection
 */
extern int FACOM_setStopBits(int stopBits);

/*
 * Set baud rate for connection
 */
extern int FACOM_setBaudRate(int baudRate);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FACOM_H_
