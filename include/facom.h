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

#define STX             0x02
#define ETX             0x03

#define ACTION_DISABLE  1
#define ACTION_ENABLE   2
#define ACTION_SET      3
#define ACTION_RESET    4

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

/*
 * Checksum of message
 */
extern unsigned char FACOM_checksum(const char *message, int msgLength);

/*
 * Send data to facom PLC
 */
extern int FACOM_write(const char *data);

/*
 * Set discrete
 */
extern int FACOM_setDiscrete(const char *address, int action);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FACOM_H_

