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

/*
 * Discrete types
 */
#define DISCRETE_X      1       // Input discrete
#define DISCRETE_Y      2       // Output discrete
#define DISCRETE_M      3       // Internal memory discrete
#define DISCRETE_S      4       // Step discrete
#define DISCRETE_T      5       // Timer discrete
#define DISCRETE_C      6       // Counter discrete

/*
 * Discrete actions
 */
#define ACTION_DISABLE  1
#define ACTION_ENABLE   2
#define ACTION_SET      3
#define ACTION_RESET    4

/*
 * PLC on/off actions
 */
#define ACTION_OFF      0
#define ACTION_ON       1

/*
 * Open connection to PLC
 */
extern int FACOM_open(const char *port, unsigned char stationNumber);

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
 * Send data to Fatek PLC
 */
extern int FACOM_write(const char *data);

/*
 * Read data from Fatek PLC
 */
extern int FACOM_read(char *data, unsigned int bufferSize);

/*
 * Set mode of Facom PLC (on/off)
 */
extern int FACOM_run(unsigned char run);

/*
 * Start Fatek PLC (set run mode on)
 */
extern int FACOM_start(void);

/*
 * Stop Fatek PLC (set run mode off)
 */
extern int FACOM_stop(void);

/*
 * Control discrete
 */
extern int FACOM_setDiscrete(unsigned char discreteType,
                             int discreteNumber,
                             unsigned char action);

/*
 * Set multiple status of continuous discrete
 */
extern int FACOM_setDiscretes(unsigned char discreteType,
                              int discreteNumber,
                              unsigned char discreteCount,
                              unsigned char *data);

/*
 * Read continuous discrete state
 */
extern int FACOM_getDiscretes(unsigned char discreteType,
                              int discreteNumber,
                              unsigned char discreteCount,
                              unsigned char *data);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FACOM_H_

