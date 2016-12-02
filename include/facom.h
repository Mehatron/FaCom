#ifndef _FACOM_H_
#define _FACOM_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*
 * Open connection to PLC
 */
int FACOM_open(const char *port);

/*
 * Close connection to PLC
 */
void FACOM_close(void);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FACOM_H_

