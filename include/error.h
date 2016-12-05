#ifndef _FACOM_ERROR_H_
#define _FACOM_ERROR_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define SUCCESS                         0
#define ERROR                          -1

#define ERROR_WRONG_PARAMETERS         -2

#define ERROR_OPEN_PORT               -51
#define ERROR_CLOSE_PORT              -52
#define ERROR_GET_PORT_OPTIONS        -53
#define ERROR_SET_PORT_OPTIONS        -54

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FACOM_ERROR_H_

