#ifndef _FACOM_PRIVATE_H_
#define _FACOM_PRIVATE_H_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*
 * Cast digit to character
 */
#define digToChar(dig) ((dig) + '0')

/*
 * Cast hex digit to character
 */
#define hexDigToChar(hexDig) ((hexDig) + 'A' - 10)

/*
 * Cast hexadecimal digit to character
 */
extern void FACOM_intToString(int number, char *str);

/*
 * Cast hexadecimal digit to character
 */
extern void FACOM_hexToString(unsigned char number, char *str);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FACOM_PRIVATE_H_

