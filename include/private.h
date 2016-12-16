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
extern void FACOM_intToHexString(unsigned char number, char *str);

/*
 * Count number of digits in number (integer)
 */
extern int FACOM_numberOfDigits(int number);

/*
 * Check for error while set commands
 */
extern int FACOM_checkForErrors(void);

/*
 * Get discrete address (for sending to Fate PLC)
 */
extern int FACOM_getDiscreteAddress(unsigned char discreteType,
                                    int discreteNumber,
                                    char *address);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FACOM_PRIVATE_H_

