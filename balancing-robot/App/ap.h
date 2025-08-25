/*
 * ap.h
 *
 *  Created on: Aug 11, 2025
 *      Author: USER
 */

#ifndef AP_H_
#define AP_H_

#include "hw.h"

void apInit(void);
void apMain(void);

void Process_UART_Command(char* cmd);

#endif /* AP_H_ */
