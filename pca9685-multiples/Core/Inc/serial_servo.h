#ifndef SERIAL_SERVO_H
#define SERIAL_SERVO_H

#include "main.h"

void serial_servo_run(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

#endif /* SERIAL_SERVO_H */
