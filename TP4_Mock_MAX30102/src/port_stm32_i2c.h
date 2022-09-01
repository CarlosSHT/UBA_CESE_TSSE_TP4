/*
 * port_stm32_i2c.h
 *
 *  Created on: Aug 23, 2022
 *      Author: carlo
 */

#ifndef INC_PORT_STM32_I2C_H_
#define INC_PORT_STM32_I2C_H_

#define TOU_I2C_PORT 10; // timeout definido de 10ms

#include "stddef.h"
#include "stdint.h"
#include "stm32f4xx_hal_i2c.h"

typedef enum
{
    port_ERROR = -1,
    port_OK
} error_port_t;

/**
 * @brief Permite inicializar un objeto i2c para la capa HAL de stm32
 *
 * @param hi2c1 : objeto i2c
 * @return error_port_t : estado de error, port_ERROR en caso no se pueda inicializar,
 * caso contrario devolverá port_OK
 */
error_port_t port_i2c_init(I2C_HandleTypeDef *hi2c1);

/**
 * @brief Lee el valor de un registro y devuelve el valor
 * leido o -1 en caso se produzca un error
 *
 * @param dev_addr : dirección del dispositivo I2C
 * @param reg_addr : valor de registro a leer
 * @param reg_addr_size : tamaño del registro en bytes
 * @param timeout : tiempo de espera
 * @return int16_t : valor del registro leido, o -1 en caso haya un error
 */
int16_t port_i2c_read_mem(uint16_t dev_addr,
                          uint16_t reg_addr,
                          uint16_t reg_addr_size,
                          uint32_t *timeout);

/**
 * @brief Reliza una lectura de varios registros de forma continua
 *
 * @param dev_addr : Valor de dirección del dispositivo i2c
 * @param reg_addr : registro inicial desde donde se leeerá
 * @param reg_addr_size : tamaño de la dirección de registro en bytes
 * @param buffer : puntero a la primera dirección leída
 * @param buffer_size : tamaño de lectura en bytes
 * @param timeout : timeout de 10
 * @return error_port_t : port_OK en caso se lea correctamente, en
 * otro caso devolverá port_ERROR
 */
error_port_t port_i2c_multi_read_mem(uint16_t dev_addr,
                                     uint16_t reg_addr,
                                     uint16_t reg_addr_size,
                                     uint8_t *buffer,
                                     uint16_t buffer_size,
                                     uint32_t *timeout);

/**
 * @brief Permite realizar la escritura de 1 o varios registros utilizando una
 * sola función
 *
 * @param dev_addr : Valor de dirección del dispositivo i2c
 * @param reg_addr : dirección del registro inicial desde donde se realizará la escritura
 * @param reg_addr_size : tamaño de la dirección en bytes
 * @param buffer : puntero con el valor de dirección del primer registro a escribir
 * @param buffer_size : tamaño de escritura en bytes
 * @param timeout : tiempo de espera
 * @return error_port_t : valor de salida por_OK en caso realice una secuencia correcta,
 * caso contrario devolverá port_ERROR
 */
error_port_t port_i2c_write_mem(uint16_t dev_addr,
                                uint16_t reg_addr,
                                uint16_t reg_addr_size,
                                uint8_t *buffer,
                                uint16_t buffer_size,
                                uint32_t *timeout);

/**
 * @brief Permite verificar si el dispositivo esta conectado utilizando un tiempo
 * de espera y número de intentos de conexión
 *
 * @param dev_addr : Dirección del dispositivo i2c
 * @param ms_waiting : tiempo de espera hasta que responda
 * @param n_retrys : número de intentos
 * @return error_port_t : en caso no encuentre el dispositivo devolverá port_ERROR,
 * pero si se encuentra devolverá port_OK
 */
error_port_t port_i2c_check_device(uint16_t dev_addr,
                                   uint16_t ms_waiting,
                                   uint8_t n_retrys);

#endif /* INC_PORT_STM32_I2C_H_ */
