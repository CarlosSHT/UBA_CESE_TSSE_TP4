/*
 * max30102.h
 *
 *      Author: Carlos Herrera Trujillo
 */

#ifndef MAX30102_INC_MAX30102_H_
#define MAX30102_INC_MAX30102_H_

/** \brief Breve descripción del archivo
 **
 ** Descripción completa del archivo
 **
 ** \addtogroup Modulo Nombre del módulo
 ** \brief Breve descripción del conjunto de archivos
 ** @{ */

/* === Cabecera C++ ============================================================================ */
// #ifdef __cplusplus
// extern "C"
// {
// #endif

/* === Inclusiones de archivos externos ======================================================== */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "port_stm32_i2c.h"

/* === Definicion y Macros ===================================================================== */

/* == Declaraciones de tipos de datos ========================================================== */
typedef bool bool_t;

/**
 * @brief Definiciones utilizadas en las funciones del programa
 *
 */
typedef enum
{
    max_ERROR = -1,
    max_OK,
    conf_Error,
    conf_OK
} error_MAX_t;

typedef uint8_t max_ledAmp_t;

typedef enum
{
    max_Disconnected,
    max_Connected
} max_connectState_t;

typedef enum
{
    average_1samples,
    average_2samples,
    average_4samples,
    average_8samples,
    average_16samples,
    average_32Asamples,
    average_32Bsamples,
    average_32Csamples,
} max_smpAve_t;

typedef enum
{
    empty_0samplesFIFO,
    empty_1samplesFIFO,
    empty_2samplesFIFO,
    empty_3samplesFIFO,
    empty_4samplesFIFO,
    empty_5samplesFIFO,
    empty_6samplesFIFO,
    empty_7samplesFIFO,
    empty_8samplesFIFO,
    empty_9samplesFIFO,
    empty_10samplesFIFO,
    empty_11samplesFIFO,
    empty_12samplesFIFO,
    empty_13samplesFIFO,
    empty_14samplesFIFO,
    empty_15samplesFIFO,
} max_almostData_t;

typedef enum
{
    mode_HeartRate = 2,
    mode_SPO2 = 3,
    mode_MultiLED = 7,
} max_modectrl_t;

typedef enum
{
    adcRange__07_81pA,
    adcRange__15_63pA,
    adcRange__31_25pA,
    adcRange__62_50pA,
} max_SPO2adcrange_t;

typedef enum
{
    rate_SPO2_50sps,
    rate_SPO2_100sps,
    rate_SPO2_200sps,
    rate_SPO2_400sps,
    rate_SPO2_800sps,
    rate_SPO2_1000sps,
    rate_SPO2_1600sps,
    rate_SPO2_32000sps,
} max_SPO2rate_t;

typedef enum
{
    pulseWidth_69us,
    pulseWidth_118us,
    pulseWidth_215us,
    pulseWidth_411us,
} max_PWcontrol_t;

typedef enum
{
    ledRED_pulseAmp,
    ledIR_pulseAmp,
} max_led_t;

typedef enum
{
    slot1_MLcontrol,
    slot2_MLcontrol,
    slot3_MLcontrol,
    slot4_MLcontrol,
} max_slotnumber_t;

typedef enum
{
    max_Disable,
    max_Enable,
} max_enableflag_t;

/**
 * @brief Estructura principal del objeto sensor
 * donde se guarda el modo de funcionamiento (heart rate o SPO2),
 * número de muestras disponibles en el objeto, 32 muestras de led
 * rojo y 32 de leds IR, así como el valor de detección de interrupción
 * y una dirección de memoria donde se asignará las variables privadas sin acceso
 *
 */
typedef struct max30102_e
{
    max_modectrl_t mode_leds;
    uint8_t samples_cntr;
    uint32_t red_samples[32];
    uint32_t ir_samples[32];
    bool_t irq_flag;
    void *private_vars;
} max30102_t;

/* === Declaraciones de variables externas ===================================================== */

/* === Declaraciones de funciones internas ===================================================== */

/**
 * @brief Realiza una lectura de configuración del registro utilizando una mascara de bit
 * se realiza un corrimiento hacia la derecha hasta que el primer bit sea 1, por
 * ejemplo, si la mascara de bits es 0x38 entonces el valor de salida será entre 0x00 a 0x07
 *
 * @param reg_addr : valor de registro a leer
 * @param bit_mask : mascara de bit utilizado para cada configuración
 * @param value : valor de retorno pasado por puntero
 * @return error_MAX_t : en caso se pueda realizar una lectura y la mascara de bits no sea
 * cero devolvera max_OK, caso contrario max_ERROR
 */
error_MAX_t _max30102_read_config(uint8_t reg_addr, uint8_t bit_mask, uint8_t *value);

/**
 * @brief Permite realizar la configuración del registro utilizando la mascara de bit
 * donde se puede limitar los valores de entrada ("value") en rangos por cada valor de máscara,
 * por ejemplo, si la mascara es 0x30, el valor admitido será desde 0x00 a 0x03
 *
 * @param reg_addr : valor de registro a escribir
 * @param bit_mask : mascara de bits utilizado para la configuración
 * @param value : valor de entrada de tamaños correspondientes a cada mascara de bits
 * @return error_MAX_t : en caso se realice la escritura con exito devuelve  un max_OK,
 * en caso contrario devolverá max_ERROR
 */
error_MAX_t _max30102_write_config(uint8_t reg_addr, uint8_t bit_mask, uint8_t value);

/**
 * @brief Permite realizar una lectura directa del registro en una salida de 8 bits
 *
 * @param reg_addr : valor de registro a leer
 * @param value: valor de salida de 8 bits
 * @return error_MAX_t : en caso no se pueda realizar una lectura devolverá max_ERROR,
 * caso contrario max_OK
 */
error_MAX_t _max30102_read_register(uint8_t reg_addr, uint8_t *value);

/**
 * @brief Permite obtener una lectura de varios registros leidos automaticamente y devueltos
 * a traves de un puntero
 *
 * @param reg_addr : registro de 8 bits inicial a leer
 * @param value : puntero al conjunto de valores leidos
 * @param value_bsize : cantidad de bytes a leer
 * @return error_MAX_t : max_OK en caso se realice una lectura exitosa, caso contrario devolverá
 * max_ERROR
 */
error_MAX_t _max30102_multiread_register(uint8_t reg_addr, uint8_t *value, uint16_t value_bsize);

/**
 * @brief Realiza una escritura directa en un registro de 8 bits
 *
 * @param reg_addr : registro a escribir
 * @param value : valor de 8 bits a escribir
 * @return error_MAX_t : estado de la operación, max OK en caso de exito,
 * max_ERROR en caso no se pueda escribir
 */
error_MAX_t _max30102_write_register(uint8_t reg_addr, uint8_t *value);

/**
 * @brief Función utilizada para verificar el estado de activación de los bits de interrupción
 *
 * @param reg_val : registro a verificar
 * @param reg_bm : mascara de bits, generalmente 1 solo bit en alto
 * @return bool_t : en caso el bit de la msacara se encuentre en alto en el registro devuelve true,
 * caso contrario devolverá false
 */
bool_t _max30102_checkBitMask(uint8_t reg_val, uint8_t reg_bm);

/* === Declaraciones de funciones externas ===================================================== */

/**
 * @brief Permite conocer el estado de conexión del dispositivo MAX30102
 * al realizar una llamda por bus I2C y verificar la recepción por parte del sensor
 *
 * @return max_connectState_t : en caso este conectado devolverá max_Connected, caso contrario
 * max_Disconnected
 */
max_connectState_t MAX30102_isConnected();

/**
 * @brief Realiza una lectura del registro PART ID a fin de verificar
 * que el dispositivo detectado con la dirección 0x57 tenga una segunda
 * verificación de presencia
 *
 * @return uint8_t : devuelve el valor en 8 bits del registro 0x57
 */
uint8_t MAX30102_get_PartID_device(void);

/**
 * @brief Inicializa el objeto sensor a traves del tipo max30102_t,
 * verifica el estado de conexión, valor de partID, en caso sea correcto
 * se hace una designación de memoria para las variable privadas de flags.
 * Por último se establece con valores por defecto todos los registros
 * configurables
 *
 * @param sensor : objeto sensor max30102_t
 * @return error_MAX_t : en caso todo suceda correctamente devuelve max_OK, caso contrario
 * max_ERROR
 */
error_MAX_t MAX30102_Init(max30102_t *sensor);

/**
 * @brief Establece los valores por defecto para el uso del sensor en modo
 * SPO2
 *
 */
void MAX30102_defaultConfig(void);

/** MAPA DE REGISTROS **/

/**
 * @brief Lee el valor del registro de interrupción 1, cuando
 * ocurre un evento de interrupción el bit se establece en 0.
 * Una vez leído el registro se limpian los valores (establecen en 1)
 *
 * @return uint8_t : valor del registro de interrupción 1 de 8 bits
 */
uint8_t MAX30102_get_IRQstatus1(void);

/**
 * @brief Lee el valor del registro de interrupción 2, cuando
 * ocurre un evento de interrupción el bit se establece en 0.
 * Una vez leído el registro se limpian los valores (establecen en 1)
 *
 * @return uint8_t : valor del registro de interrupción 2 de 8 bits
 */
uint8_t MAX30102_get_IRQstatus2(void);

/**
 * @brief Permite establecer el valor del bit de interrupción
 *  almost full, cambiando el valor del bit a 1.
 *
 *
 * @param flag max_Enable habilita la interrupción, max_Disable
 * deshabilita la interrupción por almosft full enable
 */
void MAX30102_set_AFullEnable(max_enableflag_t flag);

/**
 * @brief Se obtiene el estado de bit de interrupción habilitado
 *
 * @return max_enableflag_t : max_Enable el bit se encuentra habilitado,
 * caso contrario max_Disable
 */
max_enableflag_t MAX30102_get_AFullEnable(void);

/**
 * @brief Permite habilitar la interrupción de detección de nueva muestra
 * se guarda en el FIFO
 *
 * @param flag: max_Enable habilita las interrupciones por cada muestra nueva,
 * caso contrario se establece con max_Disable
 */
void MAX30102_set_PPGreadyEnable(max_enableflag_t flag);

/**
 * @brief Obtiene el valor del bit de interrupción para PPG ready
 *
 * @return max_enableflag_t : max_Enable si está habilitado la interrupción,
 * max_Disable en caso que no se encuentre habilitado
 */
max_enableflag_t MAX30102_get_PPGreadyEnable(void);

/**
 * @brief Habilita el bit de interrupcion por desbodamiento ambiental
 *
 * @param flag : max_Enable lo habilitado, caso contrario se utiliza
 * max_Disable
 */
void MAX30102_set_ALCovfEnable(max_enableflag_t flag);

/**
 * @brief Permite obtener el valor de estado del bit de interrupción
 * de luz ambiental
 *
 * @return max_enableflag_t : max_Enable si está habilitado, caso contrario
 * devuelve max_Disable
 */
max_enableflag_t MAX30102_get_ALCovfEnable(void);

/**
 * @brief Establece la habilitación o deshabilitación del bit de interrupción
 * DIE Temp Ready , que permite conocer si se realizó la conversión de temperatura
 *
 * @param flag : max_Enable habilita la interrupción de DIE TEMP, en otro caso
 * con max_Disable permite deshabilitarlo
 */
void MAX30102_set_DTempReadyEnable(max_enableflag_t flag);

/**
 * @brief Permite conocer si el flag de activación de interrupción
 * DIE Temp Ready se encuentra activado o desactivado
 *
 * @return max_enableflag_t : max_Enable en caso que se encuentre
 * habilitado la interrupción de DIE Temp, caso contrario devuelve
 * max_Disable
 */
max_enableflag_t MAX30102_get_DTempReadyEnable(void);

/*** FIFO Pointers ***/

/**
 * @brief Permite limpiar (configurar) el valor del
 * registro de FIFO writer pointer
 *
 * @param value : parametro de etnrada de 5 bits (0x00 a 0x1F)
 */
void MAX30102_set_FifoWRptr(uint8_t value);

/**
 * @brief Permite obtener la dirección de locación, es decir
 * cuantas muestras nuevas se han cargado en el regitro FIFO
 *
 * @return uint8_t : valor de salida de 5 bits
 */
uint8_t MAX30102_get_FifoWRptr(void);

/**
 * @brief Permite limpiar (configurar)el valor del registro
 * FIFO reader pointer
 *
 * @param value : Valor de 5 bits desde 0x00 a 0x1F
 */
void MAX30102_set_FifoRDptr(uint8_t value);

/**
 * @brief Permite obtener la locación de la última muestra leída
 *
 * @return uint8_t : Valor de slaida de 5 bits desde 0x00 a 0x1F
 */
uint8_t MAX30102_get_FifoRDptr(void);

/**
 * @brief Permite limpiar (establecer en 0x00) el registro de FIFO
 * overflow counter
 *
 * @param value : valor de 5 bits sin signo a escribir en el FIFO Overflow
 * counter (0x00 a 0x1F)
 */
void MAX30102_set_FifoOVFcntr(uint8_t value);

/**
 * @brief Obtiene el valor del registro FIFO overflow counter, es decir
 * la cantidad de muestras perdidas
 *
 * @return uint8_t : cantidad de muestras perdidas desde 0x00 hasta 0x1F muestras
 */
uint8_t MAX30102_get_FifoOVFcntr(void);

/*** CONFIGURATION ***/

/**
 * @brief Configura el valor de muestras a promediar
 *
 * @param n_smp Desde 1 a 32 muestras
 */
void MAX30102_set_SMP_AVE_conf(max_smpAve_t n_smp);

/**
 * @brief Se obtiene el número de muestras que promediará el sensor
 * para luego guardarlo en el FIFO     *
 *
 * @return max_smpAve_t : 1, 2, 4, 8, 16 o 32 muestras
 */
max_smpAve_t MAX30102_get_SMP_AVE_conf(void);

/**
 * @brief Establece el flag RollOver en modo habilitado o no,
 * en caso se habilite una vez se llene el FIFO va a la dirección 0 de esta
 * y continua guardando muestras
 *
 * @param bit_flag : true se habilita, caso contrario no permite
 * que se guarden nuevas muestras
 */
void MAX30102_set_RollOverEnable(bool_t bit_flag);

/**
 * @brief Obtiene el valor del flag RollOver
 *
 * @return bool_t : true si está habilitado, false en caso contrario
 */
bool_t MAX30102_get_RollOverEnable(void);

/**
 * @brief Se establece la cantidad de muestras previas a llenar
 * el registros FIFO para la activación de la interrupción AFULL
 *
 * @param value : De 0 muestas a 15 muestras (0x00 - 0x0F)
 */
void MAX30102_set_AFULL_conf(max_almostData_t value);

/**
 * @brief Se obtiene el valor del registro almost full
 *
 * @return max_almostData_t : valor entre 0x00 a 0x0F
 */
max_almostData_t MAX30102_get_AFULL_conf(void);

/**
 * @brief Establece el modo de funcionamientro del sensor
 *
 * @param mode : 0x02 Heart rate, 0x03 SPO2, 0x07 MultiLed, otros
 * valores no tienen efecto en configurar el modo
 */
void MAX30102_set_ModeCtrl_conf(max_modectrl_t mode);

/**
 * @brief Se lee el modo de funcionamiento en el que está configurador
 * el sensor
 *
 * @return max_modectrl_t : Modo HeartRate, modo SPO2 o multiled
 */
max_modectrl_t MAX30102_get_ModeCtrl_conf(void);

/**
 * @brief Configura el valor del rango ADC para el SPO2
 *
 * @param adc_val : valor entre 0 a 3.
 */
void MAX30102_set_SPO2adc_conf(max_SPO2adcrange_t adc_val);

/**
 * @brief Se obtiene el valor de rango de ADC de control con resolución
 * de 18 bits
 *
 * @return max_SPO2adcrange_t : valor de retorno de 0x00 a 0x03 en pico Amperes
 */
max_SPO2adcrange_t MAX30102_get_SPO2adc_conf(void);

/**
 * @brief Establece la frecuencia de muestreo de control para el SPO2
 * desde 0x00 a 0x07 correlativamente a 50mps hasta 3200mps
 *
 * @param rate
 */
void MAX30102_set_SPO2rate_conf(max_SPO2rate_t rate);

/**
 * @brief Obtiene el valor de frencuencia de muestreo para el modo SPO2
 *
 * @return max_SPO2rate_t : valor de retorno desde 50 muestras por segundo a 3200 muestras por segundo
 */
max_SPO2rate_t MAX30102_get_SPO2rate_conf(void);

/**
 * @brief Establece el valor del ancho de pulso de ambos leds (rojo e IR)
 * de acuerdo a la siguiente lista de valores
 *
 *
    pulseWidth_69us,    ADC 15 bits
    pulseWidth_118us,   ADC 16 bits
    pulseWidth_215us,   ADC 17 bits
    pulseWidth_411us,   ADC 18 bits
    *
    * @param pw_value : parametro a configurar para el ancho de pulso
    */
void MAX30102_set_LedPW_conf(max_PWcontrol_t pw_value);

/**
 * @brief Permite obtener el valor del ancho del pulso en microsegundos
 * con tipo de objeto max_PWcontrol_t
 *
 * @return max_PWcontrol_t :  pulseWidth_69us, pulseWidth_118us, pulseWidth_215us o pulseWidth_411us
 */
max_PWcontrol_t MAX30102_get_LedPW_conf(void);

/**
 * @brief Configura en valor de 8 bits la amplitud del pulso
 *
 * @param led : ledRED_pulseAmp oledIR_pulseAmp
 * @param amp_value : valor de la amplitud desde 0x00 a 0xFF
 */
void MAX30102_set_PulAmp_conf(max_led_t led, max_ledAmp_t amp_value);

/**
 * @brief Obtiene la amplitud del pulso configurado por tipo de led
 *
 * @param led : ledRED_pulseAmp oledIR_pulseAmp
 * @return max_ledAmp_t : valor de la amplitud del pulso
 */
max_ledAmp_t MAX30102_get_PulAmp_conf(max_led_t led);

/**
 * @brief Establece el led a configuraren cada slot
 *
 * @param led : ledRED_pulseAmp oledIR_pulseAmp
 * @param n_slot : número de slot
 */
void MAX30102_set_LedSlot_conf(max_led_t led, max_slotnumber_t n_slot);

/**
 * @brief Permite obtener para que led se configuro el slot a consultar
 *
 * @param n_slot : Número de slot a consultar
 * @return max_led_t : se obtiene a que led está configurado IR o Red
 */
max_led_t MAX30102_get_LedSlot_conf(max_slotnumber_t n_slot);

/**
 * @brief Habilita el estado de suspensión (bajo consumo) del sensor
 *
 * @param flag : max_Enable para entrar en suspensión,
 * caso contrario se desactivba con max_Disable
 */
void MAX30102_set_Shutdown_conf(max_enableflag_t flag);

/**
 * @brief Permite conocer el estado de suspensión del sensor
 * haciendo u na lectura del bit SHDN en el registro 0x09
 *
 */
max_enableflag_t MAX30102_get_Shutdown_conf(void);

/**
 * @brief Reinicia el sensor borrando toda la configuración que posea
 *
 */
void MAX30102_set_Reset_conf(void);

/*** DIE TEMPERATURE ***/

/**
 * @brief Obtiene el valor entero de la temperatura con un tipo de dato
 * int8_t que permite leer desde -128 a 127 en pasos de 1°
 *
 * @return uint8_t : valor de temperatura con resoluación de 1°
 */
uint8_t MAX30102_get_IntegerTemp(void);

/**
 * @brief Permite obtener una fracción de temperatura dada por la resolución
 * de 0.0625° en un rango de número de 4 bits
 *
 * @return uint8_t : valor de una fracción de temperatura
 */
uint8_t MAX30102_get_FractionTemp(void);

/**
 * @brief Obtiene el valor total de temperatura entre -128 a 127
 * con resolución de 0.0625°
 *
 * @return float : valor de temperatura calculado
 */
float MAX30102_get_TemperatureFloat(void);

/**
 * @brief Habilita el inicio de conversión de lectura de temperatura
 *
 * @param flag : max_Enable inicia la conversión, automaticamente en el sensor
 * vuelve a 0 (max_Disable)
 */
void MAX30102_set_TempEnable(max_enableflag_t flag);

/**
 * @brief Obtiene el valor del flag conversión de temperatura del
 *  sensor.
 *
 * @param flag : obtiene un max_Disable una vez termina la conversión de temperatura, caso
 * contrario mientras esta en la conversión posee el valor de max_Enable
 */
void MAX30102_get_TempEnable(max_enableflag_t *flag);

/**
 * @brief Verifica los registros de interrupciones 0x00 y 0x01
 * luego actualiza cada flag privado del objeto tipo max30102_t
 *
 * @param max_sensor : objeto inicializado tipo max30102_t
 * @return error_MAX_t : la función devuelve max_OK
 */
error_MAX_t MAX30102_check_IrqFlags(max30102_t *max_sensor);

/**
 * @brief Lee los datos de las muestras Rojo e IR del sensor si existe una diferencia
 * entre los valores de los punteros de escritura y lectura. Se revisa previamente si
 * el flag de interrupción por "almost full" fue activado.
 *
 * @param max_sensor : objeto sensor
 * @return error_MAX_t : si no hay flag de interrupción devuelve max_Error, caso contrario continua
 * el proceso y devuelve max_OK
 */
error_MAX_t MAX30102_read_FIFODataSamples(max30102_t *max_sensor);

/* === Ciere de documentacion ================================================================== */
// #ifdef __cplusplus
// }
// #endif
#endif /* MAX30102_INC_MAX30102_H_ */
