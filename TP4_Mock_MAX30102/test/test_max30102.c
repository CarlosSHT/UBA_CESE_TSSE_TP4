#include "unity.h"
#include "max30102.h"
#include "mock_port_stm32_i2c.h"

#define MAX30102_ADDRESS 0x57 << 1

void test_que_verifica_el_retorno_de_error_por_mascara_de_bit_vacia(void)
{
    uint8_t direccion_registro, mascara_bit, valor_salida;

    direccion_registro = 0x02;
    mascara_bit = 0;
    valor_salida = 0;

    TEST_ASSERT_EQUAL(max_ERROR, _max30102_read_config(direccion_registro, mascara_bit, &valor_salida));
}

void test_verifica_la_aplicacion_de_la_mascara_y_devuelve_valor_corrido_hacia_la_derecha_n_bits_hasta_que_sea_un_bit_de_valor_de_uno(void)
{
    uint8_t direccion_registro, mascara_bit, valor_salida;

    direccion_registro = 0x02;
    mascara_bit = 0x70;
    valor_salida = 0;

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x02, 1, NULL, 0xFF);
    TEST_ASSERT_EQUAL(max_OK, _max30102_read_config(direccion_registro, mascara_bit, &valor_salida));
    TEST_ASSERT_EQUAL(0x07, valor_salida);
}

void test_verifica_conexion_de_dispositivo(void)
{
    port_i2c_check_device_ExpectAndReturn(MAX30102_ADDRESS, 200, 5, port_OK);
    TEST_ASSERT_EQUAL(max_Connected, MAX30102_isConnected());
}

void test_verificar_el_part_ID_del_dispositivo(void)
{
    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0xFF, 1, NULL, 0x15);
    TEST_ASSERT_EQUAL(0x15, MAX30102_get_PartID_device());
}

void test_verifica_las_correctas_configuraciones_por_defecto_para_funcionamiento_en_modo_heartrate_y_spo2(void)
{
    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x02, 1, NULL, 0x80);
    TEST_ASSERT_EQUAL(max_Enable, MAX30102_get_AFullEnable());

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x02, 1, NULL, 0x80);
    TEST_ASSERT_EQUAL(max_Disable, MAX30102_get_PPGreadyEnable());

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x02, 1, NULL, 0x80);
    TEST_ASSERT_EQUAL(max_Disable, MAX30102_get_ALCovfEnable());

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x03, 1, NULL, 0x02);
    TEST_ASSERT_EQUAL(max_Enable, MAX30102_get_DTempReadyEnable());

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x08, 1, NULL, 0x10);
    TEST_ASSERT_EQUAL(max_Enable, MAX30102_get_RollOverEnable());

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x0C, 1, NULL, 0x1E);
    TEST_ASSERT_EQUAL(0x1E, MAX30102_get_PulAmp_conf(ledRED_pulseAmp));

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x0D, 1, NULL, 0x1E);
    TEST_ASSERT_EQUAL(0x1E, MAX30102_get_PulAmp_conf(ledIR_pulseAmp));

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x08, 1, NULL, 0x30);
    TEST_ASSERT_EQUAL(average_2samples, MAX30102_get_SMP_AVE_conf());

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x08, 1, NULL, 0x3E);
    TEST_ASSERT_EQUAL(empty_14samplesFIFO, MAX30102_get_AFULL_conf());

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x09, 1, NULL, 0x03);
    TEST_ASSERT_EQUAL(mode_SPO2, MAX30102_get_ModeCtrl_conf());

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x0A, 1, NULL, 0x04);
    TEST_ASSERT_EQUAL(rate_SPO2_100sps, MAX30102_get_SPO2rate_conf());

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x0A, 1, NULL, 0x04);
    TEST_ASSERT_EQUAL(pulseWidth_69us, MAX30102_get_LedPW_conf());

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x0A, 1, NULL, 0x04);
    TEST_ASSERT_EQUAL(adcRange__07_81pA, MAX30102_get_SPO2adc_conf());
}

void test_que_verifica_la_temperatura_obtenida(void)
{
    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x1F, 1, NULL, 0x1F);
    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x20, 1, NULL, 0x0A);
    TEST_ASSERT_EQUAL_FLOAT(31.625, MAX30102_get_TemperatureFloat());
}

void test_verifica_si_hay_valores_nuevos_disponibles_luego_de_recibir_una_interrupcion_almostfull(void)
{
    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x04, 1, NULL, 0x1E);

    TEST_ASSERT_EQUAL(30, MAX30102_get_FifoWRptr());

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x06, 1, NULL, 0x16);

    TEST_ASSERT_EQUAL(22, MAX30102_get_FifoRDptr());

    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x04, 1, NULL, 0x1E);
    port_i2c_read_mem_ExpectAndReturn(MAX30102_ADDRESS, 0x06, 1, NULL, 0x16);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT8(0, MAX30102_get_FifoWRptr() - MAX30102_get_FifoRDptr());
}
