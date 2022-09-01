/*
 * max30102.c
 *
 *  Created on: 21 ago. 2022
 *      Author: Carlos Herrera
 */

#include "max30102.h"

#define MAX30102_ADDR 0x57
#define MAX_REG_SIZE 1
#define MAX_DAT_SIZE 1
#define MAX_I2C_TOUT 10
#define MAX_ID_IC 0x15
#define MAX_N_RETRYS 5
#define MAX_TIME_RETRY 200
#define BITMASK_EMPTY 0x00
#define TEMP_FRACT_STEP 0.0625
/*** Register Maps and Descriptions ***/
/* STATUS REGISTERS */
#define MAX_IRQ_STA1 0x00
#define MAX_IRQ_STA2 0x01
#define MAX_IRQ_EN1 0x02
#define MAX_IRQ_EN2 0x03
// FIFO
#define MAX_FIFO_WR_PTR 0x04
#define MAX_OVF_CNTR 0x05
#define MAX_FIFO_RD_PTR 0x06
#define MAX_FIFO_DATA 0x07
/* CONFIGURATION REGISTERS */
#define MAX_FIFO_CONF 0x08
#define MAX_MODE_CONF 0x09
#define MAX_SPO2_CONF 0x0A
#define MAX_RESERVED1 0x0B
#define MAX_LED_PAMP1 0x0C
#define MAX_LED_PAMP2 0x0D
#define MAX_RESERVED2 0x0E
#define MAX_RESERVED3 0x0F
#define MAX_MLED_CTRL1 0x11
#define MAX_MLED_CTRL2 0x12
#define MAX_RESERVED4 0x13 // to 0x17
#define MAX_RESERVED5 0x18 // to 0x1E
/* DIE TEMPERATURE REGISTERS */
#define MAX_DTEMP_INTGR 0x1F
#define MAX_DTEMP_FRACT 0x20
#define MAX_DTEMP_CONF 0x21
#define MAX_RESERVED6 0x22 // to 0x2F
/* PART ID REGISTERS */
#define MAX_REV_ID 0xFE
#define MAX_PART_ID 0xFF

/*** Bit Masks ***/
#define BM_MASK_FULL 0xFF
#define BM_A_FULL 0b10000000
#define BM_PPG_RDY 0b01000000
#define BM_ALC_OVF 0b00100000
#define BM_PWR_RDY 0b00000001
#define BM_DIE_TEMP_RDY 0b00000010
#define BM_A_FULL_EN 0b10000000
#define BM_PPG_RDY_EN 0b01000000
#define BM_ALC_OVF_EN 0b00100000
#define BM_DIE_TEMP_RDY_EN 0b00000010
#define BM_FIFO_WR_PTR 0b00011111
#define BM_OVF_COUNTER 0b00011111
#define BM_FIFO_RD_PTR 0b00011111
#define BM_FIFO_DATA 0b11111111
#define BM_SMP_AVE 0b11100000
#define BM_FIFO_ROLLOVER_EN 0b00010000
#define BM_FIFO_A_FULL 0b00001111
#define BM_SHDN 0b10000000
#define BM_RESET 0b01000000
#define BM_MODE 0b00000111
#define BM_SPO2_ADC_RGE 0b01100000
#define BM_SPO2_SR 0b00011100
#define BM_LED_PW 0b00000011
#define BM_LED1_PA 0b11111111
#define BM_LED2_PA 0b11111111
#define BM_SLOT2 0b01110000
#define BM_SLOT1 0b00000111
#define BM_SLOT4 0b01110000
#define BM_SLOT3 0b00000111
#define BM_TINT 0b11111111
#define BM_TFRAC 0b00001111
#define BM_TEMP_EN 0b00000001
#define BM_REV_ID 0b11111111
#define BM_PART_ID 0b11111111

typedef struct max30102_irq_flags_e
{
    bool_t irqflag_A_FULL;
    bool_t irqflag_PPG_RDY;
    bool_t irqflag_ALC_OVF;
    bool_t irqflag_PWR_RDY;
    bool_t irqflag_DIE_TEMP_RDY;
} max30102_irq_flags_t;

max30102_t *spo2_maxsensor;
max30102_irq_flags_t *private_flags;

void MAX30102_defaultConfig(void);

max_connectState_t MAX30102_isConnected()
{
    error_port_t state_i2c_port;
    state_i2c_port = port_i2c_check_device(MAX30102_ADDR << 1, MAX_TIME_RETRY,
                                           MAX_N_RETRYS);

    if (state_i2c_port != port_OK)
        return max_Disconnected;

    return max_Connected;
}

error_MAX_t _max30102_read_config(uint8_t reg_addr, uint8_t bit_mask,
                                  uint8_t *value)
{
    uint8_t ret_val = 0, aux_bmask = bit_mask;

    if (bit_mask == BITMASK_EMPTY)
        return max_ERROR;

    if ((_max30102_read_register(reg_addr, value)) != max_OK)
        return max_ERROR;

    for (uint8_t var = 0; var < 8 * MAX_DAT_SIZE; ++var)
    {
        if ((aux_bmask & 0x01) == 0x00)
        {
            ret_val++;
            aux_bmask = aux_bmask >> 1;
        }
    }

    *value = (*value & bit_mask) >> ret_val;

    return max_OK;
}

error_MAX_t _max30102_write_config(uint8_t reg_addr, uint8_t bit_mask,
                                   uint8_t value)
{
    uint8_t ret_val = 0, aux_bmask = bit_mask, val2write = value, saved_val = 0;

    if ((_max30102_read_register(reg_addr, &saved_val)) != max_OK)
        return max_ERROR;

    for (uint8_t var = 0; var < 8 * MAX_DAT_SIZE; ++var)
    {
        if ((aux_bmask & 0x01) == 0x00)
        {
            ret_val++;
            aux_bmask = aux_bmask >> 1;
        }
    }

    val2write = (val2write << ret_val) & bit_mask;

    saved_val = saved_val & ~bit_mask;
    val2write = saved_val | val2write;

    if ((_max30102_write_register(reg_addr, &val2write)) != max_OK)
        return max_ERROR;

    return max_OK;
}

error_MAX_t _max30102_multiwrite_register(uint8_t reg_addr, uint8_t *value,
                                          uint16_t value_bsize)
{
    error_port_t state_i2c_port;
    state_i2c_port = port_i2c_write_mem(MAX30102_ADDR << 1, (uint16_t)reg_addr, MAX_REG_SIZE, value, value_bsize, NULL);
    if (state_i2c_port != port_OK)
        return max_ERROR;
    return max_OK;
}
error_MAX_t _max30102_multiread_register(uint8_t reg_addr, uint8_t *value,
                                         uint16_t value_bsize)
{
    error_port_t state_i2c_port;

    state_i2c_port = port_i2c_multi_read_mem(MAX30102_ADDR << 1, (uint16_t)reg_addr,
                                             MAX_REG_SIZE, value, value_bsize, NULL);
    if (state_i2c_port != port_OK)
        return max_ERROR;
    return max_OK;
}

error_MAX_t _max30102_read_register(uint8_t reg_addr, uint8_t *value)
{
    int16_t aux_val;

    aux_val = port_i2c_read_mem(MAX30102_ADDR << 1, (uint16_t)reg_addr,
                                MAX_REG_SIZE, NULL);
    if (aux_val < 0)
        return max_ERROR;
    *value = aux_val;
    return max_OK;
}

error_MAX_t _max30102_write_register(uint8_t reg_addr, uint8_t *value)
{
    error_port_t state_i2c_port;

    state_i2c_port = port_i2c_write_mem(MAX30102_ADDR << 1, (uint16_t)reg_addr,
                                        MAX_REG_SIZE, value, MAX_DAT_SIZE, NULL);
    if (state_i2c_port != port_OK)
        return max_ERROR;
    return max_OK;
}

uint8_t MAX30102_get_PartID_device(void)
{
    uint8_t aux_partid;
    _max30102_read_config(MAX_PART_ID, BM_PART_ID, &aux_partid);
    return aux_partid;
}

error_MAX_t MAX30102_Init(max30102_t *sensor)
{
    if (MAX30102_isConnected() != max_Connected)
    {
        return max_ERROR;
    }

    if (MAX30102_get_PartID_device() != MAX_ID_IC)
    {
        return max_ERROR;
    }

    spo2_maxsensor = sensor;
    spo2_maxsensor->private_vars = (void *)malloc(sizeof(max30102_irq_flags_t));
    private_flags = (max30102_irq_flags_t *)spo2_maxsensor->private_vars;
    memset(private_flags, false, sizeof(max30102_irq_flags_t));

    MAX30102_defaultConfig();
    return max_OK;
}

void MAX30102_defaultConfig(void)
{
    MAX30102_set_Reset_conf();
    MAX30102_set_FifoWRptr(0);
    MAX30102_set_FifoRDptr(0);
    MAX30102_set_FifoOVFcntr(0);
    MAX30102_set_AFullEnable(max_Enable);
    MAX30102_set_PPGreadyEnable(max_Disable);
    MAX30102_set_ALCovfEnable(max_Disable);
    MAX30102_set_DTempReadyEnable(max_Enable);
    MAX30102_set_RollOverEnable(max_Enable);
    MAX30102_set_PulAmp_conf(ledRED_pulseAmp, 0x1E);
    MAX30102_set_PulAmp_conf(ledIR_pulseAmp, 0x1E);
    MAX30102_set_SMP_AVE_conf(average_2samples);
    MAX30102_set_AFULL_conf(empty_14samplesFIFO);
    MAX30102_set_ModeCtrl_conf(mode_SPO2);
    MAX30102_set_SPO2rate_conf(rate_SPO2_100sps);
    MAX30102_set_LedPW_conf(pulseWidth_69us);
    MAX30102_set_SPO2adc_conf(adcRange__07_81pA);
}
/*** Interrupt states ***/
uint8_t MAX30102_get_IRQstatus1(void)
{
    uint8_t reg_val;
    _max30102_read_config(MAX_IRQ_STA1, BM_MASK_FULL, &reg_val);
    return reg_val;
}
uint8_t MAX30102_get_IRQstatus2(void)
{
    uint8_t reg_val;
    _max30102_read_config(MAX_IRQ_STA2, BM_MASK_FULL, &reg_val);
    return reg_val;
}

void MAX30102_set_AFullEnable(max_enableflag_t flag)
{
    _max30102_write_config(MAX_IRQ_EN1, BM_A_FULL_EN, flag);
}
max_enableflag_t MAX30102_get_AFullEnable(void)
{
    uint8_t aux_afull;
    _max30102_read_config(MAX_IRQ_EN1, BM_A_FULL_EN, &aux_afull);
    return (max_enableflag_t)aux_afull;
}

void MAX30102_set_PPGreadyEnable(max_enableflag_t flag)
{
    _max30102_write_config(MAX_IRQ_EN1, BM_PPG_RDY_EN, flag);
}
max_enableflag_t MAX30102_get_PPGreadyEnable(void)
{
    uint8_t aux_ppgrdy;
    _max30102_read_config(MAX_IRQ_EN1, BM_PPG_RDY_EN, &aux_ppgrdy);
    return (max_enableflag_t)aux_ppgrdy;
}

void MAX30102_set_ALCovfEnable(max_enableflag_t flag)
{
    _max30102_write_config(MAX_IRQ_EN1, BM_ALC_OVF_EN, flag);
}
max_enableflag_t MAX30102_get_ALCovfEnable(void)
{
    uint8_t aux_alcovf;
    _max30102_read_config(MAX_IRQ_EN1, BM_ALC_OVF_EN, &aux_alcovf);
    return (max_enableflag_t)aux_alcovf;
}

void MAX30102_set_DTempReadyEnable(max_enableflag_t flag)
{
    _max30102_write_config(MAX_IRQ_EN2, BM_DIE_TEMP_RDY_EN, flag);
}
max_enableflag_t MAX30102_get_DTempReadyEnable(void)
{
    uint8_t aux_dtemp;
    _max30102_read_config(MAX_IRQ_EN2, BM_DIE_TEMP_RDY_EN, &aux_dtemp);
    return (max_enableflag_t)aux_dtemp;
}

/*** FIFO Pointers ***/
void MAX30102_set_FifoWRptr(uint8_t value)
{
    uint8_t buffi2c;
    memset(&buffi2c, value, 3);
    _max30102_multiwrite_register(MAX_FIFO_WR_PTR, &buffi2c, 6);
}
uint8_t MAX30102_get_FifoWRptr(void)
{
    uint8_t ret;
    _max30102_read_config(MAX_FIFO_WR_PTR, BM_FIFO_WR_PTR, &ret);
    return ret;
}

void MAX30102_set_FifoRDptr(uint8_t value)
{
    uint8_t buffi2c;
    memset(&buffi2c, value, 3);
    _max30102_multiwrite_register(MAX_FIFO_RD_PTR, &buffi2c, 6);
}
uint8_t MAX30102_get_FifoRDptr(void)
{
    uint8_t ret;
    _max30102_read_config(MAX_FIFO_RD_PTR, BM_FIFO_RD_PTR, &ret);
    return ret;
}

void MAX30102_set_FifoOVFcntr(uint8_t value)
{
    uint8_t buffi2c;
    memset(&buffi2c, value, 3);
    _max30102_multiwrite_register(MAX_OVF_CNTR, &buffi2c, 6);
}
uint8_t MAX30102_get_FifoOVFcntr(void)
{
    uint8_t ret;
    _max30102_read_config(MAX_OVF_CNTR, BM_OVF_COUNTER, &ret);
    return ret;
}

/*** CONFIGURATION ***/
void MAX30102_set_SMP_AVE_conf(max_smpAve_t n_smp)
{
    _max30102_write_config(MAX_FIFO_CONF, BM_SMP_AVE, n_smp);
}

max_smpAve_t MAX30102_get_SMP_AVE_conf(void)
{
    uint8_t aux_n_smp;
    _max30102_read_config(MAX_FIFO_CONF, BM_SMP_AVE, &aux_n_smp);
    return (max_smpAve_t)aux_n_smp;
}

void MAX30102_set_RollOverEnable(bool_t bit_flag)
{
    uint8_t bit_en = 0;
    if (bit_flag)
        bit_en = 1;

    _max30102_write_config(MAX_FIFO_CONF, BM_FIFO_ROLLOVER_EN, bit_en);
}

bool_t MAX30102_get_RollOverEnable(void)
{
    uint8_t bit_en = 0;
    _max30102_read_config(MAX_FIFO_CONF, BM_FIFO_ROLLOVER_EN, &bit_en);

    if (bit_en == 0x00)
        return false;
    return true;
}

void MAX30102_set_AFULL_conf(max_almostData_t value)
{
    _max30102_write_config(MAX_FIFO_CONF, BM_FIFO_A_FULL, value);
}

max_almostData_t MAX30102_get_AFULL_conf(void)
{
    uint8_t aux_value;
    _max30102_read_config(MAX_FIFO_CONF, BM_FIFO_A_FULL, &aux_value);
    return (max_almostData_t)aux_value;
}

void MAX30102_set_ModeCtrl_conf(max_modectrl_t mode)
{
    spo2_maxsensor->mode_leds = mode;
    _max30102_write_config(MAX_MODE_CONF, BM_MODE, mode);
}

max_modectrl_t MAX30102_get_ModeCtrl_conf(void)
{
    uint8_t aux_mode;
    _max30102_read_config(MAX_MODE_CONF, BM_MODE, &aux_mode);
    return (max_modectrl_t)aux_mode;
}

void MAX30102_set_SPO2adc_conf(max_SPO2adcrange_t adc_val)
{
    _max30102_write_config(MAX_SPO2_CONF, BM_SPO2_ADC_RGE, (uint8_t)adc_val);
}

max_SPO2adcrange_t MAX30102_get_SPO2adc_conf(void)
{
    uint8_t aux_adc_val;
    _max30102_read_config(MAX_SPO2_CONF, BM_SPO2_ADC_RGE, &aux_adc_val);
    return (max_SPO2rate_t)aux_adc_val;
}

void MAX30102_set_SPO2rate_conf(max_SPO2rate_t rate)
{
    _max30102_write_config(MAX_SPO2_CONF, BM_SPO2_SR, (uint8_t)rate);
}

max_SPO2rate_t MAX30102_get_SPO2rate_conf(void)
{
    uint8_t aux_rate;
    _max30102_read_config(MAX_SPO2_CONF, BM_SPO2_SR, &aux_rate);
    return (max_SPO2rate_t)aux_rate;
}

void MAX30102_set_LedPW_conf(max_PWcontrol_t pw_value)
{
    _max30102_write_config(MAX_SPO2_CONF, BM_LED_PW, (uint8_t)pw_value);
}

max_PWcontrol_t MAX30102_get_LedPW_conf(void)
{
    uint8_t aux_pw_value;
    _max30102_read_config(MAX_SPO2_CONF, BM_LED_PW, &aux_pw_value);
    return (max_SPO2rate_t)aux_pw_value;
}

void MAX30102_set_PulAmp_conf(max_led_t led, max_ledAmp_t amp_value)
{
    if (led == ledRED_pulseAmp)
        _max30102_write_config(MAX_LED_PAMP1, BM_LED1_PA, (uint8_t)amp_value);
    if (led == ledIR_pulseAmp)
        _max30102_write_config(MAX_LED_PAMP2, BM_LED2_PA, (uint8_t)amp_value);
}

max_ledAmp_t MAX30102_get_PulAmp_conf(max_led_t led)
{
    uint8_t aux_amp_value;
    if (led == ledRED_pulseAmp)
        _max30102_read_config(MAX_LED_PAMP1, BM_LED1_PA, &aux_amp_value);
    if (led == ledIR_pulseAmp)
        _max30102_read_config(MAX_LED_PAMP2, BM_LED2_PA, &aux_amp_value);
    return (max_SPO2rate_t)aux_amp_value;
}

void MAX30102_set_LedSlot_conf(max_led_t led, max_slotnumber_t n_slot)
{
    uint8_t aux_regaddr, aux_bitmask;
    switch (n_slot)
    {
    case slot1_MLcontrol:
        aux_regaddr = MAX_MLED_CTRL1;
        aux_bitmask = BM_SLOT1;
        break;
    case slot2_MLcontrol:

        aux_regaddr = MAX_MLED_CTRL1;
        aux_bitmask = BM_SLOT2;
        break;
    case slot3_MLcontrol:

        aux_regaddr = MAX_MLED_CTRL2;
        aux_bitmask = BM_SLOT3;
        break;
    case slot4_MLcontrol:

        aux_regaddr = MAX_MLED_CTRL2;
        aux_bitmask = BM_SLOT4;
        break;
    default:
        break;
        _max30102_write_config(aux_regaddr, aux_bitmask, (uint8_t)(led + 1));
    }
}

max_led_t MAX30102_get_LedSlot_conf(max_slotnumber_t n_slot)
{
    uint8_t aux_regaddr, aux_bitmask, aux_ledselect;
    switch (n_slot)
    {
    case slot1_MLcontrol:
        aux_regaddr = MAX_MLED_CTRL1;
        aux_bitmask = BM_SLOT1;
        break;
    case slot2_MLcontrol:

        aux_regaddr = MAX_MLED_CTRL1;
        aux_bitmask = BM_SLOT2;
        break;
    case slot3_MLcontrol:

        aux_regaddr = MAX_MLED_CTRL2;
        aux_bitmask = BM_SLOT3;
        break;
    case slot4_MLcontrol:

        aux_regaddr = MAX_MLED_CTRL2;
        aux_bitmask = BM_SLOT4;
        break;
    default:
        break;
    }
    _max30102_read_config(aux_regaddr, aux_bitmask, &aux_ledselect);
    return (max_led_t)(aux_ledselect - 1);
}

void MAX30102_set_Shutdown_conf(max_enableflag_t flag)
{
    _max30102_write_config(MAX_MODE_CONF, BM_SHDN, (uint8_t)flag);
}

max_enableflag_t MAX30102_get_Shutdown_conf(void)
{
    uint8_t aux_flag;
    _max30102_read_config(MAX_MODE_CONF, BM_SHDN, &aux_flag);
    return (max_enableflag_t)aux_flag;
}

void MAX30102_set_Reset_conf()
{
    _max30102_write_config(MAX_MODE_CONF, BM_RESET, 1);
}

bool_t _max30102_checkBitMask(uint8_t reg_val, uint8_t reg_bm)
{
    if ((reg_val & reg_bm) == reg_bm)
    {
        return true;
    }
    return false;
}

error_MAX_t MAX30102_check_IrqFlags(max30102_t *max_sensor)
{
    uint8_t irq_reg1, irq_reg2;
    irq_reg1 = MAX30102_get_IRQstatus1();
    irq_reg2 = MAX30102_get_IRQstatus2();

    if (_max30102_checkBitMask(irq_reg1, BM_A_FULL))
    {
        // A full irq
        private_flags->irqflag_A_FULL = true;
    }

    if (_max30102_checkBitMask(irq_reg1, BM_PPG_RDY))
    {
        // PPG ready
        private_flags->irqflag_PPG_RDY = true;
    }
    if (_max30102_checkBitMask(irq_reg1, BM_ALC_OVF))
    {
        // ALC overflow
        private_flags->irqflag_ALC_OVF = true;
    }
    if (_max30102_checkBitMask(irq_reg1, BM_PWR_RDY))
    {
        // ´power ready
        private_flags->irqflag_PWR_RDY = true;
    }
    if (_max30102_checkBitMask(irq_reg2, BM_DIE_TEMP_RDY))
    {
        // dietemp ready
        private_flags->irqflag_DIE_TEMP_RDY = true;
    }

    max_sensor->irq_flag = false;
    return max_OK;
}

error_MAX_t MAX30102_read_FIFODataSamples(max30102_t *max_sensor)
{
    // leer FIFO WR PTR
    uint8_t val_fifoRDptr = 0, val_fifoWRptr = 0;
    int8_t available_samples = 0;
    uint8_t aux_redsmps[3];
    uint8_t aux_irsmps[6];
    uint8_t aud_dualsmps[6];

    max30102_irq_flags_t *aux_flagsirq = (max30102_irq_flags_t *)max_sensor->private_vars;

    if (aux_flagsirq->irqflag_A_FULL == false)
    {
        return max_ERROR;
    }

    val_fifoWRptr = MAX30102_get_FifoWRptr();
    val_fifoRDptr = MAX30102_get_FifoRDptr();

    available_samples = (int8_t)(val_fifoWRptr - val_fifoRDptr);

    if (available_samples < 1)
        available_samples += 32;

    max_sensor->samples_cntr = available_samples;

    for (uint8_t var = 0; var < available_samples; ++var)
    {
        switch (max_sensor->mode_leds)
        {
        case mode_HeartRate:
            _max30102_multiread_register(MAX_FIFO_DATA, aux_redsmps, 3);
            max_sensor->red_samples[var] = ((uint32_t)(aud_dualsmps[0] << 16) | (uint32_t)(aud_dualsmps[1] << 8) | (uint32_t)(aud_dualsmps[2])) & 0x3ffff;
            break;

        case mode_SPO2:
            _max30102_multiread_register(MAX_FIFO_DATA, aux_irsmps, 6);
            max_sensor->ir_samples[var] = ((uint32_t)(aux_irsmps[0] << 16) | (uint32_t)(aux_irsmps[1] << 8) | (uint32_t)(aux_irsmps[2])) & 0x3ffff;
            max_sensor->red_samples[var] = ((uint32_t)(aux_irsmps[3] << 16) | (uint32_t)(aux_irsmps[4] << 8) | (uint32_t)(aux_irsmps[5])) & 0x3ffff;
            break;

        case mode_MultiLED:
            _max30102_multiread_register(MAX_FIFO_DATA, aud_dualsmps, 6);
            max_sensor->ir_samples[var] = ((uint32_t)(aud_dualsmps[0] << 16) | (uint32_t)(aud_dualsmps[1] << 8) | (uint32_t)(aud_dualsmps[2])) & 0x3ffff;
            max_sensor->red_samples[var] = ((uint32_t)(aud_dualsmps[3] << 16) | (uint32_t)(aud_dualsmps[4] << 8) | (uint32_t)(aud_dualsmps[5])) & 0x3ffff;
            break;

        default:
            break;
        }
    }
    return max_OK;
}

/*** DIE TEMPERATURE ***/
uint8_t MAX30102_get_IntegerTemp(void)
{
    uint8_t temp;
    _max30102_read_config(MAX_DTEMP_INTGR, BM_MASK_FULL, &temp);
    return temp;
}
uint8_t MAX30102_get_FractionTemp(void)
{
    uint8_t frac_temp;
    _max30102_read_config(MAX_DTEMP_FRACT, BM_TFRAC, &frac_temp);
    return frac_temp;
}

void MAX30102_set_TempEnable(max_enableflag_t flag)
{
    _max30102_write_config(MAX_DTEMP_CONF, BM_DIE_TEMP_RDY_EN, (uint8_t)flag);
}
void MAX30102_get_TempEnable(max_enableflag_t *flag)
{
    uint8_t aux_flag;
    _max30102_read_config(MAX_DTEMP_CONF, BM_DIE_TEMP_RDY_EN, &aux_flag);
    *flag = (max_enableflag_t)aux_flag;
}

float MAX30102_get_TemperatureFloat(void)
{
    int8_t temp_int = 0;
    float temp_frac = 0.0, temp_tot = 0.0;
    temp_int = (int8_t)MAX30102_get_IntegerTemp();
    temp_frac = (float)MAX30102_get_FractionTemp() * TEMP_FRACT_STEP;
    temp_tot = (float)temp_int + temp_frac;
    return temp_tot;
}