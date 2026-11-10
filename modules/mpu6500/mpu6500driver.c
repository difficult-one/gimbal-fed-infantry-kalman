#include "mpu6500driver.h"



#include <string.h>

#define MPU_SPI_TIMEOUT_US 2000u
#define MPU_SPI_MAX_DATA 15u

/* SPI5 is dedicated to the MPU6500. RobotInit masks interrupts, so HAL's
 * tick-based blocking transfer cannot time out there. Use one DWT budget
 * for the address, data, and bus idle phases. */
static HAL_StatusTypeDef MpuWaitSpi(uint32_t flag, uint8_t expected, uint32_t start_us)
{
    do
    {
        if (__HAL_SPI_GET_FLAG(&MPU_HSPI, SPI_FLAG_MODF) ||
            __HAL_SPI_GET_FLAG(&MPU_HSPI, SPI_FLAG_OVR) ||
            __HAL_SPI_GET_FLAG(&MPU_HSPI, SPI_FLAG_FRE))
            return HAL_ERROR;
        if (__HAL_SPI_GET_FLAG(&MPU_HSPI, flag) == expected)
            return HAL_OK;
    } while ((uint32_t)((uint32_t)DWT_GetTimeline_us() - start_us) < MPU_SPI_TIMEOUT_US);
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef MpuExchangeByte(uint8_t tx, uint8_t *rx, uint32_t start_us)
{
    HAL_StatusTypeDef status = MpuWaitSpi(SPI_FLAG_TXE, 1u, start_us);
    if (status != HAL_OK)
        return status;

    volatile uint8_t *data_register = (volatile uint8_t *)&MPU_HSPI.Instance->DR;
    *data_register = tx;
    status = MpuWaitSpi(SPI_FLAG_RXNE, 1u, start_us);
    if (status == HAL_OK)
        *rx = *data_register;
    return status;
}

/* Call with CS high and only when no HAL transfer owns the peripheral. */
static void MpuRecoverSpi(void)
{
    __HAL_SPI_DISABLE(&MPU_HSPI);
    if (__HAL_SPI_GET_FLAG(&MPU_HSPI, SPI_FLAG_BSY))
    {
        __HAL_RCC_SPI5_FORCE_RESET();
        __HAL_RCC_SPI5_RELEASE_RESET();
        (void)HAL_SPI_Init(&MPU_HSPI);
        return;
    }
    __HAL_SPI_CLEAR_OVRFLAG(&MPU_HSPI);
    __HAL_SPI_CLEAR_MODFFLAG(&MPU_HSPI);
    __HAL_SPI_CLEAR_FREFLAG(&MPU_HSPI);
}

static HAL_StatusTypeDef MpuTransfer(uint8_t command, uint8_t data[], uint8_t length)
{
    if (MPU_HSPI.State != HAL_SPI_STATE_READY)
        return HAL_BUSY;
    if (__HAL_SPI_GET_FLAG(&MPU_HSPI, SPI_FLAG_BSY))
    {
        /* An idle HAL handle with a busy shifter is left over from a fault. */
        MPU_NSS_HIGH;
        MpuRecoverSpi();
        return HAL_TIMEOUT;
    }

    const uint8_t read = (command & 0x80u) != 0u;
    const uint32_t start_us = (uint32_t)DWT_GetTimeline_us();
    uint8_t discard;
    __HAL_SPI_ENABLE(&MPU_HSPI);
    MPU_NSS_LOW;
    HAL_StatusTypeDef status = MpuExchangeByte(command, &discard, start_us);
    for (uint8_t i = 0u; i < length && status == HAL_OK; ++i)
        status = MpuExchangeByte(read ? 0xFFu : data[i], &data[i], start_us);
    if (status == HAL_OK)
        status = MpuWaitSpi(SPI_FLAG_BSY, 0u, start_us);
    MPU_NSS_HIGH;

    if (status != HAL_OK)
        MpuRecoverSpi();
    return status;
}

HAL_StatusTypeDef mpu_write_byte(uint8_t reg, uint8_t data)
{
    return MpuTransfer(reg & 0x7Fu, &data, 1u);
}

HAL_StatusTypeDef mpu_read_bytes(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t received[MPU_SPI_MAX_DATA];
    if (!data || length == 0u || length > sizeof(received))
        return HAL_ERROR;

    HAL_StatusTypeDef status = MpuTransfer(reg | 0x80u, received, length);
    if (status == HAL_OK)
        memcpy(data, received, length);
    return status;
}

static uint8_t mpu_read_byte(uint8_t reg)
{
    uint8_t value = 0xFFu;
    (void)mpu_read_bytes(reg, &value, 1u);
    return value;
}

HAL_StatusTypeDef mpu_read_sample(int16_t raw[7], uint8_t *fresh)
{
    uint8_t data[MPU_SPI_MAX_DATA];
    *fresh = 0u;
    /* Read-to-clear DATA_RDY and the corresponding register snapshot with
     * the same CS assertion. No edge timestamp is available in polling mode. */
    HAL_StatusTypeDef status = mpu_read_bytes(MPU6500_INT_STATUS, data, sizeof(data));
    if (status != HAL_OK)
        return status;
    if (data[0] == 0xFFu) /* A high MISO line is not a valid INT_STATUS. */
        return HAL_ERROR;
    if (!(data[0] & MPU6500_RAW_DATA_RDY))
        return HAL_OK;

    for (uint8_t i = 0u; i < 7u; ++i)
        raw[i] = (int16_t)((uint16_t)data[1u + 2u*i] << 8u | data[2u + 2u*i]);
    *fresh = 1u;
    return HAL_OK;
}


/**
	* @brief  write IST8310 register through MPU6500's I2C master
  * @param  addr: the address to be written of IST8310's register
  *         data: data to be written
	* @retval   
  * @usage  call in ist8310_init() function
	*/
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* turn off slave 1 at first */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    MPU_DELAY(2);
    /* turn on slave 1 with one byte transmitting */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    MPU_DELAY(10);
}

/**
	* @brief  write IST8310 register through MPU6500's I2C Master
	* @param  addr: the address to be read of IST8310's register
	* @retval 
  * @usage  call in ist8310_init() function
	*/
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
    return retval;
}

/**
	* @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
* @param    device_address: slave device address, Address[6:0]
	* @retval   void
	* @note     
	*/
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
	   * configure the device address of the IST8310 
     * use slave1, auto transmit single measure mode 
	   */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    /* use slave0,auto read data */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    /* every eight mpu6500 internal samples one i2c master read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
    /* enable slave 0 and 1 access delay */
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
    /* enable slave 1 auto transmit */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    MPU_DELAY(6); 
    /* enable slave 0 with data_num bytes reading */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}
/**
	* @brief  Initializes the IST8310 device
	* @param  
	* @retval 
  * @usage  call in mpu6500_init() function
	*/
static uint8_t ist8310_init()
{
	  /* enable iic master mode */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    MPU_DELAY(10);
	  /* enable iic 400khz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d); 
    MPU_DELAY(10);

    /* turn on slave 1 for ist write and slave 4 to ist read */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    MPU_DELAY(10);

    /* IST8310_R_CONFB 0x01 = device rst */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

		/* soft reset */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); 
    MPU_DELAY(10);

		/* config as ready mode to access register */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); 
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    MPU_DELAY(10);

		/* normal state, no int */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    MPU_DELAY(10);
		
    /* config low noise mode, x,y,z axis 16 time 1 avg */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    MPU_DELAY(10);

    /* Set/Reset pulse duration setup,normal mode */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    MPU_DELAY(10);

    /* turn off slave1 & slave 4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);

    /* configure and turn on slave 0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    MPU_DELAY(100);
    return 0;
}
HAL_StatusTypeDef mpu6500_device_init(void)
{
    const uint8_t config[][2] = {
        {MPU6500_PWR_MGMT_1, 0x03u},
        {MPU6500_PWR_MGMT_2, 0x00u},
        {MPU6500_SMPLRT_DIV, 0x00u},
        {MPU6500_CONFIG, 0x04u},         /* Gyro 20 Hz DLPF, 9.9 ms nominal delay. */
        {MPU6500_GYRO_CONFIG, 0x10u},    /* +/-1000 deg/s, DLPF enabled. */
        {MPU6500_ACCEL_CONFIG, 0x10u},   /* +/-8 g. */
        {MPU6500_ACCEL_CONFIG_2, 0x04u}, /* Accel 20 Hz DLPF, 19.8 ms nominal delay. */
        {MPU6500_INT_PIN_CFG, 0x00u},    /* Only reading INT_STATUS clears its bits. */
        {MPU6500_INT_ENABLE, MPU6500_RAW_DATA_RDY},
    };
    MPU_NSS_HIGH;
    MPU_DELAY(100);
    HAL_StatusTypeDef status = mpu_write_byte(MPU6500_PWR_MGMT_1, 0x80u);
    if (status != HAL_OK)
        return status;
    MPU_DELAY(100);
    status = mpu_write_byte(MPU6500_SIGNAL_PATH_RESET, 0x07u);
    if (status != HAL_OK)
        return status;
    MPU_DELAY(100);

    uint8_t value;
    status = mpu_read_bytes(MPU6500_WHO_AM_I, &value, 1u);
    if (status != HAL_OK)
        return status;
    if (value != MPU6500_ID)
        return HAL_ERROR;

    for (unsigned i = 0u; i < sizeof(config)/sizeof(config[0]); ++i)
    {
        status = mpu_write_byte(config[i][0], config[i][1]);
        if (status != HAL_OK)
            return status;
        MPU_DELAY(1);
        status = mpu_read_bytes(config[i][0], &value, 1u);
        if (status != HAL_OK)
            return status;
        if (value != config[i][1])
            return HAL_ERROR;
    }

    /* Retain the optional magnetometer setup. The production EKF uses
     * accel/gyro only, so an absent magnetometer need not invalidate INS. */
    (void)ist8310_init();
    return HAL_OK;
}
