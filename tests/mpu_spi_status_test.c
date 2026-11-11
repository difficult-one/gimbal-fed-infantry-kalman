#include "mpu6500_spi.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static SPI_TypeDef spi;
SPI_HandleTypeDef hspi5 = { .Instance = &spi, .State = HAL_SPI_STATE_READY };

static uint8_t reg[128];
static unsigned received, selected, released, fault_at, now_us;
static uint8_t chip_selected, address, reading, corrupt_read_address;
static HAL_StatusTypeDef fault;
static uint8_t hardware_busy, busy_sticks_after_timeout;
static unsigned resets;

static uint8_t TransferByte(uint8_t tx)
{
    assert(chip_selected);
    if (received++ == 0u)
    {
        address = tx & 0x7Fu;
        reading = (tx & 0x80u) != 0u;
        return 0;
    }
    assert(address < sizeof(reg));
    if (!reading)
        reg[address] = tx;
    uint8_t result = reg[address];
    if (reading && address == corrupt_read_address)
        result ^= 1u;
    if (reading && address == MPU6500_INT_STATUS)
        reg[address] = 0u;
    ++address;
    return result;
}

void HAL_GPIO_WritePin(void *port, uint16_t pin, uint32_t level)
{
    (void)port;
    assert(pin == GPIO_PIN_6);
    if (level == GPIO_PIN_RESET)
    {
        assert(!chip_selected);
        chip_selected = 1u;
        received = 0;
        ++selected;
    }
    else if (chip_selected)
    {
        chip_selected = 0;
        ++released;
    }
}

uint8_t HostSpiFlag(uint32_t flag)
{
    if (flag == SPI_FLAG_BSY)
        return hardware_busy;
    if (flag == SPI_FLAG_MODF)
        return fault == HAL_ERROR && received == fault_at;
    if (flag == SPI_FLAG_TXE)
        return 1u;
    if (flag != SPI_FLAG_RXNE || !chip_selected)
        return 0u;
    if (fault == HAL_TIMEOUT && received == fault_at) {
        hardware_busy = busy_sticks_after_timeout;
        return 0u;
    }
    spi.DR = TransferByte((uint8_t)spi.DR);
    return 1u;
}

void HostSpiClearError(void)
{
    fault = HAL_OK;
}

void HostSpiReset(void)
{
    assert(!chip_selected && hspi5.State == HAL_SPI_STATE_READY);
    hardware_busy = 0;
    fault = HAL_OK;
    ++resets;
}

HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *handle)
{
    assert(handle == &hspi5);
    return HAL_OK;
}

uint64_t DWT_GetTimeline_us(void)
{
    return now_us++;
}

void DWT_Delay_ms(float delay)
{
    now_us += (unsigned)(delay * 1000);
}

static void TimeoutLeavesCallerDataAndChipSelectIntact(unsigned byte)
{
    uint8_t data[3] = { 0xAA, 0xBB, 0xCC };
    const uint8_t unchanged[3] = { 0xAA, 0xBB, 0xCC };
    unsigned previous_releases = released;
    fault = HAL_TIMEOUT;
    fault_at = byte;
    assert(mpu_read_bytes(MPU6500_ACCEL_XOUT_H, data, sizeof(data)) == HAL_TIMEOUT);
    assert(!chip_selected && released == previous_releases + 1);
    assert(received == byte);
    assert(memcmp(data, unchanged, sizeof(data)) == 0);
}

static void ResetStuckHardwareAfterTimeout(void)
{
    unsigned previous_resets = resets;
    busy_sticks_after_timeout = 1;
    TimeoutLeavesCallerDataAndChipSelectIntact(2u);
    assert(resets == previous_resets + 1u && !hardware_busy);
    busy_sticks_after_timeout = 0;

    hardware_busy = 1;
    uint8_t data[2];
    unsigned previous_selections = selected;
    assert(mpu_read_bytes(MPU6500_ACCEL_XOUT_H, data, sizeof(data)) == HAL_TIMEOUT);
    assert(resets == previous_resets + 2u && selected == previous_selections);
    assert(mpu_read_bytes(MPU6500_ACCEL_XOUT_H, data, sizeof(data)) == HAL_OK);
}

static void RawStatusControlsFreshness(void)
{
    const uint8_t packet[] = {
        0x01, 0x80, 0x00, 0x12, 0x34, 0xFF, 0xFF, 0xFE, 0xB2,
        0x7F, 0xFF, 0xFC, 0x00, 0x00, 0x00,
    };
    const int16_t expected[7] = {-32768, 4660, -1, -334, 32767, -1024, 0};
    int16_t raw[7] = {0};
    uint8_t fresh;

    memcpy(&reg[MPU6500_INT_STATUS], packet, sizeof(packet));
    unsigned previous_selections = selected;
    assert(mpu_read_sample(raw, &fresh) == HAL_OK && fresh);
    assert(selected == previous_selections + 1u && received == 16u);
    assert(memcmp(raw, expected, sizeof(raw)) == 0);
    assert(mpu_read_sample(raw, &fresh) == HAL_OK && !fresh);
    assert(memcmp(raw, expected, sizeof(raw)) == 0);

    reg[MPU6500_INT_STATUS] = 1u;
    assert(mpu_read_sample(raw, &fresh) == HAL_OK && fresh);
    assert(memcmp(raw, expected, sizeof(raw)) == 0);
    reg[MPU6500_INT_STATUS] = 0xFFu;
    assert(mpu_read_sample(raw, &fresh) == HAL_ERROR && !fresh);
    assert(memcmp(raw, expected, sizeof(raw)) == 0);
}

static void StartupRequiresIdentityAndReadback(void)
{
    reg[0x75] = 0x00u;
    assert(mpu6500_device_init() == HAL_ERROR);

    reg[0x75] = 0x70u;
    corrupt_read_address = MPU6500_ACCEL_CONFIG_2;
    assert(mpu6500_device_init() == HAL_ERROR);
    corrupt_read_address = 0u;
    assert(mpu6500_device_init() == HAL_OK);
    assert(reg[MPU6500_GYRO_CONFIG] == 0x10u);
    assert(reg[MPU6500_CONFIG] == 0x04u && reg[MPU6500_ACCEL_CONFIG_2] == 0x04u);
    assert(reg[MPU6500_SMPLRT_DIV] == 0u);
    assert(reg[MPU6500_INT_PIN_CFG] == 0u && reg[MPU6500_INT_ENABLE] == 1u);

    unsigned start_us = now_us;
    fault_at = 0u;
    fault = HAL_TIMEOUT;
    assert(mpu6500_device_init() == HAL_TIMEOUT);
    assert(!chip_selected && now_us - start_us < 120000u);
}

int main(void)
{
    TimeoutLeavesCallerDataAndChipSelectIntact(0u);
    TimeoutLeavesCallerDataAndChipSelectIntact(2u);

    fault = HAL_OK;
    reg[MPU6500_ACCEL_XOUT_H] = 0x12u;
    reg[MPU6500_ACCEL_XOUT_L] = 0x34u;
    uint8_t data[2];
    assert(mpu_read_bytes(MPU6500_ACCEL_XOUT_H, data, sizeof(data)) == HAL_OK);
    assert(data[0] == 0x12u && data[1] == 0x34u);
    assert(!chip_selected && selected == released);

    hspi5.State = HAL_SPI_STATE_BUSY_TX_RX;
    unsigned previous_selections = selected;
    assert(mpu_read_bytes(MPU6500_ACCEL_XOUT_H, data, sizeof(data)) == HAL_BUSY);
    assert(selected == previous_selections && !chip_selected);
    hspi5.State = HAL_SPI_STATE_READY;

    fault = HAL_ERROR;
    fault_at = 2u;
    assert(mpu_read_bytes(MPU6500_ACCEL_XOUT_H, data, sizeof(data)) == HAL_ERROR);
    assert(!chip_selected && data[0] == 0x12u && data[1] == 0x34u);
    ResetStuckHardwareAfterTimeout();
    RawStatusControlsFreshness();
    StartupRequiresIdentityAndReadback();

    puts("PASS: bounded MPU6500 SPI, complete data, freshness and startup readback");
    return 0;
}
