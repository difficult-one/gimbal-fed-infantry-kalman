#ifndef TEST_MPU_SPI_H
#define TEST_MPU_SPI_H

#include <stdint.h>

typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
enum { HAL_SPI_STATE_READY = 1, HAL_SPI_STATE_BUSY_TX_RX = 5 };
typedef struct { volatile uint32_t CR1, DR; } SPI_TypeDef;
typedef struct { SPI_TypeDef *Instance; uint32_t State; } SPI_HandleTypeDef;
extern SPI_HandleTypeDef hspi5;

enum { GPIO_PIN_RESET, GPIO_PIN_SET };
#define GPIOF ((void *)0)
#define GPIO_PIN_6 (1u << 6)
#define SPI_FLAG_RXNE (1u << 0)
#define SPI_FLAG_TXE (1u << 1)
#define SPI_FLAG_MODF (1u << 5)
#define SPI_FLAG_OVR (1u << 6)
#define SPI_FLAG_BSY (1u << 7)
#define SPI_FLAG_FRE (1u << 8)
#define SPI_CR1_SPE (1u << 6)
void HAL_GPIO_WritePin(void *port, uint16_t pin, uint32_t level);
uint8_t HostSpiFlag(uint32_t flag);
void HostSpiClearError(void);
void HostSpiReset(void);
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *handle);
#define __HAL_RCC_SPI5_FORCE_RESET() HostSpiReset()
#define __HAL_RCC_SPI5_RELEASE_RESET() ((void)0)
#define __HAL_SPI_GET_FLAG(handle, flag) HostSpiFlag(flag)
#define __HAL_SPI_ENABLE(handle) ((handle)->Instance->CR1 |= SPI_CR1_SPE)
#define __HAL_SPI_DISABLE(handle) ((handle)->Instance->CR1 &= ~SPI_CR1_SPE)
#define __HAL_SPI_CLEAR_OVRFLAG(handle) HostSpiClearError()
#define __HAL_SPI_CLEAR_MODFFLAG(handle) HostSpiClearError()
#define __HAL_SPI_CLEAR_FREFLAG(handle) HostSpiClearError()

#endif
