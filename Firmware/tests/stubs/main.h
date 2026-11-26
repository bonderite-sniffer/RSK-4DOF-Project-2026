/* PC test stub for CubeMX main.h: just enough HAL for the App/ code to compile
 * and run against sim.c (emulated MCP2515 + RobStride motors). */
#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { int port; } GPIO_TypeDef;
typedef struct { int id; } SPI_HandleTypeDef;
typedef struct { int id; } TIM_HandleTypeDef;

extern GPIO_TypeDef GPIOA_s, GPIOC_s, GPIOD_s;
#define GPIOA (&GPIOA_s)
#define GPIOC (&GPIOC_s)
#define GPIOD (&GPIOD_s)

#define CAN1_CS_Pin 0x0010
#define CAN1_CS_GPIO_Port GPIOA
#define CAN1_INT_Pin 0x0010
#define CAN1_INT_GPIO_Port GPIOC
#define CAN2_CS_Pin 0x0100
#define CAN2_CS_GPIO_Port GPIOD
#define CAN2_INT_Pin 0x0200
#define CAN2_INT_GPIO_Port GPIOD

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState s);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h, uint8_t *tx, uint8_t *rx, uint16_t n, uint32_t timeout);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *h);

/* Cortex-M bits used by can_bus.c. Every DWT access advances simulated time by 1 us. */
typedef struct { volatile uint32_t CTRL, CYCCNT, LAR; } DWT_Type;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
DWT_Type *sim_dwt(void);
extern CoreDebug_Type sim_coredebug;
#define DWT (sim_dwt())
#define CoreDebug (&sim_coredebug)
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define __DMB() do { } while (0)
extern uint32_t SystemCoreClock;

#endif
