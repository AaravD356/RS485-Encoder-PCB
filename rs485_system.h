#ifndef RS485_SYSTEM_H
#define RS485_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Shared RS485 encoder subsystem
 * ------------------------------
 * This system owns a round-robin poller for multiple AMT21x / AMT212 encoders
 * on a shared RS485 bus. It also services the existing rotary encoder CAN
 * request/response page from the project DBC so Jetson-side software can ask
 * for a specific encoder ID on the daisy chain and receive cached position and
 * velocity back over CAN.
 *
 * Add this controller in main.c with:
 *   RR_AddController(rs485_system_controller);
 */

/* =========================
 * RS485 / encoder defaults
 * =========================
 */
#define PROJECT_RS485_ENCODER_MAX_COUNT       16U
#define PROJECT_RS485_DEFAULT_ENCODER_COUNT   2U
#define PROJECT_RS485_DEFAULT_POLL_MS         5U
#define PROJECT_RS485_UART_TIMEOUT_MS         4U
#define PROJECT_RS485_ENCODER_RES_BITS        14U

#define PROJECT_RS485_USE_DE_PIN              1U
#define PROJECT_RS485_DE_GPIO_PORT            GPIOA
#define PROJECT_RS485_DE_GPIO_PIN             GPIO_PIN_8

#define PROJECT_RS485_ENCODER_0_NODE_ID       0xF8U
#define PROJECT_RS485_ENCODER_1_NODE_ID       0xFCU
#define PROJECT_RS485_ENCODER_2_NODE_ID       0x5CU
#define PROJECT_RS485_ENCODER_3_NODE_ID       0x60U

/* =========================
 * CAN bridge defaults
 * =========================
 * These names point at the existing DBC rotary encoder request/response page.
 *
 * Protocol mapping:
 *   request  data[0] = 0x20 + port_id  (DBC mux/cmd page)
 *   request  data[1] = rotary_encoder_id_0
 *   response data[1] = rotary_encoder_id_0
 *   response data[2:3] = rotary_encoder_position_0  (0.1 deg, little-endian)
 *   response data[4:5] = rotary_encoder_velocity_0  (0.01 deg/s, little-endian)
 *
 * Update them if this branch should answer with a different PCB family, such
 * as END_EFFECTOR_PCB_C / END_EFFECTOR_PCB_R instead of BASE_ARM_PCB_C / _R.
 */
#define PROJECT_RS485_CAN_REQUEST_ID_PARAM       "BASE_ARM_PCB_C.rotary_encoder_id_0"
#define PROJECT_RS485_CAN_RESPONSE_ID_PARAM      "BASE_ARM_PCB_R.rotary_encoder_id_0"
#define PROJECT_RS485_CAN_RESPONSE_POSITION_PARAM "BASE_ARM_PCB_R.rotary_encoder_position_0"
#define PROJECT_RS485_CAN_RESPONSE_VELOCITY_PARAM "BASE_ARM_PCB_R.rotary_encoder_velocity_0"

typedef struct
{
  uint8_t node_id;
  uint8_t resolution_bits;
  uint16_t poll_period_ms;
} rs485_encoder_config_t;

typedef struct
{
  UART_HandleTypeDef* huart;
  GPIO_TypeDef* de_gpio_port;
  uint16_t de_gpio_pin;
  bool use_manual_direction;
  uint32_t uart_timeout_ms;
} rs485_transport_config_t;

typedef struct
{
  bool online;
  uint8_t node_id;
  uint8_t resolution_bits;
  uint16_t position_raw;
  int32_t velocity_counts_per_s;
  uint32_t last_sample_tick_ms;
  uint32_t successful_reads;
  uint32_t failed_reads;
} rs485_encoder_state_t;

bool rs485_system_init(void);
void rs485_system_controller(void);

bool rs485_system_configure_transport(const rs485_transport_config_t* config);
bool rs485_system_get_transport_config(rs485_transport_config_t* out_config);

void rs485_system_clear_encoders(void);
bool rs485_system_add_encoder(const rs485_encoder_config_t* config);
uint8_t rs485_system_get_encoder_count(void);

bool rs485_system_get_encoder_state(uint8_t index, rs485_encoder_state_t* out_state);
bool rs485_system_find_encoder_by_node_id(uint8_t node_id, uint8_t* out_index);

bool rs485_system_zero_encoder(uint8_t index);
bool rs485_system_reset_encoder(uint8_t index);

#ifdef __cplusplus
}
#endif

#endif /* RS485_SYSTEM_H */
