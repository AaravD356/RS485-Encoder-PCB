#include "rs485_system.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stm32l4xx_hal.h"
#include "usart.h"

#include "can_params.h"
#include "can_system.h"

/* =========================
 *  Protocol definitions
 * =========================
 *
 * AMT21x / AMT212 RS485 protocol:
 *   - Read position:  <node_id>
 *   - Read turns:     <node_id + 0x01>   (multi-turn devices only)
 *   - Extended cmd:   <node_id + 0x02> followed by a subcommand
 *
 * Response format:
 *   - 2 bytes, low byte first
 *   - upper 2 bits are parity/check bits
 *   - lower 14 bits are payload
 *
 * CAN request / response protocol:
 *   request  data[0] = 0x20 + port_id  (DBC mux/cmd page)
 *   request  data[1] = rotary_encoder_id_0
 *   response data[1] = rotary_encoder_id_0
 *   response data[2:3] = rotary_encoder_position_0  (0.1 deg)
 *   response data[4:5] = rotary_encoder_velocity_0  (0.01 deg/s)
 */

#define RS485_AMT21_EXTENDED_OFFSET              0x02U
#define RS485_AMT21_ZERO_SUBCOMMAND              0x5EU
#define RS485_AMT21_RESET_SUBCOMMAND             0x75U
#define RS485_AMT21_RESPONSE_LEN                 2U
#define RS485_AMT21_PAYLOAD_MASK                 0x3FFFU

/* =========================
 *  Configuration structures
 * ========================= */

typedef struct
{
  const char* request_id_param;
  const char* response_id_param;
  const char* response_position_param;
  const char* response_velocity_param;
} rs485_can_config_t;

typedef struct
{
  rs485_encoder_config_t cfg;
  rs485_encoder_state_t state;
  uint16_t last_position_raw;
  uint32_t last_poll_tick_ms;
  bool has_sample;
} rs485_encoder_slot_t;

/* =========================
 *  CAN integration
 * ========================= */

static const rs485_can_config_t s_can_cfg =
{
  .request_id_param = PROJECT_RS485_CAN_REQUEST_ID_PARAM,
  .response_id_param = PROJECT_RS485_CAN_RESPONSE_ID_PARAM,
  .response_position_param = PROJECT_RS485_CAN_RESPONSE_POSITION_PARAM,
  .response_velocity_param = PROJECT_RS485_CAN_RESPONSE_VELOCITY_PARAM
};

/* =========================
 *  State
 * ========================= */

static bool s_initialized = false;
static rs485_encoder_slot_t s_encoders[PROJECT_RS485_ENCODER_MAX_COUNT];
static uint8_t s_encoder_count = 0U;
static uint8_t s_next_encoder_index = 0U;

static rs485_transport_config_t s_transport =
{
  .huart = NULL,
  .de_gpio_port = PROJECT_RS485_DE_GPIO_PORT,
  .de_gpio_pin = PROJECT_RS485_DE_GPIO_PIN,
  .use_manual_direction = (PROJECT_RS485_USE_DE_PIN != 0U),
  .uart_timeout_ms = PROJECT_RS485_UART_TIMEOUT_MS
};

/* =========================
 *  Generic helpers
 * ========================= */

static int32_t clamp_i32(int32_t value, int32_t min_value, int32_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

/* =========================
 *  CAN scaling helpers
 * ========================= */

static int32_t raw_counts_to_position_tenths_deg(uint16_t position_raw, uint8_t resolution_bits)
{
  uint32_t counts_per_rev = (1UL << resolution_bits);
  int32_t scaled = (int32_t)(((int64_t)position_raw * 3600LL) / (int64_t)counts_per_rev);
  return clamp_i32(scaled, -32768, 32767);
}

static int32_t counts_per_s_to_velocity_hundredths_deg_s(int32_t velocity_counts_per_s,
                                                          uint8_t resolution_bits)
{
  uint32_t counts_per_rev = (1UL << resolution_bits);
  int32_t scaled = (int32_t)(((int64_t)velocity_counts_per_s * 36000LL) / (int64_t)counts_per_rev);
  return clamp_i32(scaled, -32768, 32767);
}

/* =========================
 *  Transport helpers
 * ========================= */

static UART_HandleTypeDef* rs485_uart(void)
{
  return s_transport.huart;
}

static void rs485_set_direction_tx(void)
{
  if (s_transport.use_manual_direction && (s_transport.de_gpio_port != NULL))
  {
    HAL_GPIO_WritePin(s_transport.de_gpio_port, s_transport.de_gpio_pin, GPIO_PIN_SET);
  }

  for (volatile uint32_t i = 0U; i < 10U; i++)
  {
    __NOP();
  }
}

static void rs485_set_direction_rx(void)
{
  if (s_transport.use_manual_direction && (s_transport.de_gpio_port != NULL))
  {
    HAL_GPIO_WritePin(s_transport.de_gpio_port, s_transport.de_gpio_pin, GPIO_PIN_RESET);
  }

  for (volatile uint32_t i = 0U; i < 10U; i++)
  {
    __NOP();
  }
}

static void rs485_flush_rx(UART_HandleTypeDef* huart)
{
  __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
}

static void rs485_init_direction_gpio(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  if (!s_transport.use_manual_direction || (s_transport.de_gpio_port == NULL))
  {
    return;
  }

  if (s_transport.de_gpio_port == GPIOA)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  }
  else if (s_transport.de_gpio_port == GPIOB)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  }
  else if (s_transport.de_gpio_port == GPIOC)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
  else if (s_transport.de_gpio_port == GPIOD)
  {
    __HAL_RCC_GPIOD_CLK_ENABLE();
  }
  else if (s_transport.de_gpio_port == GPIOE)
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();
  }
  else if (s_transport.de_gpio_port == GPIOH)
  {
    __HAL_RCC_GPIOH_CLK_ENABLE();
  }

  HAL_GPIO_WritePin(s_transport.de_gpio_port, s_transport.de_gpio_pin, GPIO_PIN_RESET);

  gpio_init.Pin = s_transport.de_gpio_pin;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(s_transport.de_gpio_port, &gpio_init);
}

static bool rs485_uart_transceive(const uint8_t* tx_data,
                                  uint16_t tx_len,
                                  uint8_t* rx_data,
                                  uint16_t rx_len)
{
  UART_HandleTypeDef* huart = rs485_uart();
  uint32_t timeout_ms = s_transport.uart_timeout_ms;

  if (huart == NULL)
  {
    return false;
  }

  rs485_set_direction_tx();
  rs485_flush_rx(huart);

  if (HAL_UART_Transmit(huart, (uint8_t*)tx_data, tx_len, timeout_ms) != HAL_OK)
  {
    rs485_set_direction_rx();
    return false;
  }

  uint32_t tickstart = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
  {
    if ((HAL_GetTick() - tickstart) > timeout_ms)
    {
      rs485_set_direction_rx();
      return false;
    }
  }

  rs485_set_direction_rx();

  if (rx_len == 0U)
  {
    return true;
  }

  if (HAL_UART_Receive(huart, rx_data, rx_len, timeout_ms) != HAL_OK)
  {
    return false;
  }

  return true;
}

/* =========================
 *  AMT21x / AMT212 helpers
 * ========================= */

static bool amt21_response_is_valid(uint16_t raw_response)
{
  uint8_t high = (uint8_t)(raw_response >> 8);
  uint8_t low = (uint8_t)(raw_response & 0xFFU);

  uint8_t k1 = (uint8_t)((raw_response >> 15) & 0x01U);
  uint8_t k0 = (uint8_t)((raw_response >> 14) & 0x01U);

  uint8_t expected_k1 = (uint8_t)(!(((high >> 5) & 0x01U) ^
                                    ((high >> 3) & 0x01U) ^
                                    ((high >> 1) & 0x01U) ^
                                    ((low >> 7) & 0x01U) ^
                                    ((low >> 5) & 0x01U) ^
                                    ((low >> 3) & 0x01U) ^
                                    ((low >> 1) & 0x01U)));

  uint8_t expected_k0 = (uint8_t)(!(((high >> 4) & 0x01U) ^
                                    ((high >> 2) & 0x01U) ^
                                    ((high >> 0) & 0x01U) ^
                                    ((low >> 6) & 0x01U) ^
                                    ((low >> 4) & 0x01U) ^
                                    ((low >> 2) & 0x01U) ^
                                    ((low >> 0) & 0x01U)));

  return (k1 == expected_k1) && (k0 == expected_k0);
}

static uint16_t amt21_decode_position(uint16_t raw_response, uint8_t resolution_bits)
{
  uint16_t payload = (uint16_t)(raw_response & RS485_AMT21_PAYLOAD_MASK);

  if (resolution_bits >= 14U)
  {
    return payload;
  }

  return (uint16_t)(payload >> (14U - resolution_bits));
}

static int32_t unwrap_position_delta(uint16_t current,
                                     uint16_t previous,
                                     uint8_t resolution_bits)
{
  uint32_t modulo = (1UL << resolution_bits);
  int32_t delta = (int32_t)current - (int32_t)previous;
  int32_t half = (int32_t)(modulo / 2UL);

  if (delta > half)
  {
    delta -= (int32_t)modulo;
  }
  else if (delta < -half)
  {
    delta += (int32_t)modulo;
  }

  return delta;
}

static bool amt21_read_position(uint8_t node_id, uint8_t resolution_bits, uint16_t* out_position_raw)
{
  uint8_t command = node_id;
  uint8_t response[RS485_AMT21_RESPONSE_LEN] = {0U, 0U};

  if (out_position_raw == NULL)
  {
    return false;
  }

  if (!rs485_uart_transceive(&command, 1U, response, RS485_AMT21_RESPONSE_LEN))
  {
    return false;
  }

  uint16_t raw_response = (uint16_t)(((uint16_t)response[1] << 8) | response[0]);
  if (!amt21_response_is_valid(raw_response))
  {
    return false;
  }

  *out_position_raw = amt21_decode_position(raw_response, resolution_bits);
  return true;
}

static bool amt21_send_extended(uint8_t node_id, uint8_t subcommand)
{
  uint8_t command[2];

  command[0] = (uint8_t)(node_id + RS485_AMT21_EXTENDED_OFFSET);
  command[1] = subcommand;

  return rs485_uart_transceive(command, 2U, NULL, 0U);
}

/* =========================
 *  Polling / CAN helpers
 * ========================= */

static bool slot_poll_is_due(const rs485_encoder_slot_t* slot, uint32_t now)
{
  return (uint32_t)(now - slot->last_poll_tick_ms) >= slot->cfg.poll_period_ms;
}

static void rs485_handle_can_request(void)
{
  bool req = false;
  int32_t requested_id_i32 = 0;
  uint8_t requested_id = 0U;
  int32_t response_position = 0;
  int32_t response_velocity = 0;
  uint8_t encoder_index = 0U;
  rs485_encoder_state_t state;

  if (!CanParams_ProcEvent(s_can_cfg.request_id_param, &req) || !req)
  {
    return;
  }

  if (!CanParams_GetInt32(s_can_cfg.request_id_param, &requested_id_i32))
  {
    return;
  }

  requested_id = (uint8_t)requested_id_i32;

  if (rs485_system_find_encoder_by_node_id(requested_id, &encoder_index) &&
      rs485_system_get_encoder_state(encoder_index, &state) &&
      state.online)
  {
    response_position = raw_counts_to_position_tenths_deg(state.position_raw, state.resolution_bits);
    response_velocity = counts_per_s_to_velocity_hundredths_deg_s(state.velocity_counts_per_s,
                                                                  state.resolution_bits);
  }

  (void)CanParams_SetInt32(s_can_cfg.response_id_param, (int32_t)requested_id);
  (void)CanParams_SetInt32(s_can_cfg.response_position_param, response_position);
  (void)CanParams_SetInt32(s_can_cfg.response_velocity_param, response_velocity);
  (void)CanSystem_Send(s_can_cfg.response_position_param);
}

/* =========================
 *  Internal init
 * ========================= */

static void load_default_encoders(void)
{
  static const uint8_t default_node_ids[] =
  {
    PROJECT_RS485_ENCODER_0_NODE_ID,
    PROJECT_RS485_ENCODER_1_NODE_ID,
    PROJECT_RS485_ENCODER_2_NODE_ID,
    PROJECT_RS485_ENCODER_3_NODE_ID
  };

  uint8_t requested = PROJECT_RS485_DEFAULT_ENCODER_COUNT;
  uint8_t max_defaults = (uint8_t)(sizeof(default_node_ids) / sizeof(default_node_ids[0]));

  if (requested > max_defaults)
  {
    requested = max_defaults;
  }
  if (requested > PROJECT_RS485_ENCODER_MAX_COUNT)
  {
    requested = PROJECT_RS485_ENCODER_MAX_COUNT;
  }

  for (uint8_t i = 0U; i < requested; i++)
  {
    rs485_encoder_config_t cfg;
    cfg.node_id = default_node_ids[i];
    cfg.resolution_bits = PROJECT_RS485_ENCODER_RES_BITS;
    cfg.poll_period_ms = PROJECT_RS485_DEFAULT_POLL_MS;
    (void)rs485_system_add_encoder(&cfg);
  }
}

static void load_default_transport(void)
{
  if (s_transport.huart == NULL)
  {
    s_transport.huart = &huart4;
  }

  if (s_transport.uart_timeout_ms == 0U)
  {
    s_transport.uart_timeout_ms = PROJECT_RS485_UART_TIMEOUT_MS;
  }
}

bool rs485_system_init(void)
{
  if (s_initialized)
  {
    return true;
  }

  if (s_encoder_count == 0U)
  {
    load_default_encoders();
  }

  load_default_transport();
  rs485_init_direction_gpio();

  for (uint8_t i = 0U; i < s_encoder_count; i++)
  {
    s_encoders[i].state.online = false;
    s_encoders[i].state.position_raw = 0U;
    s_encoders[i].state.velocity_counts_per_s = 0;
    s_encoders[i].state.last_sample_tick_ms = 0U;
    s_encoders[i].state.successful_reads = 0U;
    s_encoders[i].state.failed_reads = 0U;
    s_encoders[i].last_position_raw = 0U;
    s_encoders[i].last_poll_tick_ms = 0U;
    s_encoders[i].has_sample = false;
  }

  s_next_encoder_index = 0U;
  s_initialized = true;
  return true;
}

/* =========================
 *  Public API
 * ========================= */

bool rs485_system_configure_transport(const rs485_transport_config_t* config)
{
  if (config == NULL)
  {
    return false;
  }
  if (config->huart == NULL)
  {
    return false;
  }
  if (config->uart_timeout_ms == 0U)
  {
    return false;
  }
  if (s_initialized)
  {
    return false;
  }

  s_transport = *config;
  return true;
}

bool rs485_system_get_transport_config(rs485_transport_config_t* out_config)
{
  if (out_config == NULL)
  {
    return false;
  }

  *out_config = s_transport;
  return true;
}

void rs485_system_clear_encoders(void)
{
  memset(s_encoders, 0, sizeof(s_encoders));
  s_encoder_count = 0U;
  s_next_encoder_index = 0U;
}

bool rs485_system_add_encoder(const rs485_encoder_config_t* config)
{
  rs485_encoder_slot_t* slot;

  if (config == NULL)
  {
    return false;
  }
  if (config->resolution_bits == 0U || config->resolution_bits > 14U)
  {
    return false;
  }
  if (config->poll_period_ms == 0U)
  {
    return false;
  }
  if (s_encoder_count >= PROJECT_RS485_ENCODER_MAX_COUNT)
  {
    return false;
  }

  for (uint8_t i = 0U; i < s_encoder_count; i++)
  {
    if (s_encoders[i].cfg.node_id == config->node_id)
    {
      return false;
    }
  }

  slot = &s_encoders[s_encoder_count++];
  memset(slot, 0, sizeof(*slot));
  slot->cfg = *config;
  slot->state.node_id = config->node_id;
  slot->state.resolution_bits = config->resolution_bits;

  return true;
}

uint8_t rs485_system_get_encoder_count(void)
{
  return s_encoder_count;
}

bool rs485_system_get_encoder_state(uint8_t index, rs485_encoder_state_t* out_state)
{
  if (out_state == NULL)
  {
    return false;
  }
  if (index >= s_encoder_count)
  {
    return false;
  }

  *out_state = s_encoders[index].state;
  return true;
}

bool rs485_system_find_encoder_by_node_id(uint8_t node_id, uint8_t* out_index)
{
  for (uint8_t i = 0U; i < s_encoder_count; i++)
  {
    if (s_encoders[i].cfg.node_id == node_id)
    {
      if (out_index != NULL)
      {
        *out_index = i;
      }
      return true;
    }
  }

  return false;
}

bool rs485_system_zero_encoder(uint8_t index)
{
  if (index >= s_encoder_count)
  {
    return false;
  }

  return amt21_send_extended(s_encoders[index].cfg.node_id, RS485_AMT21_ZERO_SUBCOMMAND);
}

bool rs485_system_reset_encoder(uint8_t index)
{
  if (index >= s_encoder_count)
  {
    return false;
  }

  return amt21_send_extended(s_encoders[index].cfg.node_id, RS485_AMT21_RESET_SUBCOMMAND);
}

/* =========================
 *  Controller
 * ========================= */

void rs485_system_controller(void)
{
  if (!s_initialized)
  {
    if (!rs485_system_init())
    {
      return;
    }
  }

  if (s_encoder_count == 0U)
  {
    rs485_handle_can_request();
    return;
  }

  uint32_t now = HAL_GetTick();
  rs485_encoder_slot_t* slot = &s_encoders[s_next_encoder_index];

  if (!slot_poll_is_due(slot, now))
  {
    s_next_encoder_index++;
    if (s_next_encoder_index >= s_encoder_count)
    {
      s_next_encoder_index = 0U;
    }
    rs485_handle_can_request();
    return;
  }

  slot->last_poll_tick_ms = now;

  uint16_t position_raw = 0U;
  if (amt21_read_position(slot->cfg.node_id, slot->cfg.resolution_bits, &position_raw))
  {
    slot->state.online = true;
    slot->state.position_raw = position_raw;
    slot->state.successful_reads++;

    if (slot->has_sample)
    {
      uint32_t dt_ms = now - slot->state.last_sample_tick_ms;
      if (dt_ms > 0U)
      {
        int32_t delta = unwrap_position_delta(position_raw,
                                              slot->last_position_raw,
                                              slot->cfg.resolution_bits);
        slot->state.velocity_counts_per_s = (int32_t)((delta * 1000L) / (int32_t)dt_ms);
      }
    }
    else
    {
      slot->state.velocity_counts_per_s = 0;
      slot->has_sample = true;
    }

    slot->last_position_raw = position_raw;
    slot->state.last_sample_tick_ms = now;
  }
  else
  {
    slot->state.online = false;
    slot->state.failed_reads++;
  }

  s_next_encoder_index++;
  if (s_next_encoder_index >= s_encoder_count)
  {
    s_next_encoder_index = 0U;
  }

  rs485_handle_can_request();
}
