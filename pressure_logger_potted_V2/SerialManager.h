#pragma once

// Define states
enum UartState
{
  CLEAN_COMM,
  DETECT_CLIENT,
  CONFIRM_CLIENT,
  COMMAND_MODE,
  CLEANUP_COMM
};

extern UartState uartState;

bool handle_uart();
void handle_uart_clean_comm();
void handle_uart_detect_client();
void handle_uart_confirm_client();
void handle_uart_command_mode();
void handle_uart_cleanup_comm();

void get_battery_voltage();
void dump_data();
uint16_t crc16_update(uint16_t crc, uint8_t data);

bool handle_uart_session();
