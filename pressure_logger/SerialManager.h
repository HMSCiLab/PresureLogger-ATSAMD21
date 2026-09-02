#pragma once
 
/**********************/
// Define states
enum UartSessionState
{
  CLEAN_COMM,
  DETECT_CLIENT,
  CONFIRM_CLIENT,
  COMMAND_MODE,
  COMMAND_MODE_EXIT,
  CLEANUP_COMM
};

extern UartSessionState uartSessionState;

bool handle_uart_session();
void update_pointer();
