#include "motor_fun.h"
#include "bsp_internal_flash.h"

extern "C"
{
  void PVD_config(void);
  void PVD_IRQHandler();
}