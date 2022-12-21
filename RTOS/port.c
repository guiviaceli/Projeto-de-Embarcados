#include <stdint.h>
#include "os.h"

cpu_t *stk_tmp;

cpu_t *PrepareStack(void *task, cpu_t *stk, int stk_size){
  stk = (cpu_t*)((int)stk + stk_size);

  *--stk = (cpu_t)INITIAL_XPSR;                       /* xPSR                                                   */

  *--stk = (cpu_t)task;                               /* Entry Point                                            */
  /// ??????????????????????
  *--stk = 0;                                         /* R14 (LR)                                               */
  *--stk = (cpu_t)0x12121212u;                        /* R12                                                    */
  *--stk = (cpu_t)0x03030303u;                        /* R3                                                     */
  *--stk = (cpu_t)0x02020202u;                        /* R2                                                     */
  *--stk = (cpu_t)0x01010101u;                        /* R1                                                     */
  *--stk = (cpu_t)0;//parameters;                       /* R0 : argument                                        */
#if FPU_ENABLE == 1
  *--stk = (cpu_t)0xFFFFFFFDu;                      /* R14                                                      */
#endif
                                                      /* Remaining registers saved on process stack             */
  *--stk = (cpu_t)0x11111111u;                        /* R11                                                    */
  *--stk = (cpu_t)0x10101010u;                        /* R10                                                    */
  *--stk = (cpu_t)0x09090909u;                        /* R9                                                     */
  *--stk = (cpu_t)0x08080808u;                        /* R8                                                     */
  *--stk = (cpu_t)0x07070707u;                        /* R7                                                     */
  *--stk = (cpu_t)0x06060606u;                        /* R6                                                     */
  *--stk = (cpu_t)0x05050505u;                        /* R5                                                     */
  *--stk = (cpu_t)0x04040404u;                        /* R4                                                     */

  return stk;
}

/* rotinas de interrup��o necess�rias */
__attribute__ ((naked)) void SVC_Handler(void)
{
    /* Make PendSV and SysTick the lowest priority interrupts. */
    *(NVIC_SYSPRI3) |= NVIC_PENDSV_PRI;
    *(NVIC_SYSPRI3) |= NVIC_SYSTICK_PRI;
    RESTORE_SP();
    RESTORE_CONTEXT();
}

__attribute__ ((naked)) void PendSV_Handler(void){
  SAVE_CONTEXT();
  SAVE_SP();

  Clear_PendSV();

  current_task->stk=stk_tmp;
  stk_tmp = scheduler();

  RESTORE_SP();
  RESTORE_CONTEXT();
}



void init_os_timer(void){
    uint32_t cpu_clock_hz = 120000000;//system_cpu_clock_get_hz();
    uint16_t valor_comparador = cpu_clock_hz/1000; //cfg_MARCA_TEMPO_HZ; //(cfg_CPU_CLOCK_HZ / cfg_MARCA_TEMPO_HZ);

    *(NVIC_SYSTICK_CTRL) = 0;                       // Desabilita SysTick Timer
    *(NVIC_SYSTICK_LOAD) = valor_comparador - 1;    // Configura a contagem
    *(NVIC_SYSTICK_CTRL) = NVIC_SYSTICK_CLK | NVIC_SYSTICK_INT | NVIC_SYSTICK_ENABLE;  // Inicia
}


void SysTick_Handler(void){


  if(os_inc_and_compare()){
      yield();
  }

}

