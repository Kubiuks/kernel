/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

pcb_t procTab[ MAX_PROCS ]; pcb_t* executing = NULL; int CURRENT_PROCS = 0;


void dispatch( ctx_t* ctx, pcb_t* prev, pcb_t* next ) {
  char prev_pid = '?', next_pid = '?';

  if( NULL != prev ) {
    memcpy( &prev->ctx, ctx, sizeof( ctx_t ) ); // preserve execution context of P_{prev}
    prev_pid = '0' + prev->pid;
  }
  if( NULL != next ) {
    memcpy( ctx, &next->ctx, sizeof( ctx_t ) ); // restore  execution context of P_{next}
    next_pid = '0' + next->pid;
  }



    PL011_putc( UART0, '[',      true );
    PL011_putc( UART0, prev_pid, true );
    PL011_putc( UART0, '-',      true );
    PL011_putc( UART0, '>',      true );
    PL011_putc( UART0, next_pid, true );
    PL011_putc( UART0, ']',      true );

    executing = next;                           // update   executing process to P_{next}

  return;
}
int find_priority(){
  int max = 0;
  int process = 0;
  for (int i=0; i<CURRENT_PROCS; i++){
    if(procTab[ i ].status == STATUS_READY && procTab[ i ].priority > max){
      max = procTab[ i ].priority;
      process = i;
    }
  }
  return process;
}

void schedule( ctx_t* ctx ) {
  // if     ( executing->pid == procTab[ 0 ].pid ) {
  //   dispatch( ctx, &procTab[ 0 ], &procTab[ 1 ] );  // context switch P_1 -> P_2
  //
  //   procTab[ 0 ].status = STATUS_READY;             // update   execution status  of P_1
  //   procTab[ 1 ].status = STATUS_EXECUTING;         // update   execution status  of P_2
  // }
  // else if( executing->pid == procTab[ 1 ].pid ) {
  //   dispatch( ctx, &procTab[ 1 ], &procTab[ 2 ] );  // context switch P_2 -> P_3
  //
  //   procTab[ 1 ].status = STATUS_READY;             // update   execution status  of P_2
  //   procTab[ 2 ].status = STATUS_EXECUTING;         // update   execution status  of P_1
  // }
  // else if( executing->pid == procTab[ 2 ].pid ) {
  //   dispatch( ctx, &procTab[ 2 ], &procTab[ 0 ] );  // context switch P_3 -> P_1
  //
  //   procTab[ 2 ].status = STATUS_READY;             // update   execution status  of P_3
  //   procTab[ 0 ].status = STATUS_EXECUTING;         // update   execution status  of P_1
  // }

  for(int i=0; i<CURRENT_PROCS; i++){
    if(procTab[ i ].status == STATUS_READY){
      procTab[ i ].priority++;
    }
  }

  int next = find_priority();

  for(int i=0; i<CURRENT_PROCS; i++){
    if(executing->pid == procTab[ i ].pid ){
      dispatch( ctx, &procTab[ i ], &procTab[ next ] );
      procTab[ i ].status = STATUS_READY;
      procTab[ i ].priority = procTab[ i ].base_priority;
      procTab[ next ].status = STATUS_EXECUTING;
      break;
    }
  }



  return;
}


extern void     main_console();
extern uint32_t tos_console;
extern void     main_P1();
extern uint32_t tos_P1;
extern void     main_P2();
extern uint32_t tos_P2;
extern void     main_P3();
extern uint32_t tos_P3;


void hilevel_handler_rst(ctx_t* ctx ) {


  /* Invalidate all entries in the process table, so it's clear they are not
   * representing valid (i.e., active) processes.
   */

  for( int i = 0; i < MAX_PROCS; i++ ) {
    procTab[ i ].status = STATUS_INVALID;
  }

  /* Automatically execute the user programs P1 and P2 by setting the fields
   * in two associated PCBs.// Include functionality relating to the console.

#include "console.h"
  Note in each case that
   *
   * - the CPSR value of 0x50 means the processor is switched into USR mode,
   *   with IRQ interrupts enabled, and
   * - the PC and SP values match the entry point and top of stack.
   */


  memset( &procTab[ 0 ], 0, sizeof( pcb_t ) ); // initialise 0-th PCB = P_1
  procTab[ 0 ].pid      = 0;
  procTab[ 0 ].status   = STATUS_READY;
  procTab[ 0 ].tos      = ( uint32_t )( &tos_console );
  procTab[ 0 ].ctx.cpsr = 0x50;
  procTab[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
  procTab[ 0 ].ctx.sp   = procTab[ 0 ].tos;
  procTab[ 0 ].priority = 0;
  procTab[ 0 ].base_priority = 0;

  memset( &procTab[ 1 ], 0, sizeof( pcb_t ) ); // initialise 0-th PCB = P_1
  procTab[ 1 ].pid      = 1;
  procTab[ 1 ].status   = STATUS_READY;
  procTab[ 1 ].tos      = ( uint32_t )( &tos_P1  );
  procTab[ 1 ].ctx.cpsr = 0x50;
  procTab[ 1 ].ctx.pc   = ( uint32_t )( &main_P1 );
  procTab[ 1 ].ctx.sp   = procTab[ 0 ].tos;
  procTab[ 1 ].priority = 0;
  procTab[ 1 ].base_priority = 0;

  memset( &procTab[ 2 ], 0, sizeof( pcb_t ) ); // initialise 1-st PCB = P_2
  procTab[ 2 ].pid      = 2;
  procTab[ 2 ].status   = STATUS_READY;
  procTab[ 2 ].tos      = ( uint32_t )( &tos_P2  );
  procTab[ 2 ].ctx.cpsr = 0x50;
  procTab[ 2 ].ctx.pc   = ( uint32_t )( &main_P2 );
  procTab[ 2 ].ctx.sp   = procTab[ 1 ].tos;
  procTab[ 2 ].priority = 0;
  procTab[ 2 ].base_priority = 0;

  memset( &procTab[ 3 ], 0, sizeof( pcb_t ) ); // initialise 1-st PCB = P_3
  procTab[ 3 ].pid      = 3;
  procTab[ 3 ].status   = STATUS_READY;
  procTab[ 3 ].tos      = ( uint32_t )( &tos_P3  );
  procTab[ 3 ].ctx.cpsr = 0x50;
  procTab[ 3 ].ctx.pc   = ( uint32_t )( &main_P3 );
  procTab[ 3 ].ctx.sp   = procTab[ 2 ].tos;
  procTab[ 3 ].priority = 0;
  procTab[ 3 ].base_priority = 0;
  /* Once the PCBs are initialised, we arbitrarily select the 0-th PCB to be
   * executed: there is no need to preserve the execution context, since it
   * is invalid on reset (i.e., no process was previously executing).
   */
   for( int i = 0; i < MAX_PROCS; i++ ) {
     if(procTab[ i ].status == STATUS_READY){
       CURRENT_PROCS++;
     }
   }


  int next = find_priority();
  dispatch( ctx, NULL, &procTab[ next ] );
  procTab[ next ].status   = STATUS_EXECUTING;

  /* Configure the mechanism for interrupt handling by
   *
   * - configuring timer st. it raises a (periodic) interrupt for each
   *   timer tick,
   * - configuring GIC st. the selected interrupts are forwarded to the
   *   processor via the IRQ interrupt signal, then
   * - enabling IRQ interrupts.
   */



  TIMER0->Timer1Load  = 0x00100000; // select period = 2^20 ticks ~= 1 sec
  TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
  TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
  TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
  TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

  GICC0->PMR          = 0x000000F0; // unmask all            interrupts
  GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
  GICC0->CTLR         = 0x00000001; // enable GIC interface
  GICD0->CTLR         = 0x00000001; // enable GIC distributor

  int_enable_irq();

  return;
}

void hilevel_handler_irq(ctx_t* ctx) {
  // Step 2: read  the interrupt identifier so we know the source.

  uint32_t id = GICC0->IAR;

  // Step 4: handle the interrupt, then clear (or reset) the source.

  if( id == GIC_SOURCE_TIMER0 ) {
    PL011_putc( UART0, 'T', true ); TIMER0->Timer1IntClr = 0x01;
    schedule(ctx);
  }

  // Step 5: write the interrupt identifier to signal we're done.

  GICC0->EOIR = id;

  return;
}

void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) {
  /* Based on the identifier (i.e., the immediate operand) extracted from the
   * svc instruction,
   *
   * - read  the arguments from preserved usr mode registers,
   * - perform whatever is appropriate for this system call, then
   * - write any return value back to preserved usr mode registers.
   */

  switch( id ) {
    case 0x00 : { // 0x00 => yield()
      schedule( ctx );

      break;
    }

    case 0x01 : { // 0x01 => write( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        PL011_putc( UART0, *x++, true );
      }

      ctx->gpr[ 0 ] = n;

      break;
    }
    case 0x02 : { //read

      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        x[i] = PL011_getc( UART0,true );
      }

      ctx->gpr[ 0 ] = n;

      break;
    }
    case 0x03 : {//fork
      PL011_putc( UART0, 'F', true );

      break;
    }
    case 0x04 : {//exit
      PL011_putc( UART0, 'E', true );

      break;
    }
    case 0x05 : {//exec
      PL011_putc( UART0, 'e', true );

      break;
    }
    case 0x06 : {//kill
      PL011_putc( UART0, 'k', true );

      break;
    }
    case 0x07 : {//nice
      PL011_putc( UART0, 'n', true );

      break;
    }

    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }

  return;
}
