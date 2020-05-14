/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

pcb_t procTab[ MAX_PROCS ]; pcb_t* executing = NULL; int CURRENT_PROCS = 0;
stack_chunk stack[ MAX_PROCS ]; int current_procs[MAX_PROCS];
fd_struct fd_array[MAX_PROCS]; pipe_struct pipes[2 * MAX_PROCS];


void dispatch( ctx_t* ctx, pcb_t* prev, pcb_t* next ) {
  char prev_pid[] = "?", next_pid[] = "?";

  if( NULL != prev ) {
    memcpy( &prev->ctx, ctx, sizeof( ctx_t ) ); // preserve execution context of P_{prev}
    itoa(prev_pid, prev->pid);
  }
  if( NULL != next ) {
    memcpy( ctx, &next->ctx, sizeof( ctx_t ) ); // restore  execution context of P_{next}
    itoa(next_pid, next->pid);
  }



    PL011_putc( UART0, '[',      true );
    if(strlen(prev_pid) == 1){
      write( STDOUT_FILENO, prev_pid, 1 );
    }
    else{
      write( STDOUT_FILENO, prev_pid, 2 );
    }
    PL011_putc( UART0, '-',      true );
    PL011_putc( UART0, '>',      true );
    if(strlen(next_pid) == 1){
      write( STDOUT_FILENO, next_pid, 1 );
    }
    else{
      write( STDOUT_FILENO, next_pid, 2 );
    }
    PL011_putc( UART0, ']',      true );

    executing = next;                           // update   executing process to P_{next}

  return;
}

// return index of a process with highest priority
uint32_t find_priority(){
  uint32_t max = 0;
  uint32_t process = 0;
  for (int i=0; i<MAX_PROCS; i++){
    if(procTab[ i ].status == STATUS_READY && procTab[ i ].priority > max){
      max = procTab[ i ].priority;
      process = i;
    }
  }
  return process;
}

void schedule( ctx_t* ctx ) {

  // every processes waiting to be executed gets its
  // priority increased by 1
  for(int i=0; i<MAX_PROCS; i++){
    if(procTab[ i ].status == STATUS_READY){
      procTab[ i ].priority++;
    }
  }

  uint32_t next = find_priority();

  // sets the priority of the currently running process to its base value
  // changes the status of the currently running process to STATUS_READY
  // process with highest priority becames currently running
  for(int i=0; i<MAX_PROCS; i++){
    if(   procTab[ i ].status == STATUS_INVALID
       || procTab[ i ].status == STATUS_TERMINATED
       || procTab[ i ].status == STATUS_WAITING){
      continue;
    }
    else if (procTab[i].pid == executing->pid){
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
extern uint32_t tos_irq;
extern uint32_t tos_svc;
extern uint32_t tos_procs;

void set_stack(pid_t pid){
  stack[ pid ].in_use = true;
  stack[ pid ].pid = pid;
}


void switch_fd_array(int index){
  if(fd_array[index].in_use == false){
     fd_array[index].in_use = true;
  } else {
    fd_array[index].in_use = false;
  }
}

void switch_pipe_in_use(int index){
  if(pipes[index].in_use == false){
     pipes[index].in_use = true;
  } else {
    pipes[index].in_use = false;
  }
}

void hilevel_handler_rst(ctx_t* ctx ) {


  /* Invalidate all entries in the process table, so it's clear they are not
   * representing valid (i.e., active) processes.
   * and Invalidate all stack chunks
   */


  for( int i = 0; i < MAX_PROCS; i++ ) {
    procTab[ i ].status = STATUS_INVALID;
    procTab[ i ].fd = -1;
    stack[ i ].in_use = false;
    stack[ i ].tos = ( uint32_t )( &tos_procs ) - (i+1) * 0x00001000;
    fd_array[ i ].in_use = false;
    fd_array[ i ].fd = i;
  }

  switch_fd_array(0);
  switch_fd_array(1);
  switch_fd_array(2);

  for( int i = 0; i < 2 * MAX_PROCS; i++ ) {
      pipes[i].fd_write = -1;
      pipes[i].fd_read = -1;
      pipes[i].in_use = false;
  }


  /*
  Note in each case that
   *
   * - the CPSR value of 0x50 means the processor is switched into USR mode,
   *   with IRQ interrupts enabled, and
   * - the PC and SP values match the entry point and top of stack.
   */


  memset( &procTab[ 0 ], 0, sizeof( pcb_t ) ); // initialise 0-th PCB = P_1
  procTab[ 0 ].pid      = 0;
  procTab[ 0 ].status   = STATUS_READY;
  procTab[ 0 ].tos      = ( uint32_t )( stack[ 0 ].tos);
  procTab[ 0 ].ctx.cpsr = 0x50;
  procTab[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
  procTab[ 0 ].ctx.sp   = procTab[ 0 ].tos;
  procTab[ 0 ].priority = 0;
  procTab[ 0 ].base_priority = 0;
  procTab[ 0 ].fd = fd_array[3].fd;

  switch_fd_array(3);
  set_stack(0);


   for( int i = 0; i < MAX_PROCS; i++ ) {
     if(procTab[ i ].status == STATUS_READY){
       CURRENT_PROCS++;
     }
   }


  uint32_t next = find_priority();
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

uint32_t stack_index_by_pid(pid_t pid){
  for(int i = 0; i < MAX_PROCS; i++){
    if(stack[ i ].pid == pid){
      return i;
    }
  }
  return -1;
}

uint32_t free_stack_chunk(){
  for(int i = 0; i < MAX_PROCS; i++){
    if(stack[ i ].in_use == false){
      return i;
    }
  }
  return -1;
}

pid_t find_index_by_pid(pid_t pid){
  for(int i = 0; i < MAX_PROCS; i++){
    if(procTab[i].pid == pid){
      return i;
    }
  }
  return -1;
}

uint32_t free_procTab_index(){
  for(int i = 0; i < MAX_PROCS; i++){
    if(procTab[ i ].status == STATUS_INVALID || procTab[ i ].status == STATUS_TERMINATED){
      return i;
    }
  }
  return -1;
}

int find_free_pipe(){
    for(int i = 0; i < 2 * MAX_PROCS; i++){
      if(pipes[i].in_use == false){
        return i;
      }
    }
    return -1;
}

int find_free_fd(){
  for(int i = MIN_FD; i < MAX_PROCS; i++){
    if(fd_array[i].in_use == false){
      return i;
    }
  }
  return -1;
}

int find_fd_by_pid(pid_t pid){
  for(int i = 0; i < MAX_PROCS; i++){
    if(procTab[i].pid == pid){
      return procTab[i].fd;
    }
  }
  return -1;
}

int find_pipe_by_fds(int fd_read, int fd_write){
  for(int i = 0; i < 2 * MAX_PROCS; i++){
    if(pipes[i].fd_read == fd_read && pipes[i].fd_write == fd_write){
      return i;
    }
  }
  return -1;;
}

// creates new entry in the procTab
// allocates new memory staack chunk for child
// and copies everything from parents ctx and stack
void initPCB(ctx_t* ctx, pid_t parent, pid_t child){
  uint32_t child_stack = free_stack_chunk();
  uint32_t parent_stack = stack_index_by_pid(parent);
  if(child_stack == -1 || parent_stack == -1){
    return;
  }
  memset( &procTab[ child ], 0, sizeof( pcb_t ) );
  procTab[ child ].pid      = child;
  procTab[ child ].status   = STATUS_READY;
  procTab[ child ].tos      = ( uint32_t )( stack[ child_stack ].tos );
  procTab[ child ].priority = procTab[ parent ].priority;
  procTab[ child ].base_priority = procTab[ parent ].base_priority;
  procTab[ child ].parent_pid = parent;
  int child_fd = find_free_fd();
  procTab[ child ].fd = child_fd;

  switch_fd_array(child_fd);

  set_stack(child);
  memcpy((uint32_t)stack[child_stack].tos - 0x00001000,
         (uint32_t)stack[parent_stack].tos - 0x00001000,
          0x00001000);
  memcpy(&procTab[ child ].ctx, ctx, sizeof(ctx_t));
  procTab[ child ].ctx.sp = (uint32_t)procTab[ child ].tos - ((uint32_t)procTab[parent].tos - ctx->sp) ;
  procTab[ parent ].ctx.gpr[0] = child;
  procTab[ child ].ctx.gpr[0] = 0;
}

// sets memory for pipe's data
// returns the index of a newly created pipe
int initPipe(int fd_read, int fd_write){
  int new_pipe_index = find_free_pipe();
  memset(&pipes[new_pipe_index].data, 0, sizeof(pipe_struct));
  switch_pipe_in_use(new_pipe_index);
  pipes[new_pipe_index].fd_read = fd_read;
  pipes[new_pipe_index].fd_write = fd_write;
  return new_pipe_index;
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
    case SYS_YIELD : { // 0x00 => yield()
      schedule( ctx );

      break;
    }

    case SYS_WRITE : { // 0x01 => write( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        PL011_putc( UART0, *x++, true );
      }

      ctx->gpr[ 0 ] = n;

      break;
    }
    case SYS_READ : { //read

      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        x[i] = PL011_getc( UART0,true );
      }


      ctx->gpr[ 0 ] = n;

      break;
    }
    case SYS_FORK : {//fork
      pid_t parent = find_index_by_pid(executing->pid);
      pid_t child = free_procTab_index();
      initPCB(ctx, parent, child);
      break;
    }
    case SYS_EXIT : {//exit
      uint32_t x = (uint32_t)(ctx->gpr[0]);

      if(x == EXIT_SUCCESS){
        int proc_i = find_index_by_pid(executing->pid);
        int stack_i = stack_index_by_pid(executing->pid);
        procTab[proc_i].status = STATUS_TERMINATED;
        stack[stack_i].in_use = false;
        stack[stack_i].pid = 0;
        pid_t parent_id = find_index_by_pid(executing->parent_pid);
        // if(procTab[parent_id].status == STATUS_WAITING){
        //   procTab[parent_id].status == STATUS_EXECUTING;
        //
        // }
        if(   procTab[parent_id].status == STATUS_INVALID
           || procTab[parent_id].status == STATUS_TERMINATED){
             executing = &procTab[0];
        } else {
          executing = &procTab[parent_id];
        }
      }
      else{

      }


      break;
    }
    case SYS_EXEC : {//exec
      uint32_t x = (uint32_t)(ctx->gpr[0]);
      ctx->pc = (uint32_t)(x);
      ctx->sp = procTab[ find_index_by_pid(executing->pid) ].tos;

      break;
    }
    case SYS_KILL : {//kill
      PL011_putc( UART0, 'k', true );

      break;
    }
    case SYS_NICE : {//nice
      PL011_putc( UART0, 'n', true );

      break;
    }

    case SYS_PIPE : {//pipe


      int pipe_id = initPipe(ctx->gpr[0], ctx->gpr[1]);

      ctx->gpr[0] = pipe_id;
      break;
    }
    case SYS_GET_PID : {//get pid of currently running process
      ctx->gpr[0] = executing->pid;

      break;
    }

    // case SYS_WAIT : {//wait for children to terminate
    //   PL011_putc( UART0, 'W', true );
    //   pid_t wait_id = executing->pid;
    //   schedule(ctx);
    //   procTab[find_index_by_pid(wait_id)].status = STATUS_WAITING;
    //   break;
    // }

    case SYS_GET_FD : {
      ctx->gpr[0] = find_fd_by_pid(ctx->gpr[0]);

      break;
    }

    case SYS_WRITE_TO_PIPE : {
      int   pipe_index = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );



      for( int i = 0; i < n; i++ ) {
         pipes[pipe_index].data[i] = *x++;
      }

      ctx->gpr[ 0 ] = n;

      break;
    }
    case SYS_READ_FROM_PIPE : { //read

      int   pipe_index = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      for( int i = 0; i < n; i++ ) {
        x[i] = pipes[pipe_index].data[i];
      }


      ctx->gpr[ 0 ] = n;

      break;
    }
    case SYS_GET_PIPE_BY_FDS : {
      ctx->gpr[0] = find_pipe_by_fds(ctx->gpr[0], ctx->gpr[1]);

      break;
    }

    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }

  return;
}
