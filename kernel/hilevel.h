/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#ifndef __HILEVEL_H
#define __HILEVEL_H

// Include functionality relating to newlib (the standard C library).

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// Include functionality relating to the platform.

#include   "GIC.h"
#include "PL011.h"
#include "SP804.h"

// Include functionality relating to the   kernel.

#include "lolevel.h"
#include     "int.h"
#include    "libc.h"

// Include functionality relating to the console.

#include "console.h"


#define MAX_PROCS 30
#define MIN_FD 3

typedef int pid_t;

typedef enum {
  STATUS_INVALID,

  STATUS_CREATED,
  STATUS_TERMINATED,

  STATUS_READY,
  STATUS_EXECUTING,
  STATUS_WAITING
} status_t;

typedef struct {
  uint32_t cpsr, pc, gpr[ 13 ], sp, lr;
} ctx_t;

typedef struct {
  bool in_use;     // indicates wheter the chunk is assigned to some process
  uint32_t    tos; // address of Top of Stack (ToS)
  pid_t pid;
} stack_chunk;

typedef struct {
       pid_t    pid;      // Process IDentifier (PID)
    status_t status;      // current status
    uint32_t    tos;      // address of Top of Stack (ToS)
       ctx_t    ctx;      // execution context
  uint32_t priority;      // current priority of the Process
  uint32_t base_priority; // base priority of the Process
  pid_t parent_pid;       // pid of the parent
  int  fd;                // file descriptor for pipes
} pcb_t;

typedef struct {
  bool in_use;     // indicates wheter the fd is assigned to some process
  int fd;          //file descriptor
} fd_struct;

typedef struct {
  bool in_use;       // indicates wheter the pipe is assigned to some process
  char  data[100];   // data being send through the channel
  int   fd_read;     // reading end of the pipe
  int   fd_write;    // writing end of the pipe
} pipe_struct;

#endif
