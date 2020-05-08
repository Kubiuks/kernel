#include "DINE.h"

int MAX_PHILOSOPHERS = 16;

extern void main_Philosopher();

// function implementing dining philosophers
void main_DINE() {


  static int current = -1;
  static int pid_of_philosophers[16];
  // forks 15 time to create 16 philosophers in total
  // break if child
  for(int i = 1; i < MAX_PHILOSOPHERS; i++){
    if(fork() == 0){
        break;
    }
  }
  current++;
  int my_id = current;
  pid_of_philosophers[current] = get_pid();


  char id[3];
  itoa(id, my_id);
  puts("Starting $ ", 9);
  puts(id, 3);

  yield();

  if(my_id == 0){
      pipe(get_fd(pid_of_philosophers[15]), get_fd(pid_of_philosophers[0]));
      pipe(get_fd(pid_of_philosophers[1]), get_fd(pid_of_philosophers[0]));
  } else if(my_id == 15){
      pipe(get_fd(pid_of_philosophers[14]), get_fd(pid_of_philosophers[15]));
      pipe(get_fd(pid_of_philosophers[0]), get_fd(pid_of_philosophers[15]));
  } else {
    pipe(get_fd(pid_of_philosophers[my_id - 1]), get_fd(pid_of_philosophers[my_id]));
    pipe(get_fd(pid_of_philosophers[my_id + 1]), get_fd(pid_of_philosophers[my_id]));
  }

  while(1){

  }
}
