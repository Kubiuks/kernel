#include "DINE.h"

int MAX_PHILOSOPHERS = 16;

// function implementing dining philosophers
void main_DINE() {

  static int current = -1;
  int id;

  //forks 15 time to create 16 philosophers in total
  for(int i = 0; i < MAX_PHILOSOPHERS - 1; i++){
    if(fork() == 0){
        break;
    }
  }
  current += 1;
  id = current;
  char index[3];
  itoa(index, id);

  //prints to console
  puts( "Starting: $ ", 10 );
  puts( index, 3);

  
  while( 1 ) {

  }

  exit( EXIT_SUCCESS );
}
