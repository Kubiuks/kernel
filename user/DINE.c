#include "DINE.h"

#define dirty 0
#define clean 1
#define missing -1
#define just_sent 2
#define NUM_MAX 12
#define NUM_MIN 5

// function implementing dining philosophers
// Chandy/Misra solution
// words "fork" and "chopstick" are used interchangeably
void main_DINE() {

  //variables philosophers share
  static int current = -1;
  static int pid_of_philosophers[16];

  //variables which philosophers have their own copy of
  int left_chopstick = missing;
  int right_chopstick = dirty;

  // forks 15 time to create 16 philosophers in total
  // break if child
  for(int i = 1; i < 16; i++){
    if(fork() == 0){
        break;
    }
  }

  // sets up the table
  current++;
  int my_id = current;
  pid_of_philosophers[current] = get_pid();


  char id[3];
  itoa(id, my_id);
  puts("Starting $ ", 9);
  puts(id, 3);
  puts("\n", 1);

  // yeild to wait for other philosophers to finish set up
  yield();

  // every philosopher creates a pipe with each of his neighbours
  int fd1[2];
  int fd2[2];
  int my_fd = get_fd(pid_of_philosophers[my_id]);

  if(my_id == 0){
      fd1[0] = get_fd(pid_of_philosophers[15]); fd1[1] = my_fd;
      fd2[0] = get_fd(pid_of_philosophers[1]); fd2[1] =  my_fd;
  } else if(my_id == 15){
      fd1[0] = get_fd(pid_of_philosophers[14]); fd1[1] = my_fd;
      fd2[0] = get_fd(pid_of_philosophers[0]); fd2[1] = my_fd;
  } else {
    fd1[0] = get_fd(pid_of_philosophers[my_id - 1]); fd1[1] = my_fd;
    fd2[0] = get_fd(pid_of_philosophers[my_id + 1]); fd2[1] = my_fd;
  }
  int left_write_pipe_id = pipe(fd1[0], fd1[1]);
  int right_write_pipe_id = pipe(fd2[0], fd2[1]);


  // yield to wait for other philosophers to create all channels of communication
  yield();

  // get the ids of read channels
  int left_read_pipe_id;
  int right_read_pipe_id;
  if(my_id == 0){
      left_read_pipe_id = get_pipe_by_fds(my_fd, get_fd(pid_of_philosophers[15]));
      right_read_pipe_id = get_pipe_by_fds(my_fd, get_fd(pid_of_philosophers[1]));
  } else if(my_id == 15){
    left_read_pipe_id = get_pipe_by_fds(my_fd, get_fd(pid_of_philosophers[14]));
    right_read_pipe_id = get_pipe_by_fds(my_fd, get_fd(pid_of_philosophers[0]));
  } else {
    left_read_pipe_id = get_pipe_by_fds(my_fd, get_fd(pid_of_philosophers[my_id - 1]));
    right_read_pipe_id = get_pipe_by_fds(my_fd, get_fd(pid_of_philosophers[my_id + 1]));
  }

  // gets a pseudo random number
  // to simulate thinking and eating times
  int x, y;
  x = rand() % (NUM_MAX + 1 - NUM_MIN) + NUM_MIN;
  y = rand() % (NUM_MAX + 1 - NUM_MIN) + NUM_MIN;

  // arrays in which philosopher store what others send
  char left_channel_data[1];
  char right_channel_data[1];

  //because initially each fork between neighbours
  //should go to the philosopher with the lower ID
  //we adjust philo 0 and philo MAX_PHILOSOPHERS-1 fork count
  //this creates acyclic graph which prevents deadlocks
  if(my_id == 0){
    left_chopstick = dirty;
  } else if(my_id == 15){
    right_chopstick = missing;
  }


  // if philosopher gets request but the fork is clean we defer giving it back
  // defer[0] refers to left fork
  // defer[1] refers to right fork
  // value 1 means fork needs to be returned immediately after eating
  // value 0 means the is no sending needed
  int defer[2];

  // philosophers think for some time
  // while thinking, their neighbours might request them to give them a fork
  // when requesting, philosopher writes:
  // 'g' for give me a fork
  // when requested, philosopher writes back:
  // 'r' when fork is dirty, which he "cleans" and gives it back
  // or when fork is clean, he writes nothing and keeps the fork for himself
  while(1){

      // imitate thinking time, different for each philosopher
      // does not change between iterations
      // taken from P5
      for( int i = 0; i < x; i++ ) {

        uint32_t lo = 1 <<  8;
        uint32_t hi = 1 << 16;

        puts("Philosopher $ ", 12);
        puts(id, 3);
        puts("is thinking \n$ ", 13);

        for( uint32_t x = lo; x < hi; x++ ) {
          int r = is_prime( x );
        }
        // during thinking if a neighbour wants our fork
        // we give it if it's dirty
        // we need to set the chopsticks to just_sent
        // otherwise we might ask for them before a neighbour receives them
        read_from_pipe(left_read_pipe_id, left_channel_data, 1);
        if(left_channel_data[0] == 'g'){
          if(left_chopstick == dirty){
              write_to_pipe(left_write_pipe_id, "r", 1);
              left_chopstick = just_sent;
          }
        }
        read_from_pipe(right_read_pipe_id, right_channel_data, 1);
        if(right_channel_data[0] == 'g'){
          if(right_chopstick == dirty){
            write_to_pipe(right_write_pipe_id, "r", 1);
            right_chopstick = just_sent;
          }
        }
      }

      puts("Philosopher $ ", 12);
      puts(id, 3);
      puts("wants to eat \n$ ", 14);




      // if any chopstick is missing, we request it
      if(left_chopstick == missing){
        write_to_pipe(left_write_pipe_id, "g", 1);
      }
      if(right_chopstick == missing){
        write_to_pipe(right_write_pipe_id, "g", 1);
      }

      // keep yielding until we get it
      while((left_chopstick == missing || left_chopstick == just_sent)
             || (right_chopstick == missing || right_chopstick == just_sent)){
        // during waiting for the forks if any neighbour wants our fork
        // we give it if its dirty or keep it if its clean and defer
        // giving it
        read_from_pipe(left_read_pipe_id, left_channel_data, 1);
        if(left_channel_data[0] == 'r'){
          left_chopstick = clean;
        } else if(left_channel_data[0] == 'g'){
          if(left_chopstick == dirty){
            write_to_pipe(left_write_pipe_id, "r", 1);
            left_chopstick = just_sent;
          } else if(left_chopstick == clean){
              defer[0] = 1;
          }
        }
        read_from_pipe(right_read_pipe_id, right_channel_data, 1);
        if(right_channel_data[0] == 'r'){
          right_chopstick = clean;
        } else if(right_channel_data[0] == 'g'){
          if(right_chopstick == dirty){
            write_to_pipe(right_write_pipe_id, "r", 1);
            right_chopstick = just_sent;
          } else if(right_chopstick == clean){
              defer[1] = 1;
          }
        }
        // because I sent a request for a fork, but the philosophers havent yet received it
        // we yield to avoid busy waiting
        if((left_chopstick == missing || left_chopstick == just_sent)
              || (right_chopstick == missing || right_chopstick == just_sent)){
                yield();
              }
        // if a neighbour asked for our chopstick and we gave it to them
        // we need to ask for it back
        if(left_chopstick == just_sent){
          write_to_pipe(left_write_pipe_id, "g", 1);
          left_chopstick = missing;
        }
        if(right_chopstick == just_sent){
          write_to_pipe(right_write_pipe_id, "g", 1);
          right_chopstick = missing;
        }
      }


      // if we have both chopsticks we can start eating
      // simulate eating time
      for( int i = 0; i < y; i++ ) {

        uint32_t lo = 1 <<  8;
        uint32_t hi = 1 << 16;

        puts("Philosopher $ ", 12);
        puts(id, 3);
        puts("is eating \n$ ", 11);

        for( uint32_t x = lo; x < hi; x++ ) {
          int r = is_prime( x );
        }
        // during eating we defer any requests
        read_from_pipe(left_read_pipe_id, left_channel_data, 1);
        if(left_channel_data[0] == 'g'){
          defer[0] = 1;
        }

        read_from_pipe(right_read_pipe_id, right_channel_data, 1);
        if(right_channel_data[0] == 'g'){
          defer[1] = 1;
        }
      }
      puts("Philosopher $ ", 12);
      puts(id, 3);
      puts("finished eating \n$ ", 17);

      // after eating we set the chopsticks to dirty
      left_chopstick = dirty;
      right_chopstick = dirty;

      // send defered chopsticks
      // we need to set the chopsticks to just_sent
      // otherwise we might ask for them before a neighbour receives them
      if(defer[0] == 1){
        write_to_pipe(left_write_pipe_id, "r", 1);
        left_chopstick = just_sent;
        defer[0] = 0;
      }
      if(defer[1] == 1){
        write_to_pipe(right_write_pipe_id, "r", 1);
        right_chopstick = just_sent;
        defer[1] = 0;
      }
  }
}
