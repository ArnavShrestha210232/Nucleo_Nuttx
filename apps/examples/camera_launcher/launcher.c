#include <nuttx/config.h>
#include <stdio.h>
#include <sched.h>
#include <unistd.h>

extern int camera_MSN2_main(int argc, char *argv[]);

int launcher_main(int argc, char *argv[])
{
  printf("Starting MSN Bus bridge task...\n");
  fflush(stdout);

  char *camera_argv[] = { NULL };
  int cam_pid = task_create("msn_bus", 100, 4096, camera_MSN2_main, camera_argv);
  if (cam_pid < 0)
    {
      printf("ERROR: MSNBUS task failed\n");
      fflush(stdout);
      return -1;
    }

  /* Yield briefly so msn_bus prints its header before launcher finishes */
  usleep(50000); 
  return 0;
}