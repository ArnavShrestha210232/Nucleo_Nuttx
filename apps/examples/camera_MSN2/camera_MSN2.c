#include <nuttx/config.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <poll.h>

#define UART_NUCLEO_DEV "/dev/ttyS1"  /* UART4 connected to Nucleo   */
#define UART_CAM_DEV    "/dev/ttyS0"  /* UART2 connected to Cam Board*/
#define BUFFER_SIZE     2048

static void configure_uart(int fd, speed_t baud)
{
  struct termios tio;
  if (tcgetattr(fd, &tio) < 0)
    {
      perror("tcgetattr failed");
      return;
    }

  cfmakeraw(&tio);               /* Raw mode: disables line-buffering & echo */
  cfsetispeed(&tio, baud);      /* Set input baud rate */
  cfsetospeed(&tio, baud);      /* Set output baud rate */
  tcsetattr(fd, TCSANOW, &tio);
}

static ssize_t write_all(int fd, const uint8_t *buf, size_t len)
{
  size_t total = 0;
  while (total < len)
    {
      ssize_t written = write(fd, buf + total, len - total);
      if (written < 0)
        {
          if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
              usleep(100);
              continue;
            }
          return -1;
        }
      total += (size_t)written;
    }
  return (ssize_t)total;
}

int camera_MSN2_main(int argc, char *argv[])
{
  static uint8_t nucleo_to_cam_buf[BUFFER_SIZE];
  static uint8_t cam_to_nucleo_buf[BUFFER_SIZE];

  printf("\n================ MSN BUS BRIDGE START ================\n");

  int fd_nucleo = open(UART_NUCLEO_DEV, O_RDWR | O_NOCTTY | O_NONBLOCK);
  int fd_cam    = open(UART_CAM_DEV,    O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_nucleo < 0 || fd_cam < 0)
    {
      printf("[MSNBUS] UART open failed: Nucleo (%s)=%d, Cam (%s)=%d\n", 
             UART_NUCLEO_DEV, fd_nucleo, UART_CAM_DEV, fd_cam);
      if (fd_nucleo >= 0) close(fd_nucleo);
      if (fd_cam >= 0) close(fd_cam);
      return -1;
    }

  configure_uart(fd_nucleo, B115200);
  configure_uart(fd_cam, B115200);

  struct pollfd fds[2];
  fds[0].fd     = fd_nucleo;
  fds[0].events = POLLIN;
  fds[1].fd     = fd_cam;
  fds[1].events = POLLIN;

  while (1)
    {
      int ret = poll(fds, 2, 10);

      if (ret < 0)
        {
          if (errno == EINTR) continue;
          usleep(10000);
          continue;
        }

      /* Nucleo (UART4) -> Cam Board (UART2) */
      if (fds[0].revents & POLLIN)
        {
          ssize_t nbytes = read(fd_nucleo, nucleo_to_cam_buf, sizeof(nucleo_to_cam_buf));
          if (nbytes > 0)
            {
              // printf("\n[DEBUG] Read %d bytes from UART4 (ttyS1)! Forwarding to UART2...\n", (int)nbytes);
              write_all(fd_cam, nucleo_to_cam_buf, (size_t)nbytes);
            }
        }

      /* Cam Board (UART2) -> Nucleo (UART4) */
      if (fds[1].revents & POLLIN)
        {
          ssize_t nbytes = read(fd_cam, cam_to_nucleo_buf, sizeof(cam_to_nucleo_buf));
          if (nbytes > 0)
            {
              // printf("\n[DEBUG] Read %d bytes from UART2 (ttyS0)! Forwarding to UART4...\n", (int)nbytes);
              write_all(fd_nucleo, cam_to_nucleo_buf, (size_t)nbytes);
            }
        }
    }

  close(fd_nucleo);
  close(fd_cam);
  return 0;
}