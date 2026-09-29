#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <nuttx/leds/userled.h>

/* LED ioctl compatibility macro */
#ifndef ULEDIOC_SETLEDS
#  ifdef ULEDIOC_SETALL
#    define ULEDIOC_SETLEDS ULEDIOC_SETALL
#  else
#    define ULEDIOC_SETLEDS ULEDIOC_SETLED
#  endif
#endif

#define PC_UART_DEV     "/dev/ttyS0"   /* USART2 (ST-LINK VCP / PC Terminal) */
#define CAM_UART_DEV    "/dev/ttyS1"   /* USART1 (Camera Hardware Interface) */

#define SHARED_RAM_ADDR 0x20008000
#define BUFFER_SIZE     256

typedef struct {
    volatile uint32_t len;   /* Total bytes in buffer */
    volatile uint32_t flag;  /* 1 = Data ready for CM0+, 0 = Buffer available for CM4 */
    uint8_t buffer[2048];    /* Shared payload buffer */
} SharedData_t;

static void *led_blink_thread(void *arg)
{
    int led_fd = open("/dev/userleds", O_WRONLY);

    while (1) {
        if (led_fd >= 0) {
            static uint32_t state = 0;
            state ^= 0x07;
            ioctl(led_fd, ULEDIOC_SETLEDS, state);
        }
        usleep(1000000);
    }

    if (led_fd >= 0) {
        close(led_fd);
    }
    return NULL;
}

static int configure_uart(int fd)
{
    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) return -1;

    cfsetspeed(&tio, B115200);
    
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tio.c_oflag &= ~OPOST;
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tio.c_cflag &= ~(CSIZE | PARENB);
    tio.c_cflag |= CS8 | CLOCAL | CREAD;

    return tcsetattr(fd, TCSANOW, &tio);
}

int main(int argc, FAR char *argv[])
{
    int fd_pc = -1;
    int fd_cam = -1;
    pthread_t thread_id;

    SharedData_t *pShared = (SharedData_t *)SHARED_RAM_ADDR;
    pShared->flag = 0;
    pShared->len = 0;

    printf("[Bridge] Initializing Transparent Camera Bridge Application...\n");

    fd_pc = open(PC_UART_DEV, O_RDWR | O_NOCTTY);
    if (fd_pc < 0) {
        printf("[Bridge] Error: Failed to open %s\n", PC_UART_DEV);
        return -1;
    }

    fd_cam = open(CAM_UART_DEV, O_RDWR | O_NOCTTY);
    if (fd_cam < 0) {
        printf("[Bridge] Error: Failed to open %s\n", CAM_UART_DEV);
        close(fd_pc);
        return -1;
    }

    configure_uart(fd_pc);
    configure_uart(fd_cam);

    pthread_create(&thread_id, NULL, led_blink_thread, NULL);

    printf("[Bridge] Transparent pass-through active: %s <--> %s\n", PC_UART_DEV, CAM_UART_DEV);

    struct pollfd fds[2];
    fds[0].fd = fd_pc;
    fds[0].events = POLLIN;
    fds[1].fd = fd_cam;
    fds[1].events = POLLIN;

    uint8_t io_buf[BUFFER_SIZE];

    while (1) {
        int poll_res = poll(fds, 2, -1);

        if (poll_res > 0) {
            /* 1. Send user-typed bytes from PC (UART2) directly to Camera (UART1) */
            if (fds[0].revents & POLLIN) {
                ssize_t bytes_read = read(fd_pc, io_buf, sizeof(io_buf));
                if (bytes_read > 0) {
                    write(fd_cam, io_buf, bytes_read);
                }
            }

            /* 2. Send camera responses from Camera (UART1) directly back to PC (UART2) */
            if (fds[1].revents & POLLIN) {
                ssize_t bytes_read = read(fd_cam, io_buf, sizeof(io_buf));
                if (bytes_read > 0) {
                    write(fd_pc, io_buf, bytes_read);

                    /* Optional sync to shared dual-core RAM if space allows */
                    if (pShared->flag == 0 && (pShared->len + bytes_read <= sizeof(pShared->buffer))) {
                        memcpy(&pShared->buffer[pShared->len], io_buf, bytes_read);
                        pShared->len += bytes_read;
                    }
                }
            }
        }
    }

    close(fd_pc);
    close(fd_cam);
    return 0;
}
