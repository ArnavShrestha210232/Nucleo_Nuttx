#include <cstddef>
cat << 'EOF' > camera_bridge_main.c
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

#define PC_UART_DEV     "/dev/ttyS0"   /* LPUART1 (ST-LINK VCP / PC) */
#define CAM_UART_DEV    "/dev/ttyS1"   /* USART1 (Camera) */

#define SHARED_RAM_ADDR 0x20008000
#define CAM_CMD_LEN     13
#define TOTAL_BUF_SIZE  8192

typedef struct {
    volatile uint32_t len;   /* Total bytes in buffer */
    volatile uint32_t flag;  /* 1 = Data ready for CM0+, 0 = Buffer available for CM4 */
    uint8_t buffer[2048];    /* Shared payload buffer */
} SharedData_t;

static const uint8_t g_cam_cmd[CAM_CMD_LEN] = {
    0x53, 0x04, 0xcc, 0x5e, 0xbd, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00
};

static uint8_t g_rx_full_buf[TOTAL_BUF_SIZE];

static void *led_blink_thread(void *arg)
{
    int led_fd = open("/dev/userleds", O_WRONLY);

    while (1) {
        if (led_fd >= 0) {
            static uint32_t state = 0;
            state ^= 0x07;
            ioctl(led_fd, ULEDIOC_SETLEDS, state);
        }
        usleep(1000000); /* HAL_Delay(1000) equivalent */
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

    printf("[Bridge] Initializing Camera Bridge Application...\n");

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

    printf("[Bridge] Listening on %s for trigger command...\n", PC_UART_DEV);

    uint8_t pc_rx_buf[CAM_CMD_LEN];

    while (1) {
        ssize_t bytes_read = 0;
        while (bytes_read < CAM_CMD_LEN) {
            ssize_t ret = read(fd_pc, &pc_rx_buf[bytes_read], CAM_CMD_LEN - bytes_read);
            if (ret > 0) {
                bytes_read += ret;
            } else if (ret < 0) {
                usleep(1000);
            }
        }

        if (memcmp(pc_rx_buf, g_cam_cmd, CAM_CMD_LEN) == 0) {
            memset(g_rx_full_buf, 0, sizeof(g_rx_full_buf));
            uint16_t total_rx_bytes = 0;

            write(fd_cam, pc_rx_buf, CAM_CMD_LEN);

            struct pollfd fds[1];
            fds[0].fd = fd_cam;
            fds[0].events = POLLIN;

            while (total_rx_bytes < TOTAL_BUF_SIZE) {
                int poll_res = poll(fds, 1, 200);

                if (poll_res > 0 && (fds[0].revents & POLLIN)) {
                    uint8_t temp_buf[256];
                    ssize_t chunk = read(fd_cam, temp_buf, sizeof(temp_buf));
                    if (chunk > 0) {
                        write(fd_pc, temp_buf, chunk);

                        size_t copy_size = (total_rx_bytes + chunk > TOTAL_BUF_SIZE) ? 
                                           (TOTAL_BUF_SIZE - total_rx_bytes) : chunk;
                        memcpy(&g_rx_full_buf[total_rx_bytes], temp_buf, copy_size);
                        total_rx_bytes += copy_size;
                    }
                } else {
                    break;
                }
            }
        }
    }

    close(fd_pc);
    close(fd_cam);
    return 0;
}