#define _POSIX_C_SOURCE 200809L

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <unistd.h>

#define PORT 2369
#define PAYLOAD_SIZE 1206

#define HW_REGS_BASE 0xFF200000
#define HW_REGS_SPAN 0x00200000
#define HW_REGS_MASK (HW_REGS_SPAN - 1)

#define WRITE_PORT_OFFSET 0x00104000
#define READ_PORT_OFFSET 0x00108000
#define STREAM_PORT_OFFSET 0x0010C000

#define AZIMUTH_WRAP_THRESHOLD 18000
#define DISPLAY_HOLD_FRAMES 3
#define SOCKET_RECEIVE_BUFFER (4 * 1024 * 1024)

static volatile sig_atomic_t stop_requested = 0;
static int terminal_active = 0;

static void request_stop(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static void restore_terminal(void)
{
    if (terminal_active) {
        static const char restore_sequence[] = "\033[?25h\033[?1049l";
        (void)write(
            STDOUT_FILENO,
            restore_sequence,
            sizeof(restore_sequence) - 1);
        terminal_active = 0;
    }
}

static void initialize_terminal(void)
{
    if (!isatty(STDOUT_FILENO)) {
        return;
    }

    /* Use a private screen, clear it once, and hide the cursor. */
    static const char setup_sequence[] =
        "\033[?1049h"
        "\033[2J"
        "\033[H"
        "\033[?25l";

    (void)write(STDOUT_FILENO, setup_sequence, sizeof(setup_sequence) - 1);
    terminal_active = 1;
}

static void get_display_size(int *display_width, int *display_height)
{
    struct winsize terminal_size;
    int columns = 80;
    int rows = 24;

    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &terminal_size) == 0) {
        if (terminal_size.ws_col > 0) {
            columns = terminal_size.ws_col;
        }
        if (terminal_size.ws_row > 0) {
            rows = terminal_size.ws_row;
        }
    }

    /*
     * Reserve two header rows and one unused bottom row. Avoiding the final
     * terminal row prevents an automatic scroll when the frame is written.
     */
    /* One Braille character represents two horizontal source cells. */
    *display_width = columns < 50 ? columns : 50;
    *display_height = rows > 3 ? rows - 3 : 1;
    if (*display_height > 25) {
        *display_height = 25;
    }
}

static uint8_t braille_bit(int dot_x, int dot_y)
{
    static const uint8_t bits[4][2] = {
        {1U << 0, 1U << 3},
        {1U << 1, 1U << 4},
        {1U << 2, 1U << 5},
        {1U << 6, 1U << 7}
    };
    return bits[dot_y][dot_x];
}

static void append_braille(
    char *frame,
    size_t frame_size,
    size_t *frame_index,
    uint8_t pattern)
{
    if (pattern == 0) {
        if (*frame_index + 1 < frame_size) {
            frame[(*frame_index)++] = ' ';
        }
        return;
    }

    /* UTF-8 encoding of U+2800 plus the eight-dot Braille pattern. */
    uint32_t codepoint = 0x2800U + pattern;
    if (*frame_index + 3 < frame_size) {
        frame[(*frame_index)++] = (char)(0xE0U | (codepoint >> 12));
        frame[(*frame_index)++] =
            (char)(0x80U | ((codepoint >> 6) & 0x3FU));
        frame[(*frame_index)++] = (char)(0x80U | (codepoint & 0x3FU));
    }
}

static void update_persistent_grid(
    const uint8_t new_grid[100][100],
    uint8_t display_grid[100][100],
    uint8_t display_age[100][100])
{
    for (int y = 0; y < 100; y++) {
        for (int x = 0; x < 100; x++) {
            if (new_grid[y][x] > 0) {
                /*
                 * Keep the strongest recent observation. This prevents a
                 * cell from oscillating between density characters when
                 * consecutive rotations contain different hit counts.
                 */
                if (new_grid[y][x] > display_grid[y][x]) {
                    display_grid[y][x] = new_grid[y][x];
                }
                display_age[y][x] = DISPLAY_HOLD_FRAMES;
            } else if (display_age[y][x] > 0) {
                display_age[y][x]--;
                if (display_age[y][x] == 0) {
                    display_grid[y][x] = 0;
                }
            } else {
                display_grid[y][x] = 0;
            }
        }
    }
}

static void render_grid(
    const uint8_t grid[100][100],
    char *frame_buf,
    size_t frame_buf_size)
{
    int display_width;
    int display_height;
    get_display_size(&display_width, &display_height);

    size_t buf_idx = 0;
    int written = snprintf(
        frame_buf,
        frame_buf_size,
        "\033[H"
        "LIVE FPGA OCCUPANCY GRID - Ctrl-C to exit\033[K\r\n"
        "Braille: %dx%d cells  hold:%d rotations  each dot=occupied\033[K\r\n",
        display_width * 2,
        display_height * 4,
        DISPLAY_HOLD_FRAMES);
    if (written < 0) {
        return;
    }
    buf_idx = (size_t)written;

    /*
     * Downsample by retaining the densest source cell covered by each
     * terminal character. Source Y is reversed so north remains at the top.
     */
    int virtual_width = display_width * 2;
    int virtual_height = display_height * 4;
    for (int display_y = 0; display_y < display_height; display_y++) {
        for (int display_x = 0; display_x < display_width; display_x++) {
            uint8_t pattern = 0;

            for (int dot_y = 0; dot_y < 4; dot_y++) {
                int virtual_y = display_y * 4 + dot_y;
                int source_y_begin = (virtual_y * 100) / virtual_height;
                int source_y_end =
                    ((virtual_y + 1) * 100) / virtual_height;

                for (int dot_x = 0; dot_x < 2; dot_x++) {
                    int virtual_x = display_x * 2 + dot_x;
                    int source_x_begin =
                        (virtual_x * 100) / virtual_width;
                    int source_x_end =
                        ((virtual_x + 1) * 100) / virtual_width;
                    int occupied = 0;

                    for (int source_y_index = source_y_begin;
                         source_y_index < source_y_end && !occupied;
                         source_y_index++) {
                        int source_y = 99 - source_y_index;
                        for (int source_x = source_x_begin;
                             source_x < source_x_end;
                             source_x++) {
                            if (grid[source_y][source_x] > 0) {
                                occupied = 1;
                                break;
                            }
                        }
                    }

                    if (occupied) {
                        pattern |= braille_bit(dot_x, dot_y);
                    }
                }
            }

            append_braille(
                frame_buf,
                frame_buf_size,
                &buf_idx,
                pattern);
        }

        if (buf_idx + 5 < frame_buf_size) {
            memcpy(frame_buf + buf_idx, "\033[K", 3);
            buf_idx += 3;
            if (display_y + 1 < display_height) {
                frame_buf[buf_idx++] = '\r';
                frame_buf[buf_idx++] = '\n';
            }
        }
    }

    /* Remove remnants after a terminal resize without clearing the frame. */
    if (buf_idx + 7 < frame_buf_size) {
        memcpy(frame_buf + buf_idx, "\033[J\033[H", 6);
        buf_idx += 6;
    }

    (void)fwrite(frame_buf, 1, buf_idx, stdout);
    (void)fflush(stdout);
}

int main(void)
{
    int sockfd;
    int mem_fd;
    struct sockaddr_in servaddr;
    uint8_t buffer[PAYLOAD_SIZE];

    /* Initialize the HPS-to-FPGA memory mapping. */
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) {
        perror("ERROR: could not open \"/dev/mem\"");
        return EXIT_FAILURE;
    }

    void *axi_virtual_base = mmap(
        NULL,
        HW_REGS_SPAN,
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        mem_fd,
        HW_REGS_BASE);
    if (axi_virtual_base == MAP_FAILED) {
        perror("ERROR: mmap failed");
        close(mem_fd);
        return EXIT_FAILURE;
    }

    uint8_t *axi_base = (uint8_t *)axi_virtual_base;
    volatile uint32_t *stream_port_ptr =
        (volatile uint32_t *)(axi_base + (STREAM_PORT_OFFSET & HW_REGS_MASK));
    volatile uint8_t *read_port_ptr =
        (volatile uint8_t *)(axi_base + (READ_PORT_OFFSET & HW_REGS_MASK));
    volatile uint8_t *write_port_ptr =
        (volatile uint8_t *)(axi_base + (WRITE_PORT_OFFSET & HW_REGS_MASK));

    /* Initialize the UDP socket used by the VLP-16 stream. */
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd == -1) {
        perror("ERROR: socket creation failed");
        munmap(axi_virtual_base, HW_REGS_SPAN);
        close(mem_fd);
        return EXIT_FAILURE;
    }

    int broadcast = 1;
    if (setsockopt(
            sockfd,
            SOL_SOCKET,
            SO_BROADCAST,
            &broadcast,
            sizeof(broadcast)) == -1) {
        perror("ERROR: setsockopt failed");
        close(sockfd);
        munmap(axi_virtual_base, HW_REGS_SPAN);
        close(mem_fd);
        return EXIT_FAILURE;
    }

    int receive_buffer_size = SOCKET_RECEIVE_BUFFER;
    if (setsockopt(
            sockfd,
            SOL_SOCKET,
            SO_RCVBUF,
            &receive_buffer_size,
            sizeof(receive_buffer_size)) == -1) {
        perror("WARNING: could not enlarge UDP receive buffer");
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);

    if (bind(
            sockfd,
            (const struct sockaddr *)&servaddr,
            sizeof(servaddr)) == -1) {
        perror("ERROR: bind failed");
        close(sockfd);
        munmap(axi_virtual_base, HW_REGS_SPAN);
        close(mem_fd);
        return EXIT_FAILURE;
    }

    printf("Waiting for VLP-16 stream...\n");

    int prev_azimuth = -1;
    int current_bank = 0;
    int exit_status = EXIT_SUCCESS;
    char frame_buf[12000];
    uint8_t occupancy_grid[100][100];
    uint8_t display_grid[100][100] = {{0}};
    uint8_t display_age[100][100] = {{0}};

    struct sigaction stop_action;
    memset(&stop_action, 0, sizeof(stop_action));
    stop_action.sa_handler = request_stop;
    sigemptyset(&stop_action.sa_mask);
    if (sigaction(SIGINT, &stop_action, NULL) == -1 ||
        sigaction(SIGTERM, &stop_action, NULL) == -1) {
        perror("WARNING: could not install signal handlers");
    }

    if (atexit(restore_terminal) != 0) {
        fprintf(stderr, "WARNING: could not register terminal cleanup\n");
    }
    initialize_terminal();

    while (!stop_requested) {
        ssize_t n = recvfrom(
            sockfd,
            buffer,
            sizeof(buffer),
            0,
            NULL,
            NULL);
        if (n == -1) {
            if (errno == EINTR && stop_requested) {
                break;
            }
            perror("ERROR: recvfrom failed");
            exit_status = EXIT_FAILURE;
            break;
        }
        if (n != PAYLOAD_SIZE) {
            continue;
        }

        /* A VLP-16 packet contains 12 data blocks. */
        for (int b = 0; b < 12; b++) {
            int offset = b * 100;

            /* Verify the little-endian 0xEEFF block flag. */
            uint16_t flag =
                (uint16_t)buffer[offset] |
                ((uint16_t)buffer[offset + 1] << 8);
            if (flag != 0xEEFF) {
                continue;
            }

            /* Azimuth is represented in hundredths of a degree. */
            uint16_t azimuth =
                (uint16_t)buffer[offset + 2] |
                ((uint16_t)buffer[offset + 3] << 8);

            /*
             * A decreasing azimuth marks the end of a rotation. Switch
             * buffers, read the completed occupancy grid, display it, and
             * clear it for reuse.
             */
            /*
             * A real rotation boundary is a large backwards jump near the
             * 360-to-0 transition. Ignore small backwards changes caused by
             * UDP packet reordering; rendering those would produce partial
             * frames.
             */
            int azimuth_wrapped =
                prev_azimuth != -1 &&
                prev_azimuth > (int)azimuth &&
                (prev_azimuth - (int)azimuth) > AZIMUTH_WRAP_THRESHOLD;

            if (azimuth_wrapped) {
                current_bank = !current_bank;
                stream_port_ptr[3] = (uint32_t)current_bank;

                for (int y = 99; y >= 0; y--) {
                    for (int x = 0; x < 100; x++) {
                        int addr = (y * 100) + x;

                        /* Account for the Avalon/BRAM one-cycle latency. */
                        volatile uint8_t dummy = read_port_ptr[addr];
                        (void)dummy;
                        __sync_synchronize();
                        uint8_t cell = read_port_ptr[addr];

                        occupancy_grid[y][x] = cell;

                        if (cell > 0) {
                            write_port_ptr[addr] = 0;
                        }
                    }
                }

                update_persistent_grid(
                    occupancy_grid,
                    display_grid,
                    display_age);
                render_grid(
                    display_grid,
                    frame_buf,
                    sizeof(frame_buf));
            }
            prev_azimuth = azimuth;

            /* Stream both firings (32 points) into the FPGA accelerator. */
            for (int i = 0; i < 16; i++) {
                int pt0_idx = offset + 4 + (i * 3);
                int pt1_idx = offset + 4 + ((i + 16) * 3);

                uint16_t dist0 =
                    (uint16_t)buffer[pt0_idx] |
                    ((uint16_t)buffer[pt0_idx + 1] << 8);
                uint16_t dist1 =
                    (uint16_t)buffer[pt1_idx] |
                    ((uint16_t)buffer[pt1_idx + 1] << 8);

                uint8_t valid = 0;
                if (dist0 > 0) {
                    valid |= 1;
                }
                if (dist1 > 0) {
                    valid |= 2;
                }

                if (valid != 0) {
                    uint32_t reg0 =
                        ((uint32_t)i & 0xF) |
                        (((uint32_t)i & 0xF) << 4) |
                        (((uint32_t)azimuth & 0xFFFF) << 8) |
                        (((uint32_t)azimuth & 0xFF) << 24);
                    uint32_t reg1 =
                        (((uint32_t)azimuth >> 8) & 0xFF) |
                        (((uint32_t)dist0 & 0xFFFF) << 8) |
                        (((uint32_t)dist1 & 0xFF) << 24);
                    uint32_t reg2 =
                        (((uint32_t)dist1 >> 8) & 0xFF) |
                        (((uint32_t)valid & 0x3) << 8);

                    stream_port_ptr[0] = reg0;
                    stream_port_ptr[1] = reg1;
                    __sync_synchronize();

                    /* Writing word 2 triggers the FPGA pipeline. */
                    stream_port_ptr[2] = reg2;
                    __sync_synchronize();
                }
            }
        }
    }

    close(sockfd);
    munmap(axi_virtual_base, HW_REGS_SPAN);
    close(mem_fd);
    return exit_status;
}
