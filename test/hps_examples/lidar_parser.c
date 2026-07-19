#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <sys/mman.h>

#define PORT 2369
#define PAYLOAD_SIZE 1206

#define HW_REGS_BASE 0xFF200000
#define HW_REGS_SPAN 0x00200000
#define HW_REGS_MASK (HW_REGS_SPAN - 1)

#define WRITE_PORT_OFFSET  0x00104000
#define READ_PORT_OFFSET   0x00108000
#define STREAM_PORT_OFFSET 0x0010C000

int main() {
    int sockfd, mem_fd;
    struct sockaddr_in servaddr;
    uint8_t buffer[PAYLOAD_SIZE];

    // --- 1. INITIALIZE MEMORY ---
    if ((mem_fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        perror("ERROR: could not open \"/dev/mem\""); return 1;
    }
    void *axi_virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, mem_fd, HW_REGS_BASE);
    if (axi_virtual_base == MAP_FAILED) return 1;

    volatile uint32_t *stream_port_ptr = (volatile uint32_t *)(axi_virtual_base + (STREAM_PORT_OFFSET & HW_REGS_MASK));
    volatile uint8_t  *read_port_ptr   = (volatile uint8_t  *)(axi_virtual_base + (READ_PORT_OFFSET & HW_REGS_MASK));
    volatile uint8_t  *write_port_ptr  = (volatile uint8_t  *)(axi_virtual_base + (WRITE_PORT_OFFSET & HW_REGS_MASK));

    // --- 2. INITIALIZE SOCKET ---
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    int broadcast = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);
    bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr));

    printf("Waiting for VLP-16 stream...\n");

    int prev_azimuth = -1;
    int current_bank = 0; // Explicitly track the active bank

    // Allocate a large buffer for the terminal frame (100x100 + newlines + escape codes)
    char frame_buf[12000]; 

    // --- 3. MAIN PARSING & VISUALIZATION LOOP ---
    while (1) {
        int n = recvfrom(sockfd, buffer, PAYLOAD_SIZE, 0, NULL, NULL);
        if (n == PAYLOAD_SIZE) {

            // A VLP-16 packet contains 12 data blocks
            for (int b = 0; b < 12; b++) {
                int offset = b * 100;

                // Verify the 0xEEFF block flag
                uint16_t flag = buffer[offset] | (buffer[offset+1] << 8);
                if (flag != 0xEEFF) continue;

                // Extract Azimuth (0 to 35999)
                uint16_t azimuth = buffer[offset+2] | (buffer[offset+3] << 8);

                // ==========================================
                //  SWAP BUFFERS AND RENDER ON ROTATION END
                // ==========================================
                if (prev_azimuth != -1 && azimuth < prev_azimuth) {

                    // 1. Direct Bank Assignment (Requires the Verilog double_buffer fix)
                    current_bank = !current_bank;
                    stream_port_ptr[3] = current_bank; 

                    // 2. Prepare the render buffer
                    int buf_idx = 0;
                    buf_idx += sprintf(frame_buf + buf_idx, "\033[2J\033[H--- LIVE OCCUPANCY GRID (100x100) ---\n");

                    // 3. Read out the 100x100 memory map & clear it
                    for (int y = 99; y >= 0; y--) {
                        for (int x = 0; x < 100; x++) {
                            int addr = (y * 100) + x;
                            uint8_t cell = read_port_ptr[addr];

                            // Visualize cell density into the buffer
                            if (cell > 50)      frame_buf[buf_idx++] = '#';
                            else if (cell > 20) frame_buf[buf_idx++] = 'o';
                            else if (cell > 0)  frame_buf[buf_idx++] = '.';
                            else                frame_buf[buf_idx++] = ' ';

                            // Clear the cell for the next rotation
                            if (cell > 0) {
                                write_port_ptr[addr] = 0;
                            }
                        }
                        frame_buf[buf_idx++] = '\n'; // Next row
                    }
                    frame_buf[buf_idx] = '\0'; // Null terminate

                    // 4. Blast the entire frame to the terminal at once
                    printf("%s", frame_buf);
                    fflush(stdout); 
                }
                prev_azimuth = azimuth;

                // ==========================================
                //  STREAM 32 POINTS INTO ACCELERATOR
                // ==========================================
                for (int i = 0; i < 16; i++) {

                    int pt0_idx = offset + 4 + (i * 3);          // Firing 1
                    int pt1_idx = offset + 4 + ((i + 16) * 3);   // Firing 2

                    uint16_t dist0 = buffer[pt0_idx] | (buffer[pt0_idx+1] << 8);
                    uint16_t dist1 = buffer[pt1_idx] | (buffer[pt1_idx+1] << 8);

                    uint8_t valid = 0;
                    if (dist0 > 0) valid |= 1;
                    if (dist1 > 0) valid |= 2;

                    if (valid) {
                        uint32_t reg0 = (i & 0xF) | ((i & 0xF) << 4) | ((azimuth & 0xFFFF) << 8) | ((azimuth & 0xFF) << 24);
                        uint32_t reg1 = ((azimuth >> 8) & 0xFF) | ((dist0 & 0xFFFF) << 8) | ((dist1 & 0xFF) << 24);
                        uint32_t reg2 = ((dist1 >> 8) & 0xFF) | ((valid & 0x3) << 8);

                        stream_port_ptr[0] = reg0;
                        stream_port_ptr[1] = reg1;
                        stream_port_ptr[2] = reg2; 
                    }
                }
            }
        }
    }
    return 0;
}
