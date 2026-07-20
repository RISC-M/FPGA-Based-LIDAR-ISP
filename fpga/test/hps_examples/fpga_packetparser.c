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
#define HW_REGS_BASE 0xFF200000 // LW HPS-to-FPGA Bridge
#define HW_REGS_SPAN 0x00005000

volatile uint32_t *fpga_ptr = NULL;
int fd;

// FPGA Bridge Initialization
int init_fpga_bridge() {
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) return -1;
    void *virtual_base = mmap(NULL, HW_REGS_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, HW_REGS_BASE);
    if (virtual_base == MAP_FAILED) {
        close(fd);
        return -1;
    }
    fpga_ptr = (uint32_t *)virtual_base;
    return 0;
}

void write_to_fpga(uint32_t offset, uint32_t value) {
    if (fpga_ptr) fpga_ptr[offset >> 2] = value;
}

int main() {
    int sockfd;
    struct sockaddr_in servaddr;
    uint8_t buffer[PAYLOAD_SIZE];

    // Initialize FPGA
    if (init_fpga_bridge() != 0) {
        perror("Failed to init FPGA bridge (did you use sudo?)");
        exit(EXIT_FAILURE);
    }
    printf("FPGA bridge initialized.\n");

    // Socket Setup
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    int broadcast = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    printf("Listening for LiDAR packets...\n");

    while (1) {
        int n = recvfrom(sockfd, buffer, PAYLOAD_SIZE, 0, NULL, NULL);
        if (n == PAYLOAD_SIZE) {
            // Example logic: Extract distance from a specific offset
            // Assuming first distance is at index 4 (Velodyne VLP-16 structure)
            uint16_t distance = buffer[4] | (buffer[5] << 8);
            
            // Send to FPGA
            write_to_fpga(0x0, distance); // Write distance to address 0x0
            
            printf("Distance: %u\n", distance);
        }
    }
    return 0;
}
