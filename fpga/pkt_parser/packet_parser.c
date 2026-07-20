#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

// Updated to match your hardware's specific configuration
#define PORT 2369 
#define PAYLOAD_SIZE 1206

// VLP-16 Elevation angles mapped by channel ID (0-15).
// Stored in hundredths of a degree (e.g., -15.00 degrees = -1500)
// Cast to uint16_t using two's complement for seamless hardware transfer.
const int16_t ELEVATION_LUT[16] = {
    -1500,   100, -1300,   300, -1100,   500,  -900,   700,
     -700,   900,  -500,  1100,  -300,  1300,  -100,  1500
};

int main() {
    int sockfd;
    struct sockaddr_in servaddr;
    uint8_t buffer[PAYLOAD_SIZE];

    // 1. Create UDP socket
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    // 2. Bind the socket
    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    printf("Listening for VLP-16 UDP packets on port %d...\n", PORT);

    // 3. Continuous parsing loop
    while (1) {
        int n = recvfrom(sockfd, buffer, PAYLOAD_SIZE, MSG_WAITALL, NULL, NULL);

        if (n == PAYLOAD_SIZE) {
            
            // Iterate through the 12 data blocks in the packet
            for (int block = 0; block < 12; block++) {
                int block_offset = block * 100;

                // Verify the Velodyne block flag (0xFFEE little-endian)
                if (buffer[block_offset] == 0xFF && buffer[block_offset + 1] == 0xEE) {
                    
                    // Extract Azimuth (hundredths of a degree)
                    uint16_t azimuth = buffer[block_offset + 2] | (buffer[block_offset + 3] << 8);

                    // Each block contains 2 firing sequences of the 16 lasers
                    for (int seq = 0; seq < 2; seq++) {
                        for (int channel = 0; channel < 16; channel++) {
                            
                            // Calculate exact byte offset for this specific laser return
                            int data_offset = block_offset + 4 + (seq * 48) + (channel * 3);
                            
                            // Extract Distance (2mm increments)
                            uint16_t distance = buffer[data_offset] | (buffer[data_offset + 1] << 8);
                            
                            // Map Elevation from the lookup table
                            uint16_t elevation = (uint16_t)ELEVATION_LUT[channel];

                            // Filter out empty returns
                            if (distance > 0) {
                                
                                // Print just the first laser of the first block to avoid console flood
                                if (block == 0 && seq == 0 && channel == 0) {
                                    printf("Azimuth: %5u | Distance: %5u | Elevation: %5u\n", 
                                           azimuth, distance, elevation);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    close(sockfd);
    return 0;
}
