#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define PORT 2369 
#define PAYLOAD_SIZE 1206

int main() {
    int sockfd;
    struct sockaddr_in servaddr;
    uint8_t buffer[PAYLOAD_SIZE];

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // --- FIX: Allow broadcast packets ---
    int broadcast = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
        perror("Setsockopt (SO_BROADCAST) failed");
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY; 
    servaddr.sin_port = htons(PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    printf("Listening for VLP-16 broadcast on port %d...\n", PORT);

    while (1) {
        int n = recvfrom(sockfd, buffer, PAYLOAD_SIZE, 0, NULL, NULL);
        if (n == PAYLOAD_SIZE) {
            // Your parsing logic goes here...
            printf("Received packet, size: %d\n", n);
        }
    }
    close(sockfd);
    return 0;
}
