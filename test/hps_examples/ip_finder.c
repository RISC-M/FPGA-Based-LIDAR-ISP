#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/udp.h>

int main() {
    int raw_socket = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (raw_socket < 0) {
        printf("Error: Run as root!\n");
        return 1;
    }

    unsigned char buffer[2048];
    printf("Listening to EVERYTHING on the physical wire...\n");

    while (1) {
        int n = recvfrom(raw_socket, buffer, 2048, 0, NULL, NULL);
        if (n < 0) continue;

        struct ethhdr *eth = (struct ethhdr *)buffer;
        
        // Only look at standard IPv4 packets (EtherType 0x0800)
        if (ntohs(eth->h_proto) == 0x0800) {
            
            struct iphdr *iph = (struct iphdr *)(buffer + sizeof(struct ethhdr));
            struct in_addr source_ip, dest_ip;
            source_ip.s_addr = iph->saddr;
            dest_ip.s_addr = iph->daddr;

            printf("\n>>> PACKET CAUGHT! <<<\n");
            printf("Source IP (LiDAR) : %s\n", inet_ntoa(source_ip));
            printf("Dest IP (Target)  : %s\n", inet_ntoa(dest_ip));
            printf("Protocol          : %d (17 = UDP)\n", iph->protocol);

            // If it is UDP, extract the port
            if (iph->protocol == 17) {
                struct udphdr *udph = (struct udphdr *)(buffer + sizeof(struct ethhdr) + (iph->ihl * 4));
                printf("Target UDP Port   : %d\n", ntohs(udph->dest));
                break; // Exit after finding the first UDP payload
            }
        }
    }
    return 0;
}
