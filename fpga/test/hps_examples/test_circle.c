#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define HW_REGS_BASE 0xFF200000
#define HW_REGS_SPAN 0x00200000
#define HW_REGS_MASK (HW_REGS_SPAN - 1)

#define WRITE_PORT_OFFSET  0x00104000
#define READ_PORT_OFFSET   0x00108000
#define STREAM_PORT_OFFSET 0x0010C000

void print_heatmap_pixel(int weight) {
    if (weight <= 0) {
        printf("\x1b[48;2;0;0;0m  \x1b[0m"); 
    } else {
        if (weight > 100) weight = 100;
        int r, g, b;
        if (weight < 33) {
            r = (weight * 255) / 33;
            g = 0; b = 0;
        } else if (weight < 66) {
            r = 255;
            g = ((weight - 33) * 255) / 33;
            b = 0;
        } else {
            r = 255; g = 255;
            b = ((weight - 66) * 255) / 34;
        }
        printf("\x1b[48;2;%d;%d;%dm  \x1b[0m", r, g, b);
    }
}

int main() {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

    void *axi = mmap(NULL, HW_REGS_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, HW_REGS_BASE);
    
    // Stream port is 32-bit registers
    volatile uint32_t *acc_stream = (volatile uint32_t *)(axi + (STREAM_PORT_OFFSET & HW_REGS_MASK));
    
    // Grid memory is 8-bit OCC_ENTRY. Use 8-bit pointers to map 1-to-1 to your 10,000 cells!
    volatile uint8_t *write_port = (volatile uint8_t *)(axi + (WRITE_PORT_OFFSET & HW_REGS_MASK));
    volatile uint8_t *read_port  = (volatile uint8_t *)(axi + (READ_PORT_OFFSET & HW_REGS_MASK));

    // ==========================================
    // PHASE 1: SCRUB
    // ==========================================
    acc_stream[3] = 1; // HPS mode
    __sync_synchronize();
    usleep(1000);

    printf("Scrubbing 10,000 cells...\n");
    for(int i = 0; i < 10000; i++) write_port[i] = 0;
    __sync_synchronize();

    // ==========================================
    // PHASE 2: INJECT DATA
    // ==========================================
    acc_stream[3] = 0; // Hardware mode
    __sync_synchronize();
    usleep(1000);

    printf("Injecting 360-degree point cloud...\n");

    for (int i = 0; i < 360; i += 2) {
        uint16_t az1 = (i + 1) * 100;
        uint16_t d1  = 1000; 
        
        // THE FIX: Aim the lasers UP (ID 15) to bypass the ground filter!
        uint8_t  l1  = 15; 
        
        uint16_t az0 = i * 100;
        uint16_t d0  = 1000; 
        
        // THE FIX: Aim the lasers UP (ID 15) to bypass the ground filter!
        uint8_t  l0  = 15; 

        uint8_t valid = 3; 

        uint32_t reg0 = (l0 & 0xF) | ((l1 & 0xF) << 4) | ((uint32_t)az0 << 8) | (((uint32_t)az1 & 0xFF) << 24);
        uint32_t reg1 = ((uint32_t)az1 >> 8) | ((uint32_t)d0 << 8) | (((uint32_t)d1 & 0xFF) << 24);
        uint32_t reg2 = ((uint32_t)d1 >> 8) | ((uint32_t)valid << 8); 

        acc_stream[0] = reg0;
        acc_stream[1] = reg1;
        __sync_synchronize();
        
        acc_stream[2] = reg2; // Word 2 triggers the pipeline automatically
        __sync_synchronize();

        usleep(10); 
    }

    usleep(5000); 

    // ==========================================
    // PHASE 3: DATA EXTRACTION
    // ==========================================
    acc_stream[3] = 1; // Read mode
    __sync_synchronize();
    usleep(2000);

    printf("\n--- FPGA OUTPUT: 1000mm CIRCLE HEATMAP ---\n");
    int hit_count = 0;
    FILE *fp = fopen("output_map.txt", "w");

    for (int y = 99; y >= 0; y--) {
        for (int x = 0; x < 100; x++) {
            int addr = (y * 100) + x;
            
            // Double read to handle 1-cycle BRAM latency over Avalon bus
            volatile uint8_t dummy = read_port[addr];
            __sync_synchronize();
            uint8_t cell = read_port[addr];

            if (cell > 0) {
                hit_count++;
                if (fp) fprintf(fp, "%d %d %d\n", x, y, cell);
            }
            
            print_heatmap_pixel(cell);
        }
        printf("\x1b[48;2;50;50;50m  \x1b[0m\n");
    }

    if (fp) fclose(fp);
    printf("\n>>> Total Active Cells: %d <<<\n", hit_count);

    munmap(axi, HW_REGS_SPAN);
    close(mem_fd);
    return 0;
}
