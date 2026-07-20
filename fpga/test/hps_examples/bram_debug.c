#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define HW_REGS_BASE 0xFF200000
#define HW_REGS_SPAN 0x00200000
#define HW_REGS_MASK (HW_REGS_SPAN - 1)

#define STREAM_PORT_OFFSET 0x00004000
#define GRID_PORT_OFFSET   0x00080000

int main() {
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

    void *axi = mmap(NULL, HW_REGS_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, HW_REGS_BASE);
    
    volatile uint32_t *acc_stream = (volatile uint32_t *)(axi + (STREAM_PORT_OFFSET & HW_REGS_MASK));
    
    // We map BOTH a 32-bit pointer and an 8-bit pointer to the exact same hardware address
    volatile uint32_t *grid_32 = (volatile uint32_t *)(axi + (GRID_PORT_OFFSET & HW_REGS_MASK));
    volatile uint8_t  *grid_8  = (volatile uint8_t  *)(axi + (GRID_PORT_OFFSET & HW_REGS_MASK));

    // Give HPS access to the bank
    acc_stream[3] = 1; 
    __sync_synchronize();
    usleep(1000);

    printf("--- RAW MEMORY DUMP (BEFORE WRITE) ---\n");
    for(int i = 0; i < 4; i++) {
        printf("Word [%d] (Offset 0x%02X): 0x%08X\n", i, i*4, grid_32[i]);
    }

    printf("\n--- WRITING TEST PATTERN (USING 8-BIT POINTER) ---\n");
    grid_8[0] = 0xAA;
    grid_8[1] = 0xBB;
    grid_8[2] = 0xCC;
    grid_8[3] = 0xDD;
    grid_8[4] = 0xEE;
    __sync_synchronize();

    printf("\n--- RAW MEMORY DUMP (AFTER 8-BIT WRITE) ---\n");
    for(int i = 0; i < 4; i++) {
        printf("Word [%d] (Offset 0x%02X): 0x%08X\n", i, i*4, grid_32[i]);
        printf("  Bytes: [0]=0x%02X, [1]=0x%02X, [2]=0x%02X, [3]=0x%02X\n",
               grid_8[i*4+0], grid_8[i*4+1], grid_8[i*4+2], grid_8[i*4+3]);
    }

    printf("\n--- WRITING TEST PATTERN (USING 32-BIT POINTER) ---\n");
    grid_32[0] = 0x11223344;
    grid_32[1] = 0x55667788;
    __sync_synchronize();

    printf("\n--- RAW MEMORY DUMP (AFTER 32-BIT WRITE) ---\n");
    for(int i = 0; i < 4; i++) {
        printf("Word [%d] (Offset 0x%02X): 0x%08X\n", i, i*4, grid_32[i]);
        printf("  Bytes: [0]=0x%02X, [1]=0x%02X, [2]=0x%02X, [3]=0x%02X\n",
               grid_8[i*4+0], grid_8[i*4+1], grid_8[i*4+2], grid_8[i*4+3]);
    }

    munmap(axi, HW_REGS_SPAN);
    close(mem_fd);
    return 0;
}
