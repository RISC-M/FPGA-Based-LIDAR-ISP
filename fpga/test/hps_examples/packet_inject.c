#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdint.h>  // <-- Fixes 'uint32_t' and 'int8_t' errors!

// ============================================================================
// CYCLONE V HARDWARE BASE ADDRESSES
// ============================================================================
#define ALT_LWFPGASLVS_OFST  0xFF200000 // Physical Base for Lightweight H2F Bridge
#define HW_REGS_SPAN         0x00080000 // 512 KB memory span

// Replace these offsets with the exact values from your Quartus hps_0.h header
#define ACC_STREAM_OFFSET    0x00000000 // Offset of accelerator_0 on the LW bridge
#define HPS_GRID_OFFSET      0x00004000 // Offset of double_buffer read port 

int main() {
    int fd;
    void *virtual_base;
    volatile uint32_t *acc_stream = NULL;
    volatile int8_t   *grid_mem    = NULL;

    // 1. Open the physical memory device
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        perror("ERROR: could not open \"/dev/mem\"");
        return 1;
    }

    // 2. Map the physical bridge addresses into our application user space
    virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), 
                        MAP_SHARED, fd, ALT_LWFPGASLVS_OFST);
    
    if (virtual_base == MAP_FAILED) {
        perror("ERROR: mmap() failed");
        close(fd);
        return 1;
    }

    // 3. Resolve the actual pointer addresses using absolute pointer math
    acc_stream = (volatile uint32_t *)((uint8_t *)virtual_base + ACC_STREAM_OFFSET);
    grid_mem    = (volatile int8_t   *)((uint8_t *)virtual_base + HPS_GRID_OFFSET);

    printf("DE10-Nano Diagnostic Probe Online.\n");
    printf("Streaming Port Mapped to virtual address: %p\n", (void *)acc_stream);
    printf("Grid Memory Port Mapped to virtual address: %p\n", (void *)grid_mem);

    // ============================================================================
    // EXECUTE PROBE METHOD 1 & 2: SIGNATURE PACKET INJECTION
    // ============================================================================
    printf("\nInjecting a predictable Signature Packet into the pipeline...\n");

    // We choose a configuration where Azimuth = 0 degrees (Cos=1, Sin=0)
    // This allows bypass of complex scaling math bugs to see if logic works.
    uint32_t word0 = 0x00000000; // Laser ID = 0, Azimuth[0] = 0
    uint32_t word1 = 0x01F40000; // Distance[0] = 500 mm (0x01F4 = 500), Azimuth[1] upper bits 
    uint32_t word2 = 0x00000300; // Valid flags = 2'b11 (0x300 triggers validation), Dist[1] upper

    // Sequentially write into the deserializer registers
    acc_stream[0] = word0;
    acc_stream[1] = word1;
    __sync_synchronize();        // Compiler and hardware memory barrier
    
    acc_stream[2] = word2;        // Word 2 written -> stream_valid pulses high in FPGA!
    __sync_synchronize();

    // Give the hardware pipeline 10 microseconds to calculate and commit to RAM
    usleep(10); 

    // Flip the double buffer switch (Reg 3) to freeze the frame and pass control to HPS
    printf("Toggling double buffer ping-pong switch...\n");
    acc_stream[3] = 1;
    __sync_synchronize();
    usleep(10);

    // Scan your HPS Read grid to see if the weight registered
    // We expect the point to land inside the memory structure.
    printf("Scanning grid memory for hardware feedback...\n");
    
    int hits_found = 0;
    // Iterate through the bank structure (`MEM_DEPTH` is 5000 entries total across banks)
    for (int i = 0; i < 5000; i++) {
        if (grid_mem[i] != 0) {
            printf("Found Pipeline Output! Cell Index: %d, Value Added: %d\n", i, grid_mem[i]);
            hits_found++;
        }
    }

    if (hits_found == 0) {
        printf("\nDIAGNOSIS RESULT: PIPELINE DROPPED PACKET.\n");
        printf("The hardware absorbed the writes but didn't write to memory.\n");
        printf("Check if 'clk' or 'rst_n' are floating or disconnected in Platform Designer.\n");
    } else {
        printf("\nDIAGNOSIS RESULT: BUS AND CONTROL STRUCURE IS ALIVE!\n");
        printf("Detected %d modified cell(s).\n", hits_found);
    }

    // Clean up memory maps
    if (munmap(virtual_base, HW_REGS_SPAN) != 0) {
        perror("ERROR: munmap() failed");
    }
    
    close(fd);
    return 0;
}
