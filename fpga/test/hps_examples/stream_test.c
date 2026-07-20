#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <stdint.h>

// ============================================================================
// CYCLONE V HARDWARE BASE ADDRESSES
// ============================================================================
#define ALT_LWFPGASLVS_OFST  0xFF200000 // Physical Base for Lightweight H2F Bridge
#define HW_REGS_SPAN         0x00200000 // 2 MB memory span (Expanded to reach grid)

#define ACC_STREAM_OFFSET    0x00000000 // Offset of accelerator streaming port
#define HPS_GRID_OFFSET      0x00080000 // Offset of double_buffer read port 

int main() {
    int fd;
    void *virtual_base;
    volatile uint32_t *acc_stream = NULL;
    volatile int8_t   *grid_mem   = NULL;

    // 1. Open the physical memory device
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        perror("ERROR: could not open \"/dev/mem\"");
        return 1;
    }

    // 2. Map the physical bridge addresses into user space
    virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), 
                        MAP_SHARED, fd, ALT_LWFPGASLVS_OFST);
    
    if (virtual_base == MAP_FAILED) {
        perror("ERROR: mmap() failed");
        close(fd);
        return 1;
    }

    // 3. Resolve pointer addresses
    acc_stream = (volatile uint32_t *)((uint8_t *)virtual_base + ACC_STREAM_OFFSET);
    grid_mem   = (volatile int8_t   *)((uint8_t *)virtual_base + HPS_GRID_OFFSET);

    printf("DE10-Nano Lidar Streaming Interface Initialized.\n");
    printf("Hardware edge-detector fences enabled.\n\n");

    // ============================================================================
    // BRAM SCRUBBING (Clear Power-On Garbage Data)
    // ============================================================================
    printf("Scrubbing garbage data from FPGA Block RAMs...\n");
    
    // Switch to Bank 0 for the HPS and clear it
    acc_stream[3] = 0;
    __sync_synchronize();
    usleep(10);
    for (int i = 0; i < 5000; i++) {
        grid_mem[i] = 0;
    }

    // Switch to Bank 1 for the HPS and clear it
    acc_stream[3] = 1;
    __sync_synchronize();
    usleep(10);
    for (int i = 0; i < 5000; i++) {
        grid_mem[i] = 0;
    }

    // Leave the switch on Bank 1 so the HPS reads Bank 1.
    // This forces the FPGA execution pipeline to write new data into Bank 0.
    acc_stream[3] = 1;
    __sync_synchronize();
    usleep(10);

    // ============================================================================
    // SIMULATED LIDAR STREAMING LOOP
    // ============================================================================
    printf("Streaming 5 consecutive points into the pipeline...\n");

    // We will inject 5 identical packets.
    // If the pipeline is working perfectly, the target cell should have a value
    // of exactly 100 (5 injections * 20 weight per hit).
    uint32_t word0 = 0x00000000; // Laser ID = 0, Azimuth[0] = 0
    uint32_t word1 = 0x01F40000; // Distance[0] = 500 mm, Azimuth[1] upper bits 
    uint32_t word2 = 0x00000300; // Valid flags = 2'b11, Dist[1] upper bits

    for (int i = 0; i < 5; i++) {
        // Write Word 0
        acc_stream[0] = word0;
        __sync_synchronize(); // FORCE bus to drop write-enable line
        
        // Write Word 1
        acc_stream[1] = word1;
        __sync_synchronize(); // FORCE bus to drop write-enable line
        
        // Write Word 2 (Triggers the 1-cycle pipeline execution)
        acc_stream[2] = word2;
        __sync_synchronize(); // FORCE bus to drop write-enable line
        
        usleep(5); 
    }

    printf("Streaming complete. Waiting for pipeline to drain...\n");
    usleep(50); // Give the pipeline time to finish the last Read-Modify-Write

    // ============================================================================
    // FRAME SWAP AND VERIFICATION
    // ============================================================================
    
    // Toggle the ping-pong buffer so the HPS can now read Bank 0 (where FPGA just wrote)
    printf("Swapping memory banks (Bank 0 active for HPS)...\n");
    acc_stream[3] = 0;
    __sync_synchronize();
    usleep(10); 

    printf("\n--- GRID MEMORY VERIFICATION ---\n");
    int hits_found = 0;
    int corruptions_found = 0;

    for (int i = 0; i < 5000; i++) {
        if (grid_mem[i] != 0) {
            hits_found++;
            printf("Hit at Cell [%d]: Weight = %d\n", i, grid_mem[i]);
            
            // If the value is anything other than 100 (5 * 20 weight), we have a multi-fire glitch.
            if (grid_mem[i] != 100) {
                corruptions_found++;
            }
        }
    }

    if (hits_found == 0) {
        printf("\nFAIL: No data found in the grid. Pipeline stalled or buffer didn't swap.\n");
    } else if (corruptions_found > 0) {
        printf("\nFAIL: %d cells had unexpected values. Bus stretching is still happening!\n", corruptions_found);
    } else {
        printf("\nSUCCESS! The pipeline perfectly accumulated 5 discrete points without multi-firing!\n");
    }

    // Cleanup
    if (munmap(virtual_base, HW_REGS_SPAN) != 0) {
        perror("ERROR: munmap() failed");
    }
    
    close(fd);
    return 0;
}
