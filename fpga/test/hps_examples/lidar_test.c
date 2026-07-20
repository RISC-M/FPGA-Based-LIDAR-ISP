#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>

// Cyclone V Lightweight HPS-to-FPGA Bridge
#define HW_REGS_BASE 0xFF200000
#define HW_REGS_SPAN 0x00200000 // 2MB span covers the full bridge range
#define HW_REGS_MASK (HW_REGS_SPAN - 1)

// Your locked Qsys Base Addresses
#define WRITE_PORT_OFFSET 0x00104000
#define READ_PORT_OFFSET  0x00108000
#define STREAM_PORT_OFFSET 0x0010C000

int main() {
    int fd;
    void *virtual_base;
    volatile uint32_t *write_ptr;
    volatile uint32_t *read_ptr;
    volatile uint32_t *stream_ptr;

    // 1. Open the physical memory device
    if ((fd = open("/dev/mem", (O_RDWR | O_SYNC))) == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

    // 2. Map the physical AXI bridge to a virtual address
    virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, HW_REGS_BASE);
    if (virtual_base == MAP_FAILED) {
        printf("ERROR: mmap() failed...\n");
        close(fd);
        return 1;
    }

    // 3. Calculate pointers using the Qsys offsets
    write_ptr  = (uint32_t *)(virtual_base + (WRITE_PORT_OFFSET & HW_REGS_MASK));
    read_ptr   = (uint32_t *)(virtual_base + (READ_PORT_OFFSET & HW_REGS_MASK));
    stream_ptr = (uint32_t *)(virtual_base + (STREAM_PORT_OFFSET & HW_REGS_MASK));

    printf("Memory map complete. Accessing LiDAR Accelerator...\n");

    // --- TEST SEQUENCE ---

    // Write a test value to the Write Port
    uint32_t test_val = 0xDEADBEEF;
    *write_ptr = test_val;
    printf("Wrote 0x%08X to Write Port at 0x%08X\n", test_val, WRITE_PORT_OFFSET);

    // Read back to verify communication
    // Note: If your accelerator is standard BRAM, you might read back what you wrote.
    uint32_t read_val = *read_ptr;
    printf("Read back from Read Port: 0x%08X\n", read_val);

    // ---------------------

    // 4. Cleanup
    if (munmap(virtual_base, HW_REGS_SPAN) != 0) {
        printf("ERROR: munmap() failed...\n");
        close(fd);
        return 1;
    }
    close(fd);
    
    printf("Test finished successfully.\n");
    return 0;
}
