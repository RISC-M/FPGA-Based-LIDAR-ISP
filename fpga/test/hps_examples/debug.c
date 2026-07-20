#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

// --- Lightweight AXI Bridge Defines ---
#define HW_REGS_BASE 0xFF200000
#define HW_REGS_SPAN 0x00200000 
#define HW_REGS_MASK (HW_REGS_SPAN - 1)

// --- Exact Qsys Memory Offsets ---
#define WRITE_PORT_OFFSET  0x00104000 // 8-bit width
#define READ_PORT_OFFSET   0x00108000 // 8-bit width
#define STREAM_PORT_OFFSET 0x0010C000 // 32-bit width

int main() {
    int fd = open("/dev/mem", (O_RDWR | O_SYNC));
    if (fd == -1) {
        printf("ERROR: could not open \"/dev/mem\"...\n");
        return 1;
    }

    void *axi_virtual_base = mmap(NULL, HW_REGS_SPAN, (PROT_READ | PROT_WRITE), MAP_SHARED, fd, HW_REGS_BASE);
    if (axi_virtual_base == MAP_FAILED) {
        printf("ERROR: mmap() failed for AXI Bridge...\n");
        close(fd);
        return 1;
    }

    printf("\n--- Testing Custom LiDAR Accelerator ---\n");

    // =================================================================
    // TEST 1: The 8-Bit Ports (Write & Read)
    // =================================================================
    volatile uint8_t *write_port_ptr = (volatile uint8_t *)(axi_virtual_base + (WRITE_PORT_OFFSET & HW_REGS_MASK));
    volatile uint8_t *read_port_ptr  = (volatile uint8_t *)(axi_virtual_base + (READ_PORT_OFFSET & HW_REGS_MASK));

    uint8_t test_val_8bit = 0xAB; 
    printf("\n[Test 1] Writing 8-bit value (0x%02X) to write_port (0x%08X)...\n", test_val_8bit, WRITE_PORT_OFFSET);
    *write_port_ptr = test_val_8bit; 
    printf("[Success] 8-bit write transaction finished safely.\n");

    printf("[Test 2] Reading 8-bit value from read_port (0x%08X)...\n", READ_PORT_OFFSET);
    uint8_t read_val_8bit = *read_port_ptr;
    printf("[Result] Read back 0x%02X\n", read_val_8bit);

    // =================================================================
    // TEST 3: The 32-Bit Stream Port (Write-Only)
    // =================================================================
    volatile uint32_t *stream_port_ptr = (volatile uint32_t *)(axi_virtual_base + (STREAM_PORT_OFFSET & HW_REGS_MASK));

    uint32_t test_val_32bit = 0xDEADBEEF;
    printf("\n[Test 3] Writing 32-bit value (0x%08X) to stream_port (0x%08X)...\n", test_val_32bit, STREAM_PORT_OFFSET);
    *stream_port_ptr = test_val_32bit; 
    printf("[Success] 32-bit stream transaction finished safely!\n");

    printf("\n--- Hardware Verification Complete ---\n\n");

    munmap(axi_virtual_base, HW_REGS_SPAN);
    close(fd);
    return 0;
}
