module lidar_accelerator (
    // ==========================================
    // FPGA Clock and Reset Pins
    // ==========================================
    input  logic        clk_clk,        // Connect to physical 50MHz pin (e.g., PIN_V11)
    input  logic        reset_reset_n,  // Connect to physical push button (e.g., PIN_AH17)

    // ==========================================
    // Your Custom LiDAR Fabric Connections
    // ==========================================
    input  logic [73:0] ARV_LIDAR_DATA,  // Your 74-bit packed input stream
    input  logic        ARV_HW_SWITCH,   // Your calibration toggle switch

    // ==========================================
    // HPS Dedicated DDR3 Memory Ports
    // (Names must match Qsys exactly for the TCL script)
    // ==========================================
    output logic [14:0] memory_mem_a,
    output logic [2:0]  memory_mem_ba,
    output logic        memory_mem_ck,
    output logic        memory_mem_ck_n,
    output logic        memory_mem_cke,
    output logic        memory_mem_cs_n,
    output logic        memory_mem_ras_n,
    output logic        memory_mem_cas_n,
    output logic        memory_mem_we_n,
    output logic        memory_mem_reset_n,
    inout  logic [31:0] memory_mem_dq,
    inout  logic [3:0]  memory_mem_dqs,
    inout  logic [3:0]  memory_mem_dqs_n,
    output logic        memory_mem_odt,
    output logic [3:0]  memory_mem_dm,
    input  logic        memory_oct_rzqin,

    // ==========================================
    // HPS Dedicated Peripheral I/O Ports
    // ==========================================
    output logic        hps_0_hps_io_hps_io_emac1_inst_TX_CLK,
    output logic        hps_0_hps_io_hps_io_emac1_inst_TXD0,
    output logic        hps_0_hps_io_hps_io_emac1_inst_TXD1,
    output logic        hps_0_hps_io_hps_io_emac1_inst_TXD2,
    output logic        hps_0_hps_io_hps_io_emac1_inst_TXD3,
    input  logic        hps_0_hps_io_hps_io_emac1_inst_RXD0,
    inout  logic        hps_0_hps_io_hps_io_emac1_inst_MDIO,
    output logic        hps_0_hps_io_hps_io_emac1_inst_MDC,
    input  logic        hps_0_hps_io_hps_io_emac1_inst_RX_CTL,
    output logic        hps_0_hps_io_hps_io_emac1_inst_TX_CTL,
    input  logic        hps_0_hps_io_hps_io_emac1_inst_RX_CLK,
    input  logic        hps_0_hps_io_hps_io_emac1_inst_RXD1,
    input  logic        hps_0_hps_io_hps_io_emac1_inst_RXD2,
    input  logic        hps_0_hps_io_hps_io_emac1_inst_RXD3,
    
    inout  logic        hps_0_hps_io_hps_io_sdio_inst_CMD,
    inout  logic        hps_0_hps_io_hps_io_sdio_inst_D0,
    inout  logic        hps_0_hps_io_hps_io_sdio_inst_D1,
    inout  logic        hps_0_hps_io_hps_io_sdio_inst_CLK,
    inout  logic        hps_0_hps_io_hps_io_sdio_inst_D2,
    inout  logic        hps_0_hps_io_hps_io_sdio_inst_D3,
    
    inout  logic        hps_0_hps_io_hps_io_usb1_inst_D0,
    inout  logic        hps_0_hps_io_hps_io_usb1_inst_D1,
    inout  logic        hps_0_hps_io_hps_io_usb1_inst_D2,
    inout  logic        hps_0_hps_io_hps_io_usb1_inst_D3,
    inout  logic        hps_0_hps_io_hps_io_usb1_inst_D4,
    inout  logic        hps_0_hps_io_hps_io_usb1_inst_D5,
    inout  logic        hps_0_hps_io_hps_io_usb1_inst_D6,
    inout  logic        hps_0_hps_io_hps_io_usb1_inst_D7,
    input  logic        hps_0_hps_io_hps_io_usb1_inst_CLK,
    output logic        hps_0_hps_io_hps_io_usb1_inst_STP,
    input  logic        hps_0_hps_io_hps_io_usb1_inst_DIR,
    input  logic        hps_0_hps_io_hps_io_usb1_inst_NXT,
    
    output logic        hps_0_hps_io_hps_io_spim1_inst_CLK,
    output logic        hps_0_hps_io_hps_io_spim1_inst_MOSI,
    input  logic        hps_0_hps_io_hps_io_spim1_inst_MISO,
    output logic        hps_0_hps_io_hps_io_spim1_inst_SS0,
    
    input  logic        hps_0_hps_io_hps_io_uart0_inst_RX,
    output logic        hps_0_hps_io_hps_io_uart0_inst_TX,
    
    inout  logic        hps_0_hps_io_hps_io_i2c0_inst_SDA,
    inout  logic        hps_0_hps_io_hps_io_i2c0_inst_SCL,
    inout  logic        hps_0_hps_io_hps_io_i2c1_inst_SDA,
    inout  logic        hps_0_hps_io_hps_io_i2c1_inst_SCL,
    
    inout  logic        hps_0_hps_io_hps_io_gpio_inst_GPIO09,
    inout  logic        hps_0_hps_io_hps_io_gpio_inst_GPIO35,
    inout  logic        hps_0_hps_io_hps_io_gpio_inst_GPIO40,
    inout  logic        hps_0_hps_io_hps_io_gpio_inst_GPIO53,
    inout  logic        hps_0_hps_io_hps_io_gpio_inst_GPIO54,
    inout  logic        hps_0_hps_io_hps_io_gpio_inst_GPIO61
);

    // Instantiate your Qsys system
    soc_system u0 (
        .clk_clk                               (clk_clk),                             
        .reset_reset_n                         (reset_reset_n),                       

        // Tie off unused fabric input reset requests
        .hps_0_f2h_cold_reset_req_reset_n     (1'b1),     
        .hps_0_f2h_debug_reset_req_reset_n    (1'b1),     
        .hps_0_f2h_stm_hw_events_stm_hwevents (28'b0),  
        .hps_0_f2h_warm_reset_req_reset_n     (1'b1),     
        .hps_0_h2f_reset_reset_n               (), // Open unless your fabric needs an HPS-controlled reset

        // Custom LiDAR Conduit Connection
        .lidar_raw_io_data                     (ARV_LIDAR_DATA),                    
        .lidar_raw_io_switch                   (ARV_HW_SWITCH),                  

        // External DDR3 Memory 
        .memory_mem_a                          (memory_mem_a),                         
        .memory_mem_ba                         (memory_mem_ba),                        
        .memory_mem_ck                         (memory_mem_ck),                        
        .memory_mem_ck_n                       (memory_mem_ck_n),                      
        .memory_mem_cke                        (memory_mem_cke),                       
        .memory_mem_cs_n                       (memory_mem_cs_n),                      
        .memory_mem_ras_n                      (memory_mem_ras_n),                     
        .memory_mem_cas_n                      (memory_mem_cas_n),                     
        .memory_mem_we_n                       (memory_mem_we_n),                      
        .memory_mem_reset_n                    (memory_mem_reset_n),                   
        .memory_mem_dq                         (memory_mem_dq),                        
        .memory_mem_dqs                        (memory_mem_dqs),                       
        .memory_mem_dqs_n                      (memory_mem_dqs_n),                     
        .memory_mem_odt                        (memory_mem_odt),                       
        .memory_mem_dm                         (memory_mem_dm),                        
        .memory_oct_rzqin                      (memory_oct_rzqin),                     

        // Board Peripherals
        .hps_0_hps_io_hps_io_emac1_inst_TX_CLK (hps_0_hps_io_hps_io_emac1_inst_TX_CLK), 
        .hps_0_hps_io_hps_io_emac1_inst_TXD0   (hps_0_hps_io_hps_io_emac1_inst_TXD0),   
        .hps_0_hps_io_hps_io_emac1_inst_TXD1   (hps_0_hps_io_hps_io_emac1_inst_TXD1),   
        .hps_0_hps_io_hps_io_emac1_inst_TXD2   (hps_0_hps_io_hps_io_emac1_inst_TXD2),   
        .hps_0_hps_io_hps_io_emac1_inst_TXD3   (hps_0_hps_io_hps_io_emac1_inst_TXD3),   
        .hps_0_hps_io_hps_io_emac1_inst_RXD0   (hps_0_hps_io_hps_io_emac1_inst_RXD0),   
        .hps_0_hps_io_hps_io_emac1_inst_MDIO   (hps_0_hps_io_hps_io_emac1_inst_MDIO),   
        .hps_0_hps_io_hps_io_emac1_inst_MDC    (hps_0_hps_io_hps_io_emac1_inst_MDC),    
        .hps_0_hps_io_hps_io_emac1_inst_RX_CTL (hps_0_hps_io_hps_io_emac1_inst_RX_CTL), 
        .hps_0_hps_io_hps_io_emac1_inst_TX_CTL (hps_0_hps_io_hps_io_emac1_inst_TX_CTL), 
        .hps_0_hps_io_hps_io_emac1_inst_RX_CLK (hps_0_hps_io_hps_io_emac1_inst_RX_CLK), 
        .hps_0_hps_io_hps_io_emac1_inst_RXD1   (hps_0_hps_io_hps_io_emac1_inst_RXD1),   
        .hps_0_hps_io_hps_io_emac1_inst_RXD2   (hps_0_hps_io_hps_io_emac1_inst_RXD2),   
        .hps_0_hps_io_hps_io_emac1_inst_RXD3   (hps_0_hps_io_hps_io_emac1_inst_RXD3),   
        
        .hps_0_hps_io_hps_io_sdio_inst_CMD     (hps_0_hps_io_hps_io_sdio_inst_CMD),     
        .hps_0_hps_io_hps_io_sdio_inst_D0      (hps_0_hps_io_hps_io_sdio_inst_D0),      
        .hps_0_hps_io_hps_io_sdio_inst_D1      (hps_0_hps_io_hps_io_sdio_inst_D1),      
        .hps_0_hps_io_hps_io_sdio_inst_CLK     (hps_0_hps_io_hps_io_sdio_inst_CLK),     
        .hps_0_hps_io_hps_io_sdio_inst_D2      (hps_0_hps_io_hps_io_sdio_inst_D2),      
        .hps_0_hps_io_hps_io_sdio_inst_D3      (hps_0_hps_io_hps_io_sdio_inst_D3),      
        
        .hps_0_hps_io_hps_io_usb1_inst_D0      (hps_0_hps_io_hps_io_usb1_inst_D0),      
        .hps_0_hps_io_hps_io_usb1_inst_D1      (hps_0_hps_io_hps_io_usb1_inst_D1),      
        .hps_0_hps_io_hps_io_usb1_inst_D2      (hps_0_hps_io_hps_io_usb1_inst_D2),      
        .hps_0_hps_io_hps_io_usb1_inst_D3      (hps_0_hps_io_hps_io_usb1_inst_D3),      
        .hps_0_hps_io_hps_io_usb1_inst_D4      (hps_0_hps_io_hps_io_usb1_inst_D4),      
        .hps_0_hps_io_hps_io_usb1_inst_D5      (hps_0_hps_io_hps_io_usb1_inst_D5),      
        .hps_0_hps_io_hps_io_usb1_inst_D6      (hps_0_hps_io_hps_io_usb1_inst_D6),      
        .hps_0_hps_io_hps_io_usb1_inst_D7      (hps_0_hps_io_hps_io_usb1_inst_D7),      
        .hps_0_hps_io_hps_io_usb1_inst_CLK     (hps_0_hps_io_hps_io_usb1_inst_CLK),     
        .hps_0_hps_io_hps_io_usb1_inst_STP     (hps_0_hps_io_hps_io_usb1_inst_STP),     
        .hps_0_hps_io_hps_io_usb1_inst_DIR     (hps_0_hps_io_hps_io_usb1_inst_DIR),     
        .hps_0_hps_io_hps_io_usb1_inst_NXT     (hps_0_hps_io_hps_io_usb1_inst_NXT),     
        
        .hps_0_hps_io_hps_io_spim1_inst_CLK    (hps_0_hps_io_hps_io_spim1_inst_CLK),    
        .hps_0_hps_io_hps_io_spim1_inst_MOSI   (hps_0_hps_io_hps_io_spim1_inst_MOSI),   
        .hps_0_hps_io_hps_io_spim1_inst_MISO   (hps_0_hps_io_hps_io_spim1_inst_MISO),   
        .hps_0_hps_io_hps_io_spim1_inst_SS0    (hps_0_hps_io_hps_io_spim1_inst_SS0),    
        
        .hps_0_hps_io_hps_io_uart0_inst_RX     (hps_0_hps_io_hps_io_uart0_inst_RX),     
        .hps_0_hps_io_hps_io_uart0_inst_TX     (hps_0_hps_io_hps_io_uart0_inst_TX),     
        
        .hps_0_hps_io_hps_io_i2c0_inst_SDA     (hps_0_hps_io_hps_io_i2c0_inst_SDA),     
        .hps_0_hps_io_hps_io_i2c0_inst_SCL     (hps_0_hps_io_hps_io_i2c0_inst_SCL),     
        .hps_0_hps_io_hps_io_i2c1_inst_SDA     (hps_0_hps_io_hps_io_i2c1_inst_SDA),     
        .hps_0_hps_io_hps_io_i2c1_inst_SCL     (hps_0_hps_io_hps_io_i2c1_inst_SCL),     
        
        .hps_0_hps_io_hps_io_gpio_inst_GPIO09  (hps_0_hps_io_hps_io_gpio_inst_GPIO09),  
        .hps_0_hps_io_hps_io_gpio_inst_GPIO35  (hps_0_hps_io_hps_io_gpio_inst_GPIO35),  
        .hps_0_hps_io_hps_io_gpio_inst_GPIO40  (hps_0_hps_io_hps_io_gpio_inst_GPIO40),  
        .hps_0_hps_io_hps_io_gpio_inst_GPIO53  (hps_0_hps_io_hps_io_gpio_inst_GPIO53),  
        .hps_0_hps_io_hps_io_gpio_inst_GPIO54  (hps_0_hps_io_hps_io_gpio_inst_GPIO54),  
        .hps_0_hps_io_hps_io_gpio_inst_GPIO61  (hps_0_hps_io_hps_io_gpio_inst_GPIO61)   
    );

endmodule
