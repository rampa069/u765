#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cstring>
#include "Vu765_test.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

double sc_time_stamp() {
    return 0;
}

static Vu765_test *tb;
static VerilatedVcdC *trace;
static int tickcount;

static unsigned char sdbuf[512];
static FILE *edsk;
static int reading;
static int read_ptr;

// Structure for tracking scan operation results
struct ScanResult {
    int timestamp;
    int status0;
    int status1;
    int status2;
    std::string operation;
    std::string result;
    bool match_found;
};

// Log of scan operations for analysis
static std::vector<ScanResult> scan_results;

// Buffer for test data to be compared with sector data
static uint8_t compare_data[512];

// Flags for TC and interrupts
static bool tc_active = false;
static bool int_out_active = false;
static bool int_out_previous = false;

// Function to read a block from the disk image
int img_read(int sd_rd) {
    if (!sd_rd) return 0;
    printf("img_read: %02x lba: %d\n", sd_rd, tb->sd_lba);
    int lba = tb->sd_lba;
    fseek(edsk, lba << 9, SEEK_SET);
    fread(&sdbuf, 512, 1, edsk);
    reading = 1;
    read_ptr = 0;
    return 0;
}

// Basic clock cycle
void tick(int c) {
    static int sd_rd = 0;
    static int sd_wr = 0;

    tb->clk_sys = c;
    
    // Handle TC signal
    tb->tc = tc_active ? 1 : 0;
    
    // Detect rising edges on int_out
    int_out_previous = int_out_active;
    int_out_active = tb->int_out;
    
    tb->eval();
    trace->dump(tickcount++);

    if (c) {
        if (reading) {
            tb->sd_ack = 1;
            tb->sd_buff_wr = 1;
            tb->sd_buff_dout = sdbuf[read_ptr];
            tb->sd_buff_addr = read_ptr;
            read_ptr++;
            if (read_ptr == 512) reading = 0;
        } else {
            tb->sd_ack = 0;
            tb->sd_buff_wr = 0;
        }

        if (sd_rd != tb->sd_rd) img_read(tb->sd_rd);
        sd_rd = tb->sd_rd;
        
        // Handle SD write operations (for completeness)
        if (tb->sd_wr && !sd_wr) {
            printf("SD Write request to LBA %d\n", tb->sd_lba);
            tb->sd_ack = 1; 
        } else if (!tb->sd_wr && sd_wr) {
            tb->sd_ack = 0;
        }
        sd_wr = tb->sd_wr;
    }
}

void wait(int t) {
    for (int i=0; i<t; i++) {
        tick(1);
        tick(0);
    }
}

// Read the u765 status register
int readstatus() {
    int dout;

    tb->a0 = 0;
    tick(1);
    tick(0);
    tb->nRD = 0;
    tb->nWR = 1;
    tick(1);
    tick(0);
    tick(1);
    tick(0);
    dout = tb->dout;
    tb->nRD = 1;
    tick(1);
    tick(0);
    
    // Interpret status register bits
    printf("READ STATUS = 0x%02x [ ", dout);
    if (dout & 0x80) printf("RQM ");
    if (dout & 0x40) printf("DIO ");
    if (dout & 0x20) printf("EXM ");
    if (dout & 0x10) printf("CB ");
    if (dout & 0x08) printf("D3B ");
    if (dout & 0x04) printf("D2B ");
    if (dout & 0x02) printf("D1B ");
    if (dout & 0x01) printf("D0B ");
    printf("]\n");
    
    return dout;
}

// Send a byte to the u765 controller with timeout
void sendbyte(int byte, int timeout_ms = 1000) {
    int timeout_ticks = timeout_ms * 100; // Rough approximation
    int counter = 0;
    
    // Wait for controller to be ready with timeout
    while ((readstatus() & 0xcf) != 0x80) {
        counter++;
        if (counter >= timeout_ticks) {
            printf("TIMEOUT: Controller not ready to receive byte after %d ms\n", timeout_ms);
            printf("Last status: 0x%02x\n", readstatus());
            return;
        }
        
        // Don't check status too frequently
        wait(10);
    }
    
    // Send the byte
    tb->a0 = 1;
    tick(1);
    tick(0);
    tb->nRD = 1;
    tb->nWR = 0;
    tb->din = byte;
    printf("Sending byte: 0x%02x\n", byte);
    tick(1);
    tick(0);
    tick(1);
    tick(0);
    tb->nWR = 1;
    tick(1);
    tick(0);
}

// Read a byte from the u765 controller with timeout
int readbyte(int timeout_ms = 1000) {
    int byte;
    int timeout_ticks = timeout_ms * 100; // Rough approximation
    int counter = 0;
    
    // Wait for controller to be ready with timeout
    while ((readstatus() & 0xcf) != 0xc0) {
        counter++;
        if (counter >= timeout_ticks) {
            printf("TIMEOUT: Controller not ready to provide data after %d ms\n", timeout_ms);
            printf("Last status: 0x%02x\n", readstatus());
            return -1;
        }
        
        // Don't check status too frequently
        wait(10);
    }
    
    // Read the byte
    tb->a0 = 1;
    tick(1);
    tick(0);
    tb->nRD = 0;
    tb->nWR = 1;
    tick(1);
    tick(0);
    tick(1);
    tick(0);
    byte = tb->dout;
    tb->nRD = 1;
    tick(1);
    tick(0);
    printf("READ DATA = 0x%02x\n", byte);
    return byte;
}

// Read result after a command
void read_result(const char* operation) {
    printf("--- COMMAND RESULT (%s) ----\n", operation);
    
    // Create a new scan result entry
    ScanResult result;
    result.timestamp = tickcount;
    result.operation = operation;
    
    // ST0
    int st0 = readbyte();
    printf("ST0 = 0x%02x\n", st0);
    result.status0 = st0;
    
    // ST1
    int st1 = readbyte();
    printf("ST1 = 0x%02x\n", st1);
    result.status1 = st1;
    
    // ST2
    int st2 = readbyte();
    printf("ST2 = 0x%02x\n", st2);
    result.status2 = st2;
    
    // Analyze ST2 bits to determine scan result
    // Bits 2-3 in ST2 indicate the scan result:
    // 00 = empty (not satisfied)
    // 01 = empty (not satisfied)
    // 10 = satisfied (SCAN LOW/HIGH)
    // 11 = satisfied (SCAN EQUAL)
    switch ((st2 >> 2) & 0x03) {
        case 0:
        case 1:
            result.result = "No match found";
            result.match_found = false;
            break;
        case 2:
            result.result = "Match found (SCAN LOW/HIGH satisfied)";
            result.match_found = true;
            break;
        case 3:
            result.result = "Match found (SCAN EQUAL satisfied)";
            result.match_found = true;
            break;
    }
    
    // Add the result to our log
    scan_results.push_back(result);
    
    // Read the rest of the result bytes
    printf("C   = 0x%02x\n", readbyte()); // Cylinder
    printf("H   = 0x%02x\n", readbyte()); // Head
    printf("R   = 0x%02x\n", readbyte()); // Record
    printf("N   = 0x%02x\n", readbyte()); // Number (sector size)
}

// Set Terminal Count status
void set_tc(bool active) {
    tc_active = active;
    printf("Setting TC to %s\n", active ? "ACTIVE" : "INACTIVE");
}

// Check if interrupt is active
bool check_int_out() {
    return int_out_active;
}

// Acknowledge an interrupt
void cmd_sense_interrupt() {
    printf("=== SENSE INTERRUPT STATUS ===\n");
    sendbyte(0x08);
    printf("ST0 = 0x%02x\n", readbyte());
    printf("PCN = 0x%02x\n", readbyte());
}

// Standard FDC commands
void cmd_recalibrate() {
    printf("=== RECALIBRATE ===\n");
    sendbyte(0x07);
    sendbyte(0x00);
}

void cmd_seek(int ncn) {
    printf("=== SEEK ===\n");
    sendbyte(0x0f);
    sendbyte(0x00);
    sendbyte(ncn);
}

void cmd_read_data(int c, int h, int r, int n, int eot, int gpl, int dtl) {
    printf("=== READ DATA ===\n");
    sendbyte(0x06);
    sendbyte(h << 2);  // Head << 2 | Drive
    sendbyte(c);       // Cylinder
    sendbyte(h);       // Head
    sendbyte(r);       // Record (sector)
    sendbyte(n);       // Number (sector size)
    sendbyte(eot);     // End of track
    sendbyte(gpl);     // Gap length
    sendbyte(dtl);     // Data length (if N=0)
    
    // Read data bytes
    int status, byte;
    int offset = 0;
    
    while(true) {
        status = readstatus();
        if ((status & 0xe0) != 0xa0) {
            // End of read
            break;
        }
        
        tb->a0 = 1;
        tb->nRD = 0;
        tb->nWR = 1;
        tick(1);
        tick(0);
        tick(1);
        tick(0);
        byte = tb->dout;
        tb->nRD = 1;
        tick(1);
        tick(0);
        
        // Store for later comparison
        if (offset < 512) {
            sdbuf[offset] = byte;
        }
        
        printf("%02x ", byte);
        offset++;
        if ((offset % 16) == 0) printf("\n");
    }
    printf("\n");
    
    // Read result bytes
    read_result("READ DATA");
}

void cmd_scan_equal(int c, int h, int r, int n, int eot, int gpl, int stp) {
    printf("=== SCAN EQUAL ===\n");
    sendbyte(0x11);  // Opcode SCAN EQUAL
    sendbyte(h << 2);  // Head << 2 | Drive
    sendbyte(c);       // Cylinder
    sendbyte(h);       // Head
    sendbyte(r);       // Record (sector)
    sendbyte(n);       // Number (sector size)
    sendbyte(eot);     // End of track
    sendbyte(gpl);     // Gap length
    sendbyte(stp);     // Step
    
    // AHORA: Enviar datos byte a byte para comparación
    int status;
    int offset = 0;
    int sector_size = (n == 2) ? 512 : 128 << n;
    
    printf("Sending %d bytes for comparison...\n", sector_size);
    
    while(offset < sector_size) {
        // Esperar a que el controlador esté listo para recibir datos
        do {
            status = readstatus();
        } while ((status & 0xcf) != 0x80);
        
        // Enviar byte de comparación
        sendbyte(compare_data[offset]);
        
        printf("%02x ", compare_data[offset]);
        offset++;
        if ((offset % 16) == 0) printf("\n");
    }
    printf("\n");
    
    // Esperar un poco para procesar
    wait(500);
    
    // Leer resultado
    cmd_sense_interrupt();

    read_result("SCAN EQUAL");
    // Reconocer la interrupción después de un comando SCAN
     cmd_sense_interrupt();
}

void cmd_scan_low_or_equal(int c, int h, int r, int n, int eot, int gpl, int stp) {
    printf("=== SCAN LOW OR EQUAL EQUAL ===\n");
    sendbyte(0x19);  // Opcode SCAN EQUAL
    sendbyte(h << 2);  // Head << 2 | Drive
    sendbyte(c);       // Cylinder
    sendbyte(h);       // Head
    sendbyte(r);       // Record (sector)
    sendbyte(n);       // Number (sector size)
    sendbyte(eot);     // End of track
    sendbyte(gpl);     // Gap length
    sendbyte(stp);     // Step
    
    // AHORA: Enviar datos byte a byte para comparación
    int status;
    int offset = 0;
    int sector_size = (n == 2) ? 512 : 128 << n;
    
    printf("Sending %d bytes for comparison...\n", sector_size);
    
    while(offset < sector_size) {
        // Esperar a que el controlador esté listo para recibir datos
        do {
            status = readstatus();
        } while ((status & 0xcf) != 0x80);
        
        // Enviar byte de comparación
        sendbyte(compare_data[offset]);
        
        printf("%02x ", compare_data[offset]);
        offset++;
        if ((offset % 16) == 0) printf("\n");
    }
    printf("\n");
    
    // Esperar un poco para procesar
    wait(500);
    
    // Leer resultado
    read_result("SCAN EQUAL");

}

void cmd_scan_high_or_equal(int c, int h, int r, int n, int eot, int gpl, int stp) {
    printf("=== SCAN HIGH OR EQUAL ===\n");
    sendbyte(0x1D);  // Opcode SCAN EQUAL
    sendbyte(h << 2);  // Head << 2 | Drive
    sendbyte(c);       // Cylinder
    sendbyte(h);       // Head
    sendbyte(r);       // Record (sector)
    sendbyte(n);       // Number (sector size)
    sendbyte(eot);     // End of track
    sendbyte(gpl);     // Gap length
    sendbyte(stp);     // Step
    
    // AHORA: Enviar datos byte a byte para comparación
    int status;
    int offset = 0;
    int sector_size = (n == 2) ? 512 : 128 << n;
    
    printf("Sending %d bytes for comparison...\n", sector_size);
    
    while(offset < sector_size) {
        // Esperar a que el controlador esté listo para recibir datos
        do {
            status = readstatus();
        } while ((status & 0xcf) != 0x80);
        
        // Enviar byte de comparación
        sendbyte(compare_data[offset]);
        
        printf("%02x ", compare_data[offset]);
        offset++;
        if ((offset % 16) == 0) printf("\n");
    }
    printf("\n");
    
    // Esperar un poco para procesar
    wait(500);
    
    // Leer resultado
    read_result("SCAN EQUAL");
    // Reconocer la interrupción después de un comando SCAN
    cmd_sense_interrupt();
}

// Mount a disk image
void mount(FILE *edsk, int dno) {
    int fsize;

    fseek(edsk, 0, SEEK_END);
    fsize = ftell(edsk);
    tb->img_size = fsize;
    tb->img_mounted = 1<<dno;
    tick(1);
    tick(0);
    tb->img_mounted = 0;
    wait(1000);
}

// Prepare test data for SCAN comparison
void prepare_test_data() {
    printf("Preparing test data for SCAN operations...\n");
    
    // Fill the compare data buffer with different patterns for testing
    
    // 1. Exact match (same as sector)
    for (int i = 0; i < 128; i++) {
        compare_data[i] = i & 0xFF;
    }
    
    // 2. Higher values (for LOW test)
    for (int i = 128; i < 256; i++) {
        compare_data[i] = (i & 0xFF) + 10;
    }
    
    // 3. Lower values (for HIGH test)
    for (int i = 256; i < 384; i++) {
        compare_data[i] = (i & 0xFF) - 10;
    }
    
    // 4. Mixed values
    for (int i = 384; i < 512; i++) {
        compare_data[i] = (i % 3 == 0) ? (i & 0xFF) : 
                         ((i % 3 == 1) ? (i & 0xFF) + 5 : (i & 0xFF) - 5);
    }
}

// Analyze SCAN test results
void analyze_scan_results() {
    printf("\n=== SCAN TEST RESULTS ANALYSIS ===\n");
    printf("----------------------------------------\n");
    printf("| # | Operation           | Result            | Match | ST0 | ST1 | ST2 |\n");
    printf("----------------------------------------\n");
    
    for (size_t i = 0; i < scan_results.size(); i++) {
        const ScanResult& result = scan_results[i];
        printf("| %2zu | %-19s | %-18s | %-5s | %02X  | %02X  | %02X  |\n",
               i + 1,
               result.operation.c_str(),
               result.result.c_str(),
               result.match_found ? "Yes" : "No",
               result.status0,
               result.status1,
               result.status2);
    }
    printf("----------------------------------------\n");
    
    // Summary
    int matches = 0;
    for (const auto& result : scan_results) {
        if (result.match_found) matches++;
    }
    
    printf("\nSummary: %d out of %zu SCAN operations found matches.\n", 
           matches, scan_results.size());
    
    // Check ST2 bits for correct operation
    bool correct_st2_bits = true;
    for (const auto& result : scan_results) {
        // SCAN EQUAL should set bits 2-3 to 11 (0x0C) when satisfied
        if (result.operation == "SCAN EQUAL" && result.match_found && 
            ((result.status2 & 0x0C) != 0x0C)) {
            correct_st2_bits = false;
        }
        
        // SCAN LOW/HIGH should set bits 2-3 to 10 (0x08) when satisfied
        if ((result.operation == "SCAN LOW OR EQUAL" || 
             result.operation == "SCAN HIGH OR EQUAL") && 
            result.match_found && ((result.status2 & 0x0C) != 0x08)) {
            correct_st2_bits = false;
        }
    }
    
    printf("ST2 bits correctly set: %s\n", correct_st2_bits ? "Yes" : "No");
}

// Run a complete test of all SCAN functions
void test_scan_functions() {
    printf("\n=== TESTING uPD765 SCAN FUNCTIONS ===\n");
    
    // We'll use the initialization from main() to avoid reset issues
    
    // First, let's explicitly specify parameters to ensure controller is configured
    printf("\n-- Initialization: Setting controller parameters with SPECIFY command --\n");
    sendbyte(0x03, 2000); // SPECIFY command with longer timeout
    wait(100);
    sendbyte(0x8F, 2000); // SRT=8, HUT=F
    wait(100);
    sendbyte(0x05, 2000); // HLT=5 ms, Non-DMA mode
    wait(500);
    
    printf("Controller status after SPECIFY: 0x%02x\n", readstatus());
    
    // Prepare test data
    prepare_test_data();
    
    // 1. First, recalibrate to track 0
    printf("\n-- Step 1: Recalibrate drive --\n");
    cmd_recalibrate();
    wait(1000);
    
    // Handle interrupt
    if (check_int_out()) {
        cmd_sense_interrupt();
    }
    
    // 2. Seek to a test track (track 10)
    printf("\n-- Step 2: Seek to track 10 --\n");
    cmd_seek(10);
    wait(1000);
    
    // Handle interrupt
    if (check_int_out()) {
        cmd_sense_interrupt();
    }
    
    // 3. Read sector 1 to see what's in it
    printf("\n-- Step 3: Read track 10, sector 1 to analyze content --\n");
    cmd_read_data(10, 0, 1, 2, 9, 0x2A, 0xFF);
    wait(100);
    
    // 4. Modify our test data based on what we read
    printf("\n-- Step 4: Preparing comparison data --\n");
    // Create data that's exactly the same
    for (int i = 0; i < 512; i++) {
        compare_data[i] = sdbuf[i];
    }
    
    // 5. Test SCAN EQUAL with exact match
    printf("\n-- Step 5: SCAN EQUAL with exact match --\n");
    //cmd_scan_equal(10, 0, 1, 2, 9, 0x2A, 1);
    cmd_scan_equal(0, 0, 1, 2, 5, 0x2A, 1);
    wait(100);
    
    // 6. Test SCAN EQUAL with non-matching data
    printf("\n-- Step 6: SCAN EQUAL with non-matching data --\n");
    // Modify first few bytes to be different
    for (int i = 0; i < 10; i++) {
        compare_data[i] = sdbuf[i] ^ 0xFF; // Invert bits
    }
    cmd_scan_equal(10, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 7. Test SCAN LOW OR EQUAL with data higher than sector
    printf("\n-- Step 7: SCAN LOW OR EQUAL with data higher than sector --\n");
    // Make compare data higher than sector data
    for (int i = 0; i < 512; i++) {
        compare_data[i] = sdbuf[i] + 10;
    }
    cmd_scan_low_or_equal(10, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 8. Test SCAN LOW OR EQUAL with data equal to sector
    printf("\n-- Step 8: SCAN LOW OR EQUAL with data equal to sector --\n");
    // Restore exact match
    for (int i = 0; i < 512; i++) {
        compare_data[i] = sdbuf[i];
    }
    cmd_scan_low_or_equal(10, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 9. Test SCAN LOW OR EQUAL with data lower than sector
    printf("\n-- Step 9: SCAN LOW OR EQUAL with data lower than sector --\n");
    // Make compare data lower than sector data
    for (int i = 0; i < 512; i++) {
        compare_data[i] = sdbuf[i] - 10;
    }
    cmd_scan_low_or_equal(10, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 10. Test SCAN HIGH OR EQUAL with data lower than sector
    printf("\n-- Step 10: SCAN HIGH OR EQUAL with data lower than sector --\n");
    // Keep compare data lower than sector data
    cmd_scan_high_or_equal(10, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 11. Test SCAN HIGH OR EQUAL with data equal to sector
    printf("\n-- Step 11: SCAN HIGH OR EQUAL with data equal to sector --\n");
    // Restore exact match
    for (int i = 0; i < 512; i++) {
        compare_data[i] = sdbuf[i];
    }
    cmd_scan_high_or_equal(10, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 12. Test SCAN HIGH OR EQUAL with data higher than sector
    printf("\n-- Step 12: SCAN HIGH OR EQUAL with data higher than sector --\n");
    // Make compare data higher than sector data
    for (int i = 0; i < 512; i++) {
        compare_data[i] = sdbuf[i] + 10;
    }
    cmd_scan_high_or_equal(10, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 13. Test STP parameter with value 2 (skip every other sector)
    printf("\n-- Step 13: SCAN EQUAL with STP=2 --\n");
    // Restore exact match
    for (int i = 0; i < 512; i++) {
        compare_data[i] = sdbuf[i];
    }
    cmd_scan_equal(10, 0, 1, 2, 9, 0x2A, 2);
    wait(100);
    
    // 14. Test multi-sector scan with EOT > sector
    printf("\n-- Step 14: SCAN EQUAL with multi-sector (EOT=3) --\n");
    cmd_scan_equal(10, 0, 1, 2, 3, 0x2A, 1);
    wait(200);
    
    // Analyze the results
    analyze_scan_results();
}

int main(int argc, char **argv) {
    // Verify command line arguments
    if (argc < 2) {
        printf("Usage: %s <disk_image.dsk> [debug_level]\n", argv[0]);
        printf("  debug_level: 0=regular test, 1=diagnostic only\n");
        return -1;
    }

    // Debug level (0=regular, 1=diagnostic only)
    int debug_level = (argc > 2) ? atoi(argv[2]) : 0;

    // Initialize disk for testing
    edsk = fopen(argv[1], "rb");
    if (!edsk) {
        printf("Cannot open disk image: %s\n", argv[1]);
        return -1;
    }

    // Initialize Verilator
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);
    trace = new VerilatedVcdC;
    tickcount = 0;

    // Create test bench instance
    tb = new Vu765_test;
    tb->trace(trace, 99);
    trace->open("u765_scan_test.vcd");

    // Initial configuration
    printf("Resetting controller...\n");
    tb->reset = 1;
    tb->ce = 1;
    tb->nWR = 1;
    tb->nRD = 1;
    tick(1);
    tick(0);
    wait(10);
    tb->reset = 0;
    wait(100);

    printf("Checking initial controller status...\n");
    int status = readstatus();
    printf("Initial status: 0x%02x\n", status);
    
    // Set motor, ready and density flags
    printf("Setting up drive parameters...\n");
    tb->motor = 3;       // Motor on for both drives
    tb->ready = 3;       // Both drives ready
    tb->available = 3;   // Both drives available
    tb->density = 3;     // Double density (CF2DD) for both drives
    wait(100);
    
    status = readstatus();
    printf("Status after setup: 0x%02x\n", status);

    reading = 0;
    printf("Mounting disk image...\n");
    mount(edsk, 0);
    wait(1000);
    
    status = readstatus();
    printf("Status after mount: 0x%02x\n", status);
    
    // Check if RQM bit is active - if not, the controller might be in a bad state
    if (!(status & 0x80)) {
        printf("WARNING: Controller not ready (RQM bit not set)\n");
        printf("Trying to reset again...\n");
        tb->reset = 1;
        wait(50);
        tb->reset = 0;
        wait(100);
        status = readstatus();
        printf("Status after second reset: 0x%02x\n", status);
    }

    if (debug_level == 0) {
        // Run the SCAN tests
        printf("Starting SCAN function tests...\n");
        test_scan_functions();
    } else {
        printf("Debug mode - running diagnostic only\n");
        printf("Testing controller responsiveness...\n");
        
        // Simple test to see if we can send a command
        printf("Trying to send SPECIFY command...\n");
        sendbyte(0x03, 2000); // SPECIFY command with longer timeout
        if (readstatus() & 0x80) {
            printf("Controller accepted SPECIFY command\n");
            sendbyte(0x8F, 2000); // SRT=8, HUT=F
            if (readstatus() & 0x80) {
                printf("First parameter accepted\n");
                sendbyte(0x05, 2000); // HLT=5 ms, Non-DMA mode
                printf("SPECIFY command completed\n");
            }
        }
        
        wait(100);
        status = readstatus();
        printf("Final status: 0x%02x\n", status);
    }
    
    // Close files and free resources
    fclose(edsk);
    trace->close();
    delete tb;
    delete trace;
    
    return 0;
}
