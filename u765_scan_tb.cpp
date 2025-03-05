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

// Archivo de testbench específico para comandos SCAN
// Compilar con: make scan_tb

double sc_time_stamp() {
    return 0;
}

// Estructura para datos de prueba
struct TestData {
    int sector;
    uint8_t data[512];
};

// Estructura para tracking de informes de escaneo
struct ScanResult {
    int tick;
    int status0;
    int status1;
    int status2;
    bool equal;
    bool high;
    bool low;
    std::string result_desc;
};

static Vu765_test *tb;
static VerilatedVcdC *trace;
static int tickcount;

static unsigned char sdbuf[512];
static FILE *edsk;
static int reading;
static int read_ptr;
static std::vector<ScanResult> scan_results;

// Variables globales para tracking de estados
static bool tc_active = false;
static bool int_out_active = false;
static bool int_out_previous = false;
static int interrupt_count = 0;

// Sector con datos para pruebas
static uint8_t sector_data[512];

// Ciclo de reloj básico
void tick(int c) {
    static int sd_rd = 0;
    static int sd_wr = 0;

    tb->clk_sys = c;
    
    // Manejar TC e interrupciones
    tb->tc = tc_active ? 1 : 0;
    
    // Detectar flancos de subida en int_out
    int_out_previous = int_out_active;
    int_out_active = tb->int_out;
    
    if (!int_out_previous && int_out_active) {
        interrupt_count++;
    }
    
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

        if (sd_rd != tb->sd_rd) {
            // Leer bloque del disco
            if (tb->sd_rd) {
                printf("img_read: %02x lba: %d\n", tb->sd_rd, tb->sd_lba);
                int lba = tb->sd_lba;
                fseek(edsk, lba << 9, SEEK_SET);
                fread(sdbuf, 512, 1, edsk);
                reading = 1;
                read_ptr = 0;
            }
        }
        
        // Si hay un write, preparar para aceptar la escritura desde el SD
        if (tb->sd_wr && !sd_wr) {
            printf("SD Write request to LBA %d\n", tb->sd_lba);
            // Aquí no implementamos la escritura real - solo simulamos la respuesta
            tb->sd_ack = 1;
            tb->sd_ack = 0;  // Pulso corto de ack
        }
        
        sd_rd = tb->sd_rd;
        sd_wr = tb->sd_wr;
    }
}

void wait(int t) {
    for (int i=0; i<t; i++) {
        tick(1);
        tick(0);
    }
}

// Lee el registro de estado del u765
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
    
    // Interpretar los bits del registro de estado
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

// Envía un byte al controlador u765
void sendbyte(int byte) {
    while ((readstatus() & 0xcf) != 0x80) {};
    tb->a0 = 1;
    tick(1);
    tick(0);
    tb->nRD = 1;
    tb->nWR = 0;
    tb->din = byte;
    tick(1);
    tick(0);
    tick(1);
    tick(0);
    tb->nWR = 1;
    tick(1);
    tick(0);
}

// Lee un byte del controlador u765
int readbyte() {
    int byte;

    while ((readstatus() & 0xcf) != 0xc0) {};
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

// Lee el resultado de un comando
void read_result() {
    printf("--- COMMAND RESULT ----\n");
    int st0 = readbyte();
    int st1 = readbyte();
    int st2 = readbyte();
    int c = readbyte();
    int h = readbyte();
    int r = readbyte();
    int n = readbyte();
    
    printf("ST0 = 0x%02x\n", st0);
    printf("ST1 = 0x%02x\n", st1);
    printf("ST2 = 0x%02x\n", st2);
    printf("C   = 0x%02x\n", c);
    printf("H   = 0x%02x\n", h);
    printf("R   = 0x%02x\n", r);
    printf("N   = 0x%02x\n", n);
    
    // Analizar el resultado del scan
    ScanResult result;
    result.tick = tickcount;
    result.status0 = st0;
    result.status1 = st1;
    result.status2 = st2;
    
    // Interpretar ST2 para determinar resultado de comparación
    result.equal = (st2 & 0x08) == 0; // SH bit (bit 3) es 0 si igualó
    result.high = (st2 & 0x04) != 0;  // SN bit (bit 2) es 1 si sector > data
    result.low = (st2 & 0x04) == 0 && !result.equal; // Si SN=0 pero no igual, entonces sector < data
    
    if (result.equal) {
        result.result_desc = "Datos iguales";
    } else if (result.high) {
        result.result_desc = "Datos del sector > Datos comparados";
    } else if (result.low) {
        result.result_desc = "Datos del sector < Datos comparados";
    } else {
        result.result_desc = "Resultado indeterminado";
    }
    
    scan_results.push_back(result);
}

// Comando para enviar o recibir datos del sector actual
// Declaración de funciones 
void set_tc(bool active);
void analyze_scan_results();

void transfer_sector_data(bool write) {
    int status, byte;
    int offs = 0;
    long chksum = 0;

    while(true) {
        status = readstatus();
        if ((status & 0xe0) != 0xa0) {
            // Ya no en modo transferencia
            printf("Fin de transferencia, suma: %ld\n", chksum);
            return;
        }
        
        if (write) {
            // Enviar datos al controlador
            tb->a0 = 1;
            tb->nRD = 1;
            tb->nWR = 0;
            tb->din = sector_data[offs % 512];
            tick(1);
            tick(0);
            tick(1);
            tick(0);
            tb->nWR = 1;
            tick(1);
            tick(0);
            chksum += sector_data[offs % 512];
            offs++;
        } else {
            // Leer datos del controlador
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
            chksum += byte;
            printf("%02x ", byte);
            offs++;
            if ((offs % 16) == 0) printf("\n %03x ", offs);
        }
        
        // Detener después de 512 bytes si es necesario
        if (offs >= 512 && tc_active) {
            // Activar Terminal Count para finalizar operación
            printf("Activando TC después de %d bytes\n", offs);
            set_tc(true);
            wait(5);
            set_tc(false);
            break;
        }
    }
}

// Nueva función para configurar Terminal Count
void set_tc(bool active) {
    tc_active = active;
    printf("Setting TC to %s\n", active ? "ACTIVE" : "INACTIVE");
}

// Reconoce una interrupción pendiente
void cmd_sense_interrupt() {
    printf("=== SENSE INTERRUPT STATUS ===\n");
    sendbyte(0x08);
    // Lee el resultado (ST0 y Present Cylinder Number)
    printf("ST0 = 0x%02x\n", readbyte());
    printf("PCN = 0x%02x\n", readbyte());
}

// Comandos básicos del FDC 
void cmd_recalibrate() {
    printf("=== RECALIBRATE ===\n");
    sendbyte(0x07);
    sendbyte(0x00);
}

void cmd_seek(int ncn) {
    printf("=== SEEK ===\n");
    sendbyte(0x0f);
    sendbyte(0x00);  // Unidad 0
    sendbyte(ncn);   // Cilindro destino
}

void cmd_read_id(int head) {
    printf("=== READ ID ===\n");
    sendbyte(0x0a);
    sendbyte(head << 2);
    read_result();
}

void cmd_read_data(int c, int h, int r, int n, int eot, int gpl, int dtl) {
    printf("=== READ DATA (Cylinder %d, Head %d, Sector %d) ===\n", c, h, r);
    sendbyte(0x06);        // Comando READ DATA
    sendbyte(h << 2);      // Unidad 0, Cabeza h
    sendbyte(c);           // Cilindro
    sendbyte(h);           // Cabeza
    sendbyte(r);           // Sector inicial
    sendbyte(n);           // Tamaño del sector (0=128, 1=256, 2=512, 3=1024)
    sendbyte(eot);         // Último sector en pista
    sendbyte(gpl);         // Gap length
    sendbyte(dtl);         // Data length (si N=0)

    transfer_sector_data(false);  // Leer datos del sector
    read_result();
}

void cmd_write_data(int c, int h, int r, int n, int eot, int gpl, int dtl) {
    printf("=== WRITE DATA (Cylinder %d, Head %d, Sector %d) ===\n", c, h, r);
    sendbyte(0x05);        // Comando WRITE DATA
    sendbyte(h << 2);      // Unidad 0, Cabeza h
    sendbyte(c);           // Cilindro
    sendbyte(h);           // Cabeza
    sendbyte(r);           // Sector inicial
    sendbyte(n);           // Tamaño del sector (2=512 bytes)
    sendbyte(eot);         // Último sector en pista
    sendbyte(gpl);         // Gap length
    sendbyte(dtl);         // Data length (si N=0)

    transfer_sector_data(true);   // Escribir datos al sector
    read_result();
}

// Comandos de escaneo
void cmd_scan_equal(int c, int h, int r, int n, int eot, int gpl, int stp) {
    printf("=== SCAN EQUAL (Cylinder %d, Head %d, Sector %d) ===\n", c, h, r);
    sendbyte(0x11);        // Comando SCAN EQUAL
    sendbyte(h << 2);      // Unidad 0, Cabeza h
    sendbyte(c);           // Cilindro
    sendbyte(h);           // Cabeza
    sendbyte(r);           // Sector inicial
    sendbyte(n);           // Tamaño del sector (2=512 bytes)
    sendbyte(eot);         // Último sector en pista
    sendbyte(gpl);         // Gap length
    sendbyte(stp);         // Step (01 = contiguo, 02 = saltar cada 2)

    transfer_sector_data(true);   // Enviar datos para comparar
    read_result();
}

void cmd_scan_low_or_equal(int c, int h, int r, int n, int eot, int gpl, int stp) {
    printf("=== SCAN LOW OR EQUAL (Cylinder %d, Head %d, Sector %d) ===\n", c, h, r);
    sendbyte(0x19);        // Comando SCAN LOW OR EQUAL
    sendbyte(h << 2);      // Unidad 0, Cabeza h
    sendbyte(c);           // Cilindro
    sendbyte(h);           // Cabeza
    sendbyte(r);           // Sector inicial
    sendbyte(n);           // Tamaño del sector (2=512 bytes)
    sendbyte(eot);         // Último sector en pista
    sendbyte(gpl);         // Gap length
    sendbyte(stp);         // Step (01 = contiguo, 02 = saltar cada 2)

    transfer_sector_data(true);   // Enviar datos para comparar
    read_result();
}

void cmd_scan_high_or_equal(int c, int h, int r, int n, int eot, int gpl, int stp) {
    printf("=== SCAN HIGH OR EQUAL (Cylinder %d, Head %d, Sector %d) ===\n", c, h, r);
    sendbyte(0x1d);        // Comando SCAN HIGH OR EQUAL
    sendbyte(h << 2);      // Unidad 0, Cabeza h
    sendbyte(c);           // Cilindro
    sendbyte(h);           // Cabeza
    sendbyte(r);           // Sector inicial
    sendbyte(n);           // Tamaño del sector (2=512 bytes)
    sendbyte(eot);         // Último sector en pista
    sendbyte(gpl);         // Gap length
    sendbyte(stp);         // Step (01 = contiguo, 02 = saltar cada 2)

    transfer_sector_data(true);   // Enviar datos para comparar
    read_result();
}

// Preparar datos para las pruebas de SCAN
void prepare_test_data() {
    // Inicializar datos para las pruebas
    printf("Preparando datos para pruebas de SCAN\n");

    // Primero rellenar todo con un patrón reconocible
    for (int i = 0; i < 512; i++) {
        sector_data[i] = i & 0xFF;
    }
    
    // Rellenar el resto con valores específicos para tests
    for (int i = 10; i < 20; i++) {
        sector_data[i] = 0xAA;  // Valor mayor que el esperado 
    }
    
    for (int i = 30; i < 40; i++) {
        sector_data[i] = 0x55;  // Valor menor que el esperado
    }
    
    for (int i = 50; i < 60; i++) {
        sector_data[i] = i & 0xFF;  // Valores iguales a los esperados
    }
}

// Monta una imagen de disco
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

// Test específico para comandos SCAN
void test_scan_commands() {
    printf("\n=== TEST DE COMANDOS SCAN DEL uPD765 ===\n");
    
    // Inicialización básica
    tb->reset = 1;
    wait(10);
    tb->reset = 0;
    wait(50);
    
    // Configuración de hardware
    tb->motor = 1;
    tb->ready = 1;
    tb->available = 1;
    tb->density = 1;  // CF2DD
    wait(100);
    
    // Preparar datos para el test
    prepare_test_data();
    
    // Inicializar la unidad y ubicarse en el sector de prueba
    printf("\n-- Inicialización de la unidad --\n");
    cmd_recalibrate();
    wait(500);
    
    // Reconocer interrupción de recalibrado
    if (int_out_active) {
        cmd_sense_interrupt();
    }
    
    // Mover a la pista deseada
    int test_track = 10;
    cmd_seek(test_track);
    wait(500);
    
    // Reconocer interrupción de seek
    if (int_out_active) {
        cmd_sense_interrupt();
    }
    
    // Escribir datos de prueba en el sector 1
    printf("\n-- Escribiendo datos de prueba en el sector 1 --\n");
    cmd_write_data(test_track, 0, 1, 2, 9, 0x2A, 0xFF);
    wait(100);
    
    // 1. Test SCAN EQUAL
    printf("\n-- Test 1: SCAN EQUAL --\n");
    // Preparar datos que sean iguales al sector
    memcpy(sector_data, sdbuf, 512);
    cmd_scan_equal(test_track, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 2. Test SCAN EQUAL negativo (datos no iguales)
    printf("\n-- Test 2: SCAN EQUAL (negativo) --\n");
    // Modificar algunos bytes para que no sean iguales
    for (int i = 0; i < 10; i++) {
        sector_data[i] = 0xFF;
    }
    cmd_scan_equal(test_track, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 3. Test SCAN LOW OR EQUAL (datos iguales)
    printf("\n-- Test 3: SCAN LOW OR EQUAL (datos iguales) --\n");
    memcpy(sector_data, sdbuf, 512);
    cmd_scan_low_or_equal(test_track, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 4. Test SCAN LOW OR EQUAL (datos de sector < datos comparados)
    printf("\n-- Test 4: SCAN LOW OR EQUAL (sector < datos) --\n");
    for (int i = 0; i < 10; i++) {
        sector_data[i] = sdbuf[i] + 10;  // Datos comparados son mayores
    }
    cmd_scan_low_or_equal(test_track, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 5. Test SCAN LOW OR EQUAL (datos de sector > datos comparados)
    printf("\n-- Test 5: SCAN LOW OR EQUAL (sector > datos) --\n");
    for (int i = 0; i < 10; i++) {
        sector_data[i] = sdbuf[i] - 10;  // Datos comparados son menores
    }
    cmd_scan_low_or_equal(test_track, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 6. Test SCAN HIGH OR EQUAL (datos iguales)
    printf("\n-- Test 6: SCAN HIGH OR EQUAL (datos iguales) --\n");
    memcpy(sector_data, sdbuf, 512);
    cmd_scan_high_or_equal(test_track, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 7. Test SCAN HIGH OR EQUAL (datos de sector > datos comparados)
    printf("\n-- Test 7: SCAN HIGH OR EQUAL (sector > datos) --\n");
    for (int i = 0; i < 10; i++) {
        sector_data[i] = sdbuf[i] - 10;  // Datos comparados son menores
    }
    cmd_scan_high_or_equal(test_track, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 8. Test SCAN HIGH OR EQUAL (datos de sector < datos comparados)
    printf("\n-- Test 8: SCAN HIGH OR EQUAL (sector < datos) --\n");
    for (int i = 0; i < 10; i++) {
        sector_data[i] = sdbuf[i] + 10;  // Datos comparados son mayores
    }
    cmd_scan_high_or_equal(test_track, 0, 1, 2, 9, 0x2A, 1);
    wait(100);
    
    // 9. Test SCAN con step diferente (STP=2)
    printf("\n-- Test 9: SCAN con STP=2 (saltar cada 2 sectores) --\n");
    memcpy(sector_data, sdbuf, 512);
    cmd_scan_equal(test_track, 0, 1, 2, 9, 0x2A, 2);
    wait(100);
    
    // Mostrar resultados
    analyze_scan_results();
}

// Analiza los resultados de las pruebas SCAN
void analyze_scan_results() {
    printf("\n=== RESUMEN DE RESULTADOS DE SCAN ===\n");
    printf("--------------------------------------------------------------------------------------------------------\n");
    printf("| Test | Tick     | ST0    | ST1    | ST2    | Igual  | Mayor  | Menor  | Resultado                    |\n");
    printf("--------------------------------------------------------------------------------------------------------\n");
    
    for (size_t i = 0; i < scan_results.size(); i++) {
        const ScanResult& result = scan_results[i];
        printf("| %-4zu | %-8d | 0x%02X   | 0x%02X   | 0x%02X   | %-6s | %-6s | %-6s | %-29s |\n", 
               i + 1, 
               result.tick, 
               result.status0, 
               result.status1, 
               result.status2,
               result.equal ? "Sí" : "No",
               result.high ? "Sí" : "No",
               result.low ? "Sí" : "No",
               result.result_desc.c_str());
    }
    printf("--------------------------------------------------------------------------------------------------------\n");
    
    // Validación de pruebas
    printf("\nAnálisis de resultados:\n");
    
    if (scan_results.size() >= 1 && scan_results[0].equal) {
        printf("✓ Test 1 (SCAN EQUAL): Correcto - encontró datos iguales\n");
    } else {
        printf("✗ Test 1 (SCAN EQUAL): Incorrecto - debería haber encontrado datos iguales\n");
    }
    
    if (scan_results.size() >= 2 && !scan_results[1].equal) {
        printf("✓ Test 2 (SCAN EQUAL negativo): Correcto - no encontró datos iguales\n");
    } else {
        printf("✗ Test 2 (SCAN EQUAL negativo): Incorrecto - no debería haber encontrado datos iguales\n");
    }
    
    if (scan_results.size() >= 3 && scan_results[2].equal) {
        printf("✓ Test 3 (SCAN LOW OR EQUAL, datos iguales): Correcto - encontró datos iguales\n");
    } else {
        printf("✗ Test 3 (SCAN LOW OR EQUAL, datos iguales): Incorrecto - debería haber encontrado datos iguales\n");
    }
    
    if (scan_results.size() >= 4 && scan_results[3].low) {
        printf("✓ Test 4 (SCAN LOW OR EQUAL, sector < datos): Correcto - encontró datos menores\n");
    } else {
        printf("✗ Test 4 (SCAN LOW OR EQUAL, sector < datos): Incorrecto - debería haber encontrado datos menores\n");
    }
    
    if (scan_results.size() >= 5 && !scan_results[4].equal && !scan_results[4].low) {
        printf("✓ Test 5 (SCAN LOW OR EQUAL, sector > datos): Correcto - no encontró coincidencia\n");
    } else {
        printf("✗ Test 5 (SCAN LOW OR EQUAL, sector > datos): Incorrecto - no debería haber encontrado coincidencia\n");
    }
    
    if (scan_results.size() >= 6 && scan_results[5].equal) {
        printf("✓ Test 6 (SCAN HIGH OR EQUAL, datos iguales): Correcto - encontró datos iguales\n");
    } else {
        printf("✗ Test 6 (SCAN HIGH OR EQUAL, datos iguales): Incorrecto - debería haber encontrado datos iguales\n");
    }
    
    if (scan_results.size() >= 7 && scan_results[6].high) {
        printf("✓ Test 7 (SCAN HIGH OR EQUAL, sector > datos): Correcto - encontró datos mayores\n");
    } else {
        printf("✗ Test 7 (SCAN HIGH OR EQUAL, sector > datos): Incorrecto - debería haber encontrado datos mayores\n");
    }
    
    if (scan_results.size() >= 8 && !scan_results[7].equal && !scan_results[7].high) {
        printf("✓ Test 8 (SCAN HIGH OR EQUAL, sector < datos): Correcto - no encontró coincidencia\n");
    } else {
        printf("✗ Test 8 (SCAN HIGH OR EQUAL, sector < datos): Incorrecto - no debería haber encontrado coincidencia\n");
    }
    
    if (scan_results.size() >= 9) {
        printf("✓ Test 9 (SCAN con STP=2): Completado\n");
    } else {
        printf("✗ Test 9 (SCAN con STP=2): No completado\n");
    }
}

int main(int argc, char **argv) {
    // Verificar argumentos de línea de comando
    if (argc < 2) {
        printf("Uso: %s <archivo.dsk>\n", argv[0]);
        return -1;
    }

    // Inicializar disco de prueba
    edsk = fopen(argv[1], "rb");
    if (!edsk) {
        printf("No se puede abrir %s.\n", argv[1]);
        return -1;
    }

    // Inicializar variables de Verilator
    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);
    trace = new VerilatedVcdC;
    tickcount = 0;

    // Crear una instancia de nuestro módulo bajo prueba
    tb = new Vu765_test;
    tb->trace(trace, 99);
    trace->open("u765_scan_test.vcd");

    // Configuración inicial
    tb->reset = 1;
    tb->ce = 1;
    tb->nWR = 1;
    tb->nRD = 1;
    tick(1);
    tick(0);
    tick(1);
    tick(0);
    tb->reset = 0;

    reading = 0;
    mount(edsk, 0);
    
    // Ejecutar el test de comandos SCAN
    test_scan_commands();
    
    // Cerrar archivos y liberar recursos
    fclose(edsk);
    trace->close();
    delete tb;
    delete trace;
    
    return 0;
}
