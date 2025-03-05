#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
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

// Estructura para almacenar información sobre las interrupciones
struct InterruptInfo {
    int timestamp;
    int status;
    std::string cause;
    bool acknowledged;
};

// Registro de interrupciones para análisis posterior
static std::vector<InterruptInfo> interrupt_log;

// Agregamos flags para terminal count e interrupción
static bool tc_active = false;
static bool int_out_active = false;
static bool int_out_previous = false;
static int interrupt_count = 0;
static int unacknowledged_interrupts = 0;

// Función para leer un bloque de la imagen de disco
int img_read(int sd_rd) {
    if (!sd_rd) return 0;
    printf("img_read: %02x lba: %d\n", sd_rd, tb->sd_lba);
    int lba = tb->sd_lba;
    fseek(edsk, lba << 9, SEEK_SET);
    fread(&sdbuf, 512, 1, edsk);
    reading = 1;
    read_ptr = 0;
}

// Ciclo de reloj básico
void tick(int c) {
    static int sd_rd = 0;
    static int sd_wr = 0;
    int status;

    tb->clk_sys = c;
    
    // Añadimos manejo de señales tc e int_out
    tb->tc = tc_active ? 1 : 0;
    
    // Detectamos flancos de subida en int_out
    int_out_previous = int_out_active;
    int_out_active = tb->int_out;
    
    if (!int_out_previous && int_out_active) {
        // Flanco de subida en int_out (nueva interrupción)
        interrupt_count++;
        unacknowledged_interrupts++;
        
        // Guardar información sobre esta interrupción
        status = tb->a0 == 0 ? tb->dout : -1; // Solo es válido si a0=0
        InterruptInfo info;
        info.timestamp = tickcount;
        info.status = status;
        info.acknowledged = false;
        
        // Intentar determinar la causa
        if (status != -1) {
            if (status & 0x80) info.cause = "Comando completado";
            else if (status & 0x40) info.cause = "Ejecución de fase";
            else if (status & 0x20) info.cause = "Datos listos";
            else info.cause = "Desconocida";
        } else {
            info.cause = "Desconocida (a0 no es 0)";
        }
        
        interrupt_log.push_back(info);
        
        printf("--- NUEVA INTERRUPCIÓN [%d] en tick %d ---\n", interrupt_count, tickcount);
        printf("Estado: %s (0x%02x)\n", info.cause.c_str(), status);
        printf("Interrupciones sin reconocer: %d\n", unacknowledged_interrupts);
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

        if (sd_rd != tb->sd_rd) img_read(tb->sd_rd);
        sd_rd = tb->sd_rd;
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
    printf("ST0 = 0x%02x\n", readbyte());
    printf("ST1 = 0x%02x\n", readbyte());
    printf("ST2 = 0x%02x\n", readbyte());
    printf("C   = 0x%02x\n", readbyte());
    printf("H   = 0x%02x\n", readbyte());
    printf("R   = 0x%02x\n", readbyte());
    printf("N   = 0x%02x\n", readbyte());
}

// Lee datos de un sector
void read_data() {
    int status, byte;
    int offs=0;
    long chksum=0;

    while(true) {
        while (((status=readstatus()) & 0xcf) != 0xc0) {};
        if ((status & 0x20) != 0x20) {
            printf("Data sum: %ld\n", chksum);
            return;
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
        chksum += byte;
        printf("%02x ", byte);
        offs++;
        if ((offs%16)==0) printf("\n %03x ", offs);
    }
}

// Comandos FDC estándar
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


void cmd_read_id(int head) {
    printf("=== READ ID ===\n");
    sendbyte(0x0a);
    sendbyte(head << 2);
    read_result();
}

void cmd_read(int c, int h, int r, int n, int eot, int gpl, int dtl) {
    printf("=== READ ===\n");
    sendbyte(0x06);
    sendbyte(h << 2);
    sendbyte(c);
    sendbyte(h);
    sendbyte(r);
    sendbyte(n);
    sendbyte(eot);
    sendbyte(gpl);
    sendbyte(dtl);

    read_data();
    read_result();
}

// Nueva función para configurar Terminal Count
void set_tc(bool active) {
    tc_active = active;
    printf("Setting TC to %s\n", active ? "ACTIVE" : "INACTIVE");
}

// Funciones para gestión de interrupciones
bool check_int_out() {
    return int_out_active;
}

// Reconoce una interrupción pendiente
void acknowledge_interrupt() {
    if (unacknowledged_interrupts > 0) {
        // Buscamos la última interrupción no reconocida
        for (int i = interrupt_log.size() - 1; i >= 0; i--) {
            if (!interrupt_log[i].acknowledged) {
                interrupt_log[i].acknowledged = true;
                unacknowledged_interrupts--;
                printf("Interrupción [%d] reconocida. Quedan %d sin reconocer.\n", 
                       i + 1, unacknowledged_interrupts);
                break;
            }
        }
    }
}

// Comando Sense Interrupt Status - crucial para manejar interrupciones
void cmd_sense_interrupt() {
    printf("=== SENSE INTERRUPT STATUS ===\n");
    sendbyte(0x08);
    // Lee el resultado (ST0 y Present Cylinder Number)
    printf("ST0 = 0x%02x\n", readbyte());
    printf("PCN = 0x%02x\n", readbyte());
    
    // Marcar interrupción como reconocida
    acknowledge_interrupt();
}

// Realiza un análisis de las interrupciones registradas
void analyze_interrupts() {
    printf("\n=== ANÁLISIS DE INTERRUPCIONES ===\n");
    printf("Total de interrupciones: %d\n", interrupt_count);
    printf("Interrupciones sin reconocer: %d\n", unacknowledged_interrupts);
    
    printf("\nRegistro de interrupciones:\n");
    printf("------------------------------------------\n");
    printf("| # | Timestamp | Estado  | Causa                | Reconocida |\n");
    printf("------------------------------------------\n");
    
    for (size_t i = 0; i < interrupt_log.size(); i++) {
        const InterruptInfo& info = interrupt_log[i];
        printf("| %2zu | %9d | 0x%02x | %-20s | %-10s |\n", 
               i + 1, info.timestamp, info.status, 
               info.cause.c_str(), 
               info.acknowledged ? "Sí" : "No");
    }
    printf("------------------------------------------\n");
    
    // Análisis de posibles problemas
    if (unacknowledged_interrupts > 0) {
        printf("\n¡ALERTA! Hay %d interrupciones sin reconocer.\n", unacknowledged_interrupts);
        printf("Esto podría ser la causa del cuelgue del sistema.\n");
        
        // Mostrar las interrupciones sin reconocer
        printf("Interrupciones sin reconocer:\n");
        for (size_t i = 0; i < interrupt_log.size(); i++) {
            if (!interrupt_log[i].acknowledged) {
                printf("  - Interrupción #%zu, causa: %s\n", 
                       i + 1, interrupt_log[i].cause.c_str());
            }
        }
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

// Secuencia de arranque del PCW basada en la ROM analizada
void pcw_boot_sequence() {
    printf("\n=== INICIANDO SECUENCIA DE ARRANQUE PCW ===\n");
    
    // Inicialización similar a la ROM PCW
    tb->reset = 1;
    wait(10);
    tb->reset = 0;
    wait(50);
    
    // Primera fase - Recalibración y configuración
    printf("\n=== FASE 1: RECALIBRACIÓN Y CONFIGURACIÓN ===\n");
    cmd_recalibrate();
    wait(1000);
    
    // Comprobar interrupciones después de la recalibración
    if (check_int_out()) {
        printf("Interrupción detectada después de recalibrar.\n");
        // Usar Sense Interrupt Status para reconocer interrupción
        cmd_sense_interrupt();
    }
    
    // Segunda fase - Lectura del sector de arranque (Track 0, Sector 1)
    printf("\n=== FASE 2: LEYENDO SECTOR DE ARRANQUE ===\n");
    cmd_read(0, 0, 1, 2, 0xff, 0x2A, 0xff);
    wait(100);
    
    // Comprobar interrupciones después de leer el sector de arranque
    if (check_int_out()) {
        printf("Interrupción detectada después de leer el sector de arranque.\n");
        // Las interrupciones después de Read Data deberían haberse manejado 
        // automáticamente por la fase de resultados del comando
    }
    
    // Tercera fase - Lectura del sistema desde la pista 1
    printf("\n=== FASE 3: LEYENDO COMPONENTES DEL SISTEMA ===\n");
    
    // Buscar pista 1
    cmd_seek(1);
    wait(500);
    
    // Comprobar interrupciones después del seek
    if (check_int_out()) {
        printf("Interrupción detectada después de seek a pista 1.\n");
        // Usar Sense Interrupt Status para reconocer interrupción
        cmd_sense_interrupt();
    }
    
    // Leer sectores del sistema (pista 1, sectores varios)
    // Primero el CCP (Console Command Processor)
    printf("\n-- Leyendo CCP (Console Command Processor) --\n");
    cmd_read(1, 0, 0x01, 2, 0x09, 0x2A, 0xff);
    wait(100);
    
    // Comprobar interrupciones después de leer CCP
    if (check_int_out()) {
        printf("Interrupción detectada después de leer CCP.\n");
        // Ver nota anterior sobre Read Data
    }
    
    // Luego el BDOS (Basic Disk Operating System)
    printf("\n-- Leyendo BDOS (Basic Disk Operating System) --\n");
    cmd_read(1, 0, 0x0A, 2, 0x12, 0x2A, 0xff);
    wait(100);
    
    // Comprobar interrupciones después de leer BDOS
    if (check_int_out()) {
        printf("Interrupción detectada después de leer BDOS.\n");
    }
    
    // Finalmente el BIOS (Basic Input/Output System)
    printf("\n-- Leyendo BIOS (Basic Input/Output System) --\n");
    cmd_read(1, 0, 0x13, 2, 0x1A, 0x2A, 0xff);
    wait(100);
    
    // Comprobar interrupciones después de leer BIOS
    if (check_int_out()) {
        printf("Interrupción detectada después de leer BIOS.\n");
    }
    
    // Cuarta fase - Inicialización de sistema y configuración
    printf("\n=== FASE 4: CONFIGURACIÓN DEL SISTEMA ===\n");
    cmd_seek(2);
    wait(500);
    
    // Comprobar interrupciones después del seek
    if (check_int_out()) {
        printf("Interrupción detectada después de seek a pista 2.\n");
        // Usar Sense Interrupt Status para reconocer interrupción
        cmd_sense_interrupt();
    }
    
    // Leer archivos de configuración (simulado)
    cmd_read(2, 0, 1, 2, 5, 0x2A, 0xff);
    wait(100);
    
    // Comprobar interrupciones después de leer configuración
    if (check_int_out()) {
        printf("Interrupción detectada después de leer configuración.\n");
    }
    
    // Quinta fase - Punto crítico donde podría ocurrir el cuelgue antes del prompt
    printf("\n=== FASE 5: PUNTO CRÍTICO - ANTES DEL PROMPT ===\n");
    
    // Simulamos la verificación de estado que podría causar el cuelgue
    printf("Verificando estado del controlador:\n");
    int status = readstatus();
    
    // Verificar si hay interrupción pendiente
    bool int_pending = check_int_out();
    printf("¿Interrupción pendiente?: %s\n", int_pending ? "SÍ" : "NO");
    
    // Si hay una interrupción pendiente y no se maneja, podría causar el cuelgue
    if (int_pending) {
        printf("ATENCIÓN: Interrupción no manejada detectada en el punto crítico\n");
        printf("Esto podría ser la causa del cuelgue antes del prompt\n");
        
        // Probar usando Sense Interrupt Status para limpiar la interrupción
        printf("\n-- Probando Sense Interrupt Status --\n");
        cmd_sense_interrupt();
        
        if (!check_int_out()) {
            printf("¡Éxito! La interrupción se limpió con Sense Interrupt Status\n");
        } else {
            printf("La interrupción persiste después de Sense Interrupt Status\n");
            
            // Si persiste, probar otros métodos...
            printf("\n-- Probando otros métodos de limpieza --\n");
            
            // Terminal Count
            set_tc(true);
            wait(10);
            set_tc(false);
            wait(10);
            
            // Leer datos
            tb->a0 = 1;
            tb->nRD = 0;
            wait(5);
            tb->nRD = 1;
            wait(5);
        }
    }
    
    // Intentar iniciar el prompt (simulado)
    printf("\n-- Intentando mostrar el prompt --\n");
    wait(100);
    
    // Analizar el estado de las interrupciones
    analyze_interrupts();
    
    printf("\n=== FIN DE LA SECUENCIA DE ARRANQUE ===\n");
}

// Test específico para el problema de interrupciones
void test_interrupciones() {
    printf("\n=== TEST ESPECÍFICO: MANEJO DE INTERRUPCIONES ===\n");
    
    // Inicialización básica
    tb->reset = 1;
    wait(10);
    tb->reset = 0;
    wait(50);
    
    // Configuración de hardware
    tb->motor = 1;
    tb->ready = 1;
    tb->available = 1;
    tb->density = 1;
    wait(1000);
    
    // 1. Generar y verificar interrupción durante recalibrado
    printf("\n-- Test 1: Interrupción por recalibrado --\n");
    cmd_recalibrate();
    wait(1000);
    bool int_after_recal = check_int_out();
    printf("Interrupción detectada: %s\n", int_after_recal ? "SÍ" : "NO");
    
    // 2. Intentar varias formas de reconocer la interrupción
    if (int_after_recal) {
        printf("\n-- Test 2: Métodos para reconocer interrupciones --\n");
        
        // Método A: Sense Interrupt Status (el correcto para Recalibrate/Seek)
        printf("Método A: Sense Interrupt Status\n");
        cmd_sense_interrupt();
        bool cleared_a = !check_int_out();
        printf("Interrupción borrada: %s\n", cleared_a ? "SÍ" : "NO");
        
        // Si no se borró, intentar otros métodos
        if (!cleared_a) {
            // Método B: Terminal Count
            printf("Método B: Activar Terminal Count\n");
            set_tc(true);
            wait(10);
            set_tc(false);
            wait(10);
            bool cleared_b = !check_int_out();
            printf("Interrupción borrada: %s\n", cleared_b ? "SÍ" : "NO");
            
            // Método C: Leer estado
            printf("Método C: Leer registro de estado\n");
            int status = readstatus();
            bool cleared_c = !check_int_out();
            printf("Interrupción borrada: %s\n", cleared_c ? "SÍ" : "NO");
        }
    }
    
    // 3. Simular la secuencia exacta que podría ocurrir en el PCW
    printf("\n-- Test 3: Secuencia de interrupciones múltiples --\n");
    
    // Generar una secuencia de comandos que cause múltiples interrupciones
    cmd_recalibrate();
    wait(500);
    
    // Verificar interrupción pero NO manejarla (simulando un bug)
    bool int_after_cmd1 = check_int_out();
    printf("Interrupción después de recalibrar: %s\n", int_after_cmd1 ? "SÍ" : "NO");
    //acknowledge_interrupt();
    
    // Ejecutar otro comando sin manejar la interrupción anterior
    cmd_seek(1);
    wait(500);
    
    // Verificar interrupciones
    bool int_after_cmd2 = check_int_out();
    printf("Interrupción después de seek: %s\n", int_after_cmd2 ? "SÍ" : "NO");
    
    //acknowledge_interrupt();
    
    // Intentar leer en este estado (con interrupciones pendientes)
    printf("Intentando leer datos con interrupciones pendientes...\n");
    cmd_read(1, 0, 1, 2, 5, 0x2A, 0xff);
    wait(100);
    
    // Verificar estado final
    bool int_final = check_int_out();
    printf("Interrupción final: %s\n", int_final ? "SÍ" : "NO");
    
    if (int_final) {
        printf("\n-- Manejo correcto de interrupciones --\n");
        // Limpiar todas las interrupciones pendientes con Sense Interrupt Status
        while (check_int_out()) {
            printf("Ejecutando Sense Interrupt Status...\n");
            cmd_sense_interrupt();
            wait(10);
        }
    }
    
    // 4. Prueba específica para la ROM del PCW
    printf("\n-- Test 4: Verificación de secuencia PCW --\n");
    
    // Examinar la ROM del PCW para ver si hay instrucciones específicas de manejo
    printf("La ROM del PCW contiene:\n");
    printf("- Operaciones OUT al puerto 0xF8 (PORT_CTRL)\n");
    printf("- Lecturas del bit 5 para verificación de estado\n");
    printf("- No se observa claramente un comando Sense Interrupt Status\n");
    
    // Simular la secuencia del PCW (basada en la ROM)
    cmd_recalibrate();
    wait(500);
    
    // Verificar si el PCW podría estar intentando una forma alternativa
    // de manejar las interrupciones basada en los puertos 0xF8/0xF7
    if (check_int_out()) {
        printf("Simulando manejo de interrupciones estilo PCW...\n");
        
        // Simular la secuencia vista en la ROM (update_config)
        tb->a0 = 0;  // Dirección A0=0 (registro de estado)
        wait(5);
        
        // Simular comportamiento en port_config_loop de la ROM
        int status = readstatus();
        printf("Verificando bit 5 (EXM): %s\n", (tb->int_out) ? "Activo" : "Inactivo");
        
        // Verificar si la interrupción se borró con este método
        bool cleared_pcw = !check_int_out();
        printf("Interrupción borrada con método PCW: %s\n", cleared_pcw ? "SÍ" : "NO");
        
        // Si no se borró, intentar con Sense Interrupt Status
        if (!cleared_pcw) {
            printf("El método PCW no funciona, usando Sense Interrupt Status...\n");
            cmd_sense_interrupt();
        }
    }
    
    // Analizar resultados
    analyze_interrupts();
}

int main(int argc, char **argv) {
    // Verificar argumentos de línea de comando
    if (argc < 2) {
        printf("Uso: %s <archivo.dsk> [test_mode]\n", argv[0]);
        printf("  test_mode: 0=boot completo, 1=test interrupciones\n");
        return -1;
    }

    // Modo de prueba (0=boot normal, 1=test interrupciones)
    int test_mode = (argc > 2) ? atoi(argv[2]) : 0;

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
    trace->open("pcw_u765.vcd");

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

    tb->motor = 1;
    tb->ready = 1;
    tb->available = 1;
    tb->density = 1;

    wait(1000);

    // Ejecutar el test seleccionado
    if (test_mode == 0) {
        // Ejecutar secuencia completa de arranque del PCW
        pcw_boot_sequence();
    } else if (test_mode == 1) {
        // Ejecutar test específico de interrupciones
        test_interrupciones();
    } else {
        printf("Modo de prueba no válido\n");
    }
    
    // Imprimir resumen final
    printf("\n=== RESUMEN FINAL DE LA PRUEBA ===\n");
    analyze_interrupts();
    
    // Cerrar archivos y liberar recursos
    fclose(edsk);
    trace->close();
    delete tb;
    delete trace;
    
    return 0;
}
