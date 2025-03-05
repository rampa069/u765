# Configuración del entorno
VERILATOR_ROOT = /opt/homebrew/Cellar/verilator/5.034/share/verilator
VINC = $(VERILATOR_ROOT)/include
CXX = clang++
CXXFLAGS = -std=c++17 -I obj_dir -I$(VINC)
LDFLAGS = -DOPT=-DVL_DEBUG 

# Archivos fuente de Verilator
VERILATOR_SRC = $(VINC)/verilated.cpp \
                $(VINC)/verilated_vcd_c.cpp \
                $(VINC)/verilated_threads.cpp

# Nombre del proyecto y archivos de entrada
PROJECT = u765
VERILOG_FILES = u765_test.sv u765.sv
TB_FILE = u765_tb.cpp
SCAN_TB_FILE = scan_command_tb.cpp

# Regla por defecto
all: verilate compile

# Regla para compilar el testbench principal
compile: $(PROJECT)_tb

# Regla para compilar ambos testbenches
both: $(PROJECT)_tb scan_tb

# Testbench principal
$(PROJECT)_tb: verilate
	$(CXX) $(CXXFLAGS) $(VERILATOR_SRC) $(TB_FILE) obj_dir/*.cpp $(LDFLAGS) -o $(PROJECT)_tb

# Testbench para comandos SCAN
scan_tb: verilate
	cp scan_command_tb.cpp u765_scan_tb.cpp  # Copiar el archivo con el nombre esperado
	$(CXX) $(CXXFLAGS) $(VERILATOR_SRC) u765_scan_tb.cpp obj_dir/*.cpp $(LDFLAGS) -o scan_tb

# Regla para limpiar
clean:
	rm -rf obj_dir
	rm -f $(PROJECT)_tb scan_tb
	rm -f *.vcd

# Regla para la compilación de Verilator
verilate:
	verilator --trace -Wno-fatal --threads 1 --top-module u765_test -cc $(VERILOG_FILES)

# Ayuda
help:
	@echo "Objetivos disponibles:"
	@echo "  all        - Compila Verilator y el testbench principal"
	@echo "  both       - Compila ambos testbenches"
	@echo "  compile    - Compila solo el testbench principal"
	@echo "  scan_tb    - Compila solo el testbench de comandos SCAN"
	@echo "  clean      - Limpia archivos generados"
	@echo "  verilate   - Solo ejecuta Verilator"
	@echo "  help       - Muestra esta ayuda"
