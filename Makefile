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

# Regla por defecto
all: verilate compile

# Regla para limpiar
clean:
	rm -rf obj_dir
	rm -f $(PROJECT)_tb

# Regla para la compilación de Verilator
verilate:
	verilator --trace -Wno-fatal --threads 1 --top-module u765_test -cc $(VERILOG_FILES)

# Regla para la compilación de C++
compile:
	$(CXX) $(CXXFLAGS) $(VERILATOR_SRC) $(TB_FILE) obj_dir/*.cpp $(LDFLAGS) -o $(PROJECT)_tb

