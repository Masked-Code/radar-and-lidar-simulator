# Makefile for Radar/LiDAR Signal Processing Simulation
# Author: Radar/LiDAR Simulation Team
# Version: 1.0
# Date: 2025

# ============================================================================
# BUILD CONFIGURATION
# ============================================================================

# Project name and version
PROJECT_NAME = radar-lidar-simulator
PROJECT_VERSION = 1.0.0

# Compiler and build tools
CC = gcc
LD = gcc
AR = ar
STRIP = strip

# Build directories
SRC_DIR = src
INC_DIR = include
OBJ_DIR = obj
BIN_DIR = bin
LIB_DIR = lib
DOC_DIR = docs
TEST_DIR = tests
EXAMPLE_DIR = examples

# Create directories if they don't exist
$(shell mkdir -p $(OBJ_DIR) $(BIN_DIR) $(LIB_DIR))

# ============================================================================
# COMPILER FLAGS
# ============================================================================

# Basic compiler flags
CFLAGS = -std=c11 -Wall -Wextra -Wpedantic -Wformat=2 -Wformat-security
CFLAGS += -Wstrict-prototypes -Wmissing-prototypes -Wold-style-definition
CFLAGS += -Wredundant-decls -Wnested-externs -Wmissing-include-dirs
CFLAGS += -Wswitch-default -Wswitch-enum -Wunused-parameter
CFLAGS += -Wuninitialized -Wmaybe-uninitialized -Wshadow

# Include directories
CFLAGS += -I$(INC_DIR)

# Math library
LDFLAGS += -lm

# FFTW library
CFLAGS += -DFFTW_ENABLE_FLOAT
LDFLAGS += -lfftw3 -lfftw3f

# Threading support
CFLAGS += -pthread
LDFLAGS += -pthread

# Platform-specific settings
ifeq ($(OS),Windows_NT)
    # Windows-specific settings
    CFLAGS += -D_WIN32_WINNT=0x0600 -DWIN32_LEAN_AND_MEAN
    LDFLAGS += -lws2_32 -lwinmm
    EXECUTABLE_EXT = .exe
    LIBRARY_EXT = .dll
    STATIC_LIB_EXT = .lib
else
    # Unix/Linux-specific settings
    CFLAGS += -D_GNU_SOURCE
    LDFLAGS += -ldl -lrt
    EXECUTABLE_EXT =
    LIBRARY_EXT = .so
    STATIC_LIB_EXT = .a
endif

# Build type configuration
BUILD_TYPE ?= Release

ifeq ($(BUILD_TYPE),Debug)
    CFLAGS += -g -O0 -DDEBUG -D_DEBUG
    CFLAGS += -fno-omit-frame-pointer -fno-optimize-sibling-calls
    STRIP_CMD = @echo "Skipping strip for debug build"
else ifeq ($(BUILD_TYPE),Release)
    CFLAGS += -O3 -DNDEBUG
    CFLAGS += -fomit-frame-pointer -ffast-math
    STRIP_CMD = $(STRIP)
else ifeq ($(BUILD_TYPE),Profile)
    CFLAGS += -g -O2 -pg -DPROFILE
    LDFLAGS += -pg
    STRIP_CMD = @echo "Skipping strip for profile build"
else
    $(error Unknown build type: $(BUILD_TYPE). Use Debug, Release, or Profile)
endif

# ============================================================================
# SOURCE FILES
# ============================================================================

# Core source files
CORE_SOURCES = \
    $(SRC_DIR)/signal_generator.c \
    $(SRC_DIR)/target_simulator.c \
    $(SRC_DIR)/fft_processor.c \
    $(SRC_DIR)/doppler_analyzer.c \
    $(SRC_DIR)/range_detector.c \
    $(SRC_DIR)/utils.c

# Main program source
MAIN_SOURCE = $(SRC_DIR)/main.c

# Example sources
EXAMPLE_SOURCES = \
    $(EXAMPLE_DIR)/basic_radar.c \
    $(EXAMPLE_DIR)/multi_target.c \
    $(EXAMPLE_DIR)/doppler_demo.c

# Test sources
TEST_SOURCES = \
    $(TEST_DIR)/test_fft.c \
    $(TEST_DIR)/test_signals.c \
    $(TEST_DIR)/test_doppler.c

# ============================================================================
# OBJECT FILES
# ============================================================================

# Core object files
CORE_OBJECTS = $(CORE_SOURCES:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

# Main program object
MAIN_OBJECT = $(MAIN_SOURCE:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)

# Example objects
EXAMPLE_OBJECTS = $(EXAMPLE_SOURCES:$(EXAMPLE_DIR)/%.c=$(OBJ_DIR)/%.o)

# Test objects
TEST_OBJECTS = $(TEST_SOURCES:$(TEST_DIR)/%.c=$(OBJ_DIR)/%.o)

# All object files
ALL_OBJECTS = $(CORE_OBJECTS) $(MAIN_OBJECT) $(EXAMPLE_OBJECTS) $(TEST_OBJECTS)

# ============================================================================
# TARGET EXECUTABLES
# ============================================================================

# Main executable
MAIN_TARGET = $(BIN_DIR)/$(PROJECT_NAME)$(EXECUTABLE_EXT)

# Example executables
EXAMPLE_TARGETS = \
    $(BIN_DIR)/basic_radar$(EXECUTABLE_EXT) \
    $(BIN_DIR)/multi_target$(EXECUTABLE_EXT) \
    $(BIN_DIR)/doppler_demo$(EXECUTABLE_EXT)

# Test executables
TEST_TARGETS = \
    $(BIN_DIR)/test_fft$(EXECUTABLE_EXT) \
    $(BIN_DIR)/test_signals$(EXECUTABLE_EXT) \
    $(BIN_DIR)/test_doppler$(EXECUTABLE_EXT)

# Static library
STATIC_LIB = $(LIB_DIR)/lib$(PROJECT_NAME)$(STATIC_LIB_EXT)

# ============================================================================
# BUILD TARGETS
# ============================================================================

# Default target
.PHONY: all
all: $(MAIN_TARGET) $(STATIC_LIB)
	@echo "Build complete: $(BUILD_TYPE)"
	@echo "Main executable: $(MAIN_TARGET)"
	@echo "Static library: $(STATIC_LIB)"

# Main executable
$(MAIN_TARGET): $(MAIN_OBJECT) $(CORE_OBJECTS)
	@echo "Linking main executable: $@"
	$(LD) -o $@ $^ $(LDFLAGS)
	$(STRIP_CMD) $@

# Static library
$(STATIC_LIB): $(CORE_OBJECTS)
	@echo "Creating static library: $@"
	$(AR) rcs $@ $^

# Examples
.PHONY: examples
examples: $(EXAMPLE_TARGETS)

$(BIN_DIR)/basic_radar$(EXECUTABLE_EXT): $(OBJ_DIR)/basic_radar.o $(CORE_OBJECTS)
	@echo "Linking example: $@"
	$(LD) -o $@ $^ $(LDFLAGS)
	$(STRIP_CMD) $@

$(BIN_DIR)/multi_target$(EXECUTABLE_EXT): $(OBJ_DIR)/multi_target.o $(CORE_OBJECTS)
	@echo "Linking example: $@"
	$(LD) -o $@ $^ $(LDFLAGS)
	$(STRIP_CMD) $@

$(BIN_DIR)/doppler_demo$(EXECUTABLE_EXT): $(OBJ_DIR)/doppler_demo.o $(CORE_OBJECTS)
	@echo "Linking example: $@"
	$(LD) -o $@ $^ $(LDFLAGS)
	$(STRIP_CMD) $@

# Tests
.PHONY: tests
tests: $(TEST_TARGETS)

$(BIN_DIR)/test_fft$(EXECUTABLE_EXT): $(OBJ_DIR)/test_fft.o $(CORE_OBJECTS)
	@echo "Linking test: $@"
	$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_DIR)/test_signals$(EXECUTABLE_EXT): $(OBJ_DIR)/test_signals.o $(CORE_OBJECTS)
	@echo "Linking test: $@"
	$(LD) -o $@ $^ $(LDFLAGS)

$(BIN_DIR)/test_doppler$(EXECUTABLE_EXT): $(OBJ_DIR)/test_doppler.o $(CORE_OBJECTS)
	@echo "Linking test: $@"
	$(LD) -o $@ $^ $(LDFLAGS)

# ============================================================================
# OBJECT FILE COMPILATION
# ============================================================================

# Core source objects
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	@echo "Compiling: $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Example objects
$(OBJ_DIR)/%.o: $(EXAMPLE_DIR)/%.c
	@echo "Compiling example: $<"
	$(CC) $(CFLAGS) -c $< -o $@

# Test objects
$(OBJ_DIR)/%.o: $(TEST_DIR)/%.c
	@echo "Compiling test: $<"
	$(CC) $(CFLAGS) -c $< -o $@

# ============================================================================
# UTILITY TARGETS
# ============================================================================

# Run tests
.PHONY: test
test: tests
	@echo "Running tests..."
	@for test in $(TEST_TARGETS); do \
		echo "Running $$test..."; \
		$$test || exit 1; \
	done
	@echo "All tests passed!"

# Run examples
.PHONY: run-examples
run-examples: examples
	@echo "Running examples..."
	@echo "1. Basic Radar Example:"
	$(BIN_DIR)/basic_radar$(EXECUTABLE_EXT)
	@echo "\n2. Multi-Target Example:"
	$(BIN_DIR)/multi_target$(EXECUTABLE_EXT)
	@echo "\n3. Doppler Demo:"
	$(BIN_DIR)/doppler_demo$(EXECUTABLE_EXT)

# Generate documentation
.PHONY: docs
docs:
	@echo "Generating documentation..."
	@if command -v doxygen >/dev/null 2>&1; then \
		doxygen Doxyfile; \
		echo "Documentation generated in $(DOC_DIR)/html/"; \
	else \
		echo "Doxygen not found. Please install doxygen to generate documentation."; \
	fi

# Static analysis
.PHONY: analyze
analyze:
	@echo "Running static analysis..."
	@if command -v cppcheck >/dev/null 2>&1; then \
		cppcheck --enable=all --inconclusive --std=c11 \
		         --suppress=missingIncludeSystem \
		         -I$(INC_DIR) $(SRC_DIR)/ $(EXAMPLE_DIR)/ $(TEST_DIR)/; \
	else \
		echo "cppcheck not found. Please install cppcheck for static analysis."; \
	fi

# Format code
.PHONY: format
format:
	@echo "Formatting code..."
	@if command -v clang-format >/dev/null 2>&1; then \
		find $(SRC_DIR) $(INC_DIR) $(EXAMPLE_DIR) $(TEST_DIR) -name "*.c" -o -name "*.h" | \
		xargs clang-format -i -style="{IndentWidth: 4, UseTab: Never}"; \
		echo "Code formatted successfully."; \
	else \
		echo "clang-format not found. Please install clang-format for code formatting."; \
	fi

# Check dependencies
.PHONY: check-deps
check-deps:
	@echo "Checking dependencies..."
	@echo -n "GCC: "; $(CC) --version | head -n1 || echo "Not found"
	@echo -n "FFTW3: "; pkg-config --modversion fftw3 2>/dev/null || echo "Not found"
	@echo -n "Make: "; make --version | head -n1 || echo "Not found"

# Install target (Unix/Linux only)
.PHONY: install
install: $(MAIN_TARGET) $(STATIC_LIB)
	@if [ "$(OS)" != "Windows_NT" ]; then \
		echo "Installing $(PROJECT_NAME)..."; \
		install -d $(DESTDIR)/usr/local/bin; \
		install -m 755 $(MAIN_TARGET) $(DESTDIR)/usr/local/bin/; \
		install -d $(DESTDIR)/usr/local/lib; \
		install -m 644 $(STATIC_LIB) $(DESTDIR)/usr/local/lib/; \
		install -d $(DESTDIR)/usr/local/include/$(PROJECT_NAME); \
		install -m 644 $(INC_DIR)/*.h $(DESTDIR)/usr/local/include/$(PROJECT_NAME)/; \
		echo "Installation complete."; \
	else \
		echo "Install target not supported on Windows."; \
	fi

# Package target
.PHONY: package
package: all examples
	@echo "Creating package..."
	@VERSION=$(PROJECT_VERSION); \
	 PACKAGE_NAME=$(PROJECT_NAME)-$$VERSION; \
	 mkdir -p $$PACKAGE_NAME/bin $$PACKAGE_NAME/lib $$PACKAGE_NAME/include $$PACKAGE_NAME/examples; \
	 cp $(BIN_DIR)/* $$PACKAGE_NAME/bin/; \
	 cp $(LIB_DIR)/* $$PACKAGE_NAME/lib/; \
	 cp $(INC_DIR)/* $$PACKAGE_NAME/include/; \
	 cp README.md $$PACKAGE_NAME/; \
	 tar -czf $$PACKAGE_NAME.tar.gz $$PACKAGE_NAME; \
	 rm -rf $$PACKAGE_NAME; \
	 echo "Package created: $$PACKAGE_NAME.tar.gz"

# Clean build artifacts
.PHONY: clean
clean:
	@echo "Cleaning build artifacts..."
	rm -rf $(OBJ_DIR) $(BIN_DIR) $(LIB_DIR)
	rm -f *.tar.gz
	@echo "Clean complete."

# Clean everything including documentation
.PHONY: distclean
distclean: clean
	@echo "Cleaning all generated files..."
	rm -rf $(DOC_DIR)/html $(DOC_DIR)/latex
	rm -f $(DOC_DIR)/*.log $(DOC_DIR)/*.aux
	@echo "Distclean complete."

# ============================================================================
# HELP TARGET
# ============================================================================

.PHONY: help
help:
	@echo "Radar/LiDAR Signal Processing Simulation - Build System"
	@echo "======================================================="
	@echo ""
	@echo "Available targets:"
	@echo "  all          - Build main executable and static library (default)"
	@echo "  examples     - Build example programs"
	@echo "  tests        - Build test programs"
	@echo "  test         - Build and run all tests"
	@echo "  run-examples - Build and run example programs"
	@echo "  docs         - Generate documentation (requires doxygen)"
	@echo "  analyze      - Run static analysis (requires cppcheck)"
	@echo "  format       - Format code (requires clang-format)"
	@echo "  check-deps   - Check for required dependencies"
	@echo "  install      - Install to system (Unix/Linux only)"
	@echo "  package      - Create distribution package"
	@echo "  clean        - Remove build artifacts"
	@echo "  distclean    - Remove all generated files"
	@echo "  help         - Show this help message"
	@echo ""
	@echo "Build types (use BUILD_TYPE=<type>):"
	@echo "  Debug        - Debug build with symbols and no optimization"
	@echo "  Release      - Release build with optimization (default)"
	@echo "  Profile      - Profile build with profiling information"
	@echo ""
	@echo "Examples:"
	@echo "  make                    # Build with default settings"
	@echo "  make BUILD_TYPE=Debug   # Build debug version"
	@echo "  make examples test      # Build examples and tests"
	@echo "  make clean all          # Clean and rebuild"

# ============================================================================
# DEPENDENCY TRACKING
# ============================================================================

# Generate dependency files
-include $(ALL_OBJECTS:.o=.d)

# Pattern rule for generating dependency files
$(OBJ_DIR)/%.d: $(SRC_DIR)/%.c
	@$(CC) $(CFLAGS) -MM -MT $(OBJ_DIR)/$*.o -MF $@ $<

$(OBJ_DIR)/%.d: $(EXAMPLE_DIR)/%.c
	@$(CC) $(CFLAGS) -MM -MT $(OBJ_DIR)/$*.o -MF $@ $<

$(OBJ_DIR)/%.d: $(TEST_DIR)/%.c
	@$(CC) $(CFLAGS) -MM -MT $(OBJ_DIR)/$*.o -MF $@ $<

# ============================================================================
# PHONY TARGET DECLARATIONS
# ============================================================================

.PHONY: all examples tests test run-examples docs analyze format \
        check-deps install package clean distclean help

# Special targets
.SUFFIXES:
.DELETE_ON_ERROR:
.SECONDARY:
