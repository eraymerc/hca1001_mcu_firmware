# HCA1001 ADC Monitor (Go / Fyne GUI)
#
# Separate from the firmware Makefile in the project root -- this one only
# builds the host-side GUI. Nothing here touches the STM32 build.
#
#   make            build for the host
#   make run        build and run
#   make test       unit + pty integration tests
#   make linux      native build -> dist/hca1001-monitor
#   make windows    cross build  -> dist/hca1001-monitor.exe
#   make release    both of the above, stripped
#   make toolchain  show which cross-compiler will be used
#   make clean      remove build output
#
# NOTE ON BUILD TIME: Fyne is a cgo project, so the first Windows cross build
# compiles the whole C side (glfw, GL bindings) for a new target and takes
# several minutes. Go caches per-target, so later builds are quick. A slow
# first run is expected, not a failure.

BINARY  := hca1001-monitor
PKG     := .
DIST    := dist

GO      ?= go
VERSION ?= 1.0.0

# Strip debug info and the symbol table for distributable builds.
LDFLAGS_RELEASE := -s -w

# Fyne needs cgo (OpenGL + windowing). This is not optional on any platform.
export CGO_ENABLED := 1

# ---------------------------------------------------------------- toolchain
#
# Windows cross-compilation needs a C compiler targeting Windows. Two work:
#
#   zig   (preferred) - `pacman -S zig` / `apt install zig`. Ships its own
#                       Windows headers and CRT, so there is nothing else to
#                       install and the result is self-contained.
#   mingw-w64         - `pacman -S mingw-w64-gcc` /
#                       `apt install gcc-mingw-w64-x86-64`.
#
# Override with `make windows TOOLCHAIN=mingw` (or TOOLCHAIN=zig).

TOOLCHAIN ?= auto

HAVE_ZIG   := $(shell command -v zig 2>/dev/null)
HAVE_MINGW := $(shell command -v x86_64-w64-mingw32-gcc 2>/dev/null)

ifeq ($(TOOLCHAIN),zig)
  WIN_TOOL := zig
else ifeq ($(TOOLCHAIN),mingw)
  WIN_TOOL := mingw
else
  ifneq ($(HAVE_ZIG),)
    WIN_TOOL := zig
  else ifneq ($(HAVE_MINGW),)
    WIN_TOOL := mingw
  else
    WIN_TOOL := none
  endif
endif

ifeq ($(WIN_TOOL),zig)
  WIN_CC  := zig cc -target x86_64-windows-gnu
  WIN_CXX := zig c++ -target x86_64-windows-gnu
  WIN_PROBE := zig
  # zig's linker does not act on the Go linker's -H windowsgui, so the
  # subsystem has to be set explicitly -- without this the app still works but
  # opens a console window behind the GUI. (mingw honours -H windowsgui.)
  WIN_CGO_LDFLAGS := -Wl,--subsystem,windows
else ifeq ($(WIN_TOOL),mingw)
  WIN_CC  := x86_64-w64-mingw32-gcc
  WIN_CXX := x86_64-w64-mingw32-g++
  WIN_PROBE := x86_64-w64-mingw32-gcc
  WIN_CGO_LDFLAGS :=
endif

.PHONY: all build run test test-visual linux windows release toolchain fmt vet tidy deps clean help

all: build

## Host build (whatever platform you are on)
build:
	$(GO) build -o $(BINARY) $(PKG)

run: build
	./$(BINARY)

## Tests. The serial test drives a socat pty pair standing in for the Nucleo
## and is skipped automatically if socat is not installed.
test:
	$(GO) test ./...

## Also dump sample plot renders and a full-window screenshot to /tmp/hca_plots
test-visual:
	@mkdir -p /tmp/hca_plots
	PLOT_OUT=/tmp/hca_plots $(GO) test -run 'TestRenderPlotImage|TestWindowScreenshot' -v ./...
	@echo "renders written to /tmp/hca_plots"

## ---------------------------------------------------------------- packaging

$(DIST):
	@mkdir -p $(DIST)

linux: | $(DIST)
	GOOS=linux GOARCH=amd64 \
		$(GO) build -ldflags '$(LDFLAGS_RELEASE)' -o $(DIST)/$(BINARY) $(PKG)
	@echo "built $(DIST)/$(BINARY)"

# -H windowsgui suppresses the console window that would otherwise open
# alongside the GUI.
windows: | $(DIST)
ifeq ($(WIN_TOOL),none)
	@echo "error: no Windows cross-compiler found. Install one of:"
	@echo "  zig        : sudo pacman -S zig          | sudo apt install zig"
	@echo "  mingw-w64  : sudo pacman -S mingw-w64-gcc | sudo apt install gcc-mingw-w64-x86-64"
	@exit 1
else
	@command -v $(WIN_PROBE) >/dev/null 2>&1 || { \
		echo "error: TOOLCHAIN=$(WIN_TOOL) selected but $(WIN_PROBE) is not installed."; \
		exit 1; }
	@echo "cross-compiling for windows/amd64 using $(WIN_TOOL) (first run takes a few minutes)"
	GOOS=windows GOARCH=amd64 CC="$(WIN_CC)" CXX="$(WIN_CXX)" \
		CGO_LDFLAGS="$(WIN_CGO_LDFLAGS)" \
		$(GO) build -ldflags '$(LDFLAGS_RELEASE) -H windowsgui' -o $(DIST)/$(BINARY).exe $(PKG)
	@file $(DIST)/$(BINARY).exe 2>/dev/null | grep -q GUI \
		&& echo "built $(DIST)/$(BINARY).exe (GUI subsystem)" \
		|| echo "built $(DIST)/$(BINARY).exe (warning: console subsystem - a terminal window will appear)"
endif

release: linux windows
	@ls -lh $(DIST)

## Report the detected cross-compiler without building anything.
toolchain:
	@echo "TOOLCHAIN=$(TOOLCHAIN) -> $(WIN_TOOL)"
	@echo "  zig      : $(if $(HAVE_ZIG),$(HAVE_ZIG),not installed)"
	@echo "  mingw-w64: $(if $(HAVE_MINGW),$(HAVE_MINGW),not installed)"
	@echo "  CC       : $(WIN_CC)"

## ------------------------------------------------------------------ hygiene

fmt:
	$(GO) fmt ./...

vet:
	$(GO) vet ./...

tidy:
	$(GO) mod tidy

deps:
	$(GO) mod download

clean:
	rm -f $(BINARY)
	rm -rf $(DIST)

help:
	@grep -E '^[a-z-]+:' $(MAKEFILE_LIST) | cut -d: -f1 | sort -u
