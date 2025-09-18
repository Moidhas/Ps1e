# PS1 Emulator (WIP)

A work-in-progress Sony PlayStation (PSX/PS1) emulator written in modern C++.  

**Legal note:** You must provide your own PlayStation BIOS and original game data you legally own. These files are not included.

---

## Project layout

```
.
├── BIOS/                      # Put your BIOS here (not tracked)
│   └── bios_file
├── Build/                     # Build outputs (created by make)
│   └── ps1                    # Main emulator binary
├── src/
│   ├── assembler.{hpp,cc}     # Tiny assembler
│   ├── BitUtils.{hpp,cc}      # Bit-field helpers, masks, shifts, sign-extend
│   ├── cop0.{hpp,cc}          # System control coprocessor
│   ├── cpu.{hpp,cc}           # MIPS R3000A core
│   ├── main.cc                
│   ├── memory.{hpp,cc}        # Memory map, loads/stores
│   ├── Ring.hpp               # Simple ring buffer
│   └── types.hpp              # Fixed-width types, common aliases
├── tests/
│   └── test.cc                # Unit Tests
├── Makefile
├── compile_flags.txt
├── note.txt                   # Scratch notes (dev)
└── README.md
```

---

## Requirements

- C++23 compiler (GNU GCC)
- `make`
- A PlayStation BIOS dumped from your own console

---

## Build

```bash
# From repo root
make

# To build test
make test
```

- Output binary: `Build/ps1`
- Test binary: `Build/test`

---

## Run

Place your BIOS in `BIOS/` as shown above. Then:

```bash
./Build/ps1
# for tests
./Build/test
```

