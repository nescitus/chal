# Chal (v1.0)

**Chal** (Gujarati for "move") is a minimal, didactic chess engine written entirely in a single file of strictly-compliant classical C (ANSI C89 / C90).

The core goal of this project is to serve as an educational resource to teach the fundamental concepts of chess engine development. The source file reads top-to-bottom, commented with minimal theories avoiding heavy abstraction.

## Architecture Guidelines

Despite being a single minimal file (`656` lines of pure C code, heavily supplemented by `~600` lines of comments), Chal implements several foundational engine mechanics:
- **0x88 Geometry**: Utilizes the 128-square 0x88 1D array allowing out-of-board detection in a single bitwise integer `AND`.
- **Zobrist Hashing**: Encodes state into incrementally updated 32-bit representations. 
- **Transposition Tables (TT)**: 65,536-entry hash table pruning search tree redundancies.
- **Negamax Alpha-Beta**: Minimax recursively written symmetrically.
- **Iterative Deepening**: Layers search depth for TT move-ordering logic. 
- **Quiescence Search**: Generates capture-trees past standard search depth bounds to resolve the "horizon effect".
- **Positional Evaluation (PSTs)**: Implements basic Piece-Square Tables valuing central knight placements, castling structures, and pawn advancement organically.
- **Killer Heuristics & MVV-LVA**: Move ordering arrays testing capture material values and historic beta-cutoffs earliest. 

## Building the Engine

The codebase adheres strictly to standard cross-platform C90, so it compiles on practically any C compiler without external libraries.

```bash
make
```

### Manual Compilation
```bash
gcc src/chal.c -O2 -Wall -Wextra -pedantic -std=gnu90 -o bin/chal
```

## Protocol (UCI)

Chal communicates over the Universal Chess Interface (UCI). It serves perfectly as the brain inside modern desktop GUIs (Arena, Cutechess, Banksia) or inside a basic console:

```text
uci
position startpos moves e2e4 e7e5
go
```

**Custom Extension Command**:
`perft [depth]`
Run mathematical leaf node calculations to debug generated move correctness (e.g. `perft 4` from standard startpos outputs exactly `197281` positions).

## Reading the Code

`src/chal.c` is explicitly simple. Code components follow a linear, top-to-bottom arrangement without dependency graphs:

1. **Constants & Types** (Bit packing representations)
2. **Direction Vectors** (Moving via arrays, not loops)
3. **Zobrist keys** (XOR encryption logic)
4. **Make/Undo** (State history vs deep copying Boards)
5. **Search Logic** (Negamax, PVS concepts)
6. **Move Generation & Evaluators**

Enjoy reading the code.
