# Noona – Chess Engine (Java)

Noona is a chess engine written from scratch in Java with the goal of learning and implementing the core principles of chess programming, including move generation, search algorithms, evaluation functions, and performance optimization.

The project intentionally begins with a simple and clear 2D array board representation to ensure correctness and maintainability before moving toward advanced optimizations such as bitboards and transposition tables.

The focus is:

- correctness first
- clarity of design
- incremental optimization
- strong engineering fundamentals

Current estimated strength: ~1200 ELO

---

## Features

- Legal move generation for all pieces
- Castling, en passant, and promotion support
- Check, checkmate, and stalemate detection
- Minimax search with alpha–beta pruning
- Piece-square table evaluation
- UCI protocol support (compatible with chess GUIs)
- Playable engine via command line or GUI

---

## Board Representation (v1.0)

The current implementation uses a simple 2D board model:

```java
Piece[][] board = new Piece[8][8];
