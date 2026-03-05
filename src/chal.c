/*
================================================================
                          C H A L                              
================================================================
   Gujarati for "move." A minimal chess engine in ANSI C90.   

   Author : Naman Thanki                                       
   Date   : 2026-03-05  (v1.0 Production Release)            

   This file is meant to be read as a book, not just run.      
   Every subsystem is a short lesson in engine design.         

   Compile:  gcc chal.c -O2 -Wall -Wextra -pedantic -std=gnu90 -o chal                            
   Protocol: Universal Chess Interface (UCI)                   
================================================================

   TABLE OF CONTENTS
   -----------------
   S1  Constants & Types         - piece encoding, move packing
   S2  Board State               - 0x88 grid, globals
   S3  Direction Tables          - geometry of all piece movement
   S4  Zobrist Hashing           - position fingerprints
   S5  Attack Detection          - is a square under fire?
   S6  Make / Undo               - reversible board updates
   S7  Move Generation           - pseudo-legal, fast
   S8  FEN Parser                - reading position strings
   S9  Evaluation                - material + bishop pair + pawn advance
   S10 Move Ordering             - MVV-LVA, hash-move, killers
   S11 Search                    - negamax + alpha-beta + QSearch + TT
   S12 Perft                     - move-gen correctness testing
   S13 UCI Loop                  - talking to chess GUIs
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <time.h>

/* ===============================================================
   S1  CONSTANTS & TYPES
   ===============================================================

   PIECES
   ------
   A piece fits in one byte.

       bit 3      bits 2-0
       +---+  +-----------+
       | C |  |   TYPE    |
       +---+  +-----------+

   C = 0 -> White,  C = 1 -> Black.
   TYPE lives in [1..6]; 0 means the square is empty.

       EMPTY=0  PAWN=1  KNIGHT=2  BISHOP=3
       ROOK=4   QUEEN=5  KING=6

   PIECE(c,t)  - build a piece byte from colour + type
   TYPE(p)     - strip colour, return piece kind
   COLOR(p)    - 0=White, 1=Black
*/

enum { EMPTY=0, PAWN, KNIGHT, BISHOP, ROOK, QUEEN, KING };
enum { WHITE=0, BLACK=1 };
#define SQ_NONE    (-1)
#define TYPE(p)    ((p) & 7)
#define COLOR(p)   ((p) >> 3)
#define PIECE(c,t) (((c) << 3) | (t))

/* Named constants for search bounds — never use raw magic numbers.
   INF  = initial alpha/beta window, wider than any real evaluation.
   MATE = checkmate score base; actual mate returns -(MATE - ply)
   so shorter mates score higher than longer ones.                  */
#define INF  50000
#define MATE 30000

/* -------------------------------------------------------------
   MOVES - packed into one int
   -------------------------------------------------------------

   We store an entire chess move in 32 bits:

       bits  0- 6  ->  from-square   (0-127)
       bits  7-13  ->  to-square     (0-127)
       bits 14-17  ->  promotion type (0 = none)

   One integer copy is faster than a struct copy. The macros
   below let us read each field with a mask + shift.

       FROM(m)          - origin square
       TO(m)            - destination square
       PROMO(m)         - promoted piece type (or 0)
       MAKE_MOVE(f,t,p) - pack all three fields
*/

typedef int Move;
#define FROM(m)          ((m) & 0x7F)
#define TO(m)            (((m) >> 7) & 0x7F)
#define PROMO(m)         (((m) >> 14) & 0xF)
#define MAKE_MOVE(f,t,p) ((f) | ((t) << 7) | ((p) << 14))

/* -------------------------------------------------------------
   TRANSPOSITION TABLE (TT)
   -------------------------------------------------------------

   Chess trees transpose - the same position is often reached
   by many different move orders. Without a cache we re-search
   the same node thousands of times.

   Each entry stores:
       key       - Zobrist hash (identifies the position)
       depth     - search depth when this entry was written
       flag      - TT_EXACT / TT_ALPHA (upper bound) / TT_BETA (lower bound)
       score     - evaluation in centipawns
       best_move - best move found here (used for move ordering)

   REPLACEMENT POLICY - depth-preferred
   ------------------------------------
   We only overwrite an existing entry if the new search depth
   is at least as deep. This prevents shallow nodes (written late
   in iterative deepening) from evicting deep, expensive entries
   written early. Shallow entries are cheap to recompute; deep
   entries are not.

       new.depth >= old.depth  ->  replace
       new.depth <  old.depth  ->  keep the deeper entry
*/

enum { TT_EXACT=0, TT_ALPHA, TT_BETA };
typedef struct {
    unsigned long key;
    int depth, flag, score;
    Move best_move;
} TTEntry;

#define TT_SIZE 65536
TTEntry tt[TT_SIZE];

/* -------------------------------------------------------------
   UNDO HISTORY
   -------------------------------------------------------------

   When the search plays a move, it must be able to take it back.
   Copying the whole 128-square board at every node would be
   catastrophic. Instead we save only the irreversible fields:

       piece_captured     - what was on the to-square (0 if none)
       ep_square_prev     - the en-passant square before the move
       castle_rights_prev - the castling bitmask before the move
       hash_prev          - the Zobrist key before the move

   Storing hash_prev lets undo_move() restore it in O(1)
   rather than re-scanning the entire board. This is the
   incremental-undo technique.
*/

typedef struct {
    Move move;
    int  piece_captured;
    int  ep_square_prev;
    int  castle_rights_prev;
    unsigned long hash_prev;
} State;

State history[1024];

/* KILLERS
   -------
   A "killer" is a quiet move (no capture) that recently caused
   a beta cutoff at the same search depth. If it was good once,
   it is likely good again. We store two killers per depth and
   try them before other quiet moves.

   Two slots per depth: when a new killer arrives we shift the
   old one to slot [1] and write the new one to slot [0].        */
#define MAX_DEPTH 64
Move killers[MAX_DEPTH][2];

/* ===============================================================
   S2  BOARD STATE
   ===============================================================

   THE 0x88 BOARD
   --------------
   Most beginners represent the board as board[64]. The problem:
   when a piece tries to move off the edge, you need range checks
   on both rank and file separately.

   The 0x88 trick doubles the array to 128 squares, arranged as
   a 16-column x 8-row grid. Only the left 8 columns are real:

       col:  0 1 2 3 4 5 6 7 | 8 9 A B C D E F
       rank7: a8...h8        | (phantom - off the board)
       rank6: a7...h7        | (phantom)
       ...
       rank0: a1...h1        | (phantom)

   Any square that steps into a phantom column has bit 3 set in
   its low nibble, so (square & 0x88) != 0. One AND instruction
   replaces two separate rank and file range comparisons:

       Legal square:   sq & 0x88 == 0
       Off the board:  sq & 0x88 != 0  <-- caught instantly

   Square numbering:  sq = rank*16 + file
       a1=0, b1=1, ..., h1=7
       a2=16,           h2=23
       ...
       a8=112,          h8=119
*/

#define SQ_IS_OFF(sq) ((sq) & 0x88)

int board[128];
int side, xside;         /* whose turn; its opposite                    */
int ep_square;           /* en-passant target square, or SQ_NONE        */
int castle_rights;       /* bits: 1=WO-O  2=WO-O-O  4=BO-O  8=BO-O-O  */
int king_sq[2];          /* king location for each colour               */
int ply;                 /* half-move counter: depth in current search  */
unsigned long hash_key;  /* Zobrist fingerprint of the current position */

/* ===============================================================
   S3  DIRECTION TABLES
   ===============================================================

   Every piece is just a set of geometric displacement vectors.

   On a 0x88 board, one rank up = +16, one file right = +1.
   Therefore:

       North-East diagonal = +16+1 = +17
       Knight short-L      = 2 ranks + 1 file = +-(32+-1)
                             or 1 rank + 2 files = +-(16+-2)
                           = +-14, +-18, +-31, +-33

   We store all directions in one flat array and give each piece
   type a [start, end) window into it:

       Piece    Window          Count   Slides?
       ------   -------------   -----   -------
       KNIGHT   [4,  12)         8      No  (leaper)
       BISHOP   [12, 16)         4      Yes (diagonal)
       ROOK     [16, 20)         4      Yes (orthogonal)
       QUEEN    [12, 20)         8      Yes (B U R)
       KING     [20, 28)         8      No  (leaper)

   Sliders step repeatedly (target += step) until blocked.
   Leapers (Knight, King) break after a single step.

   Pawn movement is asymmetric - direction depends on colour -
   so it is special-cased in generate_moves().
*/

int step_dir[] = {
    0, 0, 0, 0,                     /* padding: aligns index with piece enum    */
    -33,-31,-18,-14,14,18,31,33,    /* Knight jumps     (idx 4–11)              */
    -17,-15, 15, 17,                /* Bishop diagonals (idx 12–15)             */
    -16, -1,  1, 16,                /* Rook orthogonals (idx 16–19)             */
    -17,-16,-15,-1,1,15,16,17       /* King one-step    (idx 20–27)             */
};
int piece_offsets[] = { 0,0, 4,12,16,12,20 }; /* first dir index per piece type */
int piece_limits[]  = { 0,0,12,16,20,20,28 }; /* one-past-last dir index        */

/* ===============================================================
   S4  ZOBRIST HASHING
   ===============================================================

   To detect transpositions we need a cheap position fingerprint.

   Zobrist's insight (1970): assign a random number to every
   (colour, piece-type, square) combination, plus randoms for
   side-to-move, en-passant square, and castling rights. XOR
   all active terms together -> near-unique 32-bit key.

   XOR is its own inverse, so incremental updates are trivial:

       remove piece from sq:  hash ^= zobrist_piece[c][t][sq]
       place  piece on  sq:   hash ^= zobrist_piece[c][t][sq]
       (same operation either way - XOR toggles the contribution)

   NOTE: C90 does not guarantee 64-bit integers on all platforms.
   We use 32-bit unsigned long and accept a small (~1-in-4-billion)
   collision rate. Production engines use uint64_t from <stdint.h>.
*/

unsigned long zobrist_piece[2][7][128];
unsigned long zobrist_side;
unsigned long zobrist_ep[128];
unsigned long zobrist_castle[16];

static unsigned long rand32() {
    return ((unsigned long)rand() << 16) | ((unsigned long)rand() & 0xFFFF);
}

void init_zobrist() {
    int c, p, s;
    for (c=0; c<2; c++)
        for (p=0; p<7; p++)
            for (s=0; s<128; s++)
                zobrist_piece[c][p][s] = rand32();
    zobrist_side = rand32();
    for (s=0; s<128; s++) zobrist_ep[s]     = rand32();
    for (s=0; s<16;  s++) zobrist_castle[s] = rand32();
}

/* Full hash regeneration — used only on position setup.
   During search we update hash_key incrementally in make_move()
   and restore it from history[ply].hash_prev in undo_move().    */
unsigned long generate_hash() {
    unsigned long h = 0;
    int sq;
    for (sq=0; sq<128; sq++) {
        if (SQ_IS_OFF(sq)) { sq+=7; continue; }
        if (board[sq])
            h ^= zobrist_piece[COLOR(board[sq])][TYPE(board[sq])][sq];
    }
    if (side == BLACK)        h ^= zobrist_side;
    if (ep_square != SQ_NONE) h ^= zobrist_ep[ep_square];
    h ^= zobrist_castle[castle_rights];
    return h;
}

/* ===============================================================
   S5  ATTACK DETECTION
   ===============================================================

   is_square_attacked(sq, attacker_color)
   --------------------------------------
   Returns 1 if 'attacker_color' has any piece that attacks 'sq'.

   The key insight - reverse the perspective:
   Instead of enumerating all attacker pieces and asking "where
   can each one go?", we fire rays outward from the target square
   and ask "does a ray hit a matching attacker?"

   Think of it as a sonar ping from the target:

       +------------------------------------------+
       |  Fire a knight-ray from sq               |
       |  Does it land on an enemy knight?        |
       |  Yes -> sq is attacked by a knight       |
       +------------------------------------------+

   We check: pawns (fixed offsets), knights, king,
   diagonal sliders (bishop/queen), orthogonal sliders (rook/queen).
*/

int is_square_attacked(int sq, int ac) {
    int i, step, target, p, pt;

    /* Pawns attack diagonally. A White pawn at sq+15 or sq+17
       attacks sq. A Black pawn attacks from sq-15 or sq-17.
       We reverse: look one rank "behind" sq from attacker's view. */
    int pd = (ac==WHITE) ? -16 : 16;
    for (i=-1; i<=1; i+=2) {
        target = sq + pd + i;
        if (!SQ_IS_OFF(target) && board[target]
            && COLOR(board[target])==ac && TYPE(board[target])==PAWN) return 1;
    }

    /* Knights jump in an L — check all 8 landing squares */
    for (i=piece_offsets[KNIGHT]; i<piece_limits[KNIGHT]; i++) {
        target = sq + step_dir[i];
        if (!SQ_IS_OFF(target) && board[target]
            && COLOR(board[target])==ac && TYPE(board[target])==KNIGHT) return 1;
    }

    /* King adjacency — prevents kings from walking next to each other */
    for (i=piece_offsets[KING]; i<piece_limits[KING]; i++) {
        target = sq + step_dir[i];
        if (!SQ_IS_OFF(target) && board[target]
            && COLOR(board[target])==ac && TYPE(board[target])==KING) return 1;
    }

    /* Diagonal rays — hit by Bishop or Queen */
    for (i=piece_offsets[BISHOP]; i<piece_limits[BISHOP]; i++) {
        step = step_dir[i]; target = sq + step;
        while (!SQ_IS_OFF(target)) {
            p = board[target];
            if (p) {
                pt = TYPE(p);
                if (COLOR(p)==ac && (pt==BISHOP || pt==QUEEN)) return 1;
                break; /* blocked by any piece */
            }
            target += step;
        }
    }

    /* Orthogonal rays — hit by Rook or Queen */
    for (i=piece_offsets[ROOK]; i<piece_limits[ROOK]; i++) {
        step = step_dir[i]; target = sq + step;
        while (!SQ_IS_OFF(target)) {
            p = board[target];
            if (p) {
                pt = TYPE(p);
                if (COLOR(p)==ac && (pt==ROOK || pt==QUEEN)) return 1;
                break;
            }
            target += step;
        }
    }
    return 0;
}

/* ===============================================================
   S6  MAKE / UNDO MOVE
   ===============================================================

   make_move(m)
   ------------
   Apply move m to the live board. We first snapshot the three
   things that cannot be deduced from the move alone:
       - what piece was captured
       - the previous en-passant square
       - the previous castling rights
       - the previous Zobrist hash  <- new in v2

   All Zobrist updates are incremental:
       XOR out the "before" contribution, make the change,
       XOR in the "after" contribution.

   Castling rights use a 4-bit mask:
       bit 0 = White O-O     bit 1 = White O-O-O
       bit 2 = Black O-O     bit 3 = Black O-O-O
   Any king or rook move clears the relevant bit(s) permanently.
   When the king castles, we also physically teleport the rook.

   undo_move()
   -----------
   Pop history[ply-1] and reverse every board change.
   The Zobrist key is restored from hash_prev in O(1) - no board
   scan needed. This is the incremental-undo technique that
   replaces the naive generate_hash() call from v1.
*/

static void add_move(Move *list, int *n, int f, int t, int pr) {
    list[(*n)++] = MAKE_MOVE(f, t, pr);
}

void make_move(Move m) {
    int f=FROM(m), t=TO(m), pr=PROMO(m);
    int p=board[f], pt=TYPE(p), cap=board[t];

    /* Snapshot everything we need for undo */
    history[ply].move              = m;
    history[ply].piece_captured    = cap;
    history[ply].ep_square_prev    = ep_square;
    history[ply].castle_rights_prev= castle_rights;
    history[ply].hash_prev         = hash_key;   /* O(1) undo */

    /* En-passant capture: the captured pawn is NOT on the to-square;
       it sits one rank behind. */
    if (pt==PAWN && t==ep_square) {
        int ep_pawn = t + (side==WHITE ? -16 : 16);
        history[ply].piece_captured = board[ep_pawn];
        board[ep_pawn] = EMPTY;
        hash_key ^= zobrist_piece[xside][PAWN][ep_pawn];
    }

    /* Move the piece */
    board[t] = p;  board[f] = EMPTY;
    hash_key ^= zobrist_piece[side][pt][f];   /* XOR out from-square */
    hash_key ^= zobrist_piece[side][pt][t];   /* XOR in  to-square   */
    if (cap) hash_key ^= zobrist_piece[xside][TYPE(cap)][t]; /* remove captured */

    /* Promotion: replace the pawn on to-square with the new piece */
    if (pr) {
        board[t] = PIECE(side, pr);
        hash_key ^= zobrist_piece[side][pt][t]; /* XOR out pawn at t  */
        hash_key ^= zobrist_piece[side][pr][t]; /* XOR in  new piece  */
    }

    /* Castling rights update */
    hash_key ^= zobrist_castle[castle_rights];
    if (pt==KING) {
        king_sq[side] = t;
        /* Teleport the rook when king makes a two-square step */
        if (f==4   && t==6)   { board[7]=EMPTY; board[5]=PIECE(WHITE,ROOK); hash_key^=zobrist_piece[WHITE][ROOK][7]; hash_key^=zobrist_piece[WHITE][ROOK][5]; }
        if (f==4   && t==2)   { board[0]=EMPTY; board[3]=PIECE(WHITE,ROOK); hash_key^=zobrist_piece[WHITE][ROOK][0]; hash_key^=zobrist_piece[WHITE][ROOK][3]; }
        if (f==116 && t==118) { board[119]=EMPTY; board[117]=PIECE(BLACK,ROOK); hash_key^=zobrist_piece[BLACK][ROOK][119]; hash_key^=zobrist_piece[BLACK][ROOK][117]; }
        if (f==116 && t==114) { board[112]=EMPTY; board[115]=PIECE(BLACK,ROOK); hash_key^=zobrist_piece[BLACK][ROOK][112]; hash_key^=zobrist_piece[BLACK][ROOK][115]; }
        castle_rights &= (side==WHITE) ? ~3 : ~12;
    }
    if (f==0   || t==0)   castle_rights &= ~2;
    if (f==7   || t==7)   castle_rights &= ~1;
    if (f==112 || t==112) castle_rights &= ~8;
    if (f==119 || t==119) castle_rights &= ~4;
    hash_key ^= zobrist_castle[castle_rights];

    /* En-passant square: valid only immediately after a double pawn push */
    if (ep_square != SQ_NONE) hash_key ^= zobrist_ep[ep_square];
    ep_square = SQ_NONE;
    if (pt==PAWN && abs(t-f)==32) {
        ep_square = f + (side==WHITE ? 16 : -16);
        hash_key ^= zobrist_ep[ep_square];
    }

    /* Side flip */
    hash_key ^= zobrist_side;
    side ^= 1;  xside ^= 1;  ply++;
}

void undo_move() {
    Move m; int f, t, pr, pt;

    ply--;  side ^= 1;  xside ^= 1;

    m  = history[ply].move;
    f  = FROM(m);  t = TO(m);  pr = PROMO(m);

    board[f] = board[t];
    board[t] = history[ply].piece_captured;

    pt = TYPE(board[f]);

    /* Un-promote: restore the pawn */
    if (pr) board[f] = PIECE(side, PAWN);

    /* Un-en-passant: put the captured pawn back */
    if (pt==PAWN && t==history[ply].ep_square_prev) {
        board[t] = EMPTY;
        board[t + (side==WHITE ? -16 : 16)] = history[ply].piece_captured;
    }

    /* Un-castle: teleport the rook back */
    if (pt==KING) {
        king_sq[side] = f;
        if (f==4   && t==6)   { board[5]=EMPTY; board[7]=PIECE(WHITE,ROOK); }
        if (f==4   && t==2)   { board[3]=EMPTY; board[0]=PIECE(WHITE,ROOK); }
        if (f==116 && t==118) { board[117]=EMPTY; board[119]=PIECE(BLACK,ROOK); }
        if (f==116 && t==114) { board[115]=EMPTY; board[112]=PIECE(BLACK,ROOK); }
    }

    ep_square     = history[ply].ep_square_prev;
    castle_rights = history[ply].castle_rights_prev;
    hash_key      = history[ply].hash_prev;  /* O(1) restore — no board scan */
}

/* ===============================================================
   S7  MOVE GENERATION
   ===============================================================

   generate_moves() fills the 'moves' array and returns the count.
   Moves are PSEUDO-LEGAL: they obey piece geometry but may leave
   the king in check. Legality is verified in the search by calling
   is_square_attacked() after make_move() - this keeps the
   generator simple and fast.

   PAWN LOGIC (special-cased because direction is colour-dependent)
   ----------------------------------------------------------------
   White pawns advance toward higher ranks (+16 per rank step).
   Black pawns advance toward lower  ranks (-16 per rank step).
   d_pawn encodes both in one variable.

   Promotion fires when the pawn reaches the 7th rank (index 6
   for White, index 1 for Black) - it pushes onto the 8th (rank 7
   for White, rank 0 for Black), generating four moves (Q/R/B/N).

   En-passant captures: if the target square equals ep_square,
   the capture is allowed even though the target is empty.

   SLIDER / LEAPER LOOP (Knight, Bishop, Rook, Queen, King)
   --------------------------------------------------------
   Iterate over each direction in the piece's window of step_dir:

       target = sq + step
       while target is on the board:
           if empty    -> add quiet move; continue sliding
           if enemy    -> add capture; stop ray
           if friendly -> stop ray (blocked)
       Knights and Kings break after one step (non-sliders).

   CASTLING (King only)
   --------------------
   Four conditions must all hold:
     1. The castling right bit is set.
     2. Every square between king and rook is empty.
     3. The rook is physically present on its corner square
        (it may have been captured, invalidating the right retroactively).
     4. The king's path does not pass through an attacked square.
*/

int generate_moves(Move *moves) {
    int cnt=0, sq, target, step, p, pt, i;
    int d_pawn     = (side==WHITE) ?  16 : -16;
    int pawn_start = (side==WHITE) ?   1 :  6;
    int pawn_promo = (side==WHITE) ?   6 :  1;

    for (sq=0; sq<128; sq++) {
        if (sq & 0x88) { sq+=7; continue; }
        p = board[sq];
        if (!p || COLOR(p) != side) continue;
        pt = TYPE(p);

        /* ── Pawns ─────────────────────────────────────────── */
        if (pt == PAWN) {
            target = sq + d_pawn;
            if (!SQ_IS_OFF(target) && !board[target]) {
                if ((sq>>4) == pawn_promo) {           /* Promotion rank */
                    add_move(moves,&cnt,sq,target,QUEEN);
                    add_move(moves,&cnt,sq,target,ROOK);
                    add_move(moves,&cnt,sq,target,BISHOP);
                    add_move(moves,&cnt,sq,target,KNIGHT);
                } else {
                    add_move(moves,&cnt,sq,target,0);
                    if ((sq>>4)==pawn_start && !board[target+d_pawn])
                        add_move(moves,&cnt,sq,target+d_pawn,0);
                }
            }
            /* Diagonal captures (and en-passant) */
            for (i=-1; i<=1; i+=2) {
                target = sq + d_pawn + i;
                if (!SQ_IS_OFF(target)
                    && ((board[target] && COLOR(board[target])==xside) || target==ep_square)) {
                    if ((sq>>4)==pawn_promo) {
                        add_move(moves,&cnt,sq,target,QUEEN);
                        add_move(moves,&cnt,sq,target,ROOK);
                        add_move(moves,&cnt,sq,target,BISHOP);
                        add_move(moves,&cnt,sq,target,KNIGHT);
                    } else {
                        add_move(moves,&cnt,sq,target,0);
                    }
                }
            }
            continue;
        }

        /* ── All other pieces via direction table ──────────── */
        for (i=piece_offsets[pt]; i<piece_limits[pt]; i++) {
            step = step_dir[i];
            target = sq + step;
            while (!SQ_IS_OFF(target)) {
                if (!board[target]) {
                    add_move(moves,&cnt,sq,target,0);
                } else {
                    if (COLOR(board[target])==xside)
                        add_move(moves,&cnt,sq,target,0);
                    break; /* blocked — stop ray */
                }
                if (pt==KNIGHT || pt==KING) break; /* leapers: one step */
                target += step;
            }
        }

        /* ── Castling ───────────────────────────────────────── */
        if (pt==KING) {
            if (side==WHITE) {
                if ((castle_rights&1) && !board[5] && !board[6]
                    && board[7]==PIECE(WHITE,ROOK)
                    && !is_square_attacked(4,BLACK)
                    && !is_square_attacked(5,BLACK)
                    && !is_square_attacked(6,BLACK))
                    add_move(moves,&cnt,4,6,0);
                if ((castle_rights&2) && !board[1] && !board[2] && !board[3]
                    && board[0]==PIECE(WHITE,ROOK)
                    && !is_square_attacked(4,BLACK)
                    && !is_square_attacked(3,BLACK)
                    && !is_square_attacked(2,BLACK))
                    add_move(moves,&cnt,4,2,0);
            } else {
                if ((castle_rights&4) && !board[117] && !board[118]
                    && board[119]==PIECE(BLACK,ROOK)
                    && !is_square_attacked(116,WHITE)
                    && !is_square_attacked(117,WHITE)
                    && !is_square_attacked(118,WHITE))
                    add_move(moves,&cnt,116,118,0);
                if ((castle_rights&8) && !board[113] && !board[114] && !board[115]
                    && board[112]==PIECE(BLACK,ROOK)
                    && !is_square_attacked(116,WHITE)
                    && !is_square_attacked(115,WHITE)
                    && !is_square_attacked(114,WHITE))
                    add_move(moves,&cnt,116,114,0);
            }
        }
    }
    return cnt;
}

/* capture-only moves for quiescence search */
int generate_captures(Move *moves) {
    int cnt=0, sq, target, step, p, pt, i;
    int d_pawn     = (side==WHITE) ? 16 : -16;
    int pawn_promo = (side==WHITE) ?  6 :  1;

    for (sq=0; sq<128; sq++) {
        if (sq & 0x88) { sq+=7; continue; }
        p = board[sq];
        if (!p || COLOR(p)!=side) continue;
        pt = TYPE(p);

        if (pt==PAWN) {
            for (i=-1; i<=1; i+=2) {
                target = sq + d_pawn + i;
                if (!SQ_IS_OFF(target) && board[target] && COLOR(board[target])==xside) {
                    if ((sq>>4)==pawn_promo) {
                        add_move(moves,&cnt,sq,target,QUEEN); /* under-promo omitted for speed */
                    } else {
                        add_move(moves,&cnt,sq,target,0);
                    }
                }
            }
            continue;
        }

        for (i=piece_offsets[pt]; i<piece_limits[pt]; i++) {
            step = step_dir[i]; target = sq + step;
            while (!SQ_IS_OFF(target)) {
                if (board[target]) {
                    if (COLOR(board[target])==xside) add_move(moves,&cnt,sq,target,0);
                    break;
                }
                if (pt==KNIGHT || pt==KING) break;
                target += step;
            }
        }
    }
    return cnt;
}

/* ===============================================================
   S8  FEN PARSER
   ===============================================================

   Forsyth-Edwards Notation encodes a full chess position in one
   human-readable ASCII string. Starting position example:

       rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1
       ^------- piece placement (rank 8 down to rank 1) -------^
       ^ '/' separates ranks; digits = consecutive empty squares ^
       ^ uppercase = White pieces, lowercase = Black pieces      ^
                                                    ^side ^castle ^ep

   We parse four fields: placement, side to move, castling, ep.
*/

void parse_fen(const char *fen) {
    int rank=7, file=0, sq, color, piece, i;
    char lo;

    for (i=0; i<128; i++) board[i]=EMPTY;
    castle_rights=0; ep_square=SQ_NONE; ply=0;
    memset(killers, 0, sizeof(killers));

    /* 1. Piece placement */
    while (*fen && *fen!=' ') {
        if (*fen=='/') { file=0; rank--; }
        else if (isdigit(*fen)) { file += *fen-'0'; }
        else {
            sq    = rank*16 + file;
            color = isupper(*fen) ? WHITE : BLACK;
            lo    = (char)tolower(*fen);
            piece = (lo=='p')?PAWN  :(lo=='n')?KNIGHT:(lo=='b')?BISHOP:
                    (lo=='r')?ROOK  :(lo=='q')?QUEEN :KING;
            board[sq] = PIECE(color, piece);
            if (piece==KING) king_sq[color]=sq;
            file++;
        }
        fen++;
    }
    fen++;

    /* 2. Side to move */
    side=(*fen=='w') ? WHITE : BLACK;  xside=side^1;
    fen += 2;

    /* 3. Castling availability */
    while (*fen && *fen!=' ') {
        if (*fen=='K') { castle_rights|=1; }
        if (*fen=='Q') { castle_rights|=2; }
        if (*fen=='k') { castle_rights|=4; }
        if (*fen=='q') { castle_rights|=8; }
        fen++;
    }
    fen++;

    /* 4. En-passant target square */
    if (*fen!='-') ep_square = (fen[1]-'1')*16 + (fen[0]-'a');
}

/* ===============================================================
   S9  EVALUATION
   ===============================================================

   A static evaluation estimates who is winning if the game
   stopped right now. Measured in centipawns (100 cp = 1 pawn).

   MATERIAL (base values):
       Pawn=100  Knight=320  Bishop=330  Rook=500  Queen=900

   PIECE-SQUARE TABLES (PSTs)
   --------------------------
   Pure material counting is blind to position. It scores 1.a4
   the same as 1.e4 — both just advance a pawn one rank. The
   game against TSCP proved this: Chal played a4, f4, h4, h5, g4,
   g5 (all edge/wing pawns) and never developed pieces, because
   every pawn push scored identically.

   PSTs fix this by assigning a positional bonus to each
   (piece, square) pair, encoding centuries of chess knowledge
   into a lookup table:

       score = material_value + pst[piece][square]

   We store tables for White from rank 0 (rank 1) to rank 7 (rank 8).
   For Black, we mirror vertically: pst[Black][sq] = pst[White][mirror(sq)]
   where mirror flips rank 0<->7, 1<->6, etc.

       mirror(sq) = (7 - rank)*16 + file
                  = sq ^ 0x70     <-- elegant XOR trick

   Tables are stored as flat arrays indexed by the 0x88 square
   (only the valid 64 squares matter; phantom squares are never read).

   HOW TO READ A PST
   -----------------
   Each value is a centipawn bonus (positive = good for that piece
   on that square, negative = bad). Values are relative to 0.

   Example - Knight PST (White), reading rank by rank:

       rank 1 (home):  -50 -40 -30 -30 -30 -30 -40 -50
       rank 2:         -40 -20   0   0   0   0 -20 -40
       rank 3:         -30   0  10  15  15  10   0 -30
       rank 4:         -30   5  15  20  20  15   5 -30  <-- ideal squares
       rank 5:         -30   0  15  20  20  15   0 -30
       rank 6:         -30   5  10  15  15  10   5 -30
       rank 7:         -40 -20   0   5   5   0 -20 -40
       rank 8 (enemy): -50 -40 -30 -30 -30 -30 -40 -50

   "A knight on the rim is dim." The corners score -50, the center
   scores +20. This single table guides the engine to develop
   knights toward the center and away from the board edge.

   KING PST (middlegame)
   ---------------------
   The king must be safe. A king on e1 (center) is a liability:
   every open file is a highway for enemy rooks. After castling
   (king on g1 or c1), the pawn shield provides shelter.

       rank 1: +20 +30 +10   0   0 +10 +30 +20  <-- g1/b1 ideal after castling
       rank 2: +20 +20   0   0   0   0 +20 +20
       rank 3: -10 -20 -20 -20 -20 -20 -20 -10  <-- exposed, penalized
       ...

   This alone should stop the engine from ignoring castling.

   BISHOP PAIR BONUS (+30 cp)
   --------------------------
   Two bishops cooperate across the whole board. Empirically
   worth ~30 cp in open positions.
*/

/*
   PSTs are stored White-side-up (rank 0 = rank 1, rank 7 = rank 8).
   Index: rank * 8 + file  (NOT 0x88 index - we convert below).
   For Black: mirror vertically via  rank -> (7-rank).
*/
static const int pst_pawn[64] = {
     0,  0,  0,  0,  0,  0,  0,  0,   /* rank 1 - never reached */
     5, 10, 10,-20,-20, 10, 10,  5,   /* rank 2 - penalize early c/d advance */
     5, -5,-10,  0,  0,-10, -5,  5,
     0,  0,  0, 20, 20,  0,  0,  0,   /* rank 4 - central pawns get bonus  */
     5,  5, 10, 25, 25, 10,  5,  5,
    10, 10, 20, 30, 30, 20, 10, 10,
    50, 50, 50, 50, 50, 50, 50, 50,   /* rank 7 - almost promoting          */
     0,  0,  0,  0,  0,  0,  0,  0
};
static const int pst_knight[64] = {
   -50,-40,-30,-30,-30,-30,-40,-50,
   -40,-20,  0,  5,  5,  0,-20,-40,
   -30,  5, 10, 15, 15, 10,  5,-30,
   -30,  0, 15, 20, 20, 15,  0,-30,
   -30,  5, 15, 20, 20, 15,  5,-30,
   -30,  0, 10, 15, 15, 10,  0,-30,
   -40,-20,  0,  0,  0,  0,-20,-40,
   -50,-40,-30,-30,-30,-30,-40,-50
};
static const int pst_bishop[64] = {
   -20,-10,-10,-10,-10,-10,-10,-20,
   -10,  5,  0,  0,  0,  0,  5,-10,
   -10, 10, 10, 10, 10, 10, 10,-10,
   -10,  0, 10, 10, 10, 10,  0,-10,
   -10,  5,  5, 10, 10,  5,  5,-10,
   -10,  0,  5, 10, 10,  5,  0,-10,
   -10,  0,  0,  0,  0,  0,  0,-10,
   -20,-10,-10,-10,-10,-10,-10,-20
};
static const int pst_rook[64] = {
     0,  0,  0,  5,  5,  0,  0,  0,
    -5,  0,  0,  0,  0,  0,  0, -5,
    -5,  0,  0,  0,  0,  0,  0, -5,
    -5,  0,  0,  0,  0,  0,  0, -5,
    -5,  0,  0,  0,  0,  0,  0, -5,
    -5,  0,  0,  0,  0,  0,  0, -5,
     5, 10, 10, 10, 10, 10, 10,  5,   /* 7th rank bonus */
     0,  0,  0,  0,  0,  0,  0,  0
};
static const int pst_queen[64] = {
   -20,-10,-10, -5, -5,-10,-10,-20,
   -10,  0,  5,  0,  0,  0,  0,-10,
   -10,  5,  5,  5,  5,  5,  0,-10,
     0,  0,  5,  5,  5,  5,  0, -5,
    -5,  0,  5,  5,  5,  5,  0, -5,
   -10,  0,  5,  5,  5,  5,  0,-10,
   -10,  0,  0,  0,  0,  0,  0,-10,
   -20,-10,-10, -5, -5,-10,-10,-20
};
/* King middlegame: castle, stay behind pawns, avoid center */
static const int pst_king[64] = {
    20, 30, 10,  0,  0, 10, 30, 20,   /* rank 1: g1/b1 best after castling */
    20, 20,  0,  0,  0,  0, 20, 20,
   -10,-20,-20,-20,-20,-20,-20,-10,
   -20,-30,-30,-40,-40,-30,-30,-20,
   -30,-40,-40,-50,-50,-40,-40,-30,
   -30,-40,-40,-50,-50,-40,-40,-30,
   -30,-40,-40,-50,-50,-40,-40,-30,
   -30,-40,-40,-50,-50,-40,-40,-30
};

static const int *pst_table[7] = {
    NULL,           /* EMPTY */
    pst_pawn,       /* PAWN   */
    pst_knight,     /* KNIGHT */
    pst_bishop,     /* BISHOP */
    pst_rook,       /* ROOK   */
    pst_queen,      /* QUEEN  */
    pst_king        /* KING   */
};

/* Convert a 0x88 square to a PST index [0..63].
   PST is stored rank 0 = a1 side, rank 7 = a8 side (White view).
   For Black we mirror vertically: rank → (7-rank).            */
static int pst_score(int piece_type, int color, int sq88) {
    int rank = sq88 >> 4;
    int file = sq88 & 7;
    int pst_rank = (color==WHITE) ? rank : (7 - rank);
    return pst_table[piece_type][pst_rank * 8 + file];
}

int evaluate() {
    static const int val[7] = { 0,100,320,330,500,900,20000 };
    int sq, score=0, p, pt, color;
    int bishops[2] = {0,0};

    for (sq=0; sq<128; sq++) {
        if (sq & 0x88) { sq+=7; continue; }
        p = board[sq];
        if (!p) continue;
        pt = TYPE(p); color = COLOR(p);

        /* Material + positional bonus from PST */
        if (color==WHITE) score += val[pt] + pst_score(pt, WHITE, sq);
        else              score -= val[pt] + pst_score(pt, BLACK, sq);

        if (pt==BISHOP) bishops[color]++;
    }

    /* Bishop pair: two bishops cooperate better than bishop+knight */
    if (bishops[WHITE] >= 2) score += 30;
    if (bishops[BLACK] >= 2) score -= 30;

    return (side==WHITE) ? score : -score;
}

/* ===============================================================
   S10  MOVE ORDERING
   ===============================================================

   Alpha-beta is strongest when the best moves are tried first.
   A move that immediately raises alpha prunes all remaining
   branches - wasted work drops exponentially with ordering quality.

   We score moves in three tiers (high to low priority):

   TIER 1 - Hash move (from TT)
   ----------------------------
   The best move found in a previous search of this exact position
   is almost certainly best again. We inject it at position 0
   before sorting. This alone can cut the search tree in half.

   TIER 2 - Captures, scored by MVV-LVA
   ------------------------------------
   MVV-LVA (Most Valuable Victim - Least Valuable Attacker):
       Pawn captures Queen  -> score 1000 + 900 - 100 = 1800
       Queen captures Pawn  -> score 1000 + 100 - 900 = 200
   A cheap piece taking a valuable one is searched first.

   TIER 3 - Killer moves
   ---------------------
   Quiet moves (non-captures) that recently caused a beta cutoff
   at this depth. If they cut once, they likely cut again.
   Score: 900 (below captures, above other quiet moves).

   TIER 4 - Remaining quiet moves
   ------------------------------
   Score: 0. Searched in generation order.

   We use a simple selection sort - O(n^2) but fine for ~30 moves.
*/

static const int order_val[7] = { 0,100,300,300,500,900,0 };

static int score_move(Move m, Move hash_move, int depth) {
    int cap, sc=0;
    if (m == hash_move) return 20000; /* always first */
    cap = board[TO(m)];
    if (cap)            sc = 1000 + order_val[TYPE(cap)] - order_val[TYPE(board[FROM(m)])];
    else if (PROMO(m))  sc = 900;
    else if (depth < MAX_DEPTH
             && (m==killers[depth][0] || m==killers[depth][1])) sc = 800;
    return sc;
}

static void sort_moves(Move *moves, int n, Move hash_move, int depth) {
    int scores[256], i, j, ts;
    Move tm;
    for (i=0; i<n; i++) scores[i] = score_move(moves[i], hash_move, depth);
    for (i=0; i<n-1; i++)
        for (j=i+1; j<n; j++)
            if (scores[j] > scores[i]) {
                ts=scores[i]; tm=moves[i];
                scores[i]=scores[j]; moves[i]=moves[j];
                scores[j]=ts;       moves[j]=tm;
            }
}

/* ===============================================================
   S11  SEARCH
   ===============================================================

   NEGAMAX
   -------
   Minimax requires two alternating functions: maximiser and
   minimiser. Negamax collapses them into one using the symmetry:

       best score for me = -(best score for my opponent)

   Both sides always maximise their own negated child score.
   The single recursive function handles all plies identically.

   ALPHA-BETA PRUNING
   ------------------
   We maintain a window [alpha, beta]:
       alpha = best score we can guarantee (our lower bound)
       beta  = best score the opponent allows (our upper bound)

   If we find a move scoring >= beta, the opponent would have
   avoided this node earlier. We cut off (stop searching) and
   return beta. This can reduce effective branching from ~30 to ~6,
   transforming a depth-6 search into an effective depth-11.

       +-----------------------------------------------------+
       |  search depth 6                                     |
       |  Full minimax nodes:  ~30^6 = ~729,000,000          |
       |  Alpha-beta nodes:    ~30^3 =    ~27,000  (perfect) |
       |  In practice:               ~100,000-500,000        |
       +-----------------------------------------------------+

   KILLER MOVE STORAGE
   -------------------
   When a quiet move causes a beta cutoff, we record it as a
   "killer" for this depth. Next time we reach the same depth,
   killers are tried early (tier 3 in score_move). Two slots per
   depth: new killers shift the old one to slot [1].

   QUIESCENCE SEARCH
   -----------------
   The "horizon effect": if we stop at depth 0 mid-capture sequence,
   the evaluation may look great but the next move blows a piece.
   Example:
       depth 0 - engine evaluates "I'm up a pawn!"
       depth 1 - opponent recaptures, engine is actually down material

   Fix: at depth 0, instead of calling evaluate() immediately,
   we call qsearch() which keeps searching all captures until the
   position is "quiet" (no more captures available). Only then do
   we evaluate. This adds ~60 ELO for roughly 40 lines of code.

   TT PROBE / STORE
   ----------------
   Before searching, check the TT. A matching entry at equal or
   greater depth can give us an immediate result or tighten the
   alpha-beta window:
       TT_EXACT -> return the score directly
       TT_BETA  -> if score >= beta, cut off
       TT_ALPHA -> if score <= alpha, use as upper bound

   After searching, store the result and the best move found.
   Future visits to this position will use the stored best move
   as their first search candidate (see hash-move ordering above).

   ITERATIVE DEEPENING (search_root)
   ---------------------------------
   We don't jump straight to depth N. We search depth 1, then 2,
   then 3 ... up to max_depth. Each shallow pass populates the TT
   with good moves that act as perfect ordering for the next pass.
   The result: depth-N with iterative deepening is stronger than
   a single raw depth-N search, at almost no extra cost.
*/

int qsearch(int alpha, int beta) {
    Move moves[128];
    int cnt, i, sc, stand_pat;

    stand_pat = evaluate();
    if (stand_pat >= beta) return beta;
    if (stand_pat > alpha) alpha = stand_pat;

    cnt = generate_captures(moves);
    sort_moves(moves, cnt, 0, ply);  /* order by MVV-LVA */

    for (i=0; i<cnt; i++) {
        make_move(moves[i]);
        if (is_square_attacked(king_sq[xside], side)) { undo_move(); continue; }
        sc = -qsearch(-beta, -alpha);
        undo_move();
        if (sc >= beta) return beta;
        if (sc > alpha) alpha = sc;
    }
    return alpha;
}

int search(int depth, int alpha, int beta) {
    Move moves[256], best=0, hash_move=0;
    int cnt, i, legal=0, best_sc=-INF, old_alpha=alpha, sc;
    TTEntry *e = &tt[hash_key % TT_SIZE];

    /* TT probe - extract hash_move regardless of depth for ordering */
    if (e->key == hash_key) {
        hash_move = e->best_move;
        if (e->depth >= depth) {
            if (e->flag==TT_EXACT)                        return e->score;
            if (e->flag==TT_BETA  && e->score>=beta)      return beta;
            if (e->flag==TT_ALPHA && e->score<=alpha)     return alpha;
        }
    }

    if (depth==0) return qsearch(alpha, beta);

    cnt = generate_moves(moves);
    sort_moves(moves, cnt, hash_move, ply);

    for (i=0; i<cnt; i++) {
        make_move(moves[i]);
        if (is_square_attacked(king_sq[xside], side)) { undo_move(); continue; }
        legal++;
        sc = -search(depth-1, -beta, -alpha);
        undo_move();

        if (sc > best_sc) { best_sc=sc; best=moves[i]; }
        if (sc > alpha)    alpha = sc;
        if (alpha >= beta) {
            /* Beta cutoff - store this quiet move as a killer */
            if (!board[TO(moves[i])]) {
                int d = ply < MAX_DEPTH ? ply : MAX_DEPTH-1;
                killers[d][1] = killers[d][0];
                killers[d][0] = moves[i];
            }
            break;
        }
    }

    if (!legal)
        return is_square_attacked(king_sq[side],xside) ? -(MATE-ply) : 0;

    /* TT store - depth-preferred replacement policy */
    if (e->key != hash_key || depth >= e->depth) {
        e->key  = hash_key; e->depth=depth; e->best_move=best; e->score=best_sc;
        e->flag = (best_sc<=old_alpha) ? TT_ALPHA : (best_sc>=beta) ? TT_BETA : TT_EXACT;
    }
    return best_sc;
}

void print_move(Move m) {
    int f=FROM(m), t=TO(m), pr=PROMO(m);
    char pc=0;
    if (pr==QUEEN) pc='q'; else if (pr==ROOK) pc='r';
    else if (pr==BISHOP) pc='b'; else if (pr==KNIGHT) pc='n';
    if (pc) printf("%c%c%c%c%c",'a'+(f&7),'1'+(f>>4),'a'+(t&7),'1'+(t>>4),pc);
    else    printf("%c%c%c%c",  'a'+(f&7),'1'+(f>>4),'a'+(t&7),'1'+(t>>4));
}

void search_root(int max_depth) {
    Move moves[256], global_best=0, iter_best;
    int cnt=generate_moves(moves), d, i, sc, best_sc, legal_root=0;

    /* Initial sort before any iteration */
    sort_moves(moves, cnt, 0, 0);

    for (d=1; d<=max_depth; d++) {
        best_sc = -INF;
        iter_best = 0;

        /* Place the previous iteration's best move at front
           so it is searched first - better than re-sorting everything. */
        if (global_best) {
            for (i=0; i<cnt; i++) {
                if (moves[i]==global_best) {
                    Move tmp=moves[0]; moves[0]=moves[i]; moves[i]=tmp;
                    break;
                }
            }
        }

        for (i=0; i<cnt; i++) {
            make_move(moves[i]);
            if (is_square_attacked(king_sq[xside],side)) { undo_move(); continue; }
            if (d==1) legal_root++;

            sc = -search(d-1, -INF, INF);
            undo_move();

            if (sc > best_sc) { best_sc=sc; iter_best=moves[i]; }
        }
        if (iter_best) global_best = iter_best;
    }

    printf("bestmove ");
    if (legal_root && global_best) print_move(global_best);
    else                           printf("0000");
    printf("\n");
    fflush(stdout);
}

/* ===============================================================
   S12  PERFT
   ===============================================================

   perft(depth) counts every reachable leaf node at exactly
   'depth' half-moves from the current position. No evaluation,
   no pruning - pure move-generation correctness test.

   Expected values from the standard starting position:

       depth 1  ->         20  (all opening moves)
       depth 2  ->        400
       depth 3  ->      8,902
       depth 4  ->    197,281
       depth 5  ->  4,865,609

   Any deviation signals a bug in generate_moves or make/undo.
   Run "perft 4" or "perft 5" to verify move generator integrity.
*/

long perft(int depth) {
    Move moves[256]; int cnt, i; long nodes=0;
    if (depth==0) return 1;
    cnt = generate_moves(moves);
    for (i=0; i<cnt; i++) {
        make_move(moves[i]);
        if (!is_square_attacked(king_sq[xside], side))
            nodes += perft(depth-1);
        undo_move();
    }
    return nodes;
}

/* ===============================================================
   S13  UCI LOOP
   ===============================================================

   Universal Chess Interface (UCI) is the text protocol between
   chess engines and GUIs (Arena, Cutechess, Lichess bot, etc.).

   The engine is a pure calculator: stdin -> think -> stdout.
   The GUI handles display, clocks, and game recording.

   Commands we support:
   --------------------
   uci          -> announce id, list options, reply "uciok"
   isready      -> confirm we're initialised, reply "readyok"
   position ... -> load board from startpos or FEN, then replay moves
   go           -> start search, reply "bestmove <move>"
   perft <n>    -> count leaf nodes for move-gen debugging
   quit         -> exit cleanly

   'position startpos moves e2e4 e7e5' sets up the starting
   position and then applies the given move list one by one.
*/

static int parse_move(const char *s, Move *out) {
    int f, t, pr=0;
    Move list[256]; int cnt, i;

    /* Safety: require at least 4 characters for a valid move string */
    if (!s || strlen(s) < 4) return 0;

    f = (s[0]-'a') + (s[1]-'1')*16;
    t = (s[2]-'a') + (s[3]-'1')*16;

    if (strlen(s) > 4) {          /* promotion character present */
        char c = (char)tolower(s[4]);
        if (c=='q') pr=QUEEN; else if (c=='r') pr=ROOK;
        else if (c=='b') pr=BISHOP; else if (c=='n') pr=KNIGHT;
    }

    cnt = generate_moves(list);
    for (i=0; i<cnt; i++)
        if (FROM(list[i])==f && TO(list[i])==t && PROMO(list[i])==pr) {
            *out=list[i]; return 1;
        }
    return 0;
}

void uci_loop() {
    char line[1024], *p;
    Move m;

    srand((unsigned)time(NULL));
    init_zobrist();
    parse_fen("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
    hash_key = generate_hash();

    while (fgets(line, sizeof(line), stdin)) {
        if (!strncmp(line,"uci",3)) {
            printf("id name Chal\nid author Naman Thanki\nuciok\n");
            fflush(stdout);
        }
        else if (!strncmp(line,"isready",7)) {
            printf("readyok\n"); fflush(stdout);
        }
        else if (!strncmp(line,"perft",5)) {
            int depth=4; long nodes;
            sscanf(line,"perft %d",&depth);
            nodes = perft(depth);
            printf("perft depth %d nodes %ld\n",depth,nodes);
            fflush(stdout);
        }
        else if (!strncmp(line,"position",8)) {
            p = line+9;
            if (!strncmp(p,"startpos",8)) {
                parse_fen("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
                p += 8;
            } else if (!strncmp(p,"fen",3)) {
                p += 4; parse_fen(p);
            }
            hash_key = generate_hash();
            p = strstr(line,"moves");
            if (p) {
                p += 6;
                while (*p) {
                    char mv[6];
                    while (*p==' ') p++;
                    if (*p=='\n' || !*p) break;
                    sscanf(p, "%5s", mv);
                    if (parse_move(mv, &m)) make_move(m);
                    p += strlen(mv);
                }
            }
        }
        else if (!strncmp(line,"go",2)) {
            search_root(6);
        }
        else if (!strncmp(line,"quit",4)) {
            break;
        }
    }
}

/* ===============================================================
   ENTRY POINT
   =============================================================== */

int main() {
    setbuf(stdout, NULL);
    uci_loop();
    return 0;
}