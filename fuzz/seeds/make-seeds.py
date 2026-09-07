"""Regenerate the committed fuzz seeds.

    python3 make-seeds.py one-partition-512.img 512
    python3 make-seeds.py one-partition-4k.img 4096

Each seed is a minimal but complete RDB — RDSK, PART, FSHD, LSEG and
BADB, eight blocks, all five checksums valid — with the fuzz target's
block-size selector byte appended (see fuzz_targets/parse.rs). They are
committed because a valid RDSK is gated on a checksum that coverage
feedback cannot guess its way to, so without them libFuzzer would spend
its whole budget outside the parser.

Written in Python rather than reusing the crate's own test fixtures on
purpose: the seeds should stay valid according to the *format*, not
according to whatever this crate currently believes, so a fixture bug
cannot quietly become a fuzzing blind spot.
"""

import math
import struct
import sys

BS = int(sys.argv[2]) if len(sys.argv) > 2 else 512
SEL = int(math.log2(BS // 512))
NBLK = 8
d = bytearray(BS * NBLK)

def p32(blk, off, v):
    struct.pack_into(">I", d, blk*BS+off, v & 0xFFFFFFFF)

def pad(blk, off, ln, s):
    b = s.encode()[:ln].ljust(ln, b' ')
    d[blk*BS+off:blk*BS+off+ln] = b

def bstr(blk, off, s):
    b = s.encode()
    d[blk*BS+off] = len(b)
    d[blk*BS+off+1:blk*BS+off+1+len(b)] = b

def seal(blk, longs):
    p32(blk, 4, longs)
    p32(blk, 8, 0)
    s = 0
    for i in range(longs):
        s = (s + struct.unpack_from(">I", d, blk*BS+i*4)[0]) & 0xFFFFFFFF
    p32(blk, 8, (-s) & 0xFFFFFFFF)

END = 0xFFFFFFFF
RDSK = 0x5244534B
PART = 0x50415254
FSHD = 0x46534844
LSEG = 0x4C534547

# --- RDSK at block 0 ---
p32(0, 0, RDSK)
p32(0, 12, 7)          # HostID
p32(0, 16, BS)         # BlockBytes
p32(0, 20, 0x18)       # Flags: DISKID|CTRLRID
p32(0, 24, 4)          # BadBlockList -> block 4
p32(0, 28, 1)          # PartitionList -> block 1
p32(0, 32, 2)          # FileSysHeaderList -> block 2
p32(0, 36, END)        # DriveInit
p32(0, 64, 4)          # Cylinders
p32(0, 68, 2)          # Sectors
p32(0, 72, 1)          # Heads
p32(0, 76, 1)          # Interleave
p32(0, 80, 4)          # Park
p32(0, 96, 4)          # WritePreComp
p32(0, 100, 4)         # ReducedWrite
p32(0, 104, 3)         # StepRate
p32(0, 128, 0)         # RDBBlocksLo
p32(0, 132, 5)         # RDBBlocksHi
p32(0, 136, 3)         # LoCylinder
p32(0, 140, 3)         # HiCylinder
p32(0, 144, 2)         # CylBlocks
p32(0, 148, 0)         # AutoParkSeconds
p32(0, 152, 4)         # HighRDSKBlock
pad(0, 160, 8, "QUANTUM")
pad(0, 168, 16, "FIREBALL_TM3200S")
pad(0, 184, 4, "300")
pad(0, 188, 8, "CBM")
pad(0, 196, 16, "A4091")
pad(0, 212, 4, "40.9")
seal(0, 64)

# --- PART at block 1 ---
p32(1, 0, PART)
p32(1, 12, 7)          # HostID
p32(1, 16, END)        # Next
p32(1, 20, 1)          # Flags: bootable
bstr(1, 36, "DH0")
E = 128                # DosEnvec base
p32(1, E + 0*4, 16)    # TableSize
p32(1, E + 1*4, BS // 4)  # SizeBlock (longwords)
p32(1, E + 3*4, 1)     # Surfaces
p32(1, E + 5*4, 2)     # BlocksPerTrack
p32(1, E + 9*4, 3)     # LowCyl
p32(1, E + 10*4, 3)    # HighCyl
p32(1, E + 11*4, 30)   # NumBuffers
p32(1, E + 12*4, 0)    # BufMemType
p32(1, E + 13*4, 0x7FFFFFFE)  # MaxTransfer
p32(1, E + 14*4, 0x7FFFFFFE)  # Mask
p32(1, E + 15*4, 0)    # BootPri
p32(1, E + 16*4, 0x444F5303)  # DosType DOS\3
seal(1, 64)

# --- FSHD at block 2 ---
p32(2, 0, FSHD)
p32(2, 12, 7)
p32(2, 16, END)        # Next
p32(2, 32, 0x444F5303) # DosType
p32(2, 36, 0x0028_0001)# Version 40.1
p32(2, 40, 0x80)       # PatchFlags: SegList
p32(2, 44 + 7*4, 3)    # patched[7] = SegListBlocks -> block 3
seal(2, 64)

# --- LSEG at block 3 ---
p32(3, 0, LSEG)
p32(3, 12, 7)
p32(3, 16, END)        # Next
p32(3, 20, 0x000003F3) # HUNK_HEADER, so the payload looks like a driver
seal(3, 128)

# --- BADB at block 4 ---
p32(4, 0, 0x42414442)  # 'BADB'
p32(4, 12, 7)
p32(4, 16, END)        # Next
p32(4, 24, 6)          # first entry: badblock 6
p32(4, 28, 7)          # -> goodblock 7
seal(4, 8)             # SummedLongs 8 -> exactly one entry pair

open(sys.argv[1], "wb").write(bytes(d) + bytes([SEL]))
