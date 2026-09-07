# Implementation plan

The goal is a *complete* RDB implementation, not a convenient subset:
everything the format can express, readable and writable, so that no
consumer ever has to reach around this crate to amitools or hand-rolled
block pokes. Staged by risk (read → create → mutate), per m68k-machine's
ADR 0004. Boxes get ticked as they land; anything discovered missing
gets *added here first* so the plan stays the map.

## Already landed (0.1)

- [x] `BlockSource` trait, 512-byte blocks, `block_count` optional
- [x] Checksum verification (hostile `SummedLongs` fails, never panics)
- [x] RDSK scan over blocks 0..16, bad-checksum RDSK skipped (stale-copy case)
- [x] PART chain walk: cycle detection, off-disk detection, per-block ID+checksum
- [x] `DosEnvec` through `de_DosType`, extent computation, flags (bootable/noautomount)
- [x] `PartitionSource` — the offsetting adapter filesystem crates mount
- [x] `SeekBlockSource` (`std` only), `examples/rdbinfo.rs`
- [x] Validated against real images: AmigaOS 3.2.2 (DOS\3), AROS (DOS\7),
      two-partition soak layout; agrees with m68k-machine's 68k boot-ROM
      mounter field-for-field

## Milestone 1 — read side complete

Everything the format can say, surfaced. Nothing here writes a byte.

- [ ] **Full `DosEnvec`**: `de_TableSize` 17–20 fields (`de_Baud`,
      `de_Control`, `de_BootBlocks`); expose the raw longwords too, so a
      consumer can round-trip an envec this crate doesn't fully model.
- [ ] **Remaining RDSK fields**: vendor/product/revision (space-padded,
      not BCPL), `rdb_HostID`, `rdb_DriveInit`, controller fields,
      `rdb_HighRDSKBlock`, park/interleave/precomp geometry.
- [ ] **`rdb_Flags` semantics** as named constants (`LAST`, `LASTLUN`,
      `LASTTID`, `NORESELECT`, `DISKID`, `CTRLRID`, `SYNCH`), not a bare u32.
- [ ] **FSHD chain**: `FileSysHeaderBlock` — `fhb_DosType`,
      `fhb_Version`, `fhb_PatchFlags` and the patched fields it gates
      (`Type`, `Task`, `Lock`, `Handler`, `StackSize`, `Priority`,
      `Startup`, `GlobalVec`), `fhb_SegListBlocks` chain head. Same
      cycle/bounds/checksum discipline as PART.
- [ ] **LSEG chain**: reassemble `lsb_LoadData` runs into the driver's
      hunk-format binary. This crate reassembles bytes; it does not
      implement hunk relocation (that is the loader's job, wherever the
      driver ends up running). **This is the AROS DOS\7 critical path** —
      reading proves the layout understanding that writing will need.
- [ ] **BADB chain**: bad-block lists. Nearly extinct in practice, in
      the format forever; read them so a repartitioner can preserve them.
- [x] **Non-512 `rdb_BlockBytes` is required, not optional.** The
      survey answered itself with arithmetic: every RDB block count and
      cylinder field is 32-bit, so 512-byte device blocks cap the
      addressable disk at 2 TB. Larger `BlockBytes` is how the format
      reaches modern media at all (4 KB → 16 TB, and matches flash's
      native block size; 32 KB → 256 TB), and real systems run this
      way today — 8 TB drives carved into 2 TB partitions are in live
      use. Power-of-two sizes 512..=32 KB. **API consequence, decide
      early:** block size becomes a runtime property of the source
      rather than the `BLOCK_SIZE` const baked into `BlockSource`'s
      buffer type — this reshapes the trait, `PartitionSource`, and
      every LBA in the public API (all of which must be documented as
      *device* blocks of the source's size), so it lands before the
      API attracts consumers, not after.
- [ ] **Partition `SizeBlock` ≠ 128**: filesystem blocks larger than
      the device block are not just legal but in live use — FFS with
      32 KB blocks on 2 TB partitions is a working real-world setup.
      `Partition` already records it; extent math and `PartitionSource`
      stay in *device* blocks, documented loudly, since a
      device-block/filesystem-block confusion is exactly where a
      corruption goes silent. (Two independent knobs: `rdb_BlockBytes`
      is the device's block size, `de_SizeBlock` the filesystem's *per
      partition* — and per partition means exactly that: one 512-byte
      RDB disk can carry a 32 KB-block partition and a 4 KB-block
      partition side by side. A synthetic mixed-`SizeBlock` fixture
      goes in the test suite so nothing ever assumes one value per
      disk.)
- [ ] **Overlap validation** (`Rdb::validate()` or similar): report (a)
      any chained block — PART, FSHD, LSEG, BADB — lying outside
      `rdb_RDBBlocksLo..=Hi`, and (b) any partition extent overlapping
      the RDB area. Both are real: partitioning tools have been known
      to write RDB structures past the reserved area into the first
      partition when the area was too small — after which each side
      trashes the other, both believing they own the same blocks — so
      damaged-by-construction images exist in the wild. The
      parser still *reads* them (the data is there and a recovery tool
      needs it); validation is how a consumer learns the layout is
      mutually destructive before either side scribbles on the other.

## Milestone 2 — create from scratch

Write-path code with nothing to corrupt: build a fresh RDB on an empty
target, read it straight back, compare. The API external consumers
(Copperline's dynamic drive creation, amibake's image builds) actually
want.

- [ ] **`BlockSink`** (or `write_block` on a paired trait): the write
      seam, mirroring `BlockSource`. Decide one-trait-or-two once, here.
- [ ] **Checksum sealing**: the inverse of `checksum_ok`, shared by
      every block writer.
- [ ] **Geometry synthesis**: size-in-bytes → cylinders/heads/sectors
      the way real tools do it (and document *which* real tool's
      convention we follow, because they differ; amibake/amitools'
      choice is the pragmatic target since its images are our fixtures).
- [ ] **RDSK + PART writing**: builder API — add partitions by size or
      by cylinder range, auto or explicit `DriveName`, boot priority,
      dostype, the lot. Block allocation within `rdb_RDBBlocksLo..Hi`.
- [ ] **RDB-area sizing done right** — the lesson from the overlap
      bug above: reserve *generously* at creation (partition
      count is known, FSHD payload size is known or estimable — size
      from what will actually be stored, plus headroom for later
      edits, not a fixed small constant), and make "does not fit"
      a **hard error before any block is written**, never a silent
      overflow into partition space. The builder computes its full
      block budget up front; there is no code path that writes block
      N+1 after discovering block N was the last one.
- [ ] **FSHD + LSEG writing**: take a hunk-format filesystem binary,
      split it into LSEG blocks, chain them, patch the FSHD fields.
      **This is the other half of the AROS DOS\7 fix** — ship a
      long-name filesystem inside the image so a 3.1-era ROM can mount
      DOS\7.
- [ ] **Round-trip property**: every create test parses its own output
      and asserts equality; `rdbinfo` output diffed against `xdftool
      <img> open + part` (GPL oracle — run, never copy) in CI where
      xdftool is available.

## Milestone 3 — mutate in place

The risky stage, gated on the differential suite existing first.

- [ ] **Edit existing structures**: add/delete a partition, change
      flags/bootpri/name, grow/shrink where cylinder math allows.
      AmiPart (MIT) is the readable reference for the operations users
      actually perform, resize edge cases included.
- [ ] **Free-block management** inside the RDB area: reuse holes left
      by deleted PART/FSHD/LSEG blocks before extending toward
      `rdb_HighRDSKBlock`.
- [ ] **Never-touch guarantee**: mutation writes only inside
      `rdb_RDBBlocksLo..=Hi`, asserted in code, not just documented —
      partition contents are provably out of reach. This is precisely
      the invariant whose absence causes the overlap bug above; here it
      is structural.
- [ ] **Refuse-over-overlap**: an edit whose blocks don't fit the
      existing RDB area fails with an error naming the shortfall — the
      overlap outcome does not exist as a behaviour — and unlike a
      tool with history, there is no legacy write-anyway path to keep.
- [ ] **Expand the RDB area**: grow `rdb_RDBBlocksHi` (and
      `rdb_HighRDSKBlock`) when — and only when — validation proves the
      blocks being claimed are not inside any partition's extent. The
      realistic enabler for editing old images created with the
      historically tiny default area; pairs with the user resizing or
      moving the first partition to free the space.
- [ ] **Crash-shape discipline**: order writes so an interrupted edit
      leaves the *old* chain intact (write new blocks first, flip the
      chain pointer last). The format has no journal; ordering is all
      there is.

## Cross-cutting

- [ ] **Errors**: `Display` for every error type; `std::error::Error`
      under the `std` feature.
- [ ] **Fuzzing**: `cargo-fuzz` target for `Rdb::parse` over arbitrary
      images — the parser already refuses cycles/bounds/checksums, and
      the fuzzer's job is to prove there is no panic path left.
- [ ] **CI** (GitHub Actions): test on stable, `--no-default-features`
      build, clippy `-D warnings`, rustfmt, docs build. Differential
      job runs when the redistributable AROS fixture can be fetched or
      rebuilt (amibake's `aros68k` recipe builds from nothing).
- [ ] **MSRV**: pick one, state it in Cargo.toml and README, test it in CI.
- [ ] **crates.io**: publish at the end of milestone 1 (read-complete
      is a coherent 0.2); semver honestly from then on — the API is
      allowed to break pre-1.0 but not silently.
- [ ] **Docs**: every public item documented (`#![deny(missing_docs)]`
      once the surface settles); one worked example in the crate docs
      showing disk → partitions → `PartitionSource` → filesystem crate.

## Non-goals, so they don't creep in

- **Hunk relocation/loading** — reassembling LSEG payload bytes is in
  scope; executing or relocating them is the consumer's loader's job.
- **Filesystem contents** — one filesystem family per crate; this crate
  stops at the partition boundary, always.
- **MBR/GPT coexistence** — hybrid PC/Amiga layouts (as on shared CF
  cards) are a consumer-level composition: they can probe MBR
  themselves and hand this crate an offset `BlockSource`. Revisit only
  if a real consumer hits it.
- **Device I/O** — opening `/dev/...`, image file formats with headers
  (ADF is headerless and fine; HDF likewise; anything wrapped —
  vhd/qcow — is the consumer's problem behind `BlockSource`).
