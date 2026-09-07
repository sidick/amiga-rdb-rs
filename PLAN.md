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

- [x] **Full `DosEnvec`**: `de_TableSize` 17–20 fields (`de_Baud`,
      `de_Control`, `de_BootBlocks`); expose the raw longwords too, so a
      consumer can round-trip an envec this crate doesn't fully model.
- [x] **Remaining RDSK fields**: vendor/product/revision (space-padded,
      not BCPL), `rdb_HostID`, `rdb_DriveInit`, controller fields,
      `rdb_HighRDSKBlock`, park/interleave/precomp geometry.
- [x] **`rdb_Flags` semantics** as named constants (`LAST`, `LASTLUN`,
      `LASTTID`, `NORESELECT`, `DISKID`, `CTRLRID`, `SYNCH`), not a bare u32.
- [x] **FSHD chain**: `FileSysHeaderBlock` — `fhb_DosType`,
      `fhb_Version`, `fhb_PatchFlags` and the patched fields it gates
      (`Type`, `Task`, `Lock`, `Handler`, `StackSize`, `Priority`,
      `Startup`, `GlobalVec`), `fhb_SegListBlocks` chain head. Same
      cycle/bounds/checksum discipline as PART.
- [x] **LSEG chain**: reassemble `lsb_LoadData` runs into the driver's
      hunk-format binary. This crate reassembles bytes; it does not
      implement hunk relocation (that is the loader's job, wherever the
      driver ends up running). **This is the AROS DOS\7 critical path** —
      reading proves the layout understanding that writing will need.
- [x] **BADB chain**: bad-block lists. Nearly extinct in practice, in
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
- [x] **Partition `SizeBlock` ≠ 128**: filesystem blocks larger than
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
- [x] **Overlap validation** (`Rdb::validate()`): reports (a)
      any chained block — PART, FSHD, LSEG, BADB — lying outside
      `rdb_RDBBlocksLo..=Hi`, and (b) any partition extent overlapping
      the RDB area. Landed with (c) partitions overlapping *each other*
      as well — beyond the original item, but the same failure family,
      the same consequence, and one extra pass over the partition pairs.
      Shape: `Rdb::validate() -> Vec<ValidationIssue>` for the three
      eagerly parsed chains and the extents;
      `Rdb::validate_seg_lists(&mut S)` for LSEG, which is lazy by
      design and so needs the disk back. An inverted `Lo > Hi` area is
      itself an issue, reported once, and suppresses the two checks that
      would otherwise compare against a meaningless range.
      Both are real: partitioning tools have been known
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

- [x] **`BlockSink`**: the write seam, mirroring `BlockSource` —
      `block_size()`, `write_block(&mut self, lba, &[u8])`, optional
      `block_count()`.

      **Decided: two traits, not one.** Read-only sources are the
      common case and nearly all of this crate's work — a file opened
      for reading, a mapped image, a `&[u8]`, an emulator's read-only
      medium — and folding `write_block` into `BlockSource` would force
      every one of them to supply a method that can only fail at
      runtime, throwing away a compile-time truth. Split, "this code
      writes" is visible in the bound: `S: BlockSource + BlockSink`
      where both are needed, `S: BlockSource` where they are not, and a
      read-only source cannot be handed to a writer by accident. The
      cost is `block_size`/`block_count` on both traits — accepted;
      a type implementing both has them agree trivially, and making
      `BlockSink: BlockSource` instead would rule out a write-only
      target (a fresh image streamed out) for no gain.

      `SeekBlockSource` keeps its name and gains
      `impl BlockSink for SeekBlockSource<T> where T: Read + Write +
      Seek` — one type, both directions, so a read view and a write
      view of one file cannot disagree about the block size. A
      read-only `T` simply does not get the impl, which is the split
      earning its keep. The test `MemDisk` grew the matching impl and
      refuses a write past its end, so "the layout runs off the disk"
      surfaces instead of the sink silently growing.

      **`PartitionSink` deliberately not built** — see milestone 3.
- [x] **Checksum sealing**: `seal_checksum(block: &mut [u8],
      summed_longs: u32) -> Result<(), SealError>`, the exact inverse of
      `checksum_ok` and the shared seal for every block writer. Stores
      `SummedLongs` at byte 4, zeroes `ChkSum` at 8, sums that many
      big-endian longwords with wrapping arithmetic and stores the
      negation at 8.

      **Signature notes.** `summed_longs` is the caller's rather than
      inferred, because the count is part of what the *structure* says
      about itself and differs per block type (64 for RDSK/PART/FSHD
      whatever the device block size, `block_size / 4` for LSEG,
      header-plus-entries for BADB) — there is nothing in a half-built
      block to derive it from. It **errors rather than clamps** on a
      count no block could satisfy: clamping would seal a different
      number of longwords than asked for, leaving the block's own
      header disagreeing with the layout the caller was writing —
      better to fail where the arithmetic was done than to write a
      self-consistent lie. Two variants, `SummedLongsTooShort` (below
      `MIN_SUMMED_LONGS` = 3, so the sum would not cover `ChkSum`
      itself and *no* stored value could zero it — its own test proves
      that) and `SummedLongsTooLong`, which also catches a block too
      small to hold the header, so no length panics. Comes with
      `put_be32`, the public inverse of `be32`.

      The test-local `seal()` helper is now a three-line address-
      arithmetic wrapper over the public function, so every fixture in
      the suite — several hundred blocks — exercises the production
      sealer, and the round-trip property is proved incidentally
      everywhere on top of the deliberate sweep
      (`seal_then_checksum_ok_round_trips`: all seven block sizes ×
      six content patterns × six longword counts).
- [x] **Geometry synthesis**: `synthesize_geometry(total_bytes: u64,
      block_size: usize) -> Result<Geometry, GeometryError>`.

      **Convention: amitools' `rdbtool`, pinned against 0.8.1.** Tools
      differ and there is no right answer — geometry is invented on any
      modern medium — so interoperability decides it: amitools' images
      are this crate's fixtures and `xdftool` is the differential
      oracle the round-trip item below diffs against, so a *different*
      geometry for the same size would make every such comparison a
      false positive.

      **What it does**, established empirically by creating images at a
      spread of sizes and reading back what `rdbtool` chose. Two
      candidates, the one wasting fewer bytes wins, first wins an exact
      tie:

      1. *PC-ish*: 63 sectors; heads from the classic BIOS breakpoints
         applied to the **requested byte size** (≤ 504 MiB → 16, then
         32, 64, 128 at each doubling, 256 above 4032 MiB).
      2. *Amiga-ish*: 32 sectors, 1 head, then while cylinders > 65535,
         halve cylinders and double heads.

      Both compute `cylinders = (bytes / block_size) / (heads *
      sectors)`, **rounding down** — a geometry describes at most the
      disk asked for and the trailing partial cylinder is
      unaddressable, which is the only safe direction (rounding up puts
      a partition's last cylinder past the end of the medium).
      Candidate 2 wins almost everywhere, its cylinder being 16 KiB
      against candidate 1's ~504 KiB; the exceptions are sizes that are
      an exact multiple of candidate 1's cylinder, where both waste
      nothing and the tie hands it over — which is why `rdbtool` emits
      63 sectors for exactly 51 609 600 bytes and 32 sectors for
      10 MiB. Head and sector choices do **not** depend on block size
      (candidate 1's table is byte-based, candidate 2's start values
      fixed); only the cylinder count scales, and with it where the
      halving bites.

      **One deliberate deviation**: a candidate whose fields would not
      fit `rdb_Cylinders`/`rdb_Heads`/`rdb_CylBlocks` is discarded, and
      `GeometryError::TooLarge` returned if that leaves none.
      `rdbtool` is Python, whose integers do not wrap, so it has no
      answer here at all; a wrapped geometry describing a disk that is
      not there — with every partition placed against it pointing
      somewhere real and wrong — is the one outcome this crate will not
      produce. The threshold is petabytes past any medium. Below one
      32-block cylinder is `TooSmall` (16 383 bytes at 512 fails,
      16 384 succeeds, matching `rdbtool` exactly).

      The observed triples are unit tests
      (`geometry_matches_rdbtool_0_8_1`, 35 cases). Beyond those, the
      implementation was cross-checked against the live amitools
      `DiskGeometry` on 430 randomized sizes across all seven supported
      block sizes: zero mismatches. Run as an oracle, never copied.
- [x] **RDSK + PART writing**: `RdbBuilder`, two entry points —
      `new(Geometry)` for a caller cloning a disk it already measured,
      `for_size(total_bytes, block_size)` for one that has a size and no
      opinion (it goes through `synthesize_geometry`). Partitions are
      `PartitionSpec::by_size(bytes)` or `by_cylinders(low, high)`, with
      auto or explicit `DriveName`, bootable + `de_BootPri`, dostype, and
      every envec field overridable. `build(&mut sink) -> Result<RdbLayout,
      BuildError<S::Error>>` returns the LBAs and extents it used; the
      tests re-parse the image anyway, on the principle that the disk is
      the only authority on what is on the disk.

      **Rounding direction: down, and this contradicts the assumption the
      item was written under.** `rdbtool` 0.8.1 does *not* round up: its
      `add` is `cyls = num_bytes // rdisk.get_cylinder_bytes()`, plain
      floor division, verified against images it wrote — 10 MiB on a
      129 024-byte cylinder becomes cylinders 1..=81 (10 450 944 bytes),
      not 82, and a size below one cylinder is refused outright
      ("invalid partition range given!"). Matched exactly, including the
      refusal (`BuildError::PartitionTooSmall`), for the interoperability
      reason that pinned the geometry convention. It is also the same
      direction `synthesize_geometry` rounds, so the crate has one rule
      rather than two: a size is a ceiling, and the tool that hands out
      more than it was asked for is the one that walks off the end of
      something. `by_cylinders` exists for the caller who needs the other
      answer exactly.

      **Every default verified against `rdbtool` 0.8.1** by creating
      images and parsing the raw blocks, not from the NDK's suggestions.
      Envec: `de_TableSize` **16** (so `de_Baud`/`de_Control`/
      `de_BootBlocks` are *absent*, not zero — rdbtool only reaches 19
      when one of them is non-zero), `de_SecOrg` 0, `de_SectorPerBlock`
      1, `de_Reserved` **2**, `de_PreAlloc` 0, `de_Interleave` 0,
      `de_NumBuffers` **30**, `de_BufMemType` 0, `de_MaxTransfer`
      **0x00FFFFFF**, `de_Mask` **0x7FFFFFFE**, `de_DosType`
      **0x444F5303** (`DOS\3`). `de_SizeBlock` is the one default that is
      *not* a constant: rdbtool writes `rdb_BlockBytes / 4` (128 at 512,
      **1024 at 4096**), so `PartitionSpec::size_block_longs` is an
      `Option` and `envec_defaults::size_block_longs(block_size)` is the
      rule. RDSK: `rdb_Flags` **0x7** (LAST|LASTLUN|LASTTID), `HostID`
      **7**, `Interleave` 1, `ParkingZone`/`WritePreComp`/`ReducedWrite`
      all == `rdb_Cylinders`, `StepRate` 3, `AutoParkSeconds` 0,
      `DriveInit`/`BadBlockList`/`FileSysHeaderList` all `CHAIN_END`,
      `SummedLongs` **64 whatever the block size**. RDSK at **block 0**
      and PART blocks consecutively from block 1, both observed.

      **One deliberate deviation**: rdbtool writes
      `"RDBTOOL"`/`"IMAGE"`/`"2012"` into the disk identification strings
      while leaving `rdb_Flags` at 0x7, i.e. *without* `DISK_ID`. By the
      format's own rule those bytes then mean nothing; this crate leaves
      them zero rather than plant an unflagged invention a careless
      reader might print.
- [x] **RDB-area sizing done right** — "does not fit" is a hard error
      from `build()` **before any block is written**, and the structure
      guarantees it rather than the discipline doing so: a private
      `layout()` with no sink in scope computes the complete block
      budget, and every check lives there, so there is no path that can
      write block N+1 after discovering block N was the last one. Proved
      by test: each refusal builds into a `MemDisk` of zeros and asserts
      it is still all zeros. Refused cases: partition past the disk's
      last cylinder (sized or explicit), partition reaching into the RDB
      area, partitions overlapping each other, inverted cylinder range,
      a size below one cylinder, more PART blocks than the area holds,
      a sink smaller than the layout, a geometry describing no blocks, a
      duplicate or unstorable `DriveName`, and a sink whose block size is
      not the geometry's. Zero partitions is *not* an error — rdbtool's
      `create` + `init` produces exactly that, `rdb_PartitionList` is
      `CHAIN_END`, and it is what a caller partitioning in a later step
      wants.

      **Reserved-area policy, observed versus exposed.** rdbtool's own
      default *is* a fixed constant, which is the tension this item
      predicted: it reserves the disk's whole **first cylinder**
      (`rdb_RDBBlocksLo` 0, `rdb_RDBBlocksHi = cyl_blocks - 1`,
      `rdb_LoCylinder` 1) regardless of partition count — 0, 1, 2, 5 and
      10 partitions all give `RDBBlocksHi = 31` on a 32-block cylinder,
      with only `rdb_HighRDSKBlock` moving. Resolved in favour of
      interop for the common case and correctness for the uncommon one:
      the default is rdbtool's first cylinder **or what the layout needs
      plus `RDB_HEADROOM_BLOCKS` (16) of slack, whichever is larger**, so
      images match rdbtool's block-for-block until the point where
      matching it would mean overflowing into the first partition — at
      which point the area grows and `rdb_LoCylinder` moves up with it.
      `RdbBuilder::reserved_blocks(n)` overrides both, for a caller
      reproducing an existing image's area or one that knows what it is
      about to store. Note the ceiling this implies and rdbtool shares:
      an area of *n* blocks holds *n − 1* RDB structures, so the FSHD +
      LSEG payloads of the next item will need the override or the
      growth path, not the first cylinder.
- [x] **FSHD + LSEG writing**: `RdbBuilder::filesystem(FileSystemSpec)`,
      matching the `PartitionSpec` idiom — `FileSystemSpec::new(dostype,
      binary)` plus `version(major, minor)` and setters for the
      patch-flag-gated fields. **This is the other half of the AROS
      DOS\7 fix** — ship a long-name filesystem inside the image so a
      3.1-era ROM can mount DOS\7.

      **The binary is not parsed**, per the hunk non-goal: any bytes are
      accepted, split into `block_size - 20`-byte payloads, chained.
      `rdbtool` accepts arbitrary bytes too, which is how the defaults
      below were observed without needing a real handler to hand.

      **Every default verified against `rdbtool` 0.8.1** the same way
      the envec's were — `create + init + fsadd <file>`, then the raw
      FSHD read back. `fhb_PatchFlags` **0x180 and only 0x180**:
      `SegList` (bit 7) and `GlobalVec` (bit 8), with `fhb_GlobalVec`
      **-1** and `Type`/`Task`/`Lock`/`Handler`/`StackSize`/`Priority`/
      `Startup` left zero *and unpatched* — absent, not zero, which is
      exactly the distinction the read side's `Option`s preserve, so
      the spec's defaults are `None` and setting one turns its bit on.
      `fhb_HostID` **0** — matched even though `rdbtool` writes 7 into
      the RDSK and every PART on the same disk, because the field is as
      meaningless on an image either way and byte-identity with the
      oracle is worth more than tidiness. `fhb_Flags` 0, `fhb_Version`
      packed major<<16|minor. Layout: each FSHD immediately followed by
      its own LSEG run, filesystems after the PART blocks, chained in
      the order added — `rdbtool`'s allocation exactly (FSHD 1, LSEG
      2..7, FSHD 8, LSEG 9..14 for two 2560-byte drivers).

      **LSEG `SummedLongs` is the count actually summed**, floored: five
      header longwords plus `payload_len / 4`. Observed either side of
      every boundary — 493 bytes at 512-byte blocks gives `[128, 5]`,
      496 gives `[128, 6]`, 9000 bytes at 4 KB blocks gives
      `[1024, 1024, 217]` — and it is load-bearing rather than cosmetic:
      `rdbtool fsget` recovers a driver's *byte length* from these
      counts, so the differential extracts our image's driver
      byte-for-byte.

      **One deliberate deviation, and it is a bug on the other side.**
      `rdbtool` 0.8.1 writes that reduced count but sums the *whole
      block* for `ChkSum`. The two agree only while the bytes past the
      declared count are zero — true for a driver whose length is a
      multiple of four, false for any other, whose trailing 1..=3 bytes
      then sit outside the sum it actually took. Such a block fails
      `checksum_ok`, and would fail in a 68k ROM just the same. This
      crate writes the same count with the *correct* sum; the deviation
      is pinned by `rdbtool_writes_an_lseg_that_fails_its_own_checksum`
      rather than worked around, because a differential that tolerated
      the oracle being wrong would be testing nothing.

      **Area accounting.** `layout()`'s `needed` is now `1 + partitions
      + Σ(1 + ceil(len / (block_size - 20)))` — the FSHD payload size is
      known up front, as this plan predicted, so the whole budget is
      still computed before the sink is touched and the growth path and
      the `RdbAreaTooSmall`/`SinkTooSmall` refusals extended with no new
      error variants. This is the first thing that makes the growth path
      matter: a 50 KB driver is 103 blocks against a partition's one, so
      the default area moves well past `rdbtool`'s first cylinder and
      `rdb_LoCylinder` with it. (`rdbtool` refuses this case outright —
      "no space in RDB left" — having fixed the area at creation time.)
      Write order tightened to match: each LSEG chain, then its FSHD,
      then the PART blocks, then the RDSK — every block written only
      after everything it points at.
- [x] **Round-trip property**: every create test parses its own output
      and asserts equality; the amitools oracle (GPL — run, never copy)
      diffed against ours in CI.

      **Round-trip.** Each per-feature test already parses its own image
      and asserts `validate()` is silent; the broad one added on top is
      `rebuilding_from_the_parsed_values_reproduces_the_image` — build,
      parse, rebuild through `RdbBuilder::new(geometry)` +
      `by_cylinders` + explicit `reserved_blocks` + the envec off
      `envec_raw` + the FSHD re-added from the parsed header and
      `load_filesystem`'s bytes, and assert the two images are
      **byte-identical**. It doubles as a coverage assertion about the
      *read* surface: anything the builder writes that the parser does
      not expose would fail it.

      *Byte-identity holds for a driver that is a whole number of LSEG
      payloads, and that is the honest limit.* LSEG records no byte
      count, so `load_filesystem` returns the binary padded to a block;
      feeding that back reproduces the same blocks, but a driver that
      did not fill its last block comes back padded and the rebuilt
      final LSEG sums the whole block where the original summed only as
      far as the driver reached. The difference is two longwords in one
      block — a property of the format, not a defect in the rebuild.

      **Differential.** Four env-gated tests
      (`AMIGA_RDB_DIFFERENTIAL=1`, since amitools is not a build
      dependency): the existing `rdbtool <img> list` extent check, plus
      `rdbtool_reads_a_filesystem_this_crate_built` (`info` for the FSHD
      fields, then `fsget` extracting the driver byte-for-byte — which
      proves the chain, the split, the block order and the SummedLongs
      rule at once), `this_crate_reads_a_filesystem_rdbtool_built` (the
      reverse: rdbtool creates and `fsadd`s, we parse and assert the
      0x180/-1/host-0 defaults, `validate()` and `validate_seg_lists()`
      clean), and the checksum-bug pin above.

      **CI**: a `differential` job that `pip install --user
      amitools==0.8.1` — pinned, because every default here was observed
      against that version and a differential against a moving oracle
      tests nothing — and runs `AMIGA_RDB_DIFFERENTIAL=1 cargo test
      rdbtool`, the filter catching the four gated tests plus the three
      that pin rdbtool-observed constants. The workflow's `TODO` now
      names what is left: the boot-level AROS-fixture differential,
      which needs an image a real ROM can be pointed at.

## Milestone 3 — mutate in place

The risky stage, gated on the differential suite existing first.

- [ ] **`PartitionSink`** — the writing counterpart to
      `PartitionSource`, deferred here on purpose rather than built
      alongside `BlockSink` in milestone 2. Nothing in milestone 2 needs
      it: creating an RDB writes only inside `rdb_RDBBlocksLo..=Hi`, and
      a partition's *contents* are out of scope by the crate's founding
      non-goal — one filesystem family per crate, stopping at the
      partition boundary. The consumer that wants it is a filesystem
      crate formatting into a partition, and that consumer's needs
      (does it want the bounds check on every write? a flush? a
      grow-into-free-space story?) are unknown until one exists.
      Building it now would be guessing at an API with no caller,
      which is exactly the mistake `rdb_BlockBytes` taught us to avoid
      in the other direction. Revisit when a real consumer asks.
- [ ] **AmiPart survey first**: before designing the edit API, read
      AmiPart (MIT, so readable closely — unlike xdftool, which stays a
      run-only GPL oracle) to enumerate the operation set and its edge
      cases: what happens to `rdb_HighRDSKBlock` on delete, how resize
      rounds to cylinder boundaries, what it does with holes in the RDB
      area. The API is shaped by real usage before the first line lands.
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
- [ ] **AmiPart as second differential oracle**: once mutation lands,
      apply the same edit in both tools and diff the resulting images
      block-by-block — a second independent implementation alongside
      the xdftool round-trip diff from milestone 2, and one whose
      source can legally be consulted when the diff disagrees.

## Cross-cutting

- [x] **Errors**: `Display` for every error type; `std::error::Error`
      under the `std` feature. `Display` is `core::fmt`, so it holds in
      `no_std` too; the `Io`/`Parent` variants require `E: Display` on
      the impl only, so a source whose error is `()` still parses. Under
      `std`, `source()` returns the wrapped error, so `?` into
      `Box<dyn Error>` works and the chain survives.
- [x] **Fuzzing**: `cargo-fuzz` target for `Rdb::parse` over arbitrary
      images — the parser already refuses cycles/bounds/checksums, and
      the fuzzer's job is to prove there is no panic path left.
      `fuzz/fuzz_targets/parse.rs` parses the input as a disk image and,
      when the parse *succeeds*, keeps going into everything else
      attacker data reaches: `validate()`, `load_filesystem` for each
      FSHD, `validate_seg_lists`, and a `PartitionSource` read at both
      ends of every extent. Every `Result` is discarded on purpose — an
      `Err` is the specified behaviour, so only a panic or a sanitizer
      report counts.

      **Input mapping.** The image sits at offset 0 and the *last* byte
      is a block-size selector, `512 << (sel % 7)`, covering all seven
      supported device block sizes. Image-first keeps libFuzzer's
      mutations aligned to block boundaries (a leading selector would
      slide the whole image sideways on every insert) and makes a crash
      artifact a real disk image once the last byte is chopped off.
      The selector matters because block size is a runtime property of
      the source here and `rdb_BlockBytes` must agree with it, so
      without steering it the 4 K/32 K paths would never be entered.
      The in-memory source reports `block_count` and fails reads past
      the end, so the off-disk refusals are exercised rather than
      papered over with zero fill.

      **Seeds are committed, the corpus is not.** Reaching a valid RDSK
      by mutation means guessing a checksum, which coverage feedback
      cannot steer; `fuzz/seeds/` holds two tiny (4 K and 32 K) images —
      RDSK + PART + FSHD + LSEG + BADB, one 512-byte-block, one
      4 K-block — so all four chains are live from the first run.
      `fuzz/corpus`, `fuzz/artifacts` and `fuzz/target` are gitignored.

      **How to run** (needs nightly and `cargo install cargo-fuzz`):

      ```
      cargo +nightly fuzz run parse fuzz/corpus/parse fuzz/seeds \
          -- -max_total_time=60 -max_len=65536
      ```

      `-max_len` has to clear the 32 KB seed or libFuzzer truncates it
      into something that no longer parses. `fuzz/` is its own workspace
      root, so it never joins the parent build graph: `cargo test` and
      `cargo clippy --all-targets` at the repo root neither see nor need
      it. **CI does not run the fuzzer** — it needs nightly and a time
      budget that does not belong on a per-push job. Fine for now; the
      place for it is a scheduled job, when there is a reason.

      **What it found, first minute:** `parse_part` computed the extent
      as an unchecked `high_cyl - low_cyl + 1`, so an inverted cylinder
      range panicked in a debug build and — the worse half — *wrapped*
      in a release one, conjuring a partition claiming most of the
      address space out of two plausible u32s. Fixed: an inverted range
      is an empty extent (`block_len == 0`, which the overlap checks
      already skip), reported by the new
      `ValidationIssue::PartitionCylindersInverted` rather than refused,
      since a recovery tool needs to see the damage. The two extent
      multiplications saturate now as well — all three factors are
      attacker-controlled u32s and their product need not fit a `u64`.
      That saturation then exposed a second panic one layer out, which
      is why the target reaches into `PartitionSource` at all:
      `read_block` added `start_lba + lba` unchecked, so a saturated
      extent made an *in-range* `lba` overflow the parent LBA. It
      returns `OutOfRange` now — out of range is out of range whichever
      end it falls off. All three cases have regression tests; several
      million runs after the fixes, clean.
- [x] **CI** (GitHub Actions): test on stable, `--no-default-features`
      build, clippy `-D warnings`, rustfmt, docs build (`RUSTDOCFLAGS=-D
      warnings`), MSRV. One workflow, `.github/workflows/ci.yml`, jobs
      parallel. The **differential** job landed with milestone 2's
      round-trip item: `amitools==0.8.1` pinned via `pip install
      --user`, `AMIGA_RDB_DIFFERENTIAL=1 cargo test rdbtool`. What
      remains a `TODO` in the yaml is the boot-level differential — a
      redistributable AROS fixture (amibake's `aros68k` recipe builds
      one from nothing) mounted by a 3.1-era ROM, which is the one
      question the block-level oracle cannot answer.
- [x] **MSRV**: 1.63, in `rust-version` and tested by its own CI job
      (test + build only — clippy/rustfmt run on stable, where their
      opinions are current). Verified by actually running the suite on
      the 1.63.0 toolchain; nothing in the crate reaches past it, and
      the floor is deliberately conservative because retro tooling is
      packaged by distros that move slowly. *(README mention still
      pending.)*
- [ ] **crates.io**: publish at the end of milestone 1 (read-complete
      is a coherent 0.2); semver honestly from then on — the API is
      allowed to break pre-1.0 but not silently.
- [x] **Docs**: every public item documented (`#![deny(missing_docs)]`
      once the surface settles); one worked example in the crate docs
      showing disk → partitions → `PartitionSource` → filesystem crate.
      Denied now that milestone 1 has settled the read surface; the
      paired fields that shared one doc comment (`low_cyl`/`high_cyl`,
      the `rdb_blocks_*` and identification strings, every struct-variant
      error field) now each carry their own, which is the point — the
      *inclusive* half of a range is exactly what a reader gets wrong.
      The crate example builds its own one-partition image in hidden
      doctest lines rather than exposing a `#[doc(hidden)]` fixture
      helper, so `cargo test` runs the whole path for real and the public
      API stays what it says it is.

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
