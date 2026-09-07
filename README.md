# amiga-rdb

Amiga Rigid Disk Block (RDB) partition tables as a pure-Rust,
permissively-licensed library: `RDSK`, `PART`, `FSHD` and `LSEG` blocks,
their checksums, and the `DosEnvec` geometry that tells you where each
partition lives and how to mount it.

In: anything that can read 512-byte blocks (the `BlockSource` trait).
Out: partitions as extents plus metadata, and filesystem-driver
payloads. What's *inside* a partition is deliberately out of scope —
one filesystem family per crate; a partition composes with a filesystem
crate through a small adapter that offsets LBAs into the parent device.

`no_std` + `alloc` at the core; the `std` feature (default) adds only
conveniences. No dependencies.

## Status

Early: read side first, then create, then in-place editing. The API is
not stable yet.

## Why this exists

There was no permissively-licensed RDB implementation available as a
library: amitools is GPL-2, emulators' implementations are GPL, and the
original is in ROM. This crate is MIT OR Apache-2.0 so that emulators,
image-building tools and hobby OS projects can all use it, whatever
their own licence.

Written against the layouts documented in the AmigaOS NDK, with
[AmiPart](https://github.com/ChuckyGang/AmiPart) (MIT) and NetBSD's
Amiga disk support as permissive C references, and tested
differentially against independent implementations.

## License

Dual-licensed under either of

- MIT license ([LICENSE-MIT](LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))

at your option.
