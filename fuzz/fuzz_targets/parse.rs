//! Fuzz the whole read path: `Rdb::parse`, then everything reachable
//! from a successfully parsed (i.e. attacker-supplied) RDB: `validate`,
//! the `LSEG` walks, and `PartitionSource` over each extent.
//!
//! The parser is specified to *refuse* hostile input — cycles, off-disk
//! pointers, bad checksums, malformed `SummedLongs` — rather than to
//! trust it. This target proves the refusals are total: every `Result`
//! here is deliberately discarded, because an `Err` is a pass. Only a
//! panic (or a sanitizer report) is a finding.
//!
//! # Input mapping
//!
//! The fuzz input is a raw disk image with a one-byte block-size
//! selector appended:
//!
//! ```text
//! [ .................. image bytes .................. ][ selector ]
//! ```
//!
//! * The image is `data[..len - 1]` — at offset 0, so a corpus entry or
//!   a crash artifact is a real disk image you can point `rdbinfo` (or
//!   `rdbtool`) at after chopping the last byte, and so libFuzzer's
//!   byte-level mutations line up with block boundaries instead of
//!   sliding the whole image sideways.
//! * The selector is the last byte: `512 << (selector % 7)`, i.e. every
//!   supported device block size 512 B .. 32 KB. Block size is a runtime
//!   property of the source in this crate, and `rdb_BlockBytes` must
//!   agree with it, so without this the 4 K/32 K paths (bigger buffers,
//!   longer `lsb_LoadData` runs, a larger `de_TableSize` window) would
//!   never be reached.
//! * An empty input is an empty 512-byte-block disk.
//!
//! The disk exposes `block_count`, and a read past the end fails, so
//! the off-disk and short-disk refusals are exercised for real rather
//! than being papered over with zero fill.

#![no_main]

use amiga_rdb::{BlockSource, PartitionSource, Rdb};
use libfuzzer_sys::fuzz_target;

/// A disk image living in the fuzz input.
struct SliceDisk<'a> {
    data: &'a [u8],
    block_size: usize,
}

impl BlockSource for SliceDisk<'_> {
    type Error = ();

    fn block_size(&self) -> usize {
        self.block_size
    }

    fn read_block(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), ()> {
        let off = lba
            .checked_mul(self.block_size as u64)
            .and_then(|o| usize::try_from(o).ok())
            .ok_or(())?;
        let end = off.checked_add(self.block_size).ok_or(())?;
        if end > self.data.len() {
            return Err(());
        }
        buf.copy_from_slice(&self.data[off..end]);
        Ok(())
    }

    fn block_count(&self) -> Option<u64> {
        Some((self.data.len() / self.block_size) as u64)
    }
}

fuzz_target!(|data: &[u8]| {
    let (image, selector) = match data.split_last() {
        Some((sel, rest)) => (rest, *sel),
        None => (data, 0),
    };
    let block_size = 512usize << (selector % 7);

    let mut disk = SliceDisk {
        data: image,
        block_size,
    };

    let rdb = match Rdb::parse(&mut disk) {
        Ok(rdb) => rdb,
        // Every error is a correct outcome; nothing more to poke at.
        Err(_) => return,
    };

    // Layout checks over what was parsed. Pure arithmetic on
    // attacker-chosen u32s, which is exactly where an overflow panic
    // would live in a debug build.
    let _ = rdb.validate();

    // The LSEG walk is reachable from attacker data too, and is the
    // only part of the read path that allocates proportionally to what
    // the image claims — walk it both ways.
    for fs in &rdb.filesystems {
        let _ = rdb.load_filesystem(fs, &mut disk);
    }
    let _ = rdb.validate_seg_lists(&mut disk);

    // The adapter a filesystem crate mounts, over extents whose
    // `start_lba` and `block_len` came straight off the image. Probing
    // both ends and the far end of the address space is what catches an
    // offset addition that overflows rather than refusing.
    let mut block = vec![0u8; block_size];
    for p in &rdb.partitions {
        let mut view = PartitionSource::new(&mut disk, p);
        for lba in [0, 1, p.block_len.saturating_sub(1), p.block_len, u64::MAX] {
            let _ = view.read_block(lba, &mut block);
        }
    }
});
