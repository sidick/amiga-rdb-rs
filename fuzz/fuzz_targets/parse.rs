//! Fuzz the whole read path: `Rdb::parse`, then everything reachable
//! from a successfully parsed (i.e. attacker-supplied) RDB: `validate`,
//! the `LSEG` walks, and `PartitionSource` over each extent — and then
//! the *write* path, by editing that RDB and committing it.
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
//!
//! # The editor
//!
//! Once the parse succeeds, `RdbEditor::open` reads the same image and a
//! handful of edits are applied — every one of them derived from the
//! selector byte, so the fuzzer steers them — and committed to a
//! `VecSink` holding a copy of the image. The editor is where the
//! crate's arithmetic is densest (block allocation over an
//! attacker-chosen `rdb_RDBBlocksLo`/`Hi`, cylinder extents, area
//! expansion), so it is exactly where an overflow or an index panic
//! would live. Every edit's `Result` is discarded: refusing a hostile
//! request is the specified behaviour.
//!
//! Two things *are* asserted rather than discarded, because they are
//! the write path's contract: a commit never writes outside
//! `0..=rdb_RDBBlocksHi` (the sink checks it and panics rather than
//! reporting, so a violation is a fuzz finding), and an image a commit
//! reported success on parses back.

#![no_main]

use amiga_rdb::{BlockSink, BlockSource, PartitionSource, Rdb, RdbEditor};
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

/// A writable copy of the image, refusing a write past its end exactly
/// as `SliceDisk` refuses a read past its end — and asserting the
/// never-touch guarantee from outside the crate.
struct VecSink {
    data: Vec<u8>,
    block_size: usize,
    /// `rdb_RDBBlocksHi` as the parse read it. Nothing a commit does may
    /// land above this; `LeasedSink` is supposed to make that
    /// structural, and this is the independent check on it.
    hi: u64,
}

impl BlockSink for VecSink {
    type Error = ();

    fn block_size(&self) -> usize {
        self.block_size
    }

    fn write_block(&mut self, lba: u64, buf: &[u8]) -> Result<(), ()> {
        assert!(
            lba <= self.hi,
            "commit wrote block {lba} above hi {}",
            self.hi
        );
        let off = lba
            .checked_mul(self.block_size as u64)
            .and_then(|o| usize::try_from(o).ok())
            .ok_or(())?;
        let end = off.checked_add(self.block_size).ok_or(())?;
        if end > self.data.len() {
            return Err(());
        }
        self.data[off..end].copy_from_slice(buf);
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

    // ---- the write path, over the same attacker-chosen RDB ----------

    let mut editor = match RdbEditor::open(&mut disk) {
        Ok(editor) => editor,
        // The editor walks the LSEG chains the parser leaves lazy, so it
        // legitimately refuses images `Rdb::parse` accepted.
        Err(_) => return,
    };

    // Edits steered by the selector, each one discarded on refusal.
    let n = u32::from(selector);
    let _ = editor.set_boot_priority(0, n as i32);
    let _ = editor.set_name(0, "FUZZ");
    editor.set_rdb_flags(n);
    let _ = editor.set_lo_cylinder(n % 8);
    let _ = editor.set_geometry_cylinders(rdb.cylinders.wrapping_add(n % 4));
    let _ = editor.expand_rdb_area(rdb.rdb_blocks_hi.saturating_add(n % 8));
    if n % 2 == 0 {
        let _ = editor.remove_partition(0);
    }
    let _ = editor.add_filesystem(amiga_rdb::FileSystemSpec::new(
        0x444F_5303,
        vec![0xAB; (n as usize % 3) * 500],
    ));

    let mut sink = VecSink {
        data: image.to_vec(),
        block_size,
        // The lease the editor is entitled to, which an expansion may
        // legitimately have widened.
        hi: editor.rdb().rdb_blocks_hi as u64,
    };
    if editor.commit(&mut sink).is_ok() {
        // A commit that reported success wrote a complete, sealed
        // layout: it has to parse back.
        let mut written = SliceDisk {
            data: &sink.data,
            block_size,
        };
        let parsed = Rdb::parse(&mut written).expect("a committed RDB must parse back");
        let _ = parsed.validate();
        let _ = parsed.validate_seg_lists(&mut written);
    }
});
