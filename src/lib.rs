//! Amiga Rigid Disk Block (RDB) partition tables.
//!
//! The RDB is the Amiga's partition-table format: a `RDSK` block in the
//! first 16 blocks of a disk, chaining to `PART` blocks (one per
//! partition, each carrying a `DosEnvec` describing geometry and mount
//! parameters), `FSHD`/`LSEG` blocks (loadable filesystem drivers), and
//! `BADB` bad-block lists. Layouts follow the AmigaOS NDK's
//! `devices/hardblocks.h` and `dos/filehandler.h`.
//!
//! This crate is pure format logic. I/O comes in through one trait —
//! [`BlockSource`], "read me block N" — so the same code serves an
//! emulator holding an image file, a tool holding a raw device, and a
//! test holding a `Vec<u8>`. [`BlockSink`] is its write-side mirror,
//! kept separate so that "this code only reads" is a fact the type
//! system enforces, and [`RdbBuilder`] is what writes through it: a
//! whole partition table computed and checked before its first block
//! reaches the disk. What is *inside* a partition is out of
//! scope by design: one filesystem family per crate, composed through
//! an adapter that offsets a partition's LBAs into the parent device.
//!
//! Every LBA in this crate's API — chain pointers, [`Partition`]
//! extents, [`PartitionSource`] addressing — is a *device* block of the
//! source's [`block_size`](BlockSource::block_size), which the RDB's
//! `rdb_BlockBytes` must match. That is deliberately not the same thing
//! as a partition's *filesystem* block size (`de_SizeBlock`), which is
//! per-partition and frequently larger; conflating the two is exactly
//! where silent corruption comes from, so this crate never speaks
//! filesystem blocks.
//!
//! Everything on disk is big-endian; all multi-byte reads go through
//! [`be32`]/[`be16`] rather than any `#[repr(C)]` overlay, so the crate
//! is byte-order- and alignment-safe on any host.
//!
//! # Example
//!
//! The whole read path: a disk, [`Rdb::parse`], the partitions it found,
//! and a [`PartitionSource`] handed to whatever mounts the filesystem.
//! The source here is a `Vec<u8>` so the example is self-contained; a
//! real one is a file (with the `std` feature, [`SeekBlockSource`] wraps
//! any `Read + Seek`) or a raw device.
//!
//! ```
//! use amiga_rdb::{BlockSource, PartitionSource, Rdb};
//!
//! // The one seam: "read me block N". 512-byte blocks here; a 4 KB-sector
//! // disk says 4096 and every LBA below counts in *those* blocks.
//! struct MemDisk(Vec<u8>);
//!
//! impl BlockSource for MemDisk {
//!     type Error = std::io::Error;
//!
//!     fn block_size(&self) -> usize {
//!         512
//!     }
//!
//!     fn read_block(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), Self::Error> {
//!         let off = lba as usize * 512;
//!         let block = self.0.get(off..off + 512).ok_or_else(|| {
//!             std::io::Error::new(std::io::ErrorKind::UnexpectedEof, "past the end of the disk")
//!         })?;
//!         buf.copy_from_slice(block);
//!         Ok(())
//!     }
//!
//!     fn block_count(&self) -> Option<u64> {
//!         Some(self.0.len() as u64 / 512)
//!     }
//! }
//!
//! # // A one-partition image, built here so the example runs as a test.
//! # fn put32(d: &mut [u8], block: usize, off: usize, v: u32) {
//! #     let o = block * 512 + off;
//! #     d[o..o + 4].copy_from_slice(&v.to_be_bytes());
//! # }
//! # fn seal(d: &mut [u8], block: usize) {
//! #     let base = block * 512;
//! #     amiga_rdb::seal_checksum(&mut d[base..base + 512], 64).unwrap();
//! # }
//! # fn image() -> Vec<u8> {
//! #     let mut d = vec![0u8; 320 * 512];
//! #     put32(&mut d, 0, 0, 0x5244_534B);            // RDSK
//! #     put32(&mut d, 0, 16, 512);                   // rdb_BlockBytes
//! #     put32(&mut d, 0, 24, 0xFFFF_FFFF);           // rdb_BadBlockList
//! #     put32(&mut d, 0, 28, 1);                     // rdb_PartitionList
//! #     put32(&mut d, 0, 32, 0xFFFF_FFFF);           // rdb_FileSysHeaderList
//! #     put32(&mut d, 0, 132, 15);                   // rdb_RDBBlocksHi
//! #     seal(&mut d, 0);
//! #     put32(&mut d, 1, 0, 0x5041_5254);            // PART
//! #     put32(&mut d, 1, 16, 0xFFFF_FFFF);           // pb_Next
//! #     d[512 + 36] = 3;                             // pb_DriveName, BCPL
//! #     d[512 + 37..512 + 40].copy_from_slice(b"DH0");
//! #     put32(&mut d, 1, 128, 16);                   // de_TableSize
//! #     put32(&mut d, 1, 128 + 4, 128);              // de_SizeBlock
//! #     put32(&mut d, 1, 128 + 12, 1);               // de_Surfaces
//! #     put32(&mut d, 1, 128 + 20, 32);              // de_BlocksPerTrack
//! #     put32(&mut d, 1, 128 + 36, 2);               // de_LowCyl
//! #     put32(&mut d, 1, 128 + 40, 9);               // de_HighCyl
//! #     put32(&mut d, 1, 128 + 64, 0x444F_5303);     // de_DosType
//! #     seal(&mut d, 1);
//! #     d
//! # }
//! #
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! # let mut disk = MemDisk(image());
//! // let mut disk = MemDisk(std::fs::read("disk.hdf")?);
//! let rdb = Rdb::parse(&mut disk)?;
//!
//! // Two owners of the same blocks is a layout that parses perfectly and
//! // destroys itself on the first write, so ask before trusting it.
//! for issue in rdb.validate() {
//!     eprintln!("layout issue: {issue}");
//! }
//!
//! for partition in &rdb.partitions {
//!     println!(
//!         "{} dostype {:08X} blocks {}..{}",
//!         partition.name,
//!         partition.dos_type,
//!         partition.start_lba,
//!         partition.start_lba + partition.block_len,
//!     );
//!
//!     // The composition seam: LBA 0 of this source is the partition's
//!     // first block on the disk. A filesystem crate mounts one of these
//!     // and never sees the partition table.
//!     let mut source = PartitionSource::new(&mut disk, partition);
//!     let mut boot = vec![0u8; source.block_size()];
//!     source.read_block(0, &mut boot)?;
//!     println!("  first block starts {:02X?}", &boot[..4]);
//! }
//! # assert_eq!(rdb.partitions.len(), 1);
//! # Ok(())
//! # }
//! ```

#![cfg_attr(not(feature = "std"), no_std)]
#![deny(missing_docs)]

extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;

/// Anything that can produce fixed-size blocks by LBA.
///
/// The one seam between this crate and the world. Implementations are
/// expected to be cheap to call repeatedly with the same LBA; the crate
/// does not cache.
///
/// The block size is a runtime property of the source — a real device
/// knows its sector size, an image container knows (or is told) what it
/// holds. 512 bytes is the classic value, but the format's 32-bit block
/// and cylinder fields cap a 512-byte-block disk at 2 TB; larger
/// `rdb_BlockBytes` is how RDB reaches modern media (4 KB → 16 TB), and
/// such disks are in live use. Supported sizes are powers of two in
/// [`MIN_BLOCK_SIZE`]`..=`[`MAX_BLOCK_SIZE`].
pub trait BlockSource {
    /// How this source reports a failed read. No bound is imposed here —
    /// a test's source may fail with `()` — but a source whose error is
    /// [`Display`](core::fmt::Display) makes [`RdbError`] displayable too.
    type Error;

    /// Bytes per device block. Must be constant for the source's
    /// lifetime and a power of two in
    /// [`MIN_BLOCK_SIZE`]`..=`[`MAX_BLOCK_SIZE`]; [`Rdb::parse`]
    /// rejects anything else rather than misreading geometry.
    fn block_size(&self) -> usize;

    /// Read block `lba` into `buf`, whose length is exactly
    /// [`block_size`](Self::block_size).
    fn read_block(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Total number of blocks, if known. `None` is legitimate (a raw
    /// character device may not know); only operations that need the
    /// disk's end require it.
    fn block_count(&self) -> Option<u64> {
        None
    }
}

/// Anything that can accept fixed-size blocks by LBA — the write seam.
///
/// Deliberately a *second* trait rather than `write_block` bolted onto
/// [`BlockSource`], because read-only sources are the common case and
/// the majority of this crate's work: a `File` opened for reading, a
/// memory-mapped image, a `&[u8]`, an emulator's read-only medium. A
/// single trait would force every one of them to supply a `write_block`
/// that can only fail at runtime, which is a compile-time truth thrown
/// away. Splitting them means "this code writes" is visible in the
/// bound: anything that reads *and* writes says `S: BlockSource +
/// BlockSink`, and anything that only reads cannot be handed a sink by
/// accident. The cost is [`block_size`](Self::block_size) and
/// [`block_count`](Self::block_count) appearing on both traits — a
/// deliberate duplication, since a type implementing both will have
/// them agree trivially, and making `BlockSink: BlockSource` instead
/// would rule out a write-only target (a fresh image being streamed
/// out) for no gain.
///
/// The block size is a runtime property, on exactly the terms
/// [`BlockSource`] describes: every LBA is a *device* block of
/// [`block_size`](Self::block_size) bytes, and the RDB's
/// `rdb_BlockBytes` must agree with it.
pub trait BlockSink {
    /// How this sink reports a failed write. No bound is imposed here,
    /// as on [`BlockSource::Error`].
    type Error;

    /// Bytes per device block. Must be constant for the sink's
    /// lifetime, a power of two in
    /// [`MIN_BLOCK_SIZE`]`..=`[`MAX_BLOCK_SIZE`], and — for a type that
    /// is also a [`BlockSource`] — equal to what that trait reports.
    fn block_size(&self) -> usize;

    /// Write `buf` to block `lba`. `buf`'s length is exactly
    /// [`block_size`](Self::block_size); a sink may treat a different
    /// length as a caller bug (this crate never passes one).
    ///
    /// Whether the write has reached stable storage when this returns
    /// is the implementation's business — this crate never assumes it,
    /// and a caller that needs durability flushes the underlying object
    /// itself. Ordering, however, *is* this crate's business: the write
    /// paths are ordered so an interruption leaves the previous
    /// structure intact, which only holds if a sink does not reorder
    /// writes behind the caller's back.
    fn write_block(&mut self, lba: u64, buf: &[u8]) -> Result<(), Self::Error>;

    /// Total number of blocks, if known — `None` on the same terms as
    /// [`BlockSource::block_count`]. A sink that knows its size lets a
    /// writer refuse a layout that runs off the end *before* writing
    /// the first block rather than halfway through.
    fn block_count(&self) -> Option<u64> {
        None
    }
}

/// Smallest supported device block size (and the classic Amiga value).
pub const MIN_BLOCK_SIZE: usize = 512;

/// Largest supported device block size. 32 KB blocks put the format's
/// 32-bit block addressing at 256 TB, comfortably past current media.
pub const MAX_BLOCK_SIZE: usize = 32 * 1024;

/// How many blocks from the start of the disk the `RDSK` block may
/// legally sit in (`RDB_LOCATION_LIMIT`, NDK `devices/hardblocks.h`).
pub const RDB_LOCATION_LIMIT: u64 = 16;

/// Block identifiers, as big-endian magic numbers.
pub mod id {
    /// `RDSK` — the rigid disk block itself.
    pub const RDSK: u32 = 0x5244_534B;
    /// `PART` — one partition.
    pub const PART: u32 = 0x5041_5254;
    /// `FSHD` — filesystem header (a loadable filesystem driver).
    pub const FSHD: u32 = 0x4653_4844;
    /// `LSEG` — one chunk of a filesystem driver's load segments.
    pub const LSEG: u32 = 0x4C53_4547;
    /// `BADB` — bad-block list.
    pub const BADB: u32 = 0x4241_4442;
}

/// `rdb_Flags` bits (`RDBFF_*`, NDK `devices/hardblocks.h`).
///
/// The field stays a plain [`u32`] on [`Rdb`] — the format allows any
/// bit pattern and a reader must preserve what it did not understand —
/// but the bits that *are* defined get names here rather than being
/// spelled `1 << 4` at every call site. Most of them are SCSI-bus
/// scanning hints written by the controller's setup tool, meaningful
/// only to the driver that scans the bus; two of them
/// ([`DISK_ID`](rdb_flags::DISK_ID)/[`CTRLR_ID`](rdb_flags::CTRLR_ID)) gate
/// whether the RDSK's identification strings hold anything real, so a
/// consumer printing those must check.
pub mod rdb_flags {
    /// No disks exist after this one on this controller — the scan may
    /// stop here.
    pub const LAST: u32 = 1 << 0;
    /// No LUNs exist after this one on this target — stop probing LUNs.
    pub const LAST_LUN: u32 = 1 << 1;
    /// No target IDs exist after this one — stop probing targets.
    pub const LAST_TID: u32 = 1 << 2;
    /// The drive may not be told to reselect; the driver must keep the
    /// bus for the whole transfer.
    pub const NO_RESELECT: u32 = 1 << 3;
    /// The `rdb_DiskVendor`/`Product`/`Revision` strings are valid.
    /// Without this bit their bytes mean nothing and must not be shown.
    pub const DISK_ID: u32 = 1 << 4;
    /// The `rdb_ControllerVendor`/`Product`/`Revision` strings are
    /// valid, on the same terms.
    pub const CTRLR_ID: u32 = 1 << 5;
    /// The drive supports synchronous SCSI transfers.
    pub const SYNCH: u32 = 1 << 6;
}

/// `fhb_PatchFlags` bits (NDK `devices/hardblocks.h`, and identically
/// `fse_PatchFlags` in `dos/filehandler.h`).
///
/// The bit mask says which of the `FileSysHeaderBlock`'s tail fields are
/// meaningful and should be substituted into the device node when the
/// filesystem is mounted. Bit *n* covers the *n*th longword after
/// `PatchFlags` itself, so the mapping is positional: unset means "this
/// filesystem does not override that field", which is emphatically not
/// the same as "override it with zero" — hence the [`Option`]s on
/// [`FileSysHeader`].
///
/// Note the ordering trap: [`SEG_LIST`](fshd_patch::SEG_LIST) is bit 7
/// and [`GLOBAL_VEC`](fshd_patch::GLOBAL_VEC) bit
/// 8, because `fhb_SegListBlocks` physically precedes `fhb_GlobalVec` in
/// the block. Sources that describe "eight patched fields" and put
/// `GlobalVec` at bit 7 have silently dropped `SegList` from the count.
pub mod fshd_patch {
    /// `fhb_Type` — the device node type.
    pub const TYPE: u32 = 1 << 0;
    /// `fhb_Task` — handler task pointer (0 for a seglist-loaded handler).
    pub const TASK: u32 = 1 << 1;
    /// `fhb_Lock` — a lock to pass to the handler.
    pub const LOCK: u32 = 1 << 2;
    /// `fhb_Handler` — BSTR name of the handler to load.
    pub const HANDLER: u32 = 1 << 3;
    /// `fhb_StackSize` — handler process stack.
    pub const STACK_SIZE: u32 = 1 << 4;
    /// `fhb_Priority` — handler process priority.
    pub const PRIORITY: u32 = 1 << 5;
    /// `fhb_Startup` — startup value passed to the handler.
    pub const STARTUP: u32 = 1 << 6;
    /// `fhb_SegListBlocks` — the `LSEG` chain head. Surfaced
    /// unconditionally as
    /// [`FileSysHeader::seg_list_blocks`](super::FileSysHeader::seg_list_blocks)
    /// because the chain has to be walkable either way; this bit only
    /// records whether the FSHD asked for it to be patched in.
    pub const SEG_LIST: u32 = 1 << 7;
    /// `fhb_GlobalVec` — BCPL global vector (-1 for a non-BCPL handler).
    pub const GLOBAL_VEC: u32 = 1 << 8;
}

/// The chain terminator used by every block-pointer field
/// (`rdb_PartitionList`, `pb_Next`, ...): `0xFFFFFFFF`, i.e. `-1`, not
/// `0` — block 0 is a valid block address on a disk whose RDSK sits
/// later in the first sixteen.
pub const CHAIN_END: u32 = 0xFFFF_FFFF;

/// Is `size` a device block size this crate accepts?
#[inline]
pub fn block_size_ok(size: usize) -> bool {
    size.is_power_of_two() && (MIN_BLOCK_SIZE..=MAX_BLOCK_SIZE).contains(&size)
}

/// Read a big-endian u32 at byte offset `off`.
#[inline]
pub fn be32(block: &[u8], off: usize) -> u32 {
    u32::from_be_bytes([block[off], block[off + 1], block[off + 2], block[off + 3]])
}

/// Read a big-endian u16 at byte offset `off`.
#[inline]
pub fn be16(block: &[u8], off: usize) -> u16 {
    u16::from_be_bytes([block[off], block[off + 1]])
}

/// Write a big-endian u32 at byte offset `off` — the inverse of
/// [`be32`], and the only way this crate puts a longword into a block.
///
/// Public for the same reason [`be32`] is: a consumer assembling or
/// patching a block field this crate does not model needs the format's
/// byte order without reaching for a `#[repr(C)]` overlay that would be
/// wrong on a little-endian host.
#[inline]
pub fn put_be32(block: &mut [u8], off: usize, v: u32) {
    block[off..off + 4].copy_from_slice(&v.to_be_bytes());
}

/// Byte offsets into the five-longword header every RDB-family block
/// begins with — `RDSK`, `PART`, `FSHD`, `LSEG` and `BADB` alike.
///
/// The first three are what [`checksum_ok`] and [`seal_checksum`] share;
/// `HostID` and `Next` follow (the latter named in [`chain`]).
mod hdr {
    pub const ID: usize = 0;
    pub const SUMMED_LONGS: usize = 4;
    pub const CHK_SUM: usize = 8;
    /// Shared by every block type — the `RDSK`'s `rdb_HostID`, the
    /// `PART`'s `pb_HostID`, and so on: one longword, one offset.
    pub const HOST_ID: usize = 12;
}

/// The smallest `SummedLongs` that can produce a passing checksum: the
/// sum must cover `ChkSum` itself, which is the third longword.
///
/// A count below this is not merely unusual, it is unsatisfiable —
/// storing a value at `ChkSum` that the sum does not include cannot
/// change the sum — so [`checksum_ok`] rejects such a block and
/// [`seal_checksum`] refuses to write one.
pub const MIN_SUMMED_LONGS: u32 = (hdr::CHK_SUM / 4) as u32 + 1;

/// Verify an RDB-family block checksum.
///
/// Every RDB-family block carries `SummedLongs` (longword count, at
/// byte 4) and `ChkSum` (at byte 8) such that the first `SummedLongs`
/// big-endian longwords of the block sum to zero with 32-bit wrapping
/// arithmetic. Returns `false` for a `SummedLongs` that doesn't fit the
/// block — a malformed count must fail the check, not panic the host.
pub fn checksum_ok(block: &[u8]) -> bool {
    let longs = be32(block, hdr::SUMMED_LONGS) as usize;
    if longs == 0 || longs > block.len() / 4 {
        return false;
    }
    let mut sum: u32 = 0;
    for i in 0..longs {
        sum = sum.wrapping_add(be32(block, i * 4));
    }
    sum == 0
}

/// Why [`seal_checksum`] refused to seal a block.
///
/// Both variants describe a `summed_longs` that no block could ever
/// satisfy, so sealing anyway would produce a block [`checksum_ok`]
/// rejects — the one outcome a *writer* must never have. Refusing is
/// deliberately not clamping: clamping would silently seal a different
/// number of longwords than the caller asked for, and since the caller
/// derived that number from the structure it is writing (`SummedLongs`
/// is 64 for `RDSK`/`PART`/`FSHD`, the whole block for `LSEG`, a
/// function of the entry count for `BADB`), a clamp would mean the
/// block's own header disagrees with its layout. Better to fail where
/// the arithmetic was done than to write a self-consistent lie.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SealError {
    /// `summed_longs` is below [`MIN_SUMMED_LONGS`], so the sum would
    /// not cover `ChkSum` and no stored value could make it zero.
    SummedLongsTooShort {
        /// The count that was asked for.
        summed_longs: u32,
    },
    /// `summed_longs` longwords do not fit in the block — including the
    /// case of a block too small to hold the header at all.
    SummedLongsTooLong {
        /// The count that was asked for.
        summed_longs: u32,
        /// How many longwords the block actually holds.
        capacity: usize,
    },
}

impl core::fmt::Display for SealError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            SealError::SummedLongsTooShort { summed_longs } => write!(
                f,
                "SummedLongs {summed_longs} is below the {MIN_SUMMED_LONGS} needed \
                 to cover ChkSum itself"
            ),
            SealError::SummedLongsTooLong {
                summed_longs,
                capacity,
            } => write!(
                f,
                "SummedLongs {summed_longs} exceeds the {capacity} longwords the block holds"
            ),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for SealError {}

/// Seal an RDB-family block: store `summed_longs` and the `ChkSum` that
/// makes the block check out — the exact inverse of [`checksum_ok`].
///
/// Writes `summed_longs` at byte 4, zeroes `ChkSum` at byte 8, sums the
/// first `summed_longs` big-endian longwords with 32-bit wrapping
/// arithmetic, and stores the negation of that sum at `ChkSum`. After
/// this returns `Ok`, `checksum_ok(block)` is `true` — that round trip
/// is the whole contract, and it is what every block writer in this
/// crate depends on.
///
/// `summed_longs` is the caller's, not derived from the block, because
/// the count is part of what the *structure* says about itself and
/// differs per block type: 64 for `RDSK`, `PART` and `FSHD` (which sum
/// their first 256 bytes whatever the device block size), `block_size /
/// 4` for `LSEG` (the whole block is payload), and header-plus-entries
/// for `BADB`. There is nothing in a half-built block to infer it from.
///
/// Fails rather than panics or clamps on a count that cannot work — see
/// [`SealError`]. A block shorter than three longwords is
/// [`SealError::SummedLongsTooLong`] by the same check, so no length is
/// a panic.
pub fn seal_checksum(block: &mut [u8], summed_longs: u32) -> Result<(), SealError> {
    if summed_longs < MIN_SUMMED_LONGS {
        return Err(SealError::SummedLongsTooShort { summed_longs });
    }
    let capacity = block.len() / 4;
    let longs = summed_longs as usize;
    if longs > capacity {
        return Err(SealError::SummedLongsTooLong {
            summed_longs,
            capacity,
        });
    }

    put_be32(block, hdr::SUMMED_LONGS, summed_longs);
    put_be32(block, hdr::CHK_SUM, 0);
    let mut sum: u32 = 0;
    for i in 0..longs {
        sum = sum.wrapping_add(be32(block, i * 4));
    }
    put_be32(block, hdr::CHK_SUM, sum.wrapping_neg());
    Ok(())
}

/// A drive geometry: the cylinders/heads/sectors triple an `RDSK` block
/// stores in `rdb_Cylinders`/`rdb_Heads`/`rdb_Sectors`, and which every
/// partition's `de_LowCyl`/`de_HighCyl` is expressed in.
///
/// Geometry is a fiction on any drive made since the 1990s — the disk
/// reports LBAs and invents whatever CHS the host asks for — but the RDB
/// format has no other way to say where a partition starts, so *some*
/// triple has to be chosen and every tool has to choose the same way for
/// its images to be interchangeable. [`synthesize_geometry`] is where
/// that choice is made and documented.
///
/// The block size rides along because a triple means nothing without it:
/// the same cylinders/heads/sectors describe eight times the disk at
/// 4096-byte blocks that they do at 512.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Geometry {
    /// `rdb_Cylinders`.
    pub cylinders: u32,
    /// `rdb_Heads` — surfaces per cylinder.
    pub heads: u32,
    /// `rdb_Sectors` — device blocks per track.
    pub sectors: u32,
    /// Bytes per device block, matching the source or sink the geometry
    /// is for and what `rdb_BlockBytes` will say.
    pub block_size: usize,
}

impl Geometry {
    /// Device blocks per cylinder — `heads * sectors`, which is what
    /// `rdb_CylBlocks` and a partition's `de_Surfaces *
    /// de_BlocksPerTrack` both hold.
    ///
    /// Saturating, because the fields are public and a hand-built
    /// `Geometry` may hold anything; one from
    /// [`synthesize_geometry`] never comes close.
    pub fn cylinder_blocks(&self) -> u64 {
        (self.heads as u64).saturating_mul(self.sectors as u64)
    }

    /// Device blocks the whole geometry describes, saturating on the
    /// same terms.
    pub fn total_blocks(&self) -> u64 {
        (self.cylinders as u64).saturating_mul(self.cylinder_blocks())
    }

    /// Bytes the whole geometry describes, saturating on the same terms.
    /// For a synthesized geometry this is at most the size asked for,
    /// and short of it by less than one cylinder.
    pub fn total_bytes(&self) -> u64 {
        self.total_blocks().saturating_mul(self.block_size as u64)
    }
}

/// Why [`synthesize_geometry`] could not produce a geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GeometryError {
    /// `block_size` is not a power of two in
    /// [`MIN_BLOCK_SIZE`]`..=`[`MAX_BLOCK_SIZE`].
    UnsupportedBlockSize {
        /// The block size asked for.
        block_size: usize,
    },
    /// The disk is smaller than one cylinder of the smallest geometry
    /// the convention will produce, so there is nothing to describe.
    TooSmall {
        /// The size asked for.
        total_bytes: u64,
        /// The block size asked for.
        block_size: usize,
        /// The smallest size that does yield a geometry, in bytes.
        minimum_bytes: u64,
    },
    /// The disk is so large that no candidate geometry's fields fit the
    /// format's 32-bit `rdb_Cylinders`/`rdb_Heads`/`rdb_CylBlocks`.
    ///
    /// An error rather than a wrap on purpose: a wrapped cylinder count
    /// describes a disk that is not there, and every partition placed
    /// against it would point somewhere real and wrong. (The threshold
    /// is far past any medium — petabytes at 512-byte blocks — but it is
    /// reachable from a `u64` byte count, so it is answered rather than
    /// assumed away.)
    TooLarge {
        /// The size asked for.
        total_bytes: u64,
        /// The block size asked for.
        block_size: usize,
    },
}

impl core::fmt::Display for GeometryError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            GeometryError::UnsupportedBlockSize { block_size } => write!(
                f,
                "unsupported device block size {block_size}: \
                 must be a power of two in {MIN_BLOCK_SIZE}..={MAX_BLOCK_SIZE}"
            ),
            GeometryError::TooSmall {
                total_bytes,
                block_size,
                minimum_bytes,
            } => write!(
                f,
                "{total_bytes} bytes is too small for a geometry in {block_size}-byte blocks: \
                 at least {minimum_bytes} bytes are needed for one cylinder"
            ),
            GeometryError::TooLarge {
                total_bytes,
                block_size,
            } => write!(
                f,
                "{total_bytes} bytes in {block_size}-byte blocks exceeds what the RDB's \
                 32-bit geometry fields can describe"
            ),
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for GeometryError {}

/// Constants of the geometry convention this crate follows — amitools'
/// `rdbtool`, whose images are this crate's fixtures and its
/// differential oracle.
///
/// Two candidate geometries are generated and the one wasting fewer
/// bytes wins; see [`synthesize_geometry`] for the whole story.
mod geo {
    /// The "PC-ish" candidate's fixed sector count: 63, the largest a
    /// PC BIOS's six-bit sector field could hold, and so the number
    /// every PC-derived geometry has used since.
    pub const PC_SECTORS: u64 = 63;

    /// `(inclusive upper bound in KiB, heads)`, in order. A size at
    /// exactly a bound takes that row's head count; anything past the
    /// last row takes [`PC_HEADS_ABOVE`].
    ///
    /// The bounds are the classic BIOS translation breakpoints — 504 MB,
    /// then doubling — expressed in KiB because that is the unit the
    /// comparison is made in, and the comparison is against the
    /// *requested byte size*, not the block count, so it does not move
    /// with the block size.
    pub const PC_HEADS: [(u64, u64); 4] = [
        (504 * 1024, 16),
        (1008 * 1024, 32),
        (2016 * 1024, 64),
        (4032 * 1024, 128),
    ];

    /// Heads for anything past the last [`PC_HEADS`] row.
    pub const PC_HEADS_ABOVE: u64 = 256;

    /// The "Amiga-ish" candidate's fixed sector count: 32 blocks per
    /// track, the value AmigaOS partitioning tools have always used.
    pub const AMIGA_SECTORS: u64 = 32;

    /// The cylinder ceiling the Amiga-ish candidate halves down to.
    /// 65535, not 65536: an artefact of the 16-bit cylinder counts real
    /// controllers had, kept because deviating from it would put this
    /// crate's images a cylinder away from every other tool's.
    pub const MAX_CYLINDERS: u64 = 65535;
}

/// Choose a cylinders/heads/sectors geometry for a disk of
/// `total_bytes` in `block_size`-byte device blocks.
///
/// # Which convention, and why
///
/// **amitools' `rdbtool`, pinned against version 0.8.1.** Tools differ
/// here and there is no right answer — geometry is invented on any
/// modern medium — so the tie-breaker is interoperability: amitools'
/// images are this crate's test fixtures and `xdftool` is the
/// differential oracle milestone 2 diffs against, so producing a
/// *different* geometry for the same size would make every such
/// comparison a false positive. The behaviour below was pinned
/// empirically, by creating images at a spread of sizes and reading back
/// what `rdbtool` chose; the observed triples are unit tests.
///
/// # The convention
///
/// Two candidates are generated and the one wasting fewer bytes wins,
/// with the first winning an exact tie:
///
/// 1. **PC-ish**: 63 sectors, heads from a table of the classic BIOS
///    translation breakpoints applied to the *requested byte size*
///    (≤ 504 MiB → 16 heads, then 32, 64, 128 at each doubling, 256
///    above 4032 MiB).
/// 2. **Amiga-ish**: 32 sectors, 1 head — then, while the cylinder count
///    exceeds 65535, halve the cylinders and double the heads.
///
/// Both compute `cylinders = (total_bytes / block_size) / (heads *
/// sectors)`, **rounding the cylinder count down**. A geometry therefore
/// describes at most the size asked for and never more: the last partial
/// cylinder of a disk whose size is not a whole number of them is simply
/// unaddressable, which is what every real tool does and the only safe
/// direction to round — rounding up would place a partition's last
/// cylinder past the end of the medium.
///
/// In practice candidate 2 wins almost everywhere, its cylinders being
/// 16 KiB apart at 512-byte blocks against candidate 1's ~504 KiB. The
/// exceptions are sizes that are an exact multiple of candidate 1's
/// cylinder size, where both waste nothing and the tie hands it to
/// candidate 1 — which is why `rdbtool` emits a 63-sector geometry for
/// (say) exactly 51 609 600 bytes and a 32-sector one for 10 MiB.
///
/// # Deviations, deliberately
///
/// A candidate whose fields would not fit the format's 32-bit
/// `rdb_Cylinders`/`rdb_Heads`/`rdb_CylBlocks` is discarded rather than
/// truncated, and [`GeometryError::TooLarge`] is returned if that leaves
/// none. `rdbtool` is written in Python, whose integers do not wrap, and
/// so has no answer here at all; a wrapped geometry describing a disk
/// that is not there is the one outcome this crate will not produce. The
/// threshold is petabytes away from any real medium, so this never
/// changes the answer for a size anyone will ask for.
///
/// # Block size
///
/// The head and sector choices do *not* depend on `block_size` — the
/// candidate-1 table is byte-based and candidate 2's start values are
/// fixed — so only the cylinder count scales with it, which is what was
/// observed. `block_size` still matters to the *result*, because it
/// decides where the 65535-cylinder halving kicks in.
pub fn synthesize_geometry(total_bytes: u64, block_size: usize) -> Result<Geometry, GeometryError> {
    if !block_size_ok(block_size) {
        return Err(GeometryError::UnsupportedBlockSize { block_size });
    }
    let bs = block_size as u64;
    let total_blocks = total_bytes / bs;

    // Candidate 2 with one head is the smallest geometry the convention
    // can produce, so a disk short of one of its cylinders has no
    // geometry at all — and candidate 1's cylinder is 31.5 times larger,
    // so there is no size where it rescues one that fails here.
    let minimum_bytes = geo::AMIGA_SECTORS * bs;
    if total_blocks < geo::AMIGA_SECTORS {
        return Err(GeometryError::TooSmall {
            total_bytes,
            block_size,
            minimum_bytes,
        });
    }

    // Candidate 1 first, because the waste comparison below keeps the
    // incumbent on a tie and an exact tie must go to candidate 1.
    let candidates = [
        pc_geometry(total_bytes, total_blocks, block_size),
        amiga_geometry(total_blocks, block_size),
    ];
    let mut best: Option<(Geometry, u64)> = None;
    for g in candidates.into_iter().flatten() {
        let waste = total_bytes - g.total_bytes();
        let better = match best {
            Some((_, incumbent)) => waste < incumbent,
            None => true,
        };
        if better {
            best = Some((g, waste));
        }
    }

    match best {
        Some((g, _)) => Ok(g),
        None => Err(GeometryError::TooLarge {
            total_bytes,
            block_size,
        }),
    }
}

/// Candidate 1: 63 sectors, heads from the BIOS-breakpoint table.
///
/// The table is consulted with the *requested byte size*, not the block
/// count — so a 4 KB-block disk of a given size picks the same head
/// count as a 512-byte-block one of that size.
fn pc_geometry(total_bytes: u64, total_blocks: u64, block_size: usize) -> Option<Geometry> {
    let kib = total_bytes / 1024;
    let mut heads = geo::PC_HEADS_ABOVE;
    for &(limit_kib, h) in geo::PC_HEADS.iter() {
        if kib <= limit_kib {
            heads = h;
            break;
        }
    }
    let cylinders = total_blocks / (heads * geo::PC_SECTORS);
    representable(cylinders, heads, geo::PC_SECTORS, block_size)
}

/// Candidate 2: 32 sectors, one head, halving cylinders and doubling
/// heads until the cylinder count fits the 65535 ceiling.
///
/// The halving is applied to the *already floored* cylinder count, so an
/// odd count loses a whole cylinder rather than half of one — matching
/// what was observed, and the reason a 65537-cylinder disk ends up 16 KiB
/// short where a 65536-cylinder one is exact.
fn amiga_geometry(total_blocks: u64, block_size: usize) -> Option<Geometry> {
    let mut heads: u64 = 1;
    let mut cylinders = total_blocks / geo::AMIGA_SECTORS;
    while cylinders > geo::MAX_CYLINDERS {
        cylinders /= 2;
        // Terminates: cylinders strictly decreases while above 65535,
        // and `heads` is only checked against the u32 ceiling at the end
        // because doubling it in a u64 cannot overflow first (the loop
        // runs at most 64 times).
        heads = heads.saturating_mul(2);
    }
    representable(cylinders, heads, geo::AMIGA_SECTORS, block_size)
}

/// A candidate geometry, or `None` if it describes nothing or does not
/// fit the format's 32-bit fields.
///
/// `rdb_CylBlocks` is a single longword as much as `rdb_Heads` is, so
/// `heads * sectors` is checked too and not just the factors.
fn representable(cylinders: u64, heads: u64, sectors: u64, block_size: usize) -> Option<Geometry> {
    let max = u32::MAX as u64;
    if cylinders == 0 || cylinders > max || heads > max || sectors > max {
        return None;
    }
    heads.checked_mul(sectors).filter(|&cb| cb <= max)?;
    Some(Geometry {
        cylinders: cylinders as u32,
        heads: heads as u32,
        sectors: sectors as u32,
        block_size,
    })
}

/// Errors from parsing an RDB.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RdbError<E> {
    /// The underlying [`BlockSource`] failed.
    Io(E),
    /// The source's [`block_size`](BlockSource::block_size) is not a
    /// power of two in [`MIN_BLOCK_SIZE`]`..=`[`MAX_BLOCK_SIZE`].
    UnsupportedBlockSize {
        /// What the source said its block size was.
        block_size: usize,
    },
    /// No valid `RDSK` block in the first [`RDB_LOCATION_LIMIT`] blocks.
    ///
    /// Not necessarily damage: RDB-less images (a bare filesystem from
    /// block 0) are a real, if less common, layout — this is how a
    /// caller detects one.
    NoRdsk,
    /// A block in a chain had the wrong ID. `expected`/`found` are the
    /// magic numbers; `lba` is where.
    WrongId {
        /// The block the chain pointed at.
        lba: u64,
        /// The magic number the chain's type requires (see [`id`]).
        expected: u32,
        /// The magic number actually found there.
        found: u32,
    },
    /// A block's checksum failed. The chain is reported broken rather
    /// than the block trusted: a bad checksum on this platform usually
    /// means a bug wrote it, and silently accepting it is how images
    /// get corrupted further.
    BadChecksum {
        /// The block whose `ChkSum` did not add up.
        lba: u64,
    },
    /// A chain pointer walked past the end of the disk (only detectable
    /// when [`BlockSource::block_count`] is `Some`).
    ChainOutOfRange {
        /// The off-disk block the chain pointed at.
        lba: u64,
    },
    /// A chain revisited a block — a cycle. Without this check a
    /// crafted or corrupted image loops the parser forever.
    ChainCycle {
        /// The block the chain came back to.
        lba: u64,
    },
    /// `rdb_BlockBytes` disagrees with the source's
    /// [`block_size`](BlockSource::block_size). Every LBA in the RDB is
    /// in `rdb_BlockBytes` units; reading them through a differently
    /// sized source would silently address the wrong bytes, so the
    /// mismatch is an error, not a guess. (An image of a 4 KB-sector
    /// disk must be presented by a source that says 4096.)
    BlockBytesMismatch {
        /// `rdb_BlockBytes`, as the `RDSK` block declares it.
        block_bytes: u32,
        /// What the source says it reads in.
        block_size: usize,
    },
    /// A `PART` block's `DosEnvec` was too short to contain the fields
    /// this crate needs (`de_TableSize` below `DE_DOSTYPE`).
    EnvecTooShort {
        /// The `PART` block carrying the short envec.
        lba: u64,
        /// The `de_TableSize` it declared.
        table_size: u32,
    },
}

impl<E: core::fmt::Display> core::fmt::Display for RdbError<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            RdbError::Io(e) => write!(f, "reading a block failed: {e}"),
            RdbError::UnsupportedBlockSize { block_size } => write!(
                f,
                "unsupported device block size {block_size}: \
                 must be a power of two in {MIN_BLOCK_SIZE}..={MAX_BLOCK_SIZE}"
            ),
            RdbError::NoRdsk => write!(
                f,
                "no valid RDSK block in the first {RDB_LOCATION_LIMIT} blocks"
            ),
            RdbError::WrongId {
                lba,
                expected,
                found,
            } => write!(
                f,
                "block {lba} has ID {} where {} was expected",
                Fourcc(*found),
                Fourcc(*expected)
            ),
            RdbError::BadChecksum { lba } => {
                write!(f, "block {lba} has a bad checksum")
            }
            RdbError::ChainOutOfRange { lba } => {
                write!(
                    f,
                    "a chain pointed at block {lba}, past the end of the disk"
                )
            }
            RdbError::ChainCycle { lba } => {
                write!(f, "a chain loops back to block {lba}")
            }
            RdbError::BlockBytesMismatch {
                block_bytes,
                block_size,
            } => write!(
                f,
                "the RDB declares {block_bytes}-byte blocks but the source reads \
                 {block_size}-byte blocks"
            ),
            RdbError::EnvecTooShort { lba, table_size } => write!(
                f,
                "the DosEnvec in the PART block at {lba} declares de_TableSize {table_size}, \
                 short of the {} needed to reach de_DosType",
                de::DOS_TYPE
            ),
        }
    }
}

#[cfg(feature = "std")]
impl<E: std::error::Error + 'static> std::error::Error for RdbError<E> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            RdbError::Io(e) => Some(e),
            _ => None,
        }
    }
}

/// A block ID rendered the way the format writes it — four ASCII
/// characters, e.g. `PART` — falling back to hex for the corrupt case
/// that produced the error in the first place.
struct Fourcc(u32);

impl core::fmt::Display for Fourcc {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let b = self.0.to_be_bytes();
        if b.iter().all(|c| (0x20..0x7F).contains(c)) {
            for c in b {
                write!(f, "{}", c as char)?;
            }
            Ok(())
        } else {
            write!(f, "{:#010x}", self.0)
        }
    }
}

/// One partition, as read from a `PART` block.
///
/// All block quantities (`start_lba`, `block_len`, `cylinder_blocks`)
/// are *device* blocks of the parent source's size — never
/// `de_SizeBlock` filesystem blocks, which may be larger and vary per
/// partition on one disk.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Partition {
    /// LBA of the `PART` block this came from.
    pub part_block: u64,
    /// The drive name (`pb_DriveName`, BCPL string) — e.g. `DH0`.
    pub name: String,
    /// `pb_Flags` bit 0: bootable.
    pub bootable: bool,
    /// `pb_Flags` bit 1: present but not to be mounted automatically.
    pub no_automount: bool,
    /// First block of the partition, in disk device-block LBAs.
    /// (`de_LowCyl * cylinder_blocks`, saturating: all three factors are
    /// attacker-controlled u32s and their product need not fit a u64.)
    pub start_lba: u64,
    /// Number of device blocks in the partition:
    /// `(high_cyl - low_cyl + 1) * cylinder_blocks`, saturating.
    ///
    /// **Zero when `high_cyl < low_cyl`** — an inverted cylinder range
    /// describes no blocks, so that is its length, and
    /// [`ValidationIssue::PartitionCylindersInverted`] is how a consumer
    /// hears about it. The raw [`low_cyl`](Self::low_cyl) and
    /// [`high_cyl`](Self::high_cyl) are still exactly what was on disk.
    pub block_len: u64,
    /// `de_DosType` — e.g. `0x444F5303` (`DOS\x03`).
    pub dos_type: u32,
    /// `de_BootPri`.
    pub boot_pri: i32,
    /// `de_MaxTransfer`.
    pub max_transfer: u32,
    /// `de_Mask`.
    pub mask: u32,
    /// Device blocks per cylinder (`de_Surfaces * de_BlocksPerTrack`),
    /// kept because filesystems and repartitioners both need it.
    pub cylinder_blocks: u64,
    /// `de_LowCyl` — the partition's first cylinder.
    pub low_cyl: u32,
    /// `de_HighCyl` — its last cylinder, *inclusive*.
    pub high_cyl: u32,
    /// `de_NumBuffers` — a mount parameter the handler needs, saying
    /// nothing about the disk itself.
    pub num_buffers: u32,
    /// `de_BufMemType` — which memory those buffers want, likewise.
    pub buf_mem_type: u32,
    /// `de_SizeBlock` in longwords (128 == 512-byte filesystem blocks).
    /// Per partition: one disk can carry differently sized filesystem
    /// blocks side by side.
    pub size_block_longs: u32,
    /// `de_Baud` (envec longword 17) — serial rate for a
    /// serial-attached handler. `None` when `de_TableSize` stops short
    /// of it: absent and zero are different things, and a writer that
    /// rounds-trips must not invent a field the original did not have.
    pub baud: Option<u32>,
    /// `de_Control` (18) — handler-defined control word.
    pub control: Option<u32>,
    /// `de_BootBlocks` (19) — number of blocks reserved for boot code.
    pub boot_blocks: Option<u32>,
    /// The `DosEnvec` as raw longwords, `de_TableSize` included, so the
    /// vector holds `table_size + 1` entries — everything the envec
    /// claims, whether or not this crate models it. A consumer can
    /// round-trip an envec byte-for-byte from this.
    ///
    /// Clamped to what actually fits between the envec's offset (128)
    /// and the end of the block: `de_TableSize` is attacker-controlled
    /// and a hostile value must truncate, never read past the block.
    /// So `envec_raw.len()` is `min(table_size + 1, (block_size - 128) /
    /// 4)` — compare it against `table_size` to detect the truncation.
    pub envec_raw: Vec<u32>,
}

/// One loadable filesystem driver, as read from a `FileSysHeaderBlock`.
///
/// An RDB may carry the filesystem handlers its partitions need, so a
/// ROM that has never heard of (say) `DOS\x07` can still mount it: the
/// boot code finds the FSHD whose `dos_type` matches the partition's,
/// reassembles the driver binary from the `LSEG` chain
/// ([`Rdb::load_filesystem`]), and patches the fields this struct gates
/// into the device node.
///
/// The eight [`Option`] fields are gated by [`patch_flags`](Self::patch_flags)
/// — see [`fshd_patch`]. `None` means the FSHD does not override that
/// device-node field at all; it does *not* mean zero, and a writer that
/// round-trips this must not turn one into the other.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FileSysHeader {
    /// LBA of the `FSHD` block this came from.
    pub fshd_block: u64,
    /// `fhb_HostID` — SCSI ID of the owning controller, as on the RDSK.
    pub host_id: u32,
    /// `fhb_Flags`. No bits are defined by the NDK; carried so a
    /// rewrite does not drop whatever a tool put there.
    pub flags: u32,
    /// `fhb_DosType` — which partitions this driver serves, matched
    /// against a partition's [`Partition::dos_type`].
    pub dos_type: u32,
    /// `fhb_Version`, raw: major in the high 16 bits, minor in the low.
    /// Kept as the packed longword because that is what the format
    /// stores and what a version comparison actually wants (a plain
    /// `>` on the whole word orders releases correctly); split it with
    /// [`version_major`](Self::version_major) /
    /// [`version_minor`](Self::version_minor).
    pub version: u32,
    /// `fhb_PatchFlags` — which of the fields below are significant.
    /// Preserved raw, bits this crate does not model included.
    pub patch_flags: u32,
    /// `fhb_Type`, gated by [`fshd_patch::TYPE`].
    pub node_type: Option<u32>,
    /// `fhb_Task`, gated by [`fshd_patch::TASK`].
    pub task: Option<u32>,
    /// `fhb_Lock`, gated by [`fshd_patch::LOCK`].
    pub lock: Option<u32>,
    /// `fhb_Handler`, gated by [`fshd_patch::HANDLER`].
    pub handler: Option<u32>,
    /// `fhb_StackSize`, gated by [`fshd_patch::STACK_SIZE`].
    pub stack_size: Option<u32>,
    /// `fhb_Priority` (signed), gated by [`fshd_patch::PRIORITY`].
    pub priority: Option<i32>,
    /// `fhb_Startup` (signed), gated by [`fshd_patch::STARTUP`].
    pub startup: Option<i32>,
    /// `fhb_GlobalVec` (signed; -1 means "not BCPL"), gated by
    /// [`fshd_patch::GLOBAL_VEC`].
    pub global_vec: Option<i32>,
    /// `fhb_SegListBlocks` — head of this driver's `LSEG` chain, or
    /// [`CHAIN_END`] when the FSHD carries no binary (a header that
    /// only patches device-node fields for a filesystem already in ROM).
    ///
    /// Not an `Option` despite [`fshd_patch::SEG_LIST`] existing: the
    /// chain must be walkable to load the driver regardless of whether
    /// the FSHD asked for the pointer to be patched into the node, and
    /// `CHAIN_END` already expresses "no chain" unambiguously. The bit
    /// itself survives in [`patch_flags`](Self::patch_flags).
    pub seg_list_blocks: u32,
}

impl FileSysHeader {
    /// Major version — `fhb_Version >> 16`.
    pub fn version_major(&self) -> u16 {
        (self.version >> 16) as u16
    }

    /// Minor version — the low 16 bits of `fhb_Version`.
    pub fn version_minor(&self) -> u16 {
        self.version as u16
    }
}

/// One bad-block remapping from a `BadBlockBlock`: the drive block that
/// went bad, and the spare that stands in for it. Both are device-block
/// LBAs on the parent disk.
///
/// Effectively extinct — drives have remapped their own defects
/// internally since well before the format stopped being used — but the
/// entries exist on old images and a repartitioner that silently dropped
/// them would hand the filesystem blocks that do not read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BadBlockEntry {
    /// The failed block.
    pub bad: u32,
    /// The block substituted for it.
    pub good: u32,
}

/// A parsed RDB: the disk-level header, its partitions, its loadable
/// filesystems and its bad-block list.
///
/// The `FSHD` and `BADB` chains are parsed eagerly during
/// [`Rdb::parse`] — they are a handful of blocks and the alternative
/// would be methods taking `&mut S`, which would tie the returned value
/// to the source's lifetime. `LSEG` payloads are the exception and stay
/// lazy behind [`Rdb::load_filesystem`]: a driver binary runs to
/// hundreds of kilobytes and most consumers only want the partition
/// table.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Rdb {
    /// LBA the `RDSK` block was found at (0..16).
    pub rdsk_block: u64,
    /// `rdb_BlockBytes` — bytes per device block. Always equal to the
    /// source's [`block_size`](BlockSource::block_size) after a
    /// successful parse; carried so consumers can do byte math without
    /// the source in hand.
    pub block_bytes: u32,
    /// `rdb_Flags`. Left as a bare `u32` because the format permits
    /// bits this crate does not know and a reader must preserve them;
    /// the defined bits have names in [`rdb_flags`].
    pub flags: u32,
    /// `rdb_HostID` — the SCSI ID of the controller that owns this
    /// disk. Meaningless on non-SCSI media, preserved regardless.
    pub host_id: u32,
    /// `rdb_DriveInit` — an optional seglist pointer for drive-specific
    /// init code. On disk it is a block address; this crate does not
    /// follow it (nothing in the wild uses it), it just carries it so a
    /// rewrite does not drop it.
    pub drive_init: u32,
    /// `rdb_Cylinders` — cylinders on the drive, as the RDB declares it.
    pub cylinders: u32,
    /// `rdb_Heads` — surfaces per cylinder.
    pub heads: u32,
    /// `rdb_Sectors` — blocks per track.
    pub sectors: u32,
    /// `rdb_Interleave` — physical sector interleave. Historical: a
    /// value tuned to a controller too slow to read consecutive
    /// sectors. Kept because it is part of what the drive was formatted
    /// with, not because anything modern honours it.
    pub interleave: u32,
    /// `rdb_Park` — the cylinder to park the heads on. Dead on any
    /// drive made since parking became automatic.
    pub park: u32,
    /// `rdb_WritePreComp` — first cylinder needing write precompensation.
    pub write_pre_comp: u32,
    /// `rdb_ReducedWrite` — first cylinder needing reduced write current.
    pub reduced_write: u32,
    /// `rdb_StepRate` — head step rate in the drive's own units.
    pub step_rate: u32,
    /// `rdb_RDBBlocksLo` — first block of the area the RDB structures
    /// themselves occupy: the area a repartitioner may rewrite and a
    /// filesystem must never touch.
    pub rdb_blocks_lo: u32,
    /// `rdb_RDBBlocksHi` — the last block of that area, *inclusive*.
    pub rdb_blocks_hi: u32,
    /// `rdb_LoCylinder` — first cylinder available to partitions.
    /// Distinct from `cylinders`: the RDB area itself normally sits
    /// below this, so this is the range a partitioner may hand out.
    pub lo_cylinder: u32,
    /// `rdb_HiCylinder` — the last such cylinder, *inclusive*.
    pub hi_cylinder: u32,
    /// `rdb_CylBlocks` — device blocks per cylinder as the *drive*
    /// declares it. Each partition states its own (`de_Surfaces *
    /// de_BlocksPerTrack`) and the two are allowed to disagree, which is
    /// why both are surfaced rather than one derived from the other.
    pub cyl_blocks: u32,
    /// `rdb_AutoParkSeconds` — idle seconds before an auto-park, 0 for
    /// never. As dead as [`park`](Self::park).
    pub auto_park_seconds: u32,
    /// `rdb_HighRDSKBlock` — the highest block any RDB structure
    /// currently occupies. `rdb_blocks_hi` is the *reserved* ceiling;
    /// this is the high-water mark actually used, so a writer knows
    /// where free space in the RDB area begins.
    pub high_rdsk_block: u32,
    /// `rdb_DiskVendor` — SCSI INQUIRY identity of the drive,
    /// space-padded ASCII (*not* BCPL, unlike `pb_DriveName`). Only
    /// meaningful when `flags` has [`rdb_flags::DISK_ID`]; otherwise
    /// this and its two neighbours are whatever bytes happened to be
    /// there, so they are parsed unconditionally but must not be
    /// displayed without checking the bit.
    pub disk_vendor: String,
    /// `rdb_DiskProduct`, gated by [`rdb_flags::DISK_ID`] as above.
    pub disk_product: String,
    /// `rdb_DiskRevision`, gated by [`rdb_flags::DISK_ID`] as above.
    pub disk_revision: String,
    /// `rdb_ControllerVendor` — the same INQUIRY identity for the
    /// controller, gated the same way by [`rdb_flags::CTRLR_ID`].
    pub controller_vendor: String,
    /// `rdb_ControllerProduct`, gated by [`rdb_flags::CTRLR_ID`].
    pub controller_product: String,
    /// `rdb_ControllerRevision`, gated by [`rdb_flags::CTRLR_ID`].
    pub controller_revision: String,
    /// Head of the `FSHD` chain ([`CHAIN_END`] if none).
    pub filesys_header_list: u32,
    /// Head of the `BADB` chain ([`CHAIN_END`] if none).
    pub bad_block_list: u32,
    /// Partitions in on-disk chain order.
    pub partitions: Vec<Partition>,
    /// Loadable filesystems in `FSHD` chain order. Empty when
    /// [`filesys_header_list`](Self::filesys_header_list) is
    /// [`CHAIN_END`].
    pub filesystems: Vec<FileSysHeader>,
    /// Bad-block remappings, flattened across every `BADB` block in the
    /// chain and kept in on-disk order.
    ///
    /// Flattened rather than grouped per block because the grouping
    /// carries no information: which entries share a block is an
    /// allocation artefact of whichever tool wrote the list, and a
    /// rewrite repacks them anyway. The chain head survives in
    /// [`bad_block_list`](Self::bad_block_list) for round-tripping.
    pub bad_blocks: Vec<BadBlockEntry>,
    /// LBAs of the `BADB` blocks the parse visited, in chain order.
    ///
    /// [`bad_blocks`](Self::bad_blocks) deliberately forgets which block
    /// each entry came from, but [`validate`](Self::validate) has to know
    /// *where the blocks were* to say whether the chain strayed outside
    /// the RDB area — and unlike `PART` and `FSHD`, whose LBAs ride along
    /// on [`Partition::part_block`] and [`FileSysHeader::fshd_block`],
    /// a `BADB` block has no per-block struct to carry it. Hence a list
    /// here rather than a field that does not exist anywhere else.
    pub badb_blocks: Vec<u64>,
}

/// Which kind of RDB structure a [`ValidationIssue`] is about.
///
/// Only the four chained block types appear: the `RDSK` block is not
/// part of any chain and its legal location is bounded by
/// [`RDB_LOCATION_LIMIT`], not by `rdb_RDBBlocksLo..=Hi`, so an `RDSK`
/// outside the RDB area is normal rather than an issue.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlockKind {
    /// A `PART` partition block.
    Part,
    /// A `FSHD` filesystem-header block.
    Fshd,
    /// A `LSEG` filesystem-driver payload block.
    Lseg,
    /// A `BADB` bad-block-list block.
    Badb,
}

impl core::fmt::Display for BlockKind {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            BlockKind::Part => "PART",
            BlockKind::Fshd => "FSHD",
            BlockKind::Lseg => "LSEG",
            BlockKind::Badb => "BADB",
        })
    }
}

/// One way an RDB's *layout* is self-destructive, as reported by
/// [`Rdb::validate`].
///
/// Not an error: every one of these describes an image that parses
/// perfectly and reads back exactly what is on it. They describe two
/// owners of the same blocks — an RDB structure sitting where a
/// filesystem believes it owns the space, or two partitions claiming the
/// same extent — which is a live grenade rather than a parse failure.
/// Partitioning tools have written RDB blocks past a too-small reserved
/// area into the first partition; the image is readable right up until
/// either side writes, after which both are wrong. A recovery tool needs
/// the data, so the parser hands it over; this type is how a consumer
/// learns not to *write*.
///
/// All block quantities are *device* blocks, like everything else in
/// this crate's API. The [`Display`](core::fmt::Display) impl renders a
/// single line fit to show a user, in `no_std` as much as `std` — as do
/// [`RdbError`] and [`PartitionSourceError`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ValidationIssue {
    /// `rdb_RDBBlocksLo` is above `rdb_RDBBlocksHi`: the reserved area is
    /// empty or inverted, so *nothing* can be said about what lies
    /// inside it. Reported once, and the checks that depend on the area
    /// ([`BlockOutsideRdbArea`](Self::BlockOutsideRdbArea),
    /// [`PartitionOverlapsRdbArea`](Self::PartitionOverlapsRdbArea)) are
    /// skipped rather than producing an issue per block against a range
    /// that means nothing. Partition-versus-partition checking is
    /// unaffected — it never consults the area.
    RdbAreaInvalid {
        /// `rdb_RDBBlocksLo` as it was read.
        lo: u32,
        /// `rdb_RDBBlocksHi` as it was read — below `lo`, which is the issue.
        hi: u32,
    },
    /// A chained block sits outside `rdb_RDBBlocksLo..=rdb_RDBBlocksHi` —
    /// i.e. in space the RDB itself says is not reserved, and which a
    /// partition or a repartitioner is therefore entitled to reuse.
    BlockOutsideRdbArea {
        /// Which chain the block belongs to.
        kind: BlockKind,
        /// Where it actually is.
        lba: u64,
        /// First block of the reserved area it should have been inside.
        lo: u64,
        /// Last block of that area, inclusive.
        hi: u64,
    },
    /// A partition's extent covers part of the RDB area: the filesystem
    /// and the partition table own the same blocks. The classic
    /// damaged-by-construction image.
    PartitionOverlapsRdbArea {
        /// Index into [`Rdb::partitions`].
        index: usize,
        /// The partition's `pb_DriveName`, so a report can name it.
        name: String,
        /// First block of the partition's extent.
        start_lba: u64,
        /// Its length, so the extent is `start_lba..start_lba + block_len`.
        block_len: u64,
        /// First block of the reserved area it collides with.
        lo: u64,
        /// Last block of that area, inclusive.
        hi: u64,
    },
    /// A partition's `de_HighCyl` is below its `de_LowCyl`: the extent
    /// runs backwards and so describes no blocks at all.
    /// [`Partition::block_len`] is zero for such a partition, which
    /// keeps it out of the overlap checks — this is the issue that says
    /// why, and it is not the same as a legitimately empty partition
    /// (there is no such thing: `de_HighCyl` is inclusive, so the
    /// smallest honest partition is one cylinder).
    PartitionCylindersInverted {
        /// Index into [`Rdb::partitions`].
        index: usize,
        /// The partition's `pb_DriveName`, so a report can name it.
        name: String,
        /// `de_LowCyl` as it was read.
        low_cyl: u32,
        /// `de_HighCyl` as it was read — below `low_cyl`, which is the issue.
        high_cyl: u32,
    },
    /// Two partitions' extents intersect. Beyond the letter of the
    /// "overlap validation" plan item, but the same failure family and
    /// the same consequence — two filesystems mounting the same blocks,
    /// each destroying the other — for one extra comparison.
    PartitionsOverlap {
        /// Index into [`Rdb::partitions`] of the first partition, always
        /// the lower of the two.
        a: usize,
        /// Index of the second, always above `a`.
        b: usize,
        /// `a`'s `pb_DriveName`, so a report can name it.
        a_name: String,
        /// `b`'s `pb_DriveName`.
        b_name: String,
        /// First block both claim.
        start: u64,
        /// How many, so the shared extent is `start..start + len`.
        len: u64,
    },
}

impl core::fmt::Display for ValidationIssue {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ValidationIssue::RdbAreaInvalid { lo, hi } => write!(
                f,
                "RDB area is empty or inverted: RDBBlocksLo {lo} is above RDBBlocksHi {hi}"
            ),
            ValidationIssue::BlockOutsideRdbArea { kind, lba, lo, hi } => write!(
                f,
                "{kind} block at {lba} lies outside the RDB area {lo}..={hi}"
            ),
            ValidationIssue::PartitionOverlapsRdbArea {
                index,
                name,
                start_lba,
                block_len,
                lo,
                hi,
            } => write!(
                f,
                "partition {index} ({name}) covers blocks {start_lba}..{} \
                 and overlaps the RDB area {lo}..={hi}",
                start_lba.saturating_add(*block_len)
            ),
            ValidationIssue::PartitionCylindersInverted {
                index,
                name,
                low_cyl,
                high_cyl,
            } => write!(
                f,
                "partition {index} ({name}) has an inverted cylinder range: \
                 LowCyl {low_cyl} is above HighCyl {high_cyl}"
            ),
            ValidationIssue::PartitionsOverlap {
                a,
                b,
                a_name,
                b_name,
                start,
                len,
            } => write!(
                f,
                "partitions {a} ({a_name}) and {b} ({b_name}) both claim blocks {start}..{}",
                start.saturating_add(*len)
            ),
        }
    }
}

/// Byte offsets into a `RDSK` block (NDK `RigidDiskBlock`).
mod rdsk {
    pub const HOST_ID: usize = 12;
    pub const BLOCK_BYTES: usize = 16;
    pub const FLAGS: usize = 20;
    pub const BAD_BLOCK_LIST: usize = 24;
    pub const PARTITION_LIST: usize = 28;
    pub const FILESYS_HEADER_LIST: usize = 32;
    pub const DRIVE_INIT: usize = 36;
    // 40..64: rdb_Reserved1[6]
    pub const CYLINDERS: usize = 64;
    pub const SECTORS: usize = 68;
    pub const HEADS: usize = 72;
    pub const INTERLEAVE: usize = 76;
    pub const PARK: usize = 80;
    // 84..96: rdb_Reserved2[3]
    pub const WRITE_PRE_COMP: usize = 96;
    pub const REDUCED_WRITE: usize = 100;
    pub const STEP_RATE: usize = 104;
    // 108..128: rdb_Reserved3[5]
    pub const RDB_BLOCKS_LO: usize = 128;
    pub const RDB_BLOCKS_HI: usize = 132;
    pub const LO_CYLINDER: usize = 136;
    pub const HI_CYLINDER: usize = 140;
    pub const CYL_BLOCKS: usize = 144;
    pub const AUTO_PARK_SECONDS: usize = 148;
    pub const HIGH_RDSK_BLOCK: usize = 152;
    // 156: rdb_Reserved4

    /// The identification strings: `(byte offset, byte length)`. Not
    /// BCPL — plain space-padded ASCII, unlike `pb_DriveName` four
    /// structures away.
    pub const DISK_VENDOR: (usize, usize) = (160, 8);
    pub const DISK_PRODUCT: (usize, usize) = (168, 16);
    pub const DISK_REVISION: (usize, usize) = (184, 4);
    pub const CONTROLLER_VENDOR: (usize, usize) = (188, 8);
    pub const CONTROLLER_PRODUCT: (usize, usize) = (196, 16);
    pub const CONTROLLER_REVISION: (usize, usize) = (212, 4);
}

/// Byte offsets shared by every chained RDB block.
///
/// `PART`, `FSHD`, `LSEG` and `BADB` all begin with the same five
/// longwords — ID, SummedLongs, ChkSum, HostID, Next — which is what
/// makes one [`walk_chain`] able to serve all four.
mod chain {
    pub const NEXT: usize = 16;
}

/// Byte offsets into a `PART` block (NDK `PartitionBlock`).
mod part {
    pub const FLAGS: usize = 20;
    pub const DRIVE_NAME: usize = 36; // BCPL: length byte, then chars (32 bytes total)
    pub const ENVIRONMENT: usize = 128; // DosEnvec, longwords
}

/// `DosEnvec` field indices, in longwords from its start
/// (NDK `dos/filehandler.h`).
mod de {
    pub const TABLE_SIZE: usize = 0;
    pub const SIZE_BLOCK: usize = 1;
    pub const SEC_ORG: usize = 2;
    pub const SURFACES: usize = 3;
    pub const SECTORS_PER_BLOCK: usize = 4;
    pub const BLOCKS_PER_TRACK: usize = 5;
    pub const RESERVED: usize = 6;
    pub const PRE_ALLOC: usize = 7;
    pub const INTERLEAVE: usize = 8;
    pub const LOW_CYL: usize = 9;
    pub const HIGH_CYL: usize = 10;
    pub const NUM_BUFFERS: usize = 11;
    pub const BUF_MEM_TYPE: usize = 12;
    pub const MAX_TRANSFER: usize = 13;
    pub const MASK: usize = 14;
    pub const BOOT_PRI: usize = 15;
    pub const DOS_TYPE: usize = 16;
    /// The three optional tail fields. `de_TableSize` counts longwords
    /// *after itself*, so a field at index `i` is present exactly when
    /// `table_size >= i`.
    pub const BAUD: usize = 17;
    pub const CONTROL: usize = 18;
    pub const BOOT_BLOCKS: usize = 19;
}

/// Byte offsets into a `FSHD` block (NDK `FileSysHeaderBlock`).
mod fshd {
    pub const HOST_ID: usize = 12;
    pub const FLAGS: usize = 20;
    // 24..32: fhb_Reserved1[2]
    pub const DOS_TYPE: usize = 32;
    pub const VERSION: usize = 36;
    pub const PATCH_FLAGS: usize = 40;
    /// First of the nine patched longwords. Everything gated by
    /// [`super::fshd_patch`] is at `PATCHED + n * 4`, which is exactly
    /// why the bit numbering is positional.
    pub const PATCHED: usize = 44;
    /// Index into the patched longwords, not a byte offset — the one
    /// field read unconditionally.
    pub const SEG_LIST_INDEX: usize = 7;
}

/// Byte offsets into a `LSEG` block (NDK `LoadSegBlock`).
mod lseg {
    /// Start of `lsb_LoadData`; everything from here to the end of the
    /// block is driver payload.
    pub const LOAD_DATA: usize = 20;
}

/// Byte offsets into a `BADB` block (NDK `BadBlockBlock`).
mod badb {
    // 20: bbb_Reserved
    /// Start of `bbb_BlockPairs` — pairs of (bad, good) longwords.
    pub const ENTRIES: usize = 24;
    /// Longwords before the entries. `SummedLongs` counts the whole
    /// block header plus the entries, so the entry count is
    /// `SummedLongs - HEADER_LONGS`.
    pub const HEADER_LONGS: usize = ENTRIES / 4;
}

/// Walk one RDB block chain, verifying every block and calling `visit`.
///
/// The four chains (`PART`, `FSHD`, `LSEG`, `BADB`) differ only in the
/// ID they expect and what they do with each block, so the discipline
/// that matters — off-disk pointers, cycles, wrong IDs, bad checksums —
/// lives here once rather than four times. `buf` is the caller's scratch
/// block and holds the last-read block on return.
///
/// Bounded by a visited set rather than a maximum length: a cycle is the
/// failure mode a crafted or corrupted image produces, and a length cap
/// would either reject a legitimately long chain or still spin on one.
fn walk_chain<S, F>(
    disk: &mut S,
    head: u32,
    expected: u32,
    buf: &mut [u8],
    mut visit: F,
) -> Result<(), RdbError<S::Error>>
where
    S: BlockSource,
    F: FnMut(&[u8], u64) -> Result<(), RdbError<S::Error>>,
{
    let mut next = head;
    let mut visited: Vec<u32> = Vec::new();
    while next != CHAIN_END {
        let lba = next as u64;
        if let Some(n) = disk.block_count() {
            if lba >= n {
                return Err(RdbError::ChainOutOfRange { lba });
            }
        }
        if visited.contains(&next) {
            return Err(RdbError::ChainCycle { lba });
        }
        visited.push(next);

        disk.read_block(lba, buf).map_err(RdbError::Io)?;
        let found = be32(buf, hdr::ID);
        if found != expected {
            return Err(RdbError::WrongId {
                lba,
                expected,
                found,
            });
        }
        if !checksum_ok(buf) {
            return Err(RdbError::BadChecksum { lba });
        }

        visit(buf, lba)?;
        next = be32(buf, chain::NEXT);
    }
    Ok(())
}

impl Rdb {
    /// Find and parse the RDB on `disk`.
    ///
    /// Scans the first [`RDB_LOCATION_LIMIT`] blocks for a `RDSK` block
    /// whose checksum passes (both conditions: an `RDSK` ID with a bad
    /// sum is skipped, matching what the ROM does, so a stale copy at a
    /// lower LBA cannot shadow the live RDB), then walks the `PART`
    /// chain. `rdb_BlockBytes` must match the source's
    /// [`block_size`](BlockSource::block_size) — see
    /// [`RdbError::BlockBytesMismatch`].
    pub fn parse<S: BlockSource>(disk: &mut S) -> Result<Self, RdbError<S::Error>> {
        let block_size = disk.block_size();
        if !block_size_ok(block_size) {
            return Err(RdbError::UnsupportedBlockSize { block_size });
        }
        let mut buf = alloc::vec![0u8; block_size];

        let mut rdsk_at = None;
        let scan_end = match disk.block_count() {
            Some(n) => n.min(RDB_LOCATION_LIMIT),
            None => RDB_LOCATION_LIMIT,
        };
        for lba in 0..scan_end {
            disk.read_block(lba, &mut buf).map_err(RdbError::Io)?;
            if be32(&buf, hdr::ID) == id::RDSK && checksum_ok(&buf) {
                rdsk_at = Some(lba);
                break;
            }
        }
        let rdsk_at = rdsk_at.ok_or(RdbError::NoRdsk)?;

        let block_bytes = be32(&buf, rdsk::BLOCK_BYTES);
        if block_bytes as usize != block_size {
            return Err(RdbError::BlockBytesMismatch {
                block_bytes,
                block_size,
            });
        }

        let mut rdb = Rdb {
            rdsk_block: rdsk_at,
            block_bytes,
            flags: be32(&buf, rdsk::FLAGS),
            host_id: be32(&buf, rdsk::HOST_ID),
            drive_init: be32(&buf, rdsk::DRIVE_INIT),
            cylinders: be32(&buf, rdsk::CYLINDERS),
            heads: be32(&buf, rdsk::HEADS),
            sectors: be32(&buf, rdsk::SECTORS),
            interleave: be32(&buf, rdsk::INTERLEAVE),
            park: be32(&buf, rdsk::PARK),
            write_pre_comp: be32(&buf, rdsk::WRITE_PRE_COMP),
            reduced_write: be32(&buf, rdsk::REDUCED_WRITE),
            step_rate: be32(&buf, rdsk::STEP_RATE),
            rdb_blocks_lo: be32(&buf, rdsk::RDB_BLOCKS_LO),
            rdb_blocks_hi: be32(&buf, rdsk::RDB_BLOCKS_HI),
            lo_cylinder: be32(&buf, rdsk::LO_CYLINDER),
            hi_cylinder: be32(&buf, rdsk::HI_CYLINDER),
            cyl_blocks: be32(&buf, rdsk::CYL_BLOCKS),
            auto_park_seconds: be32(&buf, rdsk::AUTO_PARK_SECONDS),
            high_rdsk_block: be32(&buf, rdsk::HIGH_RDSK_BLOCK),
            disk_vendor: padded_ascii(&buf, rdsk::DISK_VENDOR),
            disk_product: padded_ascii(&buf, rdsk::DISK_PRODUCT),
            disk_revision: padded_ascii(&buf, rdsk::DISK_REVISION),
            controller_vendor: padded_ascii(&buf, rdsk::CONTROLLER_VENDOR),
            controller_product: padded_ascii(&buf, rdsk::CONTROLLER_PRODUCT),
            controller_revision: padded_ascii(&buf, rdsk::CONTROLLER_REVISION),
            filesys_header_list: be32(&buf, rdsk::FILESYS_HEADER_LIST),
            bad_block_list: be32(&buf, rdsk::BAD_BLOCK_LIST),
            partitions: Vec::new(),
            filesystems: Vec::new(),
            bad_blocks: Vec::new(),
            badb_blocks: Vec::new(),
        };
        let partition_list = be32(&buf, rdsk::PARTITION_LIST);

        let mut partitions = Vec::new();
        walk_chain(disk, partition_list, id::PART, &mut buf, |b, lba| {
            partitions.push(parse_part(b, lba)?);
            Ok(())
        })?;
        rdb.partitions = partitions;

        let mut filesystems = Vec::new();
        walk_chain(
            disk,
            rdb.filesys_header_list,
            id::FSHD,
            &mut buf,
            |b, lba| {
                filesystems.push(parse_fshd(b, lba));
                Ok(())
            },
        )?;
        rdb.filesystems = filesystems;

        let mut bad_blocks = Vec::new();
        let mut badb_blocks = Vec::new();
        walk_chain(disk, rdb.bad_block_list, id::BADB, &mut buf, |b, lba| {
            badb_blocks.push(lba);
            parse_badb(b, &mut bad_blocks);
            Ok(())
        })?;
        rdb.bad_blocks = bad_blocks;
        rdb.badb_blocks = badb_blocks;

        Ok(rdb)
    }

    /// Reassemble one filesystem driver's binary from its `LSEG` chain.
    ///
    /// Lazy rather than a field on [`Rdb`]: driver binaries run to
    /// hundreds of kilobytes and most consumers of a partition table
    /// never want them, so the cost is paid only when asked for. `disk`
    /// must be the source the RDB was parsed from — the chain pointers
    /// are LBAs on it — which is checked via its block size.
    ///
    /// The result is the driver in AmigaDOS hunk format, exactly as a
    /// `LoadSeg` would consume it. This crate does not parse or relocate
    /// hunks; that is the loader's job wherever the driver ends up
    /// running.
    ///
    /// **Length has block granularity.** `LSEG` records no exact byte
    /// count anywhere — each block contributes its whole `lsb_LoadData`
    /// area (`block_bytes - 20` bytes) — so the returned `Vec` is the
    /// binary followed by up to that much slack from the final block.
    /// This is not a defect in the reassembly: the hunk structure inside
    /// knows where it ends, and every real consumer finds the end by
    /// parsing hunks rather than by trusting a length.
    ///
    /// An FSHD with no chain ([`CHAIN_END`]) yields an empty `Vec`, not
    /// an error — a header that only patches device-node fields for a
    /// ROM filesystem is legitimate.
    pub fn load_filesystem<S: BlockSource>(
        &self,
        fshd: &FileSysHeader,
        disk: &mut S,
    ) -> Result<Vec<u8>, RdbError<S::Error>> {
        let block_size = disk.block_size();
        if block_size != self.block_bytes as usize {
            return Err(RdbError::BlockBytesMismatch {
                block_bytes: self.block_bytes,
                block_size,
            });
        }
        let mut buf = alloc::vec![0u8; block_size];
        let mut out = Vec::new();
        walk_chain(disk, fshd.seg_list_blocks, id::LSEG, &mut buf, |b, _lba| {
            out.extend_from_slice(&b[lseg::LOAD_DATA..]);
            Ok(())
        })?;
        Ok(out)
    }

    /// Check the *layout* for blocks with two owners.
    ///
    /// Separate from [`parse`](Self::parse) on purpose, and returning a
    /// list rather than an error: an image whose RDB structures have
    /// spilled into a partition still parses, still reads back exactly
    /// what is on it, and a recovery tool needs precisely that. Refusing
    /// to parse it would destroy the only path to the data. What a
    /// consumer needs to know before *writing* is a different question,
    /// and this is where it is answered.
    ///
    /// Reports, in a stable order:
    ///
    /// 1. every chained block the parse visited — `PART`, `FSHD`, `BADB` —
    ///    lying outside `rdb_RDBBlocksLo..=rdb_RDBBlocksHi`;
    /// 2. every partition extent (`start_lba..start_lba + block_len`,
    ///    device blocks) overlapping that same area;
    /// 3. every partition whose `de_HighCyl` is below its `de_LowCyl`;
    /// 4. every pair of partitions whose extents intersect.
    ///
    /// Check 4 goes beyond the RDB-versus-partition case, but it is the
    /// same failure — two owners, both writing — and costs one pass over
    /// the partition pairs. Checks 3 and 4 never consult the RDB area,
    /// so they run even when it is unusable.
    ///
    /// `LSEG` blocks are *not* covered here: they are lazy by design (a
    /// driver binary is hundreds of kilobytes and their LBAs are never
    /// held in memory), so checking them needs the disk back.
    /// [`validate_seg_lists`](Self::validate_seg_lists) does that, and a
    /// consumer that cares about the whole layout runs both.
    ///
    /// An empty result means the layout is self-consistent. It does not
    /// mean the image is undamaged — checksums and chain discipline are
    /// [`parse`](Self::parse)'s business, and passed already.
    pub fn validate(&self) -> Vec<ValidationIssue> {
        let mut issues = Vec::new();

        if self.rdb_blocks_lo > self.rdb_blocks_hi {
            issues.push(ValidationIssue::RdbAreaInvalid {
                lo: self.rdb_blocks_lo,
                hi: self.rdb_blocks_hi,
            });
        } else {
            let lo = self.rdb_blocks_lo as u64;
            let hi = self.rdb_blocks_hi as u64;

            let outside = |kind: BlockKind, lba: u64, issues: &mut Vec<ValidationIssue>| {
                if lba < lo || lba > hi {
                    issues.push(ValidationIssue::BlockOutsideRdbArea { kind, lba, lo, hi });
                }
            };
            for p in &self.partitions {
                outside(BlockKind::Part, p.part_block, &mut issues);
            }
            for f in &self.filesystems {
                outside(BlockKind::Fshd, f.fshd_block, &mut issues);
            }
            for &lba in &self.badb_blocks {
                outside(BlockKind::Badb, lba, &mut issues);
            }

            for (index, p) in self.partitions.iter().enumerate() {
                if p.block_len != 0
                    && p.start_lba <= hi
                    && p.start_lba.saturating_add(p.block_len) > lo
                {
                    issues.push(ValidationIssue::PartitionOverlapsRdbArea {
                        index,
                        name: p.name.clone(),
                        start_lba: p.start_lba,
                        block_len: p.block_len,
                        lo,
                        hi,
                    });
                }
            }
        }

        // Inverted cylinder ranges, which need no RDB area either — and
        // which run first of the area-independent checks because an
        // inverted extent is *why* a partition is missing from the
        // overlap results below.
        for (index, p) in self.partitions.iter().enumerate() {
            if p.high_cyl < p.low_cyl {
                issues.push(ValidationIssue::PartitionCylindersInverted {
                    index,
                    name: p.name.clone(),
                    low_cyl: p.low_cyl,
                    high_cyl: p.high_cyl,
                });
            }
        }

        // Partition-versus-partition, which needs no RDB area and so
        // runs even when the area is unusable.
        for a in 0..self.partitions.len() {
            for b in a + 1..self.partitions.len() {
                let (pa, pb) = (&self.partitions[a], &self.partitions[b]);
                let start = pa.start_lba.max(pb.start_lba);
                let end = pa
                    .start_lba
                    .saturating_add(pa.block_len)
                    .min(pb.start_lba.saturating_add(pb.block_len));
                if start < end {
                    issues.push(ValidationIssue::PartitionsOverlap {
                        a,
                        b,
                        a_name: pa.name.clone(),
                        b_name: pb.name.clone(),
                        start,
                        len: end - start,
                    });
                }
            }
        }

        issues
    }

    /// The `LSEG` half of [`validate`](Self::validate): walk every
    /// filesystem's driver chain and report blocks outside the RDB area.
    ///
    /// Takes the disk because `LSEG` chains are only ever walked on
    /// demand — [`load_filesystem`](Self::load_filesystem) is where their
    /// LBAs exist at all — so unlike the other three chains there is
    /// nothing in memory to check. The blocks are read for their chain
    /// pointers and their payload discarded, which is cheap next to
    /// reassembling the binaries.
    ///
    /// Fails only the way [`load_filesystem`](Self::load_filesystem)
    /// does: a broken chain (wrong ID, bad checksum, cycle, off-disk
    /// pointer) is a parse error, not a layout issue. When the RDB area
    /// itself is unusable ([`ValidationIssue::RdbAreaInvalid`], which
    /// [`validate`](Self::validate) reports) this returns no issues,
    /// there being no range to compare against.
    pub fn validate_seg_lists<S: BlockSource>(
        &self,
        disk: &mut S,
    ) -> Result<Vec<ValidationIssue>, RdbError<S::Error>> {
        if self.rdb_blocks_lo > self.rdb_blocks_hi {
            return Ok(Vec::new());
        }
        let block_size = disk.block_size();
        if block_size != self.block_bytes as usize {
            return Err(RdbError::BlockBytesMismatch {
                block_bytes: self.block_bytes,
                block_size,
            });
        }
        let (lo, hi) = (self.rdb_blocks_lo as u64, self.rdb_blocks_hi as u64);
        let mut buf = alloc::vec![0u8; block_size];
        let mut issues = Vec::new();
        for f in &self.filesystems {
            walk_chain(disk, f.seg_list_blocks, id::LSEG, &mut buf, |_b, lba| {
                if lba < lo || lba > hi {
                    issues.push(ValidationIssue::BlockOutsideRdbArea {
                        kind: BlockKind::Lseg,
                        lba,
                        lo,
                        hi,
                    });
                }
                Ok(())
            })?;
        }
        Ok(issues)
    }
}

/// Parse a verified `FSHD` block.
///
/// Infallible: every field is at a fixed offset well inside the
/// smallest legal block, and `PatchFlags` cannot make a read go out of
/// bounds — it only decides whether an already-in-bounds longword is
/// meaningful.
fn parse_fshd(buf: &[u8], lba: u64) -> FileSysHeader {
    let patch_flags = be32(buf, fshd::PATCH_FLAGS);
    let patched = |i: usize| be32(buf, fshd::PATCHED + i * 4);
    let gated = |i: usize| {
        if patch_flags & (1 << i) != 0 {
            Some(patched(i))
        } else {
            None
        }
    };

    FileSysHeader {
        fshd_block: lba,
        host_id: be32(buf, fshd::HOST_ID),
        flags: be32(buf, fshd::FLAGS),
        dos_type: be32(buf, fshd::DOS_TYPE),
        version: be32(buf, fshd::VERSION),
        patch_flags,
        node_type: gated(0),
        task: gated(1),
        lock: gated(2),
        handler: gated(3),
        stack_size: gated(4),
        priority: gated(5).map(|v| v as i32),
        startup: gated(6).map(|v| v as i32),
        // Bit 7 is SegList, read unconditionally below.
        global_vec: gated(8).map(|v| v as i32),
        seg_list_blocks: patched(fshd::SEG_LIST_INDEX),
    }
}

/// Append a verified `BADB` block's entries to `out`.
///
/// `SummedLongs` counts the header plus the entry longwords, so the
/// entry count is `(SummedLongs - 6) / 2`. That value is
/// attacker-controlled, and although [`checksum_ok`] has already
/// rejected a `SummedLongs` larger than the block, the arithmetic is
/// clamped to what the block physically holds anyway: a bounds check
/// that depends on a checksum passing is one refactor away from not
/// being a bounds check.
fn parse_badb(buf: &[u8], out: &mut Vec<BadBlockEntry>) {
    let summed_longs = be32(buf, hdr::SUMMED_LONGS) as usize;
    let entry_longs = summed_longs.saturating_sub(badb::HEADER_LONGS);
    let capacity_longs = (buf.len() - badb::ENTRIES) / 4;
    let pairs = entry_longs.min(capacity_longs) / 2;
    for i in 0..pairs {
        let off = badb::ENTRIES + i * 8;
        out.push(BadBlockEntry {
            bad: be32(buf, off),
            good: be32(buf, off + 4),
        });
    }
}

/// Read a fixed-width, space-padded ASCII field as a `String`.
///
/// The RDSK identification fields are SCSI INQUIRY data copied
/// verbatim: fixed width, padded with spaces — not BCPL, and not
/// NUL-terminated by specification, though real controllers pad with
/// NULs often enough that both are trimmed. Bytes become chars
/// one-for-one (latin-1-ish) rather than being trusted as UTF-8, the
/// same treatment `pb_DriveName` gets.
fn padded_ascii(buf: &[u8], (off, len): (usize, usize)) -> String {
    let field = &buf[off..off + len];
    let end = field
        .iter()
        .rposition(|&b| b != b' ' && b != 0)
        .map_or(0, |i| i + 1);
    field[..end].iter().map(|&b| b as char).collect()
}

fn parse_part<E>(buf: &[u8], lba: u64) -> Result<Partition, RdbError<E>> {
    let envec = |i: usize| be32(buf, part::ENVIRONMENT + i * 4);

    // de_TableSize counts longwords *after itself*; DOS_TYPE is the
    // last field this crate requires. Enforced before reading past it.
    let table_size = envec(de::TABLE_SIZE);
    if (table_size as usize) < de::DOS_TYPE {
        return Err(RdbError::EnvecTooShort { lba, table_size });
    }

    // How many envec longwords the block physically holds. de_TableSize
    // is attacker-controlled, so every read past DOS_TYPE — the
    // optional fields and envec_raw alike — is clamped to this, and a
    // hostile 0xFFFFFFFF truncates instead of running off the block.
    let envec_capacity = (buf.len() - part::ENVIRONMENT) / 4;
    let present = |i: usize| {
        if table_size as usize >= i && i < envec_capacity {
            Some(envec(i))
        } else {
            None
        }
    };
    // table_size + 1 longwords, TableSize itself included; saturating
    // so table_size == u32::MAX cannot wrap to zero.
    let raw_len = (table_size as usize).saturating_add(1).min(envec_capacity);
    let envec_raw: Vec<u32> = (0..raw_len).map(envec).collect();

    // BCPL string: length byte then bytes, no terminator. The name is
    // ASCII in every image ever seen, but bytes are passed through
    // as-is (lossy) rather than trusted to be UTF-8.
    let name_len = (buf[part::DRIVE_NAME] as usize).min(31);
    let name_bytes = &buf[part::DRIVE_NAME + 1..part::DRIVE_NAME + 1 + name_len];
    let name: String = name_bytes.iter().map(|&b| b as char).collect();

    let surfaces = envec(de::SURFACES) as u64;
    let blocks_per_track = envec(de::BLOCKS_PER_TRACK) as u64;
    // Two u32s widened first, so this product alone cannot exceed u64.
    // Everything downstream multiplies it by a *third* u32 and so can,
    // which is why the extent arithmetic below saturates.
    let cylinder_blocks = surfaces * blocks_per_track;
    let low_cyl = envec(de::LOW_CYL);
    let high_cyl = envec(de::HIGH_CYL);
    let flags = be32(buf, part::FLAGS);

    // `de_HighCyl` is inclusive, so the span is `high - low + 1` — but
    // nothing on disk makes `high >= low` true, and an inverted pair is
    // exactly what a corrupt (or hostile) PART block carries. Zero is
    // the honest length: an inverted range contains no blocks, and a
    // zero-length extent is already excluded from every overlap check.
    // The inversion itself is reported by
    // [`Rdb::validate`](Rdb::validate), where layout nonsense belongs —
    // refusing the parse would deny a recovery tool the only view of
    // the damage. Found by the fuzzer: subtracting unchecked panicked
    // in a debug build and, worse, wrapped in a release one, conjuring
    // a multi-exabyte partition out of two plausible u32s.
    let block_len = match high_cyl.checked_sub(low_cyl) {
        Some(span) => (span as u64 + 1).saturating_mul(cylinder_blocks),
        None => 0,
    };

    Ok(Partition {
        part_block: lba,
        name,
        bootable: flags & 1 != 0,
        no_automount: flags & 2 != 0,
        start_lba: (low_cyl as u64).saturating_mul(cylinder_blocks),
        block_len,
        dos_type: envec(de::DOS_TYPE),
        boot_pri: envec(de::BOOT_PRI) as i32,
        max_transfer: envec(de::MAX_TRANSFER),
        mask: envec(de::MASK),
        cylinder_blocks,
        low_cyl,
        high_cyl,
        num_buffers: envec(de::NUM_BUFFERS),
        buf_mem_type: envec(de::BUF_MEM_TYPE),
        size_block_longs: envec(de::SIZE_BLOCK),
        baud: present(de::BAUD),
        control: present(de::CONTROL),
        boot_blocks: present(de::BOOT_BLOCKS),
        envec_raw,
    })
}

/// The values this crate writes into a new `PART` block's `DosEnvec`
/// when the caller does not override them, and where each came from.
///
/// **Provenance: every number here was read back out of an image made by
/// amitools' `rdbtool` 0.8.1**, by creating a disk, adding partitions and
/// parsing the raw `PART` blocks — not from the NDK's suggested values
/// and not from folklore. The reason is the one
/// [`synthesize_geometry`] gives for the geometry convention:
/// `rdbtool`'s images are this crate's fixtures and its differential
/// oracle, so a default that differs from `rdbtool`'s would make every
/// such comparison a false positive, and would hand the Amiga a mount
/// parameter subtly unlike the one every other image on the machine
/// carries.
///
/// A caller who wants different values sets them on the
/// [`PartitionSpec`]; nothing here is enforced, only defaulted.
pub mod envec_defaults {
    /// `de_TableSize` — 16, the index of `de_DosType`, so the envec runs
    /// exactly as far as the last field the format's mount path needs.
    /// The three tail fields (`de_Baud`, `de_Control`, `de_BootBlocks`)
    /// are *absent* rather than zero, which is what `rdbtool` writes and
    /// what [`Partition::baud`](super::Partition::baud) and friends report as `None`.
    pub const TABLE_SIZE: u32 = 16;

    /// `de_SecOrg` — sector origin. Always zero; the field has never had
    /// another meaning.
    pub const SEC_ORG: u32 = 0;

    /// `de_SectorPerBlock` — device sectors per filesystem block. One,
    /// with `de_SizeBlock` carrying the size instead; every image in the
    /// wild says one.
    pub const SECTORS_PER_BLOCK: u32 = 1;

    /// `de_Reserved` — blocks reserved at the start of the partition for
    /// the boot block. Two, which is what a DOS-family filesystem needs
    /// and what every tool writes.
    pub const RESERVED: u32 = 2;

    /// `de_PreAlloc` — blocks reserved at the *end* of the partition.
    /// Zero: a DOS filesystem needs none.
    pub const PRE_ALLOC: u32 = 0;

    /// `de_Interleave` — filesystem-level interleave. Zero.
    pub const INTERLEAVE: u32 = 0;

    /// `de_NumBuffers` — cache buffers the handler allocates at mount.
    /// A mount parameter that says nothing about the disk; 30 is
    /// `rdbtool`'s default.
    pub const NUM_BUFFERS: u32 = 30;

    /// `de_BufMemType` — which memory those buffers want. Zero means
    /// "any", which is right for everything except a DMA controller that
    /// cannot reach fast RAM.
    pub const BUF_MEM_TYPE: u32 = 0;

    /// `de_MaxTransfer` — the largest transfer, in bytes, the driver may
    /// issue in one go.
    pub const MAX_TRANSFER: u32 = 0x00FF_FFFF;

    /// `de_Mask` — address mask for DMA-reachable memory. `0x7FFFFFFE`
    /// is the conventional "anything word-aligned in the low 2 GB".
    pub const MASK: u32 = 0x7FFF_FFFE;

    /// `de_DosType` — `DOS\x03` (FFS with international caching),
    /// `rdbtool`'s default for a new partition. Callers who want
    /// `DOS\x00` or `DOS\x07` say so.
    pub const DOS_TYPE: u32 = 0x444F_5303;

    /// `de_SizeBlock`, the filesystem block size in longwords, is *not*
    /// a constant: `rdbtool` writes `rdb_BlockBytes / 4` — 128 on a
    /// 512-byte-block disk, 1024 on a 4 KB one — so the filesystem block
    /// and the device block start out the same size. This is the one
    /// default that depends on the geometry, which is why
    /// [`PartitionSpec::size_block_longs`](super::PartitionSpec::size_block_longs) is an [`Option`] rather than
    /// carrying a number the constructor cannot know.
    ///
    /// The two remain independent knobs after that: a caller may ask for
    /// a 32 KB filesystem block on a 512-byte-block disk, which is a real
    /// and working configuration.
    pub const fn size_block_longs(block_size: usize) -> u32 {
        (block_size / 4) as u32
    }
}

/// The values this crate writes into a new `RDSK` block when the caller
/// does not override them — same provenance as [`envec_defaults`]:
/// observed in `rdbtool` 0.8.1's output, not assumed.
pub mod rdsk_defaults {
    /// `rdb_Flags` — `LAST | LASTLUN | LASTTID`
    /// ([`rdb_flags`](super::rdb_flags)): there is no disk after this
    /// one, no LUN after this one, no target after this one. The honest
    /// answer for an image file, which is not on a bus at all, and what
    /// `rdbtool` writes.
    pub const FLAGS: u32 =
        super::rdb_flags::LAST | super::rdb_flags::LAST_LUN | super::rdb_flags::LAST_TID;

    /// `rdb_HostID` — the controller's own SCSI ID. Seven by convention
    /// (the host adapter traditionally takes the highest priority ID),
    /// and meaningless on an image.
    pub const HOST_ID: u32 = 7;

    /// `rdb_Interleave` — physical sector interleave. One means "none".
    pub const INTERLEAVE: u32 = 1;

    /// `rdb_StepRate` — head step rate in the drive's own units.
    pub const STEP_RATE: u32 = 3;

    /// `rdb_AutoParkSeconds` — idle seconds before an auto-park. Zero is
    /// never, which is the only sane value for anything modern.
    pub const AUTO_PARK_SECONDS: u32 = 0;
}

/// Where a partition goes: a size the builder turns into cylinders, or
/// the cylinders themselves.
///
/// Two entry points because there are two kinds of caller. One has a
/// size — "give me a 500 MB `DH0`" — and wants the cylinder arithmetic
/// done for it. The other is reproducing a layout it already knows (a
/// clone of an existing disk, a recipe in a build file) and must be able
/// to say the exact cylinders, because rounding a size back into
/// cylinders is not guaranteed to land on the same ones.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Placement {
    /// At most this many bytes, placed after the previous partition.
    ///
    /// **Rounds the cylinder count down** — `bytes / cylinder_bytes`,
    /// floored — so a partition never claims more space than was asked
    /// for, and falls short of it by less than one cylinder. That is what
    /// `rdbtool` 0.8.1 does (`cyls = num_bytes //
    /// rdisk.get_cylinder_bytes()`, verified against the images it
    /// writes: 10 MiB on a 129 024-byte cylinder becomes 81 cylinders,
    /// 10 450 944 bytes, not 82), and it is the same direction
    /// [`synthesize_geometry`] rounds, for the same reason — a size is a
    /// ceiling, and the tool that hands out more than it was asked for is
    /// the one that walks off the end of something.
    ///
    /// A size below one whole cylinder therefore describes no partition
    /// at all and is [`BuildError::PartitionTooSmall`], not a silently
    /// rounded-up one: `de_HighCyl` is inclusive, so there is no
    /// zero-cylinder partition to write. `rdbtool` refuses the same case
    /// ("invalid partition range given!").
    ///
    /// A caller who needs "at least *n* bytes" rounds up itself, or uses
    /// [`Cylinders`](Self::Cylinders) — the point of the pair is that the
    /// exact answer is always available.
    Size(u64),
    /// Exactly these cylinders, `high` *inclusive*, as `de_LowCyl` and
    /// `de_HighCyl` will say. Placed where it says, not after the
    /// previous partition — a builder given only explicit ranges places
    /// nothing implicitly, and overlaps are refused rather than shuffled.
    Cylinders {
        /// `de_LowCyl`, the first cylinder.
        low: u32,
        /// `de_HighCyl`, the last — inclusive, so `low == high` is a
        /// legal one-cylinder partition.
        high: u32,
    },
}

/// One partition to create: where it goes and what its `PART` block will
/// say.
///
/// Construct with [`by_size`](Self::by_size) or
/// [`by_cylinders`](Self::by_cylinders), which fill every mount
/// parameter from [`envec_defaults`]; change what you need through the
/// chainable setters or by assigning the public fields directly, since a
/// caller cloning an existing disk needs to reproduce values this crate
/// has no opinion about.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PartitionSpec {
    /// Where the partition goes.
    pub placement: Placement,
    /// `pb_DriveName`, or `None` to have the builder assign the first
    /// free `DH`*n* — see [`RdbBuilder::build`] for exactly which.
    pub name: Option<String>,
    /// `pb_Flags` bit 0: the ROM may boot from this partition.
    pub bootable: bool,
    /// `pb_Flags` bit 1: mount it, but not automatically at boot.
    pub no_automount: bool,
    /// `de_BootPri` — boot priority, higher wins. Ignored unless
    /// [`bootable`](Self::bootable).
    pub boot_pri: i32,
    /// `de_DosType`.
    pub dos_type: u32,
    /// `de_SizeBlock`, in longwords — the *filesystem* block size, which
    /// is per partition and independent of `rdb_BlockBytes`.
    ///
    /// `None` takes [`envec_defaults::size_block_longs`] of the
    /// geometry's block size, which is what `rdbtool` writes and the one
    /// default the [`PartitionSpec`] constructors cannot fill in, not
    /// knowing the disk they will be built against.
    pub size_block_longs: Option<u32>,
    /// `de_SecOrg`.
    pub sec_org: u32,
    /// `de_SectorPerBlock`.
    pub sectors_per_block: u32,
    /// `de_Reserved` — boot blocks at the start of the partition.
    pub reserved: u32,
    /// `de_PreAlloc` — blocks held back at the end.
    pub pre_alloc: u32,
    /// `de_Interleave`.
    pub interleave: u32,
    /// `de_NumBuffers`.
    pub num_buffers: u32,
    /// `de_BufMemType`.
    pub buf_mem_type: u32,
    /// `de_MaxTransfer`.
    pub max_transfer: u32,
    /// `de_Mask`.
    pub mask: u32,
}

impl PartitionSpec {
    /// A partition of at least `bytes`, placed after the previous one —
    /// see [`Placement::Size`] for the rounding.
    pub fn by_size(bytes: u64) -> Self {
        Self::with_placement(Placement::Size(bytes))
    }

    /// A partition on exactly cylinders `low..=high` (inclusive).
    pub fn by_cylinders(low: u32, high: u32) -> Self {
        Self::with_placement(Placement::Cylinders { low, high })
    }

    fn with_placement(placement: Placement) -> Self {
        Self {
            placement,
            name: None,
            bootable: false,
            no_automount: false,
            boot_pri: 0,
            dos_type: envec_defaults::DOS_TYPE,
            size_block_longs: None,
            sec_org: envec_defaults::SEC_ORG,
            sectors_per_block: envec_defaults::SECTORS_PER_BLOCK,
            reserved: envec_defaults::RESERVED,
            pre_alloc: envec_defaults::PRE_ALLOC,
            interleave: envec_defaults::INTERLEAVE,
            num_buffers: envec_defaults::NUM_BUFFERS,
            buf_mem_type: envec_defaults::BUF_MEM_TYPE,
            max_transfer: envec_defaults::MAX_TRANSFER,
            mask: envec_defaults::MASK,
        }
    }

    /// Set `pb_DriveName` explicitly instead of taking an assigned one.
    pub fn named(mut self, name: &str) -> Self {
        self.name = Some(String::from(name));
        self
    }

    /// Set `de_DosType`.
    pub fn dos_type(mut self, dos_type: u32) -> Self {
        self.dos_type = dos_type;
        self
    }

    /// Mark the partition bootable (`pb_Flags` bit 0) with the given
    /// `de_BootPri`.
    pub fn bootable(mut self, boot_pri: i32) -> Self {
        self.bootable = true;
        self.boot_pri = boot_pri;
        self
    }

    /// Set `de_SizeBlock` in longwords — the filesystem block size, not
    /// the device's.
    pub fn size_block_longs(mut self, longs: u32) -> Self {
        self.size_block_longs = Some(longs);
        self
    }
}

/// Why [`RdbBuilder::build`] refused to write.
///
/// Every variant except [`Io`](Self::Io) is raised **before the first
/// block is written**: the builder computes the whole layout, checks it,
/// and only then writes, so a rejected build leaves the target exactly as
/// it found it. That is the point of the type — the failure mode this
/// crate exists to prevent is a partitioner discovering halfway through
/// that its structures do not fit and writing them into partition space
/// anyway.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BuildError<E> {
    /// The [`BlockSink`] failed on a write. The only variant that can
    /// leave the target half-written — the layout was valid and the
    /// device said no.
    Io(E),
    /// The sink's [`block_size`](BlockSink::block_size) is not a power of
    /// two in [`MIN_BLOCK_SIZE`]`..=`[`MAX_BLOCK_SIZE`].
    UnsupportedBlockSize {
        /// What the sink said its block size was.
        block_size: usize,
    },
    /// The [`Geometry`]'s block size and the sink's disagree. Every LBA
    /// the builder computes is in geometry blocks; writing them through a
    /// differently sized sink would address the wrong bytes.
    BlockSizeMismatch {
        /// [`Geometry::block_size`].
        geometry: usize,
        /// [`BlockSink::block_size`].
        sink: usize,
    },
    /// The geometry describes no blocks — a zero in `cylinders`, `heads`
    /// or `sectors`. Nothing can be placed against it.
    EmptyGeometry {
        /// The geometry as given.
        geometry: Geometry,
    },
    /// A `pb_DriveName` does not fit the 32-byte BCPL field, or is empty.
    InvalidName {
        /// The name asked for.
        name: String,
        /// The longest name the field holds.
        max: usize,
    },
    /// Two partitions were given the same `pb_DriveName`. Refused rather
    /// than silently renamed: two `DH0`s is a layout whose mounts fight
    /// each other, and the caller asked for it explicitly.
    DuplicateName {
        /// The name asked for twice.
        name: String,
    },
    /// A [`Placement::Cylinders`] range runs backwards.
    CylindersInverted {
        /// The partition's name.
        name: String,
        /// `de_LowCyl` as asked for.
        low_cyl: u32,
        /// `de_HighCyl` as asked for — below `low_cyl`, which is the issue.
        high_cyl: u32,
    },
    /// A [`Placement::Size`] is below one cylinder, so it describes no
    /// partition at all — see that variant for why this is refused
    /// rather than rounded up to one.
    PartitionTooSmall {
        /// The partition's name.
        name: String,
        /// The size asked for.
        bytes: u64,
        /// One cylinder, in bytes — the smallest partition there is.
        cylinder_bytes: u64,
    },
    /// A partition's last cylinder is past the last cylinder the geometry
    /// has. Covers both a [`Placement::Cylinders`] range that overshoots
    /// and a [`Placement::Size`] larger than the space left.
    PartitionPastEndOfDisk {
        /// The partition's name.
        name: String,
        /// The last cylinder it wanted.
        high_cyl: u32,
        /// The last cylinder the disk has (`rdb_Cylinders - 1`).
        last_cylinder: u32,
    },
    /// A partition starts below `rdb_LoCylinder`, i.e. inside the
    /// reserved RDB area. The overlap this crate exists to refuse.
    PartitionOverlapsRdbArea {
        /// The partition's name.
        name: String,
        /// The cylinder it wanted to start on.
        low_cyl: u32,
        /// The first cylinder available to partitions.
        lo_cylinder: u32,
    },
    /// Two partitions claim the same cylinders.
    PartitionsOverlap {
        /// The first partition's name, in the order they were added.
        a_name: String,
        /// The second's.
        b_name: String,
        /// The first cylinder both claim.
        low_cyl: u32,
        /// The last, inclusive.
        high_cyl: u32,
    },
    /// The reserved RDB area cannot hold the blocks the layout needs —
    /// one `RDSK` plus one `PART` per partition. Either too many
    /// partitions, or a [`RdbBuilder::reserved_blocks`] override too
    /// small for them.
    RdbAreaTooSmall {
        /// Blocks the layout needs, `RDSK` included.
        needed: u32,
        /// Blocks the area has (`rdb_RDBBlocksHi - rdb_RDBBlocksLo + 1`).
        available: u32,
    },
    /// The sink is smaller than the layout: its
    /// [`block_count`](BlockSink::block_count) is below the last block
    /// the layout would occupy. Only detectable when the sink knows its
    /// size; a sink reporting `None` is taken at its word.
    SinkTooSmall {
        /// Blocks the layout needs to exist.
        needed: u64,
        /// Blocks the sink says it has.
        available: u64,
    },
    /// Sealing a block's checksum failed.
    ///
    /// Unreachable in practice — the builder seals 64 longwords into
    /// blocks of at least [`MIN_BLOCK_SIZE`], which hold 128 — but the
    /// alternative to a variant here is an `unwrap`, and a writer that
    /// can panic on a caller's arithmetic is exactly what
    /// [`seal_checksum`] returns a `Result` to avoid.
    Seal(SealError),
}

impl<E: core::fmt::Display> core::fmt::Display for BuildError<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BuildError::Io(e) => write!(f, "writing a block failed: {e}"),
            BuildError::UnsupportedBlockSize { block_size } => write!(
                f,
                "unsupported device block size {block_size}: \
                 must be a power of two in {MIN_BLOCK_SIZE}..={MAX_BLOCK_SIZE}"
            ),
            BuildError::BlockSizeMismatch { geometry, sink } => write!(
                f,
                "the geometry is in {geometry}-byte blocks but the sink writes \
                 {sink}-byte blocks"
            ),
            BuildError::EmptyGeometry { geometry } => write!(
                f,
                "the geometry {}/{}/{} describes no blocks",
                geometry.cylinders, geometry.heads, geometry.sectors
            ),
            BuildError::InvalidName { name, max } => write!(
                f,
                "drive name {name:?} does not fit pb_DriveName: \
                 1..={max} characters are available"
            ),
            BuildError::DuplicateName { name } => {
                write!(f, "two partitions are both named {name:?}")
            }
            BuildError::CylindersInverted {
                name,
                low_cyl,
                high_cyl,
            } => write!(
                f,
                "partition {name:?} has an inverted cylinder range: \
                 LowCyl {low_cyl} is above HighCyl {high_cyl}"
            ),
            BuildError::PartitionTooSmall {
                name,
                bytes,
                cylinder_bytes,
            } => write!(
                f,
                "partition {name:?} asks for {bytes} bytes, less than the \
                 {cylinder_bytes}-byte cylinder that is the smallest partition"
            ),
            BuildError::PartitionPastEndOfDisk {
                name,
                high_cyl,
                last_cylinder,
            } => write!(
                f,
                "partition {name:?} ends on cylinder {high_cyl}, past the disk's last \
                 cylinder {last_cylinder}"
            ),
            BuildError::PartitionOverlapsRdbArea {
                name,
                low_cyl,
                lo_cylinder,
            } => write!(
                f,
                "partition {name:?} starts on cylinder {low_cyl}, inside the RDB area \
                 that ends at cylinder {}",
                lo_cylinder.saturating_sub(1)
            ),
            BuildError::PartitionsOverlap {
                a_name,
                b_name,
                low_cyl,
                high_cyl,
            } => write!(
                f,
                "partitions {a_name:?} and {b_name:?} both claim cylinders \
                 {low_cyl}..={high_cyl}"
            ),
            BuildError::RdbAreaTooSmall { needed, available } => write!(
                f,
                "the RDB area holds {available} blocks but the layout needs {needed}"
            ),
            BuildError::SinkTooSmall { needed, available } => write!(
                f,
                "the layout needs {needed} blocks but the target has {available}"
            ),
            BuildError::Seal(e) => write!(f, "sealing a block failed: {e}"),
        }
    }
}

#[cfg(feature = "std")]
impl<E: std::error::Error + 'static> std::error::Error for BuildError<E> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            BuildError::Io(e) => Some(e),
            BuildError::Seal(e) => Some(e),
            _ => None,
        }
    }
}

/// Where one partition landed: the answer to "what did you write, and
/// where", for a caller that wants to act on the result without
/// re-parsing the disk.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlacedPartition {
    /// LBA of the `PART` block written for it.
    pub part_block: u64,
    /// The `pb_DriveName` written — the assigned one if the spec left it
    /// to the builder.
    pub name: String,
    /// `de_LowCyl` as written.
    pub low_cyl: u32,
    /// `de_HighCyl` as written, *inclusive*.
    pub high_cyl: u32,
    /// First device block of the partition's data, as
    /// [`Partition::start_lba`] will report it.
    pub start_lba: u64,
    /// How many device blocks it covers, as [`Partition::block_len`]
    /// will report it.
    pub block_len: u64,
}

/// The complete block layout [`RdbBuilder::build`] computed and wrote.
///
/// Returned rather than nothing so a caller need not re-parse the disk
/// to learn where its partitions ended up — though re-parsing is exactly
/// what this crate's own tests do, on the principle that the disk is the
/// only authority on what is on the disk.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RdbLayout {
    /// LBA the `RDSK` block was written to.
    pub rdsk_block: u64,
    /// `rdb_RDBBlocksLo` as written.
    pub rdb_blocks_lo: u32,
    /// `rdb_RDBBlocksHi` as written — the reserved ceiling, *inclusive*.
    pub rdb_blocks_hi: u32,
    /// `rdb_HighRDSKBlock` as written — the highest block actually used,
    /// so the gap up to [`rdb_blocks_hi`](Self::rdb_blocks_hi) is the
    /// headroom a later edit has to work in.
    pub high_rdsk_block: u32,
    /// `rdb_LoCylinder` — the first cylinder available to partitions.
    pub lo_cylinder: u32,
    /// `rdb_HiCylinder` — the last, *inclusive*.
    pub hi_cylinder: u32,
    /// The geometry written into the `RDSK` block.
    pub geometry: Geometry,
    /// The partitions, in the order they were added and chained.
    pub partitions: Vec<PlacedPartition>,
}

/// Build a fresh RDB — an `RDSK` block and its `PART` chain — on an
/// empty target.
///
/// # The order of operations, which is the whole design
///
/// [`build`](Self::build) computes the **complete** block layout,
/// validates every part of it, and only then writes. There is no code
/// path that writes block *N+1* after discovering that block *N* was the
/// last one that fit: "does not fit" is a [`BuildError`] returned before
/// the sink is touched. This is not defensiveness for its own sake — RDB
/// images damaged by exactly that failure exist in the wild, partition
/// tables written past a too-small reserved area into the first
/// partition, after which the filesystem and the partition table each
/// destroy the other. [`Rdb::validate`] is how a reader finds such an
/// image; this is how a writer never makes one.
///
/// Blocks are written `PART` chain first and `RDSK` last, so an
/// interrupted build leaves no valid `RDSK` — an unpartitioned disk
/// rather than a partition table pointing at blocks that were never
/// written.
///
/// # Example
///
/// ```
/// use amiga_rdb::{PartitionSpec, RdbBuilder};
/// # use amiga_rdb::{BlockSink, BlockSource, Rdb};
/// # struct MemDisk(Vec<u8>);
/// # fn eof() -> std::io::Error {
/// #     std::io::Error::new(std::io::ErrorKind::UnexpectedEof, "past the end of the disk")
/// # }
/// # impl BlockSource for MemDisk {
/// #     type Error = std::io::Error;
/// #     fn block_size(&self) -> usize { 512 }
/// #     fn read_block(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), Self::Error> {
/// #         let off = lba as usize * 512;
/// #         buf.copy_from_slice(self.0.get(off..off + 512).ok_or_else(eof)?);
/// #         Ok(())
/// #     }
/// #     fn block_count(&self) -> Option<u64> { Some(self.0.len() as u64 / 512) }
/// # }
/// # impl BlockSink for MemDisk {
/// #     type Error = std::io::Error;
/// #     fn block_size(&self) -> usize { 512 }
/// #     fn write_block(&mut self, lba: u64, buf: &[u8]) -> Result<(), Self::Error> {
/// #         let off = lba as usize * 512;
/// #         self.0.get_mut(off..off + 512).ok_or_else(eof)?.copy_from_slice(buf);
/// #         Ok(())
/// #     }
/// #     fn block_count(&self) -> Option<u64> { Some(self.0.len() as u64 / 512) }
/// # }
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let mut disk = MemDisk(vec![0u8; 64 * 1024 * 1024]);
///
/// let layout = RdbBuilder::for_size(64 * 1024 * 1024, 512)?
///     .partition(PartitionSpec::by_size(16 * 1024 * 1024).bootable(0))
///     .partition(PartitionSpec::by_size(16 * 1024 * 1024).named("WORK"))
///     .build(&mut disk)?;
///
/// assert_eq!(layout.partitions[0].name, "DH0");
/// assert_eq!(layout.partitions[1].name, "WORK");
///
/// // The disk is the authority on what is on the disk.
/// let rdb = Rdb::parse(&mut disk)?;
/// assert_eq!(rdb.partitions.len(), 2);
/// assert!(rdb.validate().is_empty());
/// # Ok(())
/// # }
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RdbBuilder {
    geometry: Geometry,
    rdsk_block: u32,
    reserved_blocks: Option<u32>,
    flags: u32,
    host_id: u32,
    specs: Vec<PartitionSpec>,
}

/// Longest `pb_DriveName` the 32-byte BCPL field holds: one length byte
/// and 31 characters.
const MAX_DRIVE_NAME: usize = 31;

/// How many longwords an `RDSK`, `PART` or `FSHD` block sums over — 64,
/// i.e. the first 256 bytes, whatever the device block size.
const HEADER_SUMMED_LONGS: u32 = 64;

impl RdbBuilder {
    /// A builder for a disk of exactly this [`Geometry`] — the entry
    /// point for a caller that already has one, whether from
    /// [`synthesize_geometry`] or from an existing disk it is cloning.
    pub fn new(geometry: Geometry) -> Self {
        Self {
            geometry,
            rdsk_block: 0,
            reserved_blocks: None,
            flags: rdsk_defaults::FLAGS,
            host_id: rdsk_defaults::HOST_ID,
            specs: Vec::new(),
        }
    }

    /// A builder for a disk of `total_bytes` in `block_size`-byte blocks,
    /// with the geometry [`synthesize_geometry`] chooses — the entry
    /// point for a caller that has a size and no opinion about cylinders,
    /// which is most of them.
    pub fn for_size(total_bytes: u64, block_size: usize) -> Result<Self, GeometryError> {
        Ok(Self::new(synthesize_geometry(total_bytes, block_size)?))
    }

    /// Add a partition. Order matters: [`Placement::Size`] partitions are
    /// laid out in the order added, and the `PART` chain follows it.
    pub fn partition(mut self, spec: PartitionSpec) -> Self {
        self.specs.push(spec);
        self
    }

    /// Put the `RDSK` block somewhere other than block 0.
    ///
    /// Block 0 is where `rdbtool` writes it and where every image this
    /// crate has seen carries it; the format allows anywhere in the first
    /// [`RDB_LOCATION_LIMIT`] blocks, which exists so a disk can carry a
    /// foreign boot sector at block 0 and an RDB behind it. Values at or
    /// above the limit are not refused here — the RDSK simply would not
    /// be found, which [`build`](Self::build) leaves to the caller who
    /// deliberately asked for it.
    pub fn rdsk_block(mut self, lba: u32) -> Self {
        self.rdsk_block = lba;
        self
    }

    /// Reserve exactly this many blocks for the RDB area
    /// (`rdb_RDBBlocksLo..=rdb_RDBBlocksHi`), overriding the default
    /// policy [`build`](Self::build) documents.
    ///
    /// The count is from block 0, so it is also `rdb_RDBBlocksHi + 1`. A
    /// value too small for the `RDSK` and `PART` blocks the layout needs
    /// is [`BuildError::RdbAreaTooSmall`], not a silent overflow into the
    /// first partition.
    pub fn reserved_blocks(mut self, blocks: u32) -> Self {
        self.reserved_blocks = Some(blocks);
        self
    }

    /// Set `rdb_Flags`, overriding [`rdsk_defaults::FLAGS`].
    pub fn flags(mut self, flags: u32) -> Self {
        self.flags = flags;
        self
    }

    /// Set `rdb_HostID`, overriding [`rdsk_defaults::HOST_ID`].
    pub fn host_id(mut self, host_id: u32) -> Self {
        self.host_id = host_id;
        self
    }

    /// Compute the layout, validate it, and write it to `sink`.
    ///
    /// # What gets written where
    ///
    /// The `RDSK` block goes at [`rdsk_block`](Self::rdsk_block)
    /// (default 0, `rdbtool`'s choice), and the `PART` blocks follow it
    /// consecutively, chained in the order the partitions were added and
    /// terminated with [`CHAIN_END`]. `rdb_HighRDSKBlock` records the
    /// last block used; `rdb_RDBBlocksHi` records the reserved ceiling,
    /// which is normally higher — see below. The `FSHD` and `BADB` chain
    /// heads are `CHAIN_END`: a fresh RDB carries no loadable filesystem
    /// (that is the next milestone) and no bad blocks.
    ///
    /// # Names
    ///
    /// A [`PartitionSpec`] without a name gets the first `DH`*n* not
    /// already claimed by an *explicit* name anywhere in the build — so a
    /// mixed build of `[unnamed, "DH0", unnamed]` yields `DH1`, `DH0`,
    /// `DH2`, and never a duplicate. Two explicit names that collide are
    /// [`BuildError::DuplicateName`]; the builder renames nothing it was
    /// told.
    ///
    /// # Cylinders
    ///
    /// `rdb_LoCylinder` is the first cylinder past the reserved area and
    /// `rdb_HiCylinder` the geometry's last, both inclusive.
    /// [`Placement::Size`] partitions are packed from `rdb_LoCylinder`
    /// upward in the order added, each starting on the cylinder after the
    /// previous one's last; [`Placement::Cylinders`] partitions go
    /// exactly where they say and do not move the packing cursor past
    /// themselves except when they end above it. Every partition is then
    /// checked against the disk's end, against the RDB area, and against
    /// every other partition.
    ///
    /// # Zero partitions
    ///
    /// Legal, and supported: `rdb_PartitionList` is [`CHAIN_END`] and the
    /// result is an initialised disk with no partitions, which is what
    /// `rdbtool`'s `create` + `init` produces and what a caller
    /// partitioning in a later step wants.
    pub fn build<S: BlockSink>(&self, sink: &mut S) -> Result<RdbLayout, BuildError<S::Error>> {
        let block_size = sink.block_size();
        if !block_size_ok(block_size) {
            return Err(BuildError::UnsupportedBlockSize { block_size });
        }
        if self.geometry.block_size != block_size {
            return Err(BuildError::BlockSizeMismatch {
                geometry: self.geometry.block_size,
                sink: block_size,
            });
        }

        let layout = self.layout(sink.block_count())?;

        // Everything below is a write. Nothing above one wrote a byte,
        // which is the invariant the whole type exists for.
        let mut buf = alloc::vec![0u8; block_size];
        for (i, placed) in layout.partitions.iter().enumerate() {
            let next = match layout.partitions.get(i + 1) {
                Some(p) => p.part_block as u32,
                None => CHAIN_END,
            };
            buf.iter_mut().for_each(|b| *b = 0);
            self.fill_part(&mut buf, &self.specs[i], placed, next)?;
            sink.write_block(placed.part_block, &buf)
                .map_err(BuildError::Io)?;
        }

        // The RDSK last: until it lands the disk has no partition table
        // at all, which is a better outcome for an interrupted build
        // than a table pointing at blocks that were never written.
        buf.iter_mut().for_each(|b| *b = 0);
        self.fill_rdsk(&mut buf, &layout)?;
        sink.write_block(layout.rdsk_block, &buf)
            .map_err(BuildError::Io)?;

        Ok(layout)
    }

    /// The whole layout, or the first reason it cannot exist. Split out
    /// of [`build`](Self::build) so that "compute everything, then write"
    /// is structural rather than a discipline: this function has no sink
    /// and so cannot write.
    fn layout<E>(&self, sink_blocks: Option<u64>) -> Result<RdbLayout, BuildError<E>> {
        let g = self.geometry;
        let cyl_blocks = g.cylinder_blocks();
        if g.cylinders == 0 || cyl_blocks == 0 {
            return Err(BuildError::EmptyGeometry { geometry: g });
        }
        let last_cylinder = g.cylinders - 1;

        // Names first: the layout errors below name the partition they
        // are about, so the names have to exist before the placement.
        let names = self.assign_names()?;

        // The RDB area. `rdb_RDBBlocksLo` is 0 rather than the RDSK's own
        // block: the area is what a repartitioner owns, and that includes
        // any block before the RDSK it might move the RDSK into.
        let needed = 1u64 + self.specs.len() as u64;
        let reserved = match self.reserved_blocks {
            Some(n) => n as u64,
            None => self.default_reserved_blocks(cyl_blocks, needed),
        };
        if reserved < needed || reserved > u32::MAX as u64 {
            return Err(BuildError::RdbAreaTooSmall {
                needed: needed.min(u32::MAX as u64) as u32,
                available: reserved.min(u32::MAX as u64) as u32,
            });
        }
        // Every RDB block must sit inside the area, the RDSK included.
        let last_used = self.rdsk_block as u64 + needed - 1;
        if last_used >= reserved {
            return Err(BuildError::RdbAreaTooSmall {
                needed: (last_used + 1).min(u32::MAX as u64) as u32,
                available: reserved as u32,
            });
        }

        // The area is rounded up to a whole cylinder because a partition
        // can only start on one: a reserved area ending mid-cylinder
        // would leave the rest of that cylinder owned by nobody, or —
        // worse — by the first partition, which is the overlap.
        // (`div_ceil` would say this, but it is newer than this crate's
        // MSRV; `reserved >= 1` so the addition cannot be the problem.)
        let lo_cylinder = (reserved + cyl_blocks - 1) / cyl_blocks;
        if lo_cylinder > last_cylinder as u64 {
            return Err(BuildError::PartitionOverlapsRdbArea {
                name: String::from("<the disk>"),
                low_cyl: last_cylinder,
                lo_cylinder: lo_cylinder.min(u32::MAX as u64) as u32,
            });
        }
        let lo_cylinder = lo_cylinder as u32;

        // Placement, in the order added. `next_cyl` is where the next
        // sized partition starts; an explicit range pushes it past
        // itself, so mixing the two packs rather than colliding.
        let mut next_cyl = lo_cylinder;
        let mut partitions = Vec::with_capacity(self.specs.len());
        for (i, spec) in self.specs.iter().enumerate() {
            let name = names[i].clone();
            let (low_cyl, high_cyl) = match spec.placement {
                Placement::Cylinders { low, high } => {
                    if high < low {
                        return Err(BuildError::CylindersInverted {
                            name,
                            low_cyl: low,
                            high_cyl: high,
                        });
                    }
                    (low, high)
                }
                Placement::Size(bytes) => {
                    // Floored, matching rdbtool: a partition claims at
                    // most what was asked for. Below one cylinder there
                    // is nothing to claim — de_HighCyl is inclusive, so a
                    // zero-cylinder partition cannot be written — and
                    // that is an error rather than a rounded-up cylinder.
                    let cylinder_bytes = cyl_blocks * g.block_size as u64;
                    let want = bytes / cylinder_bytes;
                    if want == 0 {
                        return Err(BuildError::PartitionTooSmall {
                            name,
                            bytes,
                            cylinder_bytes,
                        });
                    }
                    let low = next_cyl as u64;
                    let high = low.saturating_add(want - 1);
                    if high > last_cylinder as u64 {
                        return Err(BuildError::PartitionPastEndOfDisk {
                            name,
                            high_cyl: high.min(u32::MAX as u64) as u32,
                            last_cylinder,
                        });
                    }
                    (low as u32, high as u32)
                }
            };

            if high_cyl > last_cylinder {
                return Err(BuildError::PartitionPastEndOfDisk {
                    name,
                    high_cyl,
                    last_cylinder,
                });
            }
            if low_cyl < lo_cylinder {
                return Err(BuildError::PartitionOverlapsRdbArea {
                    name,
                    low_cyl,
                    lo_cylinder,
                });
            }
            next_cyl = next_cyl.max(high_cyl.saturating_add(1));

            partitions.push(PlacedPartition {
                part_block: self.rdsk_block as u64 + 1 + i as u64,
                name,
                low_cyl,
                high_cyl,
                start_lba: low_cyl as u64 * cyl_blocks,
                block_len: (high_cyl as u64 - low_cyl as u64 + 1) * cyl_blocks,
            });
        }

        // Pairwise overlap. Quadratic on a list that is single digits in
        // every real layout and capped by the RDB area's block count in
        // any case; the same check [`Rdb::validate`] makes, made before
        // the image exists rather than after.
        for a in 0..partitions.len() {
            for b in a + 1..partitions.len() {
                let (pa, pb) = (&partitions[a], &partitions[b]);
                let low = pa.low_cyl.max(pb.low_cyl);
                let high = pa.high_cyl.min(pb.high_cyl);
                if low <= high {
                    return Err(BuildError::PartitionsOverlap {
                        a_name: pa.name.clone(),
                        b_name: pb.name.clone(),
                        low_cyl: low,
                        high_cyl: high,
                    });
                }
            }
        }

        // The target has to actually hold what the layout describes: the
        // RDB area, and every partition's last block.
        let mut needed_blocks = reserved;
        for p in &partitions {
            needed_blocks = needed_blocks.max(p.start_lba + p.block_len);
        }
        if let Some(available) = sink_blocks {
            if needed_blocks > available {
                return Err(BuildError::SinkTooSmall {
                    needed: needed_blocks,
                    available,
                });
            }
        }

        Ok(RdbLayout {
            rdsk_block: self.rdsk_block as u64,
            rdb_blocks_lo: 0,
            rdb_blocks_hi: (reserved - 1) as u32,
            high_rdsk_block: last_used as u32,
            lo_cylinder,
            hi_cylinder: last_cylinder,
            geometry: g,
            partitions,
        })
    }

    /// How many blocks to reserve when the caller did not say.
    ///
    /// **Policy: `rdbtool`'s reserved area, or what the layout needs plus
    /// headroom, whichever is larger.** `rdbtool` reserves the disk's
    /// whole first cylinder (`rdb_RDBBlocksHi = rdb_CylBlocks - 1`,
    /// `rdb_LoCylinder = 1`) — a *fixed* policy that does not scale with
    /// the partition count, which is in tension with this crate's plan to
    /// "size from what will actually be stored". The tension is resolved
    /// in favour of interoperability for the common case and correctness
    /// for the uncommon one: a first cylinder is 32 blocks at the
    /// smallest Amiga geometry and 1024 at a PC-ish one, which is roomy
    /// for any realistic partition count, so matching `rdbtool` costs
    /// nothing and keeps images comparable. Where it is *not* enough —
    /// many partitions on a small-cylinder disk, or a future `FSHD`/`LSEG`
    /// payload — the area grows to what is needed plus
    /// [`RDB_HEADROOM_BLOCKS`] of slack for later edits, rather than
    /// overflowing into the first partition.
    ///
    /// [`reserved_blocks`](Self::reserved_blocks) overrides this
    /// entirely, for a caller reproducing an existing image's area or one
    /// who knows what it is about to store.
    fn default_reserved_blocks(&self, cyl_blocks: u64, needed: u64) -> u64 {
        let rdbtool_area = cyl_blocks;
        let with_headroom = (self.rdsk_block as u64 + needed).saturating_add(RDB_HEADROOM_BLOCKS);
        rdbtool_area.max(with_headroom)
    }

    /// The `pb_DriveName` for every spec, explicit ones as given and the
    /// rest assigned.
    fn assign_names<E>(&self) -> Result<Vec<String>, BuildError<E>> {
        let mut names: Vec<Option<String>> = Vec::with_capacity(self.specs.len());
        for spec in &self.specs {
            match &spec.name {
                Some(n) => {
                    if n.is_empty() || n.len() > MAX_DRIVE_NAME {
                        return Err(BuildError::InvalidName {
                            name: n.clone(),
                            max: MAX_DRIVE_NAME,
                        });
                    }
                    if names.iter().flatten().any(|other| other == n) {
                        return Err(BuildError::DuplicateName { name: n.clone() });
                    }
                    names.push(Some(n.clone()));
                }
                None => names.push(None),
            }
        }

        // Assigned names avoid every *explicit* name, not merely the ones
        // already assigned — otherwise a build of [unnamed, "DH0"] would
        // hand out DH0 twice and the collision check above would not see
        // it, the two names having been decided in different places.
        let mut next = 0u32;
        let out = names
            .iter()
            .map(|n| match n {
                Some(n) => n.clone(),
                None => loop {
                    let candidate = alloc::format!("DH{next}");
                    next += 1;
                    if !names.iter().flatten().any(|other| *other == candidate) {
                        break candidate;
                    }
                },
            })
            .collect();
        Ok(out)
    }

    /// Fill a zeroed buffer with the `RDSK` block and seal it.
    fn fill_rdsk<E>(&self, buf: &mut [u8], layout: &RdbLayout) -> Result<(), BuildError<E>> {
        let g = layout.geometry;
        put_be32(buf, hdr::ID, id::RDSK);
        put_be32(buf, rdsk::HOST_ID, self.host_id);
        put_be32(buf, rdsk::BLOCK_BYTES, g.block_size as u32);
        put_be32(buf, rdsk::FLAGS, self.flags);
        put_be32(buf, rdsk::BAD_BLOCK_LIST, CHAIN_END);
        put_be32(
            buf,
            rdsk::PARTITION_LIST,
            match layout.partitions.first() {
                Some(p) => p.part_block as u32,
                None => CHAIN_END,
            },
        );
        put_be32(buf, rdsk::FILESYS_HEADER_LIST, CHAIN_END);
        put_be32(buf, rdsk::DRIVE_INIT, CHAIN_END);
        put_be32(buf, rdsk::CYLINDERS, g.cylinders);
        put_be32(buf, rdsk::SECTORS, g.sectors);
        put_be32(buf, rdsk::HEADS, g.heads);
        put_be32(buf, rdsk::INTERLEAVE, rdsk_defaults::INTERLEAVE);
        // Park on the cylinder past the last: the landing zone of a drive
        // whose data ends where the geometry does.
        put_be32(buf, rdsk::PARK, g.cylinders);
        put_be32(buf, rdsk::WRITE_PRE_COMP, g.cylinders);
        put_be32(buf, rdsk::REDUCED_WRITE, g.cylinders);
        put_be32(buf, rdsk::STEP_RATE, rdsk_defaults::STEP_RATE);
        put_be32(buf, rdsk::RDB_BLOCKS_LO, layout.rdb_blocks_lo);
        put_be32(buf, rdsk::RDB_BLOCKS_HI, layout.rdb_blocks_hi);
        put_be32(buf, rdsk::LO_CYLINDER, layout.lo_cylinder);
        put_be32(buf, rdsk::HI_CYLINDER, layout.hi_cylinder);
        put_be32(buf, rdsk::CYL_BLOCKS, g.cylinder_blocks() as u32);
        put_be32(
            buf,
            rdsk::AUTO_PARK_SECONDS,
            rdsk_defaults::AUTO_PARK_SECONDS,
        );
        put_be32(buf, rdsk::HIGH_RDSK_BLOCK, layout.high_rdsk_block);
        // The six identification fields stay zero — a deliberate
        // deviation from rdbtool, which writes "RDBTOOL"/"IMAGE"/"2012"
        // into the disk triple while leaving rdb_Flags at 0x7, i.e.
        // without DISK_ID set. By the format's own rule those bytes then
        // mean nothing, and a consumer that checks the bit (as this
        // crate's docs require) will not show them either way; writing a
        // vendor string for a disk that is not on a bus is an invention,
        // and an unflagged one is an invention a careless reader prints.
        seal_checksum(buf, HEADER_SUMMED_LONGS).map_err(BuildError::Seal)
    }

    /// Fill a zeroed buffer with one `PART` block and seal it.
    fn fill_part<E>(
        &self,
        buf: &mut [u8],
        spec: &PartitionSpec,
        placed: &PlacedPartition,
        next: u32,
    ) -> Result<(), BuildError<E>> {
        put_be32(buf, hdr::ID, id::PART);
        put_be32(buf, hdr::HOST_ID, self.host_id);
        put_be32(buf, chain::NEXT, next);
        let flags = (spec.bootable as u32) | ((spec.no_automount as u32) << 1);
        put_be32(buf, part::FLAGS, flags);

        // pb_DriveName is BCPL — a length byte then the characters, no
        // terminator — unlike the RDSK's identification strings four
        // structures away, which are space-padded ASCII.
        let name = placed.name.as_bytes();
        buf[part::DRIVE_NAME] = name.len() as u8;
        buf[part::DRIVE_NAME + 1..part::DRIVE_NAME + 1 + name.len()].copy_from_slice(name);

        let g = self.geometry;
        let mut env = |i: usize, v: u32| put_be32(buf, part::ENVIRONMENT + i * 4, v);
        env(de::TABLE_SIZE, envec_defaults::TABLE_SIZE);
        env(
            de::SIZE_BLOCK,
            spec.size_block_longs
                .unwrap_or_else(|| envec_defaults::size_block_longs(g.block_size)),
        );
        env(de::SEC_ORG, spec.sec_org);
        env(de::SURFACES, g.heads);
        env(de::SECTORS_PER_BLOCK, spec.sectors_per_block);
        env(de::BLOCKS_PER_TRACK, g.sectors);
        env(de::RESERVED, spec.reserved);
        env(de::PRE_ALLOC, spec.pre_alloc);
        env(de::INTERLEAVE, spec.interleave);
        env(de::LOW_CYL, placed.low_cyl);
        env(de::HIGH_CYL, placed.high_cyl);
        env(de::NUM_BUFFERS, spec.num_buffers);
        env(de::BUF_MEM_TYPE, spec.buf_mem_type);
        env(de::MAX_TRANSFER, spec.max_transfer);
        env(de::MASK, spec.mask);
        env(de::BOOT_PRI, spec.boot_pri as u32);
        env(de::DOS_TYPE, spec.dos_type);

        seal_checksum(buf, HEADER_SUMMED_LONGS).map_err(BuildError::Seal)
    }
}

/// Blocks of slack the RDB area gets past what a build actually uses,
/// when the layout needs more than `rdbtool`'s first cylinder.
///
/// Headroom exists so a later edit — one more partition, an `FSHD` and
/// its `LSEG` chain — has somewhere to go that is not the first
/// partition. Sixteen is a cylinder's worth on the smallest geometry this
/// crate produces and cheap on any disk; growing the area after the fact
/// means moving a partition, which is the expensive operation this is
/// buying insurance against.
pub const RDB_HEADROOM_BLOCKS: u64 = 16;

/// A [`BlockSource`] view of one partition: LBA 0 here is
/// `partition.start_lba` on the parent, in the parent's device blocks
/// (the block size passes through unchanged — this adapter never speaks
/// `de_SizeBlock` filesystem blocks). This is the composition seam with
/// filesystem crates — they mount one of these, never the disk.
pub struct PartitionSource<'a, S: BlockSource> {
    parent: &'a mut S,
    start_lba: u64,
    block_len: u64,
}

/// Error from [`PartitionSource`]: either the parent's error, or a read
/// past the partition's end (which the parent could not catch — the
/// block may exist on disk, just not in this partition).
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PartitionSourceError<E> {
    /// The parent [`BlockSource`] failed on the underlying read.
    Parent(E),
    /// A read past the partition's last block.
    OutOfRange {
        /// The partition-relative block asked for.
        lba: u64,
        /// How many blocks the partition has, so `lba` had to be below it.
        len: u64,
    },
}

impl<E: core::fmt::Display> core::fmt::Display for PartitionSourceError<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            PartitionSourceError::Parent(e) => {
                write!(f, "the parent device failed: {e}")
            }
            PartitionSourceError::OutOfRange { lba, len } => write!(
                f,
                "block {lba} is past the end of the partition, which has {len} blocks"
            ),
        }
    }
}

#[cfg(feature = "std")]
impl<E: std::error::Error + 'static> std::error::Error for PartitionSourceError<E> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            PartitionSourceError::Parent(e) => Some(e),
            PartitionSourceError::OutOfRange { .. } => None,
        }
    }
}

impl<'a, S: BlockSource> PartitionSource<'a, S> {
    /// View `partition` of `parent` as a [`BlockSource`] of its own,
    /// borrowing the parent for as long as the view lives.
    pub fn new(parent: &'a mut S, partition: &Partition) -> Self {
        Self {
            parent,
            start_lba: partition.start_lba,
            block_len: partition.block_len,
        }
    }
}

impl<'a, S: BlockSource> BlockSource for PartitionSource<'a, S> {
    type Error = PartitionSourceError<S::Error>;

    fn block_size(&self) -> usize {
        self.parent.block_size()
    }

    fn read_block(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), Self::Error> {
        // The second half of the guard is not paranoia about the
        // caller: a `Partition` parsed from a hostile PART block can
        // carry a saturated `start_lba` and `block_len`, and then an
        // in-range `lba` still runs off the end of the *address space*.
        // Out of range is out of range whichever end it falls off.
        let parent_lba = match self.start_lba.checked_add(lba) {
            Some(l) if lba < self.block_len => l,
            _ => {
                return Err(PartitionSourceError::OutOfRange {
                    lba,
                    len: self.block_len,
                })
            }
        };
        self.parent
            .read_block(parent_lba, buf)
            .map_err(PartitionSourceError::Parent)
    }

    fn block_count(&self) -> Option<u64> {
        Some(self.block_len)
    }
}

#[cfg(feature = "std")]
mod std_support {
    use super::{block_size_ok, BlockSink, BlockSource, MIN_BLOCK_SIZE};
    use std::io::{Read, Seek, SeekFrom, Write};

    /// A [`BlockSource`] over anything `Read + Seek` — a `File`, a
    /// `Cursor<Vec<u8>>`. The convenience the `std` feature exists for.
    ///
    /// A byte stream carries no sector size of its own, so the caller
    /// supplies it: [`new`](Self::new) assumes the classic 512, and
    /// [`with_block_size`](Self::with_block_size) takes the size of the
    /// device the image was taken from.
    ///
    /// It is also a [`BlockSink`] when — and only when — the inner type
    /// is `Write` as well. One type serves both directions rather than a
    /// `SeekBlockSink` beside it: the seek-and-transfer logic is the
    /// same and the block size must not be allowed to differ between a
    /// read view and a write view of one file. A read-only `T` simply
    /// does not get the [`BlockSink`] impl, so "this file was opened for
    /// reading" stays a compile-time fact — which is the whole reason
    /// [`BlockSink`] is a separate trait.
    pub struct SeekBlockSource<T: Read + Seek> {
        inner: T,
        block_size: usize,
        blocks: Option<u64>,
    }

    impl<T: Read + Seek> SeekBlockSource<T> {
        /// A 512-byte-block view of `inner`.
        pub fn new(inner: T) -> std::io::Result<Self> {
            Self::with_block_size(inner, MIN_BLOCK_SIZE)
        }

        /// A view of `inner` with the given device block size.
        ///
        /// `block_count` comes from the stream length; a stream whose
        /// length is not a block multiple keeps its trailing fragment
        /// invisible, the same as a real disk with a partial final
        /// sector. An unsupported `block_size` is
        /// `std::io::ErrorKind::InvalidInput`.
        pub fn with_block_size(mut inner: T, block_size: usize) -> std::io::Result<Self> {
            if !block_size_ok(block_size) {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("unsupported block size {block_size}"),
                ));
            }
            let len = inner.seek(SeekFrom::End(0))?;
            Ok(Self {
                inner,
                block_size,
                blocks: Some(len / block_size as u64),
            })
        }
    }

    impl<T: Read + Seek> BlockSource for SeekBlockSource<T> {
        type Error = std::io::Error;

        fn block_size(&self) -> usize {
            self.block_size
        }

        fn read_block(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), Self::Error> {
            self.inner
                .seek(SeekFrom::Start(lba * self.block_size as u64))?;
            self.inner.read_exact(buf)
        }

        fn block_count(&self) -> Option<u64> {
            self.blocks
        }
    }

    impl<T: Read + Write + Seek> BlockSink for SeekBlockSource<T> {
        type Error = std::io::Error;

        fn block_size(&self) -> usize {
            self.block_size
        }

        fn write_block(&mut self, lba: u64, buf: &[u8]) -> Result<(), Self::Error> {
            self.inner
                .seek(SeekFrom::Start(lba * self.block_size as u64))?;
            self.inner.write_all(buf)
        }

        /// The block count sampled when the source was constructed, and
        /// so *not* updated by a write past the end that grows a file.
        /// Deliberate: it describes the device this view was opened on,
        /// and a writer asking "does my layout fit" wants that answer,
        /// not one that moves as it writes.
        fn block_count(&self) -> Option<u64> {
            self.blocks
        }
    }
}

#[cfg(feature = "std")]
pub use std_support::SeekBlockSource;

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    /// In-memory disk for tests, with a configurable block size.
    struct MemDisk {
        data: Vec<u8>,
        block_size: usize,
    }

    impl MemDisk {
        fn new(data: Vec<u8>) -> Self {
            Self {
                data,
                block_size: 512,
            }
        }
    }

    impl BlockSource for MemDisk {
        type Error = ();

        fn block_size(&self) -> usize {
            self.block_size
        }

        fn read_block(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), ()> {
            let off = lba as usize * self.block_size;
            if off + self.block_size > self.data.len() {
                return Err(());
            }
            buf.copy_from_slice(&self.data[off..off + self.block_size]);
            Ok(())
        }

        fn block_count(&self) -> Option<u64> {
            Some((self.data.len() / self.block_size) as u64)
        }
    }

    /// The same buffer, writable — the in-memory sink the write path is
    /// developed against. It refuses a write past the end exactly as it
    /// refuses a read past the end: a sink that silently grew would hide
    /// the "layout runs off the disk" bug the checks exist to catch.
    impl BlockSink for MemDisk {
        type Error = ();

        fn block_size(&self) -> usize {
            self.block_size
        }

        fn write_block(&mut self, lba: u64, buf: &[u8]) -> Result<(), ()> {
            let off = lba as usize * self.block_size;
            if off + self.block_size > self.data.len() {
                return Err(());
            }
            self.data[off..off + self.block_size].copy_from_slice(buf);
            Ok(())
        }

        fn block_count(&self) -> Option<u64> {
            Some((self.data.len() / self.block_size) as u64)
        }
    }

    fn put32(disk: &mut [u8], bs: usize, block: usize, off: usize, v: u32) {
        let o = block * bs + off;
        disk[o..o + 4].copy_from_slice(&v.to_be_bytes());
    }

    /// Seal block `block` of a whole-disk buffer over `longs`
    /// longwords.
    ///
    /// A thin address-arithmetic wrapper over the public
    /// [`seal_checksum`] rather than a second implementation of it: the
    /// fixtures below seal several hundred blocks between them, so the
    /// production sealer gets exercised by every parse test in the file
    /// — the round-trip property (`seal_checksum` then `checksum_ok`)
    /// proved incidentally, everywhere, in addition to
    /// [`seal_then_checksum_ok_round_trips`] proving it on purpose.
    fn seal(disk: &mut [u8], bs: usize, block: usize, longs: u32) {
        let base = block * bs;
        seal_checksum(&mut disk[base..base + bs], longs).unwrap();
    }

    /// Write a fixed-width, space-padded ASCII field (the RDSK
    /// identification convention) at `off`, truncating an over-long
    /// value the way a real controller's INQUIRY copy would.
    fn put_padded(disk: &mut [u8], bs: usize, block: usize, (off, len): (usize, usize), s: &str) {
        let base = block * bs + off;
        disk[base..base + len].fill(b' ');
        let b = s.as_bytes();
        let n = b.len().min(len);
        disk[base..base + n].copy_from_slice(&b[..n]);
    }

    /// Build a minimal valid image with the given device block size:
    /// RDSK at `rdsk_block`, one PART. Geometry: 10 cylinders of 32
    /// blocks, regardless of block size.
    fn one_partition_image_bs(rdsk_block: usize, bs: usize) -> Vec<u8> {
        one_partition_image_envec(rdsk_block, bs, 16)
    }

    /// As [`one_partition_image_bs`], but with an explicit
    /// `de_TableSize` and the longwords 17..=19 filled in, so the
    /// optional-tail cases (and a hostile TableSize) share one builder.
    fn one_partition_image_envec(rdsk_block: usize, bs: usize, table_size: u32) -> Vec<u8> {
        // Sized to the geometry it declares: 10 cylinders of 32 blocks.
        let mut d = vec![0u8; 320 * bs];
        let part_block = rdsk_block + 1;

        put32(&mut d, bs, rdsk_block, 0, id::RDSK);
        put32(&mut d, bs, rdsk_block, rdsk::BLOCK_BYTES, bs as u32);
        put32(&mut d, bs, rdsk_block, rdsk::BAD_BLOCK_LIST, CHAIN_END);
        put32(
            &mut d,
            bs,
            rdsk_block,
            rdsk::PARTITION_LIST,
            part_block as u32,
        );
        put32(&mut d, bs, rdsk_block, rdsk::FILESYS_HEADER_LIST, CHAIN_END);
        put32(&mut d, bs, rdsk_block, rdsk::CYLINDERS, 10);
        put32(&mut d, bs, rdsk_block, rdsk::SECTORS, 32);
        put32(&mut d, bs, rdsk_block, rdsk::HEADS, 1);
        put32(&mut d, bs, rdsk_block, rdsk::HOST_ID, 7);
        put32(&mut d, bs, rdsk_block, rdsk::DRIVE_INIT, CHAIN_END);
        put32(&mut d, bs, rdsk_block, rdsk::INTERLEAVE, 1);
        put32(&mut d, bs, rdsk_block, rdsk::PARK, 10);
        put32(&mut d, bs, rdsk_block, rdsk::WRITE_PRE_COMP, 10);
        put32(&mut d, bs, rdsk_block, rdsk::REDUCED_WRITE, 10);
        put32(&mut d, bs, rdsk_block, rdsk::STEP_RATE, 3);
        put32(&mut d, bs, rdsk_block, rdsk::LO_CYLINDER, 2);
        put32(&mut d, bs, rdsk_block, rdsk::HI_CYLINDER, 9);
        put32(&mut d, bs, rdsk_block, rdsk::CYL_BLOCKS, 32);
        put32(&mut d, bs, rdsk_block, rdsk::AUTO_PARK_SECONDS, 0);
        // A reserved area covering the whole of the first cylinder-ish
        // prefix, well below the first partition at block 64, so the
        // fixture is a *clean* layout and validate() has something
        // meaningful to say when a test then breaks it.
        put32(&mut d, bs, rdsk_block, rdsk::RDB_BLOCKS_LO, 0);
        put32(&mut d, bs, rdsk_block, rdsk::RDB_BLOCKS_HI, 15);
        put32(
            &mut d,
            bs,
            rdsk_block,
            rdsk::HIGH_RDSK_BLOCK,
            part_block as u32,
        );
        put32(
            &mut d,
            bs,
            rdsk_block,
            rdsk::FLAGS,
            rdb_flags::DISK_ID | rdb_flags::CTRLR_ID,
        );
        // Space-padded, and DISK_PRODUCT deliberately fills its whole
        // 16 bytes so the trimmer is exercised on both cases.
        put_padded(&mut d, bs, rdsk_block, rdsk::DISK_VENDOR, "QUANTUM ");
        put_padded(
            &mut d,
            bs,
            rdsk_block,
            rdsk::DISK_PRODUCT,
            "FIREBALL_TM3200S",
        );
        put_padded(&mut d, bs, rdsk_block, rdsk::DISK_REVISION, "300 ");
        put_padded(&mut d, bs, rdsk_block, rdsk::CONTROLLER_VENDOR, "CBM     ");
        put_padded(&mut d, bs, rdsk_block, rdsk::CONTROLLER_PRODUCT, "A4091");
        put_padded(&mut d, bs, rdsk_block, rdsk::CONTROLLER_REVISION, "40.9");
        seal(&mut d, bs, rdsk_block, 64);

        write_part(
            &mut d,
            bs,
            part_block,
            CHAIN_END,
            "DH0",
            (bs / 4) as u32,
            (2, 9),
            table_size,
        );

        d
    }

    /// Write one `PART` block: name, `de_SizeBlock` in longwords, the
    /// inclusive cylinder range, `de_TableSize`, and the next-block
    /// pointer. Split out of the image builders because a second
    /// partition differs from the first only in these, and a fixture
    /// that duplicated thirty `put32`s to change three of them would
    /// drift.
    #[allow(clippy::too_many_arguments)]
    fn write_part(
        d: &mut [u8],
        bs: usize,
        block: usize,
        next: u32,
        name: &str,
        size_block_longs: u32,
        (low_cyl, high_cyl): (u32, u32),
        table_size: u32,
    ) {
        put32(d, bs, block, 0, id::PART);
        put32(d, bs, block, chain::NEXT, next);
        put32(d, bs, block, part::FLAGS, 1); // bootable
        let name = name.as_bytes();
        d[block * bs + part::DRIVE_NAME] = name.len() as u8;
        d[block * bs + part::DRIVE_NAME + 1..block * bs + part::DRIVE_NAME + 1 + name.len()]
            .copy_from_slice(name);
        let e = part::ENVIRONMENT;
        put32(d, bs, block, e + de::TABLE_SIZE * 4, table_size);
        put32(d, bs, block, e + de::SIZE_BLOCK * 4, size_block_longs);
        put32(d, bs, block, e + de::SURFACES * 4, 1);
        put32(d, bs, block, e + de::BLOCKS_PER_TRACK * 4, 32);
        put32(d, bs, block, e + de::LOW_CYL * 4, low_cyl);
        put32(d, bs, block, e + de::HIGH_CYL * 4, high_cyl);
        put32(d, bs, block, e + de::BOOT_PRI * 4, 0);
        put32(d, bs, block, e + de::DOS_TYPE * 4, 0x444F_5303);
        // Written unconditionally: whether they are *readable* is
        // de_TableSize's business, and planting them even when it says
        // they are absent is what proves the gate works.
        put32(d, bs, block, e + de::BAUD * 4, 9600);
        put32(d, bs, block, e + de::CONTROL * 4, 0x1234);
        put32(d, bs, block, e + de::BOOT_BLOCKS * 4, 2);
        seal(d, bs, block, 64);
    }

    /// A 512-byte-device-block image carrying *two* partitions chained
    /// off one RDSK, each with its own `de_SizeBlock` and cylinder
    /// range.
    ///
    /// The point of the fixture is that `de_SizeBlock` is per partition:
    /// one disk legitimately carries a 4 KB-filesystem-block partition
    /// beside a 32 KB one, and nothing in this crate may assume a single
    /// value per disk. It doubles as the two-extent fixture the overlap
    /// validation needs, since overlapping partitions are just a
    /// different cylinder range here.
    fn two_partition_image(
        (name_a, size_block_a, cyls_a): (&str, u32, (u32, u32)),
        (name_b, size_block_b, cyls_b): (&str, u32, (u32, u32)),
    ) -> Vec<u8> {
        let bs = 512;
        let mut d = one_partition_image(2);
        write_part(&mut d, bs, 3, 4, name_a, size_block_a, cyls_a, 16);
        write_part(&mut d, bs, 4, CHAIN_END, name_b, size_block_b, cyls_b, 16);
        put32(&mut d, bs, 2, rdsk::HIGH_RDSK_BLOCK, 4);
        seal(&mut d, bs, 2, 64);
        d
    }

    fn one_partition_image(rdsk_block: usize) -> Vec<u8> {
        one_partition_image_bs(rdsk_block, 512)
    }

    /// Blocks the [`fs_image`] fixture lays its extra chains out on,
    /// after the RDSK at 2 and the PART at 3.
    const FSHD_BLOCK: usize = 4;
    const LSEG_BLOCK: usize = 5; // and 6
    const BADB_BLOCK: usize = 7; // and 8

    /// Bytes of driver payload one 512-byte `LSEG` block carries.
    const LSEG_PAYLOAD: usize = 512 - lseg::LOAD_DATA;

    /// [`one_partition_image`] plus a one-entry `FSHD` chain with a
    /// two-block `LSEG` chain, and a two-block `BADB` chain. Returns the
    /// image and the exact driver bytes the `LSEG` blocks were filled
    /// with, so a reassembly test can compare against a known payload.
    ///
    /// The payload is sized to fill both blocks exactly (2 × 492 bytes);
    /// a real driver leaves slack in its last block, which the format
    /// cannot record and the reassembly therefore returns.
    fn fs_image() -> (Vec<u8>, Vec<u8>) {
        let bs = 512;
        let mut d = one_partition_image(2);
        put32(&mut d, bs, 2, rdsk::FILESYS_HEADER_LIST, FSHD_BLOCK as u32);
        put32(&mut d, bs, 2, rdsk::BAD_BLOCK_LIST, BADB_BLOCK as u32);
        seal(&mut d, bs, 2, 64);

        put32(&mut d, bs, FSHD_BLOCK, 0, id::FSHD);
        put32(&mut d, bs, FSHD_BLOCK, chain::NEXT, CHAIN_END);
        put32(&mut d, bs, FSHD_BLOCK, fshd::HOST_ID, 7);
        put32(&mut d, bs, FSHD_BLOCK, fshd::FLAGS, 0);
        put32(&mut d, bs, FSHD_BLOCK, fshd::DOS_TYPE, 0x444F_5307);
        put32(&mut d, bs, FSHD_BLOCK, fshd::VERSION, (43 << 16) | 4);
        // Type, Handler, StackSize, Priority, SegList, GlobalVec patched;
        // Task, Lock and Startup deliberately not.
        put32(
            &mut d,
            bs,
            FSHD_BLOCK,
            fshd::PATCH_FLAGS,
            fshd_patch::TYPE
                | fshd_patch::HANDLER
                | fshd_patch::STACK_SIZE
                | fshd_patch::PRIORITY
                | fshd_patch::SEG_LIST
                | fshd_patch::GLOBAL_VEC,
        );
        // Every patched longword is written, gated or not: planting a
        // value behind a clear bit is what proves the gate works.
        for (i, v) in [
            1u32,        // Type
            0xDEAD_0001, // Task      (bit clear)
            0xDEAD_0002, // Lock      (bit clear)
            0x0000_0100, // Handler
            8192,        // StackSize
            10,          // Priority
            0xDEAD_0003, // Startup   (bit clear)
            LSEG_BLOCK as u32,
            0xFFFF_FFFF, // GlobalVec (-1: not BCPL)
        ]
        .into_iter()
        .enumerate()
        {
            put32(&mut d, bs, FSHD_BLOCK, fshd::PATCHED + i * 4, v);
        }
        seal(&mut d, bs, FSHD_BLOCK, 64);

        let payload: Vec<u8> = (0..2 * LSEG_PAYLOAD).map(|i| (i % 251) as u8).collect();
        for (i, block) in [LSEG_BLOCK, LSEG_BLOCK + 1].into_iter().enumerate() {
            put32(&mut d, bs, block, 0, id::LSEG);
            put32(
                &mut d,
                bs,
                block,
                chain::NEXT,
                if i == 0 {
                    (LSEG_BLOCK + 1) as u32
                } else {
                    CHAIN_END
                },
            );
            let base = block * bs + lseg::LOAD_DATA;
            d[base..base + LSEG_PAYLOAD]
                .copy_from_slice(&payload[i * LSEG_PAYLOAD..(i + 1) * LSEG_PAYLOAD]);
            // LSEG sums the whole block: SummedLongs == block_size / 4.
            seal(&mut d, bs, block, (bs / 4) as u32);
        }

        // Two BADB blocks, so the flattening across the chain is tested
        // rather than just the entries within one block.
        for (block, next, entries) in [
            (
                BADB_BLOCK,
                (BADB_BLOCK + 1) as u32,
                &[(100u32, 900u32), (101, 901)][..],
            ),
            (BADB_BLOCK + 1, CHAIN_END, &[(102, 902)][..]),
        ] {
            put32(&mut d, bs, block, 0, id::BADB);
            put32(&mut d, bs, block, chain::NEXT, next);
            for (i, (bad, good)) in entries.iter().enumerate() {
                put32(&mut d, bs, block, badb::ENTRIES + i * 8, *bad);
                put32(&mut d, bs, block, badb::ENTRIES + i * 8 + 4, *good);
            }
            seal(
                &mut d,
                bs,
                block,
                (badb::HEADER_LONGS + entries.len() * 2) as u32,
            );
        }

        (d, payload)
    }

    #[test]
    fn parses_a_minimal_image() {
        let mut disk = MemDisk::new(one_partition_image(2));
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.rdsk_block, 2);
        assert_eq!(rdb.block_bytes, 512);
        assert_eq!(rdb.partitions.len(), 1);
        let p = &rdb.partitions[0];
        assert_eq!(p.name, "DH0");
        assert!(p.bootable);
        assert_eq!(p.dos_type, 0x444F_5303);
        assert_eq!(p.cylinder_blocks, 32);
        assert_eq!(p.start_lba, 64); // LowCyl 2 * 32
        assert_eq!(p.block_len, 256); // cyls 2..=9
    }

    /// A 4 KB-block disk parses identically: same LBAs, same extents —
    /// everything is in device blocks, whatever their size.
    #[test]
    fn parses_a_4k_block_image() {
        let mut disk = MemDisk {
            data: one_partition_image_bs(2, 4096),
            block_size: 4096,
        };
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.rdsk_block, 2);
        assert_eq!(rdb.block_bytes, 4096);
        let p = &rdb.partitions[0];
        assert_eq!(p.start_lba, 64);
        assert_eq!(p.block_len, 256);
        assert_eq!(p.size_block_longs, 1024); // 4 KB filesystem blocks
    }

    /// A 4 KB-sector image read through a 512-byte source must fail
    /// loudly, not misaddress every chained block.
    #[test]
    fn block_bytes_mismatch_is_an_error() {
        // RDSK at 0 so the 512-byte scan still lands on it.
        let mut disk = MemDisk::new(one_partition_image_bs(0, 4096));
        assert_eq!(
            Rdb::parse(&mut disk).unwrap_err(),
            RdbError::BlockBytesMismatch {
                block_bytes: 4096,
                block_size: 512
            }
        );
    }

    #[test]
    fn unsupported_source_block_size_is_an_error() {
        for bad in [0usize, 256, 768, 65536] {
            let mut disk = MemDisk {
                data: vec![0u8; 4 * 65536],
                block_size: bad,
            };
            assert_eq!(
                Rdb::parse(&mut disk).unwrap_err(),
                RdbError::UnsupportedBlockSize { block_size: bad }
            );
        }
    }

    #[test]
    fn no_rdsk_is_reported_not_invented() {
        let mut disk = MemDisk::new(vec![0u8; 32 * 512]);
        assert_eq!(Rdb::parse(&mut disk).unwrap_err(), RdbError::NoRdsk);
    }

    /// An `RDSK` ID with a bad checksum must be skipped, not trusted —
    /// this is the stale-copy-shadows-live-RDB case.
    #[test]
    fn bad_checksum_rdsk_is_skipped_in_the_scan() {
        let mut img = one_partition_image(3);
        // Plant a checksummed-wrong RDSK *earlier* than the real one.
        put32(&mut img, 512, 1, 0, id::RDSK);
        put32(&mut img, 512, 1, 4, 64);
        put32(&mut img, 512, 1, 8, 0xDEAD_BEEF);
        let mut disk = MemDisk::new(img);
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.rdsk_block, 3);
    }

    #[test]
    fn part_chain_cycle_is_an_error_not_a_hang() {
        let mut img = one_partition_image(0);
        // PART at 1 points to itself.
        put32(&mut img, 512, 1, chain::NEXT, 1);
        seal(&mut img, 512, 1, 64);
        let mut disk = MemDisk::new(img);
        assert_eq!(
            Rdb::parse(&mut disk).unwrap_err(),
            RdbError::ChainCycle { lba: 1 }
        );
    }

    #[test]
    fn part_chain_past_disk_end_is_an_error() {
        let mut img = one_partition_image(0);
        put32(&mut img, 512, 0, rdsk::PARTITION_LIST, 1000);
        seal(&mut img, 512, 0, 64);
        let mut disk = MemDisk::new(img);
        assert_eq!(
            Rdb::parse(&mut disk).unwrap_err(),
            RdbError::ChainOutOfRange { lba: 1000 }
        );
    }

    #[test]
    fn partition_source_offsets_and_bounds() {
        let mut disk = MemDisk::new(one_partition_image(2));
        // Stamp a marker at the partition's first block (LBA 64).
        disk.data[64 * 512] = 0xAB;
        let rdb = Rdb::parse(&mut disk).unwrap();
        let p = rdb.partitions[0].clone();
        let mut ps = PartitionSource::new(&mut disk, &p);
        assert_eq!(ps.block_count(), Some(256));
        assert_eq!(ps.block_size(), 512);
        let mut buf = [0u8; 512];
        ps.read_block(0, &mut buf).unwrap();
        assert_eq!(buf[0], 0xAB);
        assert!(matches!(
            ps.read_block(256, &mut buf),
            Err(PartitionSourceError::OutOfRange { lba: 256, len: 256 })
        ));
    }

    /// The full envec: TableSize 20 means longwords 1..=20 follow, so
    /// Baud/Control/BootBlocks are all present and `envec_raw` holds 21
    /// entries (TableSize itself included).
    #[test]
    fn table_size_20_envec_exposes_the_optional_tail() {
        let mut disk = MemDisk::new(one_partition_image_envec(2, 512, 20));
        let rdb = Rdb::parse(&mut disk).unwrap();
        let p = &rdb.partitions[0];
        assert_eq!(p.baud, Some(9600));
        assert_eq!(p.control, Some(0x1234));
        assert_eq!(p.boot_blocks, Some(2));
        assert_eq!(p.envec_raw.len(), 21);
        assert_eq!(p.envec_raw[de::TABLE_SIZE], 20);
        assert_eq!(p.envec_raw[de::DOS_TYPE], 0x444F_5303);
        assert_eq!(p.envec_raw[de::BOOT_BLOCKS], 2);
    }

    /// TableSize 16 stops at DosType. The tail longwords are physically
    /// present in the block (the builder writes them) and must still
    /// read as `None` — absent is not the same as zero, and a
    /// round-tripping writer must not resurrect them.
    #[test]
    fn table_size_16_envec_has_no_optional_tail() {
        let mut disk = MemDisk::new(one_partition_image_envec(2, 512, 16));
        let rdb = Rdb::parse(&mut disk).unwrap();
        let p = &rdb.partitions[0];
        assert_eq!(p.baud, None);
        assert_eq!(p.control, None);
        assert_eq!(p.boot_blocks, None);
        assert_eq!(p.envec_raw.len(), 17);
    }

    /// A hostile `de_TableSize` must clamp to what the block holds, not
    /// index past it. 512-byte block, envec at 128 → 96 longwords.
    #[test]
    fn hostile_table_size_clamps_envec_raw() {
        for bogus in [96u32, 1000, 0x7FFF_FFFF, u32::MAX] {
            let mut disk = MemDisk::new(one_partition_image_envec(2, 512, bogus));
            let rdb = Rdb::parse(&mut disk).unwrap();
            let p = &rdb.partitions[0];
            assert_eq!(p.envec_raw.len(), (512 - part::ENVIRONMENT) / 4);
            // The tail fields still fit the block, so they read fine —
            // clamping bounds the read, it does not suppress it.
            assert_eq!(p.boot_blocks, Some(2));
        }
    }

    /// The same clamp on a 4 KB block, where the *declared* size is the
    /// binding one: 20 longwords is far less than the 992 that fit.
    #[test]
    fn envec_raw_follows_table_size_when_it_fits() {
        let mut disk = MemDisk {
            data: one_partition_image_envec(2, 4096, 20),
            block_size: 4096,
        };
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.partitions[0].envec_raw.len(), 21);
    }

    #[test]
    fn identification_strings_parse_with_padding_trimmed() {
        let mut disk = MemDisk::new(one_partition_image(2));
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.disk_vendor, "QUANTUM");
        assert_eq!(rdb.disk_product, "FIREBALL_TM3200S"); // fills all 16
        assert_eq!(rdb.disk_revision, "300");
        assert_eq!(rdb.controller_vendor, "CBM");
        assert_eq!(rdb.controller_product, "A4091");
        assert_eq!(rdb.controller_revision, "40.9");
        assert!(rdb.flags & rdb_flags::DISK_ID != 0);
        assert!(rdb.flags & rdb_flags::CTRLR_ID != 0);
        assert!(rdb.flags & rdb_flags::LAST == 0);
    }

    #[test]
    fn remaining_rdsk_fields_are_surfaced() {
        let mut disk = MemDisk::new(one_partition_image(2));
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.host_id, 7);
        assert_eq!(rdb.drive_init, CHAIN_END);
        assert_eq!(rdb.interleave, 1);
        assert_eq!(rdb.park, 10);
        assert_eq!(rdb.write_pre_comp, 10);
        assert_eq!(rdb.reduced_write, 10);
        assert_eq!(rdb.step_rate, 3);
        assert_eq!(rdb.lo_cylinder, 2);
        assert_eq!(rdb.hi_cylinder, 9);
        assert_eq!(rdb.cyl_blocks, 32);
        assert_eq!(rdb.auto_park_seconds, 0);
        assert_eq!(rdb.high_rdsk_block, 3);
    }

    /// The FSHD is read, and `PatchFlags` decides which tail fields
    /// exist. The three ungated ones hold planted values in the block
    /// and must still read as `None`.
    #[test]
    fn fshd_chain_parses_with_patch_flags_gating() {
        let (img, _) = fs_image();
        let mut disk = MemDisk::new(img);
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.filesys_header_list, FSHD_BLOCK as u32);
        assert_eq!(rdb.filesystems.len(), 1);
        let f = &rdb.filesystems[0];
        assert_eq!(f.fshd_block, FSHD_BLOCK as u64);
        assert_eq!(f.host_id, 7);
        assert_eq!(f.dos_type, 0x444F_5307);
        assert_eq!(f.version_major(), 43);
        assert_eq!(f.version_minor(), 4);
        assert_eq!(f.node_type, Some(1));
        assert_eq!(f.handler, Some(0x100));
        assert_eq!(f.stack_size, Some(8192));
        assert_eq!(f.priority, Some(10));
        assert_eq!(f.global_vec, Some(-1));
        assert_eq!(f.task, None);
        assert_eq!(f.lock, None);
        assert_eq!(f.startup, None);
        // Always read, whatever the bit says — the chain must be walkable.
        assert_eq!(f.seg_list_blocks, LSEG_BLOCK as u32);
    }

    #[test]
    fn lseg_chain_reassembles_the_driver_binary() {
        let (img, payload) = fs_image();
        let mut disk = MemDisk::new(img);
        let rdb = Rdb::parse(&mut disk).unwrap();
        let bin = rdb.load_filesystem(&rdb.filesystems[0], &mut disk).unwrap();
        assert_eq!(bin.len(), 2 * LSEG_PAYLOAD);
        assert_eq!(bin, payload);
    }

    /// No `LSEG` chain is an empty binary, not an error: an FSHD that
    /// only patches device-node fields is legitimate.
    #[test]
    fn fshd_without_a_seg_list_loads_nothing() {
        let (mut img, _) = fs_image();
        put32(
            &mut img,
            512,
            FSHD_BLOCK,
            fshd::PATCHED + fshd::SEG_LIST_INDEX * 4,
            CHAIN_END,
        );
        seal(&mut img, 512, FSHD_BLOCK, 64);
        let mut disk = MemDisk::new(img);
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert!(rdb
            .load_filesystem(&rdb.filesystems[0], &mut disk)
            .unwrap()
            .is_empty());
    }

    #[test]
    fn badb_entries_are_flattened_across_the_chain() {
        let (img, _) = fs_image();
        let mut disk = MemDisk::new(img);
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.bad_block_list, BADB_BLOCK as u32);
        assert_eq!(
            rdb.bad_blocks,
            vec![
                BadBlockEntry {
                    bad: 100,
                    good: 900
                },
                BadBlockEntry {
                    bad: 101,
                    good: 901
                },
                BadBlockEntry {
                    bad: 102,
                    good: 902
                },
            ]
        );
    }

    /// A disk with no BADB chain has no entries, and the head field
    /// still round-trips.
    #[test]
    fn no_badb_chain_is_an_empty_list() {
        let mut disk = MemDisk::new(one_partition_image(2));
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.bad_block_list, CHAIN_END);
        assert!(rdb.bad_blocks.is_empty());
        assert!(rdb.filesystems.is_empty());
    }

    #[test]
    fn lseg_chain_cycle_is_an_error_not_a_hang() {
        let (mut img, _) = fs_image();
        // The second LSEG points back at the first.
        put32(
            &mut img,
            512,
            LSEG_BLOCK + 1,
            chain::NEXT,
            LSEG_BLOCK as u32,
        );
        seal(&mut img, 512, LSEG_BLOCK + 1, 128);
        let mut disk = MemDisk::new(img);
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(
            rdb.load_filesystem(&rdb.filesystems[0], &mut disk)
                .unwrap_err(),
            RdbError::ChainCycle {
                lba: LSEG_BLOCK as u64
            }
        );
    }

    #[test]
    fn fshd_chain_with_wrong_id_is_an_error() {
        let (mut img, _) = fs_image();
        put32(&mut img, 512, FSHD_BLOCK, 0, id::PART);
        seal(&mut img, 512, FSHD_BLOCK, 64);
        let mut disk = MemDisk::new(img);
        assert_eq!(
            Rdb::parse(&mut disk).unwrap_err(),
            RdbError::WrongId {
                lba: FSHD_BLOCK as u64,
                expected: id::FSHD,
                found: id::PART,
            }
        );
    }

    /// A `BADB` whose `SummedLongs` claims more entries than the block
    /// holds must clamp. Such a block also fails `checksum_ok`, so the
    /// clamp is reached here by calling the parser directly — the point
    /// is that the arithmetic is safe on its own.
    #[test]
    fn hostile_badb_summed_longs_clamps_entry_count() {
        for bogus in [128u32, 1000, u32::MAX] {
            let mut b = vec![0u8; 512];
            b[4..8].copy_from_slice(&bogus.to_be_bytes());
            let mut out = Vec::new();
            parse_badb(&b, &mut out);
            assert_eq!(out.len(), (512 - badb::ENTRIES) / 8);
        }
    }

    /// `de_SizeBlock` is a *per partition* property: one 512-byte-device
    /// -block disk carries a 4 KB-filesystem-block partition beside a
    /// 32 KB one, and everything this crate reports — extents,
    /// [`PartitionSource`] addressing — stays in the device's 512-byte
    /// blocks regardless. Nothing may assume one filesystem block size
    /// per disk.
    #[test]
    fn mixed_size_block_partitions_on_one_disk() {
        let mut disk = MemDisk::new(two_partition_image(
            ("DH0", 1024, (2, 4)), // 4 KB filesystem blocks
            ("DH1", 8192, (5, 9)), // 32 KB filesystem blocks
        ));
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.block_bytes, 512);
        assert_eq!(rdb.partitions.len(), 2);

        let (a, b) = (rdb.partitions[0].clone(), rdb.partitions[1].clone());
        assert_eq!(a.name, "DH0");
        assert_eq!(a.size_block_longs, 1024);
        assert_eq!(b.name, "DH1");
        assert_eq!(b.size_block_longs, 8192);
        // 64× apart in filesystem block size, identical arithmetic:
        // extents are device blocks, cylinders × 32.
        assert_eq!((a.start_lba, a.block_len), (64, 96)); // cyls 2..=4
        assert_eq!((b.start_lba, b.block_len), (160, 160)); // cyls 5..=9

        // ...and the adapter a filesystem crate mounts still speaks the
        // device's block size for both, not de_SizeBlock's.
        for (p, blocks) in [(&a, 96u64), (&b, 160)] {
            let ps = PartitionSource::new(&mut disk, p);
            assert_eq!(ps.block_size(), 512);
            assert_eq!(ps.block_count(), Some(blocks));
        }

        // A legal layout, mixed block sizes and all.
        assert!(rdb.validate().is_empty());
    }

    /// The fixtures are clean layouts: every chained block inside the
    /// reserved area, every partition clear of it, no two partitions
    /// claiming a block. Both halves of the check agree.
    #[test]
    fn validate_accepts_a_clean_image() {
        let (img, _) = fs_image();
        let mut disk = MemDisk::new(img);
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.rdb_blocks_lo, 0);
        assert_eq!(rdb.rdb_blocks_hi, 15);
        assert_eq!(
            rdb.badb_blocks,
            vec![BADB_BLOCK as u64, BADB_BLOCK as u64 + 1]
        );
        assert!(rdb.validate().is_empty());
        assert!(rdb.validate_seg_lists(&mut disk).unwrap().is_empty());
    }

    /// A `PART` block past `rdb_RDBBlocksHi` sits in space the RDB says
    /// is not reserved — readable, and a repartitioner is entitled to
    /// hand that block to a filesystem.
    #[test]
    fn validate_reports_a_part_block_outside_the_rdb_area() {
        let mut img = one_partition_image(2);
        put32(&mut img, 512, 2, rdsk::RDB_BLOCKS_HI, 2); // PART is at 3
        seal(&mut img, 512, 2, 64);
        let mut disk = MemDisk::new(img);
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(
            rdb.validate(),
            vec![ValidationIssue::BlockOutsideRdbArea {
                kind: BlockKind::Part,
                lba: 3,
                lo: 0,
                hi: 2,
            }]
        );
    }

    /// The real-world case: a tool wrote RDB structures past the
    /// reserved area, so the area now runs into the first partition and
    /// each side will trash the other. The image still parses — a
    /// recovery tool needs the data — and `validate` is how a consumer
    /// finds out before writing.
    #[test]
    fn validate_reports_a_partition_overlapping_the_rdb_area() {
        let mut img = one_partition_image(2);
        put32(&mut img, 512, 2, rdsk::RDB_BLOCKS_HI, 100); // DH0 starts at 64
        seal(&mut img, 512, 2, 64);
        let mut disk = MemDisk::new(img);
        let rdb = Rdb::parse(&mut disk).unwrap();
        // Still fully readable: that is the whole point.
        assert_eq!(rdb.partitions[0].start_lba, 64);
        assert_eq!(
            rdb.validate(),
            vec![ValidationIssue::PartitionOverlapsRdbArea {
                index: 0,
                name: String::from("DH0"),
                start_lba: 64,
                block_len: 256,
                lo: 0,
                hi: 100,
            }]
        );
    }

    /// Beyond the plan item, same failure family: two partitions
    /// claiming the same blocks, each filesystem destroying the other.
    #[test]
    fn validate_reports_overlapping_partitions() {
        let mut disk = MemDisk::new(two_partition_image(
            ("DH0", 1024, (2, 4)), // 64..160
            ("DH1", 8192, (4, 9)), // 128..320
        ));
        let rdb = Rdb::parse(&mut disk).unwrap();
        let issues = rdb.validate();
        assert_eq!(
            issues,
            vec![ValidationIssue::PartitionsOverlap {
                a: 0,
                b: 1,
                a_name: String::from("DH0"),
                b_name: String::from("DH1"),
                start: 128,
                len: 32,
            }]
        );
        assert_eq!(
            alloc::format!("{}", issues[0]),
            "partitions 0 (DH0) and 1 (DH1) both claim blocks 128..160"
        );
    }

    /// An inverted `de_LowCyl`/`de_HighCyl` pair. Found by the fuzzer:
    /// the span used to be computed as an unchecked `high - low + 1`,
    /// which panicked in a debug build and — far worse — wrapped in a
    /// release one, turning a backwards range into a partition claiming
    /// most of the address space. The parse still succeeds (a recovery
    /// tool needs to see the damage), the extent is empty, and
    /// validation names it.
    #[test]
    fn inverted_cylinder_range_is_an_empty_extent_not_an_overflow() {
        let mut d = one_partition_image(2);
        put32(&mut d, 512, 3, part::ENVIRONMENT + de::LOW_CYL * 4, 9);
        put32(&mut d, 512, 3, part::ENVIRONMENT + de::HIGH_CYL * 4, 2);
        seal(&mut d, 512, 3, 64);

        let mut disk = MemDisk::new(d);
        let rdb = Rdb::parse(&mut disk).unwrap();
        let p = &rdb.partitions[0];
        assert_eq!((p.low_cyl, p.high_cyl), (9, 2));
        assert_eq!(p.block_len, 0);
        // Zero-length, so it cannot collide with anything: the only
        // issue is the inversion itself.
        let issues = rdb.validate();
        assert_eq!(
            issues,
            vec![ValidationIssue::PartitionCylindersInverted {
                index: 0,
                name: String::from("DH0"),
                low_cyl: 9,
                high_cyl: 2,
            }]
        );
        assert_eq!(
            alloc::format!("{}", issues[0]),
            "partition 0 (DH0) has an inverted cylinder range: LowCyl 9 is above HighCyl 2"
        );
    }

    /// The other half of the same arithmetic: a plausible-looking
    /// geometry whose cylinder product times `de_LowCyl` does not fit a
    /// `u64`. Saturating is the only answer that is not a lie; what
    /// matters is that it does not panic or wrap.
    #[test]
    fn absurd_geometry_saturates_rather_than_overflowing() {
        let mut d = one_partition_image(2);
        let env = |i: usize| part::ENVIRONMENT + i * 4;
        put32(&mut d, 512, 3, env(de::SURFACES), u32::MAX);
        put32(&mut d, 512, 3, env(de::BLOCKS_PER_TRACK), u32::MAX);
        put32(&mut d, 512, 3, env(de::LOW_CYL), u32::MAX - 1);
        put32(&mut d, 512, 3, env(de::HIGH_CYL), u32::MAX);
        seal(&mut d, 512, 3, 64);

        let mut disk = MemDisk::new(d);
        let rdb = Rdb::parse(&mut disk).unwrap();
        let p = &rdb.partitions[0];
        assert_eq!(p.start_lba, u64::MAX);
        assert_eq!(p.block_len, u64::MAX);

        // And the adapter over that extent refuses rather than
        // overflowing the parent LBA: `block_len` says block 1 is in
        // range, but `start_lba + 1` does not exist.
        let mut parent = MemDisk::new(vec![0u8; 512]);
        let mut view = PartitionSource::new(&mut parent, p);
        let mut block = vec![0u8; 512];
        assert_eq!(
            view.read_block(1, &mut block),
            Err(PartitionSourceError::OutOfRange {
                lba: 1,
                len: u64::MAX
            })
        );
    }

    /// `LSEG` blocks are lazy, so they are checked with the disk in
    /// hand. Here the area stops at the FSHD, leaving the driver's own
    /// blocks — and the BADB chain behind them — outside it.
    #[test]
    fn validate_seg_lists_reports_lseg_outside_the_rdb_area() {
        let (mut img, _) = fs_image();
        put32(&mut img, 512, 2, rdsk::RDB_BLOCKS_HI, FSHD_BLOCK as u32);
        seal(&mut img, 512, 2, 64);
        let mut disk = MemDisk::new(img);
        let rdb = Rdb::parse(&mut disk).unwrap();

        let outside = |kind, lba| ValidationIssue::BlockOutsideRdbArea {
            kind,
            lba,
            lo: 0,
            hi: FSHD_BLOCK as u64,
        };
        // The eager chains: PART and FSHD are inside, both BADBs are not.
        assert_eq!(
            rdb.validate(),
            vec![
                outside(BlockKind::Badb, BADB_BLOCK as u64),
                outside(BlockKind::Badb, BADB_BLOCK as u64 + 1),
            ]
        );
        assert_eq!(
            rdb.validate_seg_lists(&mut disk).unwrap(),
            vec![
                outside(BlockKind::Lseg, LSEG_BLOCK as u64),
                outside(BlockKind::Lseg, LSEG_BLOCK as u64 + 1),
            ]
        );
    }

    /// An inverted reserved area says nothing about what is inside it,
    /// so it is reported once instead of once per block — but the
    /// partition-versus-partition check does not consult the area and
    /// still runs.
    #[test]
    fn validate_reports_an_inverted_rdb_area_once() {
        let mut img = two_partition_image(("DH0", 1024, (2, 4)), ("DH1", 8192, (4, 9)));
        put32(&mut img, 512, 2, rdsk::RDB_BLOCKS_LO, 16);
        put32(&mut img, 512, 2, rdsk::RDB_BLOCKS_HI, 0);
        seal(&mut img, 512, 2, 64);
        let mut disk = MemDisk::new(img);
        let rdb = Rdb::parse(&mut disk).unwrap();
        let issues = rdb.validate();
        assert_eq!(issues.len(), 2);
        assert_eq!(issues[0], ValidationIssue::RdbAreaInvalid { lo: 16, hi: 0 });
        assert!(matches!(
            issues[1],
            ValidationIssue::PartitionsOverlap { a: 0, b: 1, .. }
        ));
        assert!(rdb.validate_seg_lists(&mut disk).unwrap().is_empty());
    }

    /// The error types render one line a user can act on: which block,
    /// and both sides of every disagreement. A wrong ID prints the four
    /// characters the format actually writes.
    #[test]
    fn errors_display_as_one_useful_line() {
        let line = |e: RdbError<&str>| alloc::format!("{e}");
        assert_eq!(
            line(RdbError::WrongId {
                lba: 4,
                expected: id::FSHD,
                found: id::PART,
            }),
            "block 4 has ID PART where FSHD was expected"
        );
        // A corrupt ID is not four printable characters; hex, not mojibake.
        assert_eq!(
            line(RdbError::WrongId {
                lba: 4,
                expected: id::PART,
                found: 0,
            }),
            "block 4 has ID 0x00000000 where PART was expected"
        );
        assert_eq!(
            line(RdbError::BadChecksum { lba: 7 }),
            "block 7 has a bad checksum"
        );
        assert_eq!(
            line(RdbError::BlockBytesMismatch {
                block_bytes: 4096,
                block_size: 512,
            }),
            "the RDB declares 4096-byte blocks but the source reads 512-byte blocks"
        );
        assert_eq!(
            line(RdbError::Io("disk on fire")),
            "reading a block failed: disk on fire"
        );
        assert_eq!(
            alloc::format!(
                "{}",
                PartitionSourceError::<&str>::OutOfRange { lba: 256, len: 256 }
            ),
            "block 256 is past the end of the partition, which has 256 blocks"
        );
    }

    #[test]
    fn checksum_rejects_hostile_summed_longs() {
        let mut b = [0u8; 512];
        b[4..8].copy_from_slice(&0xFFFF_FFFFu32.to_be_bytes());
        assert!(!checksum_ok(&b));
        b[4..8].copy_from_slice(&0u32.to_be_bytes());
        assert!(!checksum_ok(&b));
    }

    /// The round-trip property in miniature, swept over every supported
    /// block size, a spread of `SummedLongs` within each, and contents
    /// chosen to make the wrapping arithmetic actually wrap: whatever
    /// [`seal_checksum`] accepts, [`checksum_ok`] must then accept.
    ///
    /// Not a `proptest` — no new dependencies — but the same shape: the
    /// generator is a cheap LCG over the block bytes, so each case is a
    /// different pattern rather than the zeros a hand-written fixture
    /// would have. The all-`0xFF` and all-zero patterns are included
    /// explicitly because they are the two the sum degenerates on.
    #[test]
    fn seal_then_checksum_ok_round_trips() {
        for bs in [512usize, 1024, 2048, 4096, 8192, 16384, 32768] {
            let capacity = (bs / 4) as u32;
            for pattern in 0u32..6 {
                let mut block = vec![0u8; bs];
                for (i, byte) in block.iter_mut().enumerate() {
                    *byte = match pattern {
                        0 => 0,
                        1 => 0xFF,
                        // A cheap LCG, so each pattern is a different
                        // spread of longwords rather than a ramp that
                        // sums to something tidy.
                        p => {
                            (((i as u32)
                                .wrapping_mul(1_103_515_245)
                                .wrapping_add(p * 12_345))
                                >> 16) as u8
                        }
                    };
                }
                for longs in [
                    MIN_SUMMED_LONGS,
                    MIN_SUMMED_LONGS + 1,
                    64,
                    capacity / 2,
                    capacity - 1,
                    capacity,
                ] {
                    seal_checksum(&mut block, longs).unwrap();
                    assert!(
                        checksum_ok(&block),
                        "bs {bs} pattern {pattern} longs {longs} did not check out"
                    );
                    // And the header says what was asked for, which is
                    // the half a clamp would have quietly changed.
                    assert_eq!(be32(&block, 4), longs);
                }
            }
        }
    }

    /// Sealing refuses counts no block can satisfy rather than clamping
    /// them, and refuses without touching the block.
    #[test]
    fn seal_refuses_impossible_summed_longs() {
        let mut block = vec![0xAAu8; 512];
        let untouched = block.clone();

        for longs in 0..MIN_SUMMED_LONGS {
            assert_eq!(
                seal_checksum(&mut block, longs),
                Err(SealError::SummedLongsTooShort {
                    summed_longs: longs
                })
            );
        }
        for longs in [129u32, 1024, u32::MAX] {
            assert_eq!(
                seal_checksum(&mut block, longs),
                Err(SealError::SummedLongsTooLong {
                    summed_longs: longs,
                    capacity: 128,
                })
            );
        }
        assert_eq!(block, untouched);

        // A block too short to hold the header is the same refusal, not
        // a panic — the property that matters is that no length panics.
        let mut tiny = [0u8; 8];
        assert_eq!(
            seal_checksum(&mut tiny, MIN_SUMMED_LONGS),
            Err(SealError::SummedLongsTooLong {
                summed_longs: MIN_SUMMED_LONGS,
                capacity: 2,
            })
        );
    }

    /// A `SummedLongs` below [`MIN_SUMMED_LONGS`] is unsatisfiable, not
    /// merely refused by convention: even a hand-built block gets no
    /// value at `ChkSum` that makes such a sum come out zero, which is
    /// why sealing declines to try.
    #[test]
    fn short_summed_longs_can_never_check_out() {
        for longs in 1u32..MIN_SUMMED_LONGS {
            let mut block = [0u8; 512];
            put_be32(&mut block, 4, longs);
            for chk in [0u32, 1, 0xFFFF_FFFF, 0x8000_0000] {
                put_be32(&mut block, 8, chk);
                // The first `longs` longwords are ID and SummedLongs,
                // both non-zero here, so the sum cannot be zero however
                // ChkSum is set.
                put_be32(&mut block, 0, id::RDSK);
                assert!(!checksum_ok(&block));
            }
        }
    }

    /// The write seam, end to end through the crate's own reader: seal a
    /// block, hand it to a [`BlockSink`], and parse the result back.
    /// A `MemDisk` is both traits, which is exactly the
    /// `BlockSource + BlockSink` bound the write path will take.
    #[test]
    fn block_sink_writes_blocks_a_parse_reads_back() {
        fn stamp<D: BlockSource + BlockSink>(disk: &mut D, src: &[u8], block: usize)
        where
            <D as BlockSink>::Error: core::fmt::Debug,
        {
            let bs = BlockSink::block_size(disk);
            disk.write_block(block as u64, &src[block * bs..(block + 1) * bs])
                .unwrap();
        }

        // A blank disk, filled a block at a time from a known-good image
        // through the sink rather than by slicing the buffer.
        let image = one_partition_image(2);
        let mut disk = MemDisk::new(vec![0u8; image.len()]);
        for block in 0..image.len() / 512 {
            stamp(&mut disk, &image, block);
        }
        assert_eq!(disk.data, image);

        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.rdsk_block, 2);
        assert_eq!(rdb.partitions[0].name, "DH0");
    }

    /// A sink refuses a block past its end, so a layout that does not
    /// fit fails at the write rather than growing the disk under it.
    #[test]
    fn block_sink_refuses_a_write_past_the_end() {
        let mut disk = MemDisk::new(vec![0u8; 4 * 512]);
        assert_eq!(disk.write_block(3, &[0u8; 512]), Ok(()));
        assert_eq!(disk.write_block(4, &[0u8; 512]), Err(()));
        assert_eq!(BlockSink::block_count(&disk), Some(4));
    }

    /// `SeekBlockSource` gains [`BlockSink`] from a `Write` inner type
    /// and keeps its block size across both directions.
    #[cfg(feature = "std")]
    #[test]
    fn seek_block_source_writes_when_the_inner_type_can() {
        let mut disk = SeekBlockSource::new(std::io::Cursor::new(vec![0u8; 8 * 512])).unwrap();
        let mut block = vec![0u8; 512];
        put_be32(&mut block, 0, id::RDSK);
        seal_checksum(&mut block, 64).unwrap();
        disk.write_block(5, &block).unwrap();

        let mut back = vec![0u8; 512];
        disk.read_block(5, &mut back).unwrap();
        assert_eq!(back, block);
        assert!(checksum_ok(&back));
        assert_eq!(BlockSink::block_size(&disk), 512);
        assert_eq!(BlockSink::block_count(&disk), Some(8));
    }

    #[test]
    fn put_be32_is_be32s_inverse() {
        let mut block = [0u8; 512];
        for (i, v) in [0u32, 1, 0x4441_5441, u32::MAX, 0x8000_0000]
            .into_iter()
            .enumerate()
        {
            put_be32(&mut block, i * 4, v);
            assert_eq!(be32(&block, i * 4), v);
        }
        // And the byte order really is the format's, not the host's.
        put_be32(&mut block, 0, 0x5244_534B);
        assert_eq!(&block[..4], b"RDSK");
    }

    /// Every case below was read out of an image `rdbtool` actually
    /// created — `rdbtool <img> create size=<n> [bs=<n>] + init`, then
    /// the geometry read back — so these are observations, not a
    /// restatement of the algorithm.
    ///
    /// **Pinned against amitools 0.8.1.** If a future amitools changes
    /// its convention this test is where it will be noticed, and the
    /// decision (follow, or diverge deliberately) belongs there rather
    /// than in silently drifting images.
    const RDBTOOL_0_8_1: &[(u64, usize, u32, u32, u32)] = &[
        // (requested bytes, block size, cylinders, heads, sectors)
        //
        // The Amiga-ish candidate wins almost everywhere: its cylinder
        // is 16 KiB, so it wastes less than the 63-sector one can.
        (512 * 1024, 512, 32, 1, 32),
        (1024 * 1024, 512, 64, 1, 32),
        (10 * 1024 * 1024, 512, 640, 1, 32),
        (100 * 1024 * 1024, 512, 6400, 1, 32),
        (512 * 1024 * 1024, 512, 32768, 1, 32),
        (700 * 1024 * 1024, 512, 44800, 1, 32),
        // Past 65535 cylinders the halving starts, so the head count
        // doubles with every doubling of the disk and the cylinder
        // count sticks at 32768.
        (1024 * 1024 * 1024, 512, 32768, 2, 32),
        (2 * 1024 * 1024 * 1024, 512, 32768, 4, 32),
        (4 * 1024 * 1024 * 1024, 512, 32768, 8, 32),
        (8 * 1024 * 1024 * 1024, 512, 32768, 16, 32),
        (16 * 1024 * 1024 * 1024, 512, 32768, 32, 32),
        (64 * 1024 * 1024 * 1024, 512, 32768, 128, 32),
        // Either side of the ceiling: 65535 cylinders is allowed,
        // 65536 is not, and 65537 loses a cylinder to the halving.
        (1_073_725_440, 512, 65535, 1, 32),
        (1_073_741_824, 512, 32768, 2, 32),
        (1_073_758_208, 512, 32768, 2, 32),
        (2_147_467_264, 512, 65535, 2, 32),
        // Sizes that are an exact multiple of the PC-ish cylinder:
        // both candidates waste nothing and the tie goes to 63 sectors.
        // The head count walks the breakpoint table.
        (516_096, 512, 1, 16, 63),
        (1_032_192, 512, 2, 16, 63),
        (51_609_600, 512, 100, 16, 63),
        (527_966_208, 512, 1023, 16, 63),
        (528_482_304, 512, 1024, 16, 63), // exactly 504 MiB: still 16 heads
        (528_482_305, 512, 1024, 16, 63), // one byte over: still 16 heads
        (1_056_964_608, 512, 1024, 32, 63),
        (2_113_929_216, 512, 1024, 64, 63),
        (4_227_858_432, 512, 1024, 128, 63),
        (8_455_716_864, 512, 1024, 256, 63),
        // Sizes that divide evenly into no cylinder at all: the count
        // rounds down and the tail is unaddressable.
        (123_456_789, 512, 7535, 1, 32),
        (33_333_333, 512, 2034, 1, 32),
        // 4 KB blocks: heads and sectors are unchanged, only the
        // cylinder count scales — and with it where the ceiling bites.
        (10 * 1024 * 1024, 4096, 80, 1, 32),
        (100 * 1024 * 1024, 4096, 800, 1, 32),
        (1024 * 1024 * 1024, 4096, 8192, 1, 32),
        (8 * 1024 * 1024 * 1024, 4096, 32768, 2, 32),
        (123_456_789, 4096, 941, 1, 32),
        // The smallest disk that has a geometry at all: one 32-block
        // cylinder. 16383 bytes is the TooSmall case, tested separately.
        (16384, 512, 1, 1, 32),
    ];

    #[test]
    fn geometry_matches_rdbtool_0_8_1() {
        for &(bytes, bs, cylinders, heads, sectors) in RDBTOOL_0_8_1 {
            let g = synthesize_geometry(bytes, bs)
                .unwrap_or_else(|e| panic!("{bytes} bytes at bs {bs}: {e}"));
            assert_eq!(
                (g.cylinders, g.heads, g.sectors),
                (cylinders, heads, sectors),
                "{bytes} bytes at bs {bs}"
            );
            assert_eq!(g.block_size, bs);
            assert_eq!(g.cylinder_blocks(), heads as u64 * sectors as u64);
        }
    }

    /// The rounding direction, asserted as a property over every pinned
    /// case rather than only where the numbers happen to be untidy: a
    /// geometry describes at most the disk asked for, and falls short by
    /// less than one cylinder. Rounding the other way would put a
    /// partition's last cylinder past the end of the medium.
    #[test]
    fn geometry_never_describes_more_than_the_disk() {
        for &(bytes, bs, ..) in RDBTOOL_0_8_1 {
            let g = synthesize_geometry(bytes, bs).unwrap();
            let cylinder_bytes = g.cylinder_blocks() * bs as u64;
            assert!(g.total_bytes() <= bytes, "{bytes} at bs {bs} overshot");
            assert!(
                bytes - g.total_bytes() < cylinder_bytes,
                "{bytes} at bs {bs} wasted a whole cylinder"
            );
        }
    }

    /// A disk short of one 32-block cylinder has no geometry, and says
    /// so rather than returning a zero-cylinder one that would describe
    /// a disk of no size.
    #[test]
    fn geometry_refuses_a_disk_below_one_cylinder() {
        assert_eq!(
            synthesize_geometry(16383, 512),
            Err(GeometryError::TooSmall {
                total_bytes: 16383,
                block_size: 512,
                minimum_bytes: 16384,
            })
        );
        assert_eq!(
            synthesize_geometry(0, 512),
            Err(GeometryError::TooSmall {
                total_bytes: 0,
                block_size: 512,
                minimum_bytes: 16384,
            })
        );
        // The floor scales with the block size: 32 blocks, whatever
        // they are worth.
        assert!(synthesize_geometry(131_071, 4096).is_err());
        assert!(synthesize_geometry(131_072, 4096).is_ok());
    }

    /// A size no 32-bit geometry can describe is an error, not a wrap.
    /// Both candidates fail here: the PC-ish one needs 2.2e12 cylinders
    /// and the Amiga-ish one 2^35 heads.
    #[test]
    fn geometry_refuses_a_disk_too_large_for_the_u32_fields() {
        assert_eq!(
            synthesize_geometry(u64::MAX, 512),
            Err(GeometryError::TooLarge {
                total_bytes: u64::MAX,
                block_size: 512,
            })
        );
        // An eighth of that in 32 KB blocks *is* describable, so the
        // refusal above is a real limit and not a blanket cap on large
        // inputs. It is also the one-candidate-survives case: the
        // PC-ish geometry needs 4.4e9 cylinders and is discarded, while
        // the Amiga-ish one lands on 2^25 heads and stands.
        let g = synthesize_geometry(u64::MAX / 8, 32768).unwrap();
        assert_eq!((g.cylinders, g.heads, g.sectors), (65535, 1 << 25, 32));
        assert!(g.cylinder_blocks() <= u32::MAX as u64);
    }

    #[test]
    fn geometry_refuses_an_unsupported_block_size() {
        for bad in [0usize, 256, 768, 65536] {
            assert_eq!(
                synthesize_geometry(1024 * 1024 * 1024, bad),
                Err(GeometryError::UnsupportedBlockSize { block_size: bad })
            );
        }
    }

    #[test]
    fn geometry_error_displays_a_line() {
        assert_eq!(
            alloc::format!(
                "{}",
                GeometryError::TooSmall {
                    total_bytes: 16383,
                    block_size: 512,
                    minimum_bytes: 16384
                }
            ),
            "16383 bytes is too small for a geometry in 512-byte blocks: \
             at least 16384 bytes are needed for one cylinder"
        );
        assert_eq!(
            alloc::format!(
                "{}",
                GeometryError::TooLarge {
                    total_bytes: 1,
                    block_size: 512
                }
            ),
            "1 bytes in 512-byte blocks exceeds what the RDB's 32-bit geometry \
             fields can describe"
        );
    }

    #[test]
    fn seal_error_displays_a_line() {
        assert_eq!(
            alloc::format!("{}", SealError::SummedLongsTooShort { summed_longs: 1 }),
            "SummedLongs 1 is below the 3 needed to cover ChkSum itself"
        );
        assert_eq!(
            alloc::format!(
                "{}",
                SealError::SummedLongsTooLong {
                    summed_longs: 200,
                    capacity: 128
                }
            ),
            "SummedLongs 200 exceeds the 128 longwords the block holds"
        );
    }

    // ---- the write path: RdbBuilder -------------------------------

    /// A zeroed target — the state every refusal test asserts the sink
    /// is still in afterwards.
    fn blank_disk(blocks: usize, bs: usize) -> MemDisk {
        MemDisk {
            data: vec![0u8; blocks * bs],
            block_size: bs,
        }
    }

    /// Ten mebibytes at 512-byte blocks: 640 cylinders of 32 blocks, so
    /// one cylinder is 16 KiB and the arithmetic in the assertions below
    /// stays checkable by eye.
    const TEN_MIB: u64 = 10 * 1024 * 1024;
    const TEN_MIB_BLOCKS: usize = (TEN_MIB / 512) as usize;

    /// Build into a blank disk and hand back both. Every builder test
    /// then parses what it wrote: the disk is the only authority on what
    /// is on the disk, and a layout that agrees with itself proves
    /// nothing.
    fn build_on(builder: RdbBuilder, blocks: usize, bs: usize) -> (MemDisk, RdbLayout, Rdb) {
        let mut disk = blank_disk(blocks, bs);
        let layout = builder.build(&mut disk).expect("build");
        let rdb = Rdb::parse(&mut disk).expect("parse back");
        assert_eq!(rdb.validate(), Vec::new(), "builder produced a bad layout");
        (disk, layout, rdb)
    }

    /// A build that must be refused, and must have written nothing.
    fn assert_refused(builder: RdbBuilder, blocks: usize, bs: usize, expected: BuildError<()>) {
        let mut disk = blank_disk(blocks, bs);
        assert_eq!(builder.build(&mut disk), Err(expected));
        assert!(
            disk.data.iter().all(|&b| b == 0),
            "a refused build wrote to the sink"
        );
    }

    /// The whole round trip: two partitions in, an image out, and every
    /// field read back off the disk — geometry, extents, names, flags,
    /// dostypes and the envec defaults.
    #[test]
    fn builder_round_trips_through_parse() {
        let (_disk, layout, rdb) = build_on(
            RdbBuilder::for_size(TEN_MIB, 512)
                .unwrap()
                .partition(
                    PartitionSpec::by_size(4 * 1024 * 1024)
                        .bootable(5)
                        .dos_type(0x444F_5307),
                )
                .partition(
                    PartitionSpec::by_size(2 * 1024 * 1024)
                        .named("WORK")
                        .size_block_longs(256),
                ),
            TEN_MIB_BLOCKS,
            512,
        );

        // The RDSK, as the geometry and the reserved-area policy decided.
        assert_eq!(rdb.rdsk_block, 0);
        assert_eq!(rdb.block_bytes, 512);
        assert_eq!((rdb.cylinders, rdb.heads, rdb.sectors), (640, 1, 32));
        assert_eq!(rdb.cyl_blocks, 32);
        assert_eq!((rdb.rdb_blocks_lo, rdb.rdb_blocks_hi), (0, 31));
        assert_eq!((rdb.lo_cylinder, rdb.hi_cylinder), (1, 639));
        assert_eq!(rdb.high_rdsk_block, 2);
        assert_eq!(rdb.flags, 0x7);
        assert_eq!(rdb.host_id, 7);
        assert_eq!(rdb.drive_init, CHAIN_END);
        assert_eq!(rdb.filesys_header_list, CHAIN_END);
        assert_eq!(rdb.bad_block_list, CHAIN_END);
        assert!(rdb.filesystems.is_empty() && rdb.bad_blocks.is_empty());

        // 4 MiB over a 16 KiB cylinder is 256 cylinders exactly, from
        // cylinder 1; 2 MiB is 128, packed straight after it.
        let a = &rdb.partitions[0];
        assert_eq!(a.part_block, 1);
        assert_eq!(a.name, "DH0");
        assert_eq!((a.low_cyl, a.high_cyl), (1, 256));
        assert_eq!((a.start_lba, a.block_len), (32, 256 * 32));
        assert_eq!(a.cylinder_blocks, 32);
        assert!(a.bootable && !a.no_automount);
        assert_eq!(a.boot_pri, 5);
        assert_eq!(a.dos_type, 0x444F_5307);

        let b = &rdb.partitions[1];
        assert_eq!(b.part_block, 2);
        assert_eq!(b.name, "WORK");
        assert_eq!((b.low_cyl, b.high_cyl), (257, 384));
        assert_eq!((b.start_lba, b.block_len), (257 * 32, 128 * 32));
        assert!(!b.bootable);
        assert_eq!(b.dos_type, envec_defaults::DOS_TYPE);
        assert_eq!(b.size_block_longs, 256);

        // The envec defaults, in the units the format stores them in.
        for p in &rdb.partitions {
            assert_eq!(p.num_buffers, 30);
            assert_eq!(p.buf_mem_type, 0);
            assert_eq!(p.max_transfer, 0x00FF_FFFF);
            assert_eq!(p.mask, 0x7FFF_FFFE);
            // de_TableSize 16 means the tail fields are *absent*, not
            // zero — which is what rdbtool writes and what a
            // round-tripping consumer must not turn into a zero.
            assert_eq!((p.baud, p.control, p.boot_blocks), (None, None, None));
            assert_eq!(p.envec_raw.len(), 17);
            assert_eq!(p.envec_raw[de::TABLE_SIZE], 16);
            assert_eq!(p.envec_raw[de::SEC_ORG], 0);
            assert_eq!(p.envec_raw[de::SECTORS_PER_BLOCK], 1);
            assert_eq!(p.envec_raw[de::SURFACES], 1);
            assert_eq!(p.envec_raw[de::BLOCKS_PER_TRACK], 32);
            assert_eq!(p.envec_raw[de::RESERVED], 2);
            assert_eq!(p.envec_raw[de::PRE_ALLOC], 0);
            assert_eq!(p.envec_raw[de::INTERLEAVE], 0);
        }
        assert_eq!(rdb.partitions[0].size_block_longs, 128);

        // And the layout says the same as the disk, since a caller may
        // act on it without re-parsing.
        assert_eq!(layout.rdsk_block, rdb.rdsk_block);
        assert_eq!(layout.rdb_blocks_hi, rdb.rdb_blocks_hi);
        assert_eq!(layout.high_rdsk_block, rdb.high_rdsk_block);
        assert_eq!(layout.lo_cylinder, rdb.lo_cylinder);
        for (placed, parsed) in layout.partitions.iter().zip(&rdb.partitions) {
            assert_eq!(placed.part_block, parsed.part_block);
            assert_eq!(placed.name, parsed.name);
            assert_eq!(
                (placed.low_cyl, placed.high_cyl),
                (parsed.low_cyl, parsed.high_cyl)
            );
            assert_eq!(
                (placed.start_lba, placed.block_len),
                (parsed.start_lba, parsed.block_len)
            );
        }
    }

    /// Assigned names step around the explicit ones wherever they
    /// appear, including an explicit name the assignment would otherwise
    /// have reached later.
    #[test]
    fn builder_assigns_names_around_explicit_ones() {
        let (_disk, _layout, rdb) = build_on(
            RdbBuilder::for_size(TEN_MIB, 512)
                .unwrap()
                .partition(PartitionSpec::by_size(1024 * 1024))
                .partition(PartitionSpec::by_size(1024 * 1024).named("DH0"))
                .partition(PartitionSpec::by_size(1024 * 1024))
                .partition(PartitionSpec::by_size(1024 * 1024).named("SCRATCH")),
            TEN_MIB_BLOCKS,
            512,
        );
        let names: Vec<&str> = rdb.partitions.iter().map(|p| p.name.as_str()).collect();
        assert_eq!(names, ["DH1", "DH0", "DH2", "SCRATCH"]);
    }

    /// Sizes round **down** to whole cylinders, pinned against what
    /// `rdbtool` 0.8.1 does with the same requests on the same geometry
    /// (`create chs=1000,4,63`, cylinder = 129 024 bytes): 10 MiB
    /// becomes cylinders 1..=81 and 3 MB the 23 that follow, exactly as
    /// `rdbtool` places them.
    #[test]
    fn builder_size_rounds_down_like_rdbtool() {
        let geometry = Geometry {
            cylinders: 1000,
            heads: 4,
            sectors: 63,
            block_size: 512,
        };
        let (_disk, _layout, rdb) = build_on(
            RdbBuilder::new(geometry)
                .partition(PartitionSpec::by_size(10 * 1024 * 1024))
                .partition(PartitionSpec::by_size(3_000_000))
                // Either side of a cylinder boundary: one byte short of
                // two cylinders is one cylinder, exactly two is two.
                .partition(PartitionSpec::by_size(258_047))
                .partition(PartitionSpec::by_size(258_048)),
            30_000,
            512,
        );
        let extents: Vec<(u32, u32)> = rdb
            .partitions
            .iter()
            .map(|p| (p.low_cyl, p.high_cyl))
            .collect();
        assert_eq!(extents, [(1, 81), (82, 104), (105, 105), (106, 107)]);
    }

    /// Explicit cylinder ranges go exactly where they say, and a sized
    /// partition after one packs from the cylinder it left free.
    #[test]
    fn builder_places_explicit_cylinder_ranges() {
        let (_disk, _layout, rdb) = build_on(
            RdbBuilder::for_size(TEN_MIB, 512)
                .unwrap()
                .partition(PartitionSpec::by_cylinders(100, 199))
                .partition(PartitionSpec::by_size(1024 * 1024))
                // One cylinder is legal: de_HighCyl is inclusive.
                .partition(PartitionSpec::by_cylinders(500, 500)),
            TEN_MIB_BLOCKS,
            512,
        );
        let extents: Vec<(u32, u32)> = rdb
            .partitions
            .iter()
            .map(|p| (p.low_cyl, p.high_cyl))
            .collect();
        assert_eq!(extents, [(100, 199), (200, 263), (500, 500)]);
    }

    /// A partition-less RDB is legal — `rdbtool`'s `create` + `init`
    /// produces one — and is what a caller that partitions in a later
    /// step wants.
    #[test]
    fn builder_writes_an_rdb_with_no_partitions() {
        let (disk, layout, rdb) = build_on(
            RdbBuilder::for_size(TEN_MIB, 512).unwrap(),
            TEN_MIB_BLOCKS,
            512,
        );
        assert!(rdb.partitions.is_empty());
        assert!(layout.partitions.is_empty());
        // The chain head is CHAIN_END, not 0: block 0 is a real address.
        assert_eq!(be32(&disk.data, rdsk::PARTITION_LIST), CHAIN_END);
        // Nothing but the RDSK is in use, but the area is still reserved.
        assert_eq!(rdb.high_rdsk_block, 0);
        assert_eq!(rdb.rdb_blocks_hi, 31);
        assert_eq!(rdb.lo_cylinder, 1);
    }

    /// 4 KB device blocks: every LBA is in *those* blocks, and
    /// `de_SizeBlock` follows the device block size the way `rdbtool`
    /// writes it (1024 longwords) rather than staying at 128.
    #[test]
    fn builder_round_trips_at_4k_blocks() {
        let (_disk, _layout, rdb) = build_on(
            RdbBuilder::for_size(TEN_MIB, 4096)
                .unwrap()
                .partition(PartitionSpec::by_size(4 * 1024 * 1024).bootable(0)),
            (TEN_MIB / 4096) as usize,
            4096,
        );
        assert_eq!(rdb.block_bytes, 4096);
        assert_eq!((rdb.cylinders, rdb.heads, rdb.sectors), (80, 1, 32));
        assert_eq!(rdb.cyl_blocks, 32);
        assert_eq!((rdb.rdb_blocks_lo, rdb.rdb_blocks_hi), (0, 31));
        assert_eq!(rdb.lo_cylinder, 1);

        let p = &rdb.partitions[0];
        // A cylinder is 32 * 4096 = 128 KiB, so 4 MiB is 32 of them.
        assert_eq!((p.low_cyl, p.high_cyl), (1, 32));
        assert_eq!((p.start_lba, p.block_len), (32, 32 * 32));
        assert_eq!(p.size_block_longs, 1024);
        assert_eq!(p.max_transfer, 0x00FF_FFFF);
        assert_eq!(p.mask, 0x7FFF_FFFE);
    }

    /// The reserved area is `rdbtool`'s first cylinder by default, and
    /// grows past it — pushing `rdb_LoCylinder` up with it — when the
    /// `PART` blocks plus headroom do not fit. The alternative, which
    /// this crate will not produce, is RDB blocks written into the first
    /// partition.
    #[test]
    fn reserved_area_grows_past_the_first_cylinder_when_needed() {
        let mut builder = RdbBuilder::for_size(TEN_MIB, 512).unwrap();
        for _ in 0..40 {
            builder = builder.partition(PartitionSpec::by_size(16 * 1024));
        }
        let (_disk, layout, rdb) = build_on(builder, TEN_MIB_BLOCKS, 512);

        // 40 PART blocks + the RDSK is 41, past the 32-block cylinder,
        // so the area is 41 + RDB_HEADROOM_BLOCKS and the first
        // partition starts on cylinder 2 rather than 1.
        assert_eq!(rdb.rdb_blocks_hi, 41 + RDB_HEADROOM_BLOCKS as u32 - 1);
        assert_eq!(rdb.high_rdsk_block, 40);
        assert_eq!(rdb.lo_cylinder, 2);
        assert_eq!(layout.partitions[0].low_cyl, 2);
        assert_eq!(rdb.partitions.len(), 40);
        assert_eq!(rdb.partitions[39].part_block, 40);
    }

    /// An explicit reserved area is honoured as given — the override a
    /// caller reproducing an existing image needs.
    #[test]
    fn explicit_reserved_blocks_are_honoured() {
        let (_disk, _layout, rdb) = build_on(
            RdbBuilder::for_size(TEN_MIB, 512)
                .unwrap()
                .reserved_blocks(64)
                .partition(PartitionSpec::by_size(1024 * 1024)),
            TEN_MIB_BLOCKS,
            512,
        );
        assert_eq!(rdb.rdb_blocks_hi, 63);
        assert_eq!(rdb.lo_cylinder, 2);
        assert_eq!(rdb.partitions[0].low_cyl, 2);
    }

    /// Every way a layout can fail to fit, each refused **before a byte
    /// is written** — the property the whole build path is shaped
    /// around, asserted by handing each case a disk of zeros and
    /// checking it is still zeros afterwards.
    #[test]
    fn build_refuses_a_layout_that_does_not_fit() {
        let ten_mib = || RdbBuilder::for_size(TEN_MIB, 512).unwrap();

        // A cylinder range past the geometry's last cylinder.
        assert_refused(
            ten_mib().partition(PartitionSpec::by_cylinders(600, 700)),
            TEN_MIB_BLOCKS,
            512,
            BuildError::PartitionPastEndOfDisk {
                name: String::from("DH0"),
                high_cyl: 700,
                last_cylinder: 639,
            },
        );

        // A size larger than the space left after the RDB area.
        assert_refused(
            ten_mib().partition(PartitionSpec::by_size(TEN_MIB)),
            TEN_MIB_BLOCKS,
            512,
            BuildError::PartitionPastEndOfDisk {
                name: String::from("DH0"),
                high_cyl: 640,
                last_cylinder: 639,
            },
        );

        // A partition reaching into the reserved RDB area.
        assert_refused(
            ten_mib().partition(PartitionSpec::by_cylinders(0, 10)),
            TEN_MIB_BLOCKS,
            512,
            BuildError::PartitionOverlapsRdbArea {
                name: String::from("DH0"),
                low_cyl: 0,
                lo_cylinder: 1,
            },
        );

        // Two partitions claiming the same cylinders.
        assert_refused(
            ten_mib()
                .partition(PartitionSpec::by_cylinders(10, 20))
                .partition(PartitionSpec::by_cylinders(15, 25)),
            TEN_MIB_BLOCKS,
            512,
            BuildError::PartitionsOverlap {
                a_name: String::from("DH0"),
                b_name: String::from("DH1"),
                low_cyl: 15,
                high_cyl: 20,
            },
        );

        // More PART blocks than the caller's own reserved area holds.
        let mut cramped = ten_mib().reserved_blocks(4);
        for _ in 0..5 {
            cramped = cramped.partition(PartitionSpec::by_size(16 * 1024));
        }
        assert_refused(
            cramped,
            TEN_MIB_BLOCKS,
            512,
            BuildError::RdbAreaTooSmall {
                needed: 6,
                available: 4,
            },
        );

        // A target smaller than the layout: the geometry says 640
        // cylinders, the sink has 100 blocks.
        assert_refused(
            ten_mib().partition(PartitionSpec::by_size(1024 * 1024)),
            100,
            512,
            BuildError::SinkTooSmall {
                needed: 65 * 32,
                available: 100,
            },
        );

        // A geometry with no blocks in it at all.
        let empty = Geometry {
            cylinders: 0,
            heads: 1,
            sectors: 32,
            block_size: 512,
        };
        assert_refused(
            RdbBuilder::new(empty),
            TEN_MIB_BLOCKS,
            512,
            BuildError::EmptyGeometry { geometry: empty },
        );

        // A sink whose blocks are not the geometry's.
        assert_refused(
            ten_mib().partition(PartitionSpec::by_size(1024 * 1024)),
            (TEN_MIB / 4096) as usize,
            4096,
            BuildError::BlockSizeMismatch {
                geometry: 512,
                sink: 4096,
            },
        );
    }

    /// The same discipline for specs that are wrong rather than too big:
    /// nothing written, and an error naming the partition.
    #[test]
    fn build_refuses_a_malformed_spec() {
        let ten_mib = || RdbBuilder::for_size(TEN_MIB, 512).unwrap();

        assert_refused(
            ten_mib().partition(PartitionSpec::by_cylinders(200, 100)),
            TEN_MIB_BLOCKS,
            512,
            BuildError::CylindersInverted {
                name: String::from("DH0"),
                low_cyl: 200,
                high_cyl: 100,
            },
        );

        assert_refused(
            ten_mib()
                .partition(PartitionSpec::by_size(1024 * 1024).named("DH0"))
                .partition(PartitionSpec::by_size(1024 * 1024).named("DH0")),
            TEN_MIB_BLOCKS,
            512,
            BuildError::DuplicateName {
                name: String::from("DH0"),
            },
        );

        let long = "X".repeat(32);
        assert_refused(
            ten_mib().partition(PartitionSpec::by_size(1024 * 1024).named(&long)),
            TEN_MIB_BLOCKS,
            512,
            BuildError::InvalidName {
                name: long,
                max: 31,
            },
        );

        // Below one 16 KiB cylinder there is no partition to write, and
        // rounding up would hand out space the caller did not ask for.
        assert_refused(
            ten_mib().partition(PartitionSpec::by_size(16 * 1024 - 1)),
            TEN_MIB_BLOCKS,
            512,
            BuildError::PartitionTooSmall {
                name: String::from("DH0"),
                bytes: 16 * 1024 - 1,
                cylinder_bytes: 16 * 1024,
            },
        );
    }

    /// The write order: `PART` blocks first, `RDSK` last, so an
    /// interrupted build leaves a disk with no partition table rather
    /// than one pointing at blocks that were never written.
    #[test]
    fn build_writes_the_rdsk_last() {
        /// A sink that fails on the *n*th write, recording what it got.
        struct FlakySink {
            data: Vec<u8>,
            writes: usize,
            fail_after: usize,
        }
        impl BlockSink for FlakySink {
            type Error = ();
            fn block_size(&self) -> usize {
                512
            }
            fn write_block(&mut self, lba: u64, buf: &[u8]) -> Result<(), ()> {
                if self.writes == self.fail_after {
                    return Err(());
                }
                self.writes += 1;
                let off = lba as usize * 512;
                self.data[off..off + 512].copy_from_slice(buf);
                Ok(())
            }
            fn block_count(&self) -> Option<u64> {
                Some(self.data.len() as u64 / 512)
            }
        }

        let mut sink = FlakySink {
            data: vec![0u8; TEN_MIB_BLOCKS * 512],
            writes: 0,
            fail_after: 2,
        };
        let err = RdbBuilder::for_size(TEN_MIB, 512)
            .unwrap()
            .partition(PartitionSpec::by_size(1024 * 1024))
            .partition(PartitionSpec::by_size(1024 * 1024))
            .partition(PartitionSpec::by_size(1024 * 1024))
            .build(&mut sink)
            .unwrap_err();
        assert_eq!(err, BuildError::Io(()));
        // Two PART blocks landed; block 0 is untouched, so the disk has
        // no RDSK and a parse says so rather than following a chain into
        // blocks that do not exist.
        assert_eq!(be32(&sink.data, 512), id::PART);
        assert!(sink.data[..512].iter().all(|&b| b == 0));
        let mut disk = MemDisk {
            data: sink.data,
            block_size: 512,
        };
        assert_eq!(Rdb::parse(&mut disk), Err(RdbError::NoRdsk));
    }

    /// Every `BuildError` renders one line fit to show a user, in
    /// `no_std` as much as `std` — the same contract the parse and
    /// geometry errors hold to.
    #[test]
    fn build_errors_display_as_one_useful_line() {
        let cases: [(BuildError<&str>, &str); 6] = [
            (
                BuildError::Io("device is read-only"),
                "writing a block failed: device is read-only",
            ),
            (
                BuildError::BlockSizeMismatch {
                    geometry: 512,
                    sink: 4096,
                },
                "the geometry is in 512-byte blocks but the sink writes 4096-byte blocks",
            ),
            (
                BuildError::DuplicateName {
                    name: String::from("DH0"),
                },
                "two partitions are both named \"DH0\"",
            ),
            (
                BuildError::PartitionPastEndOfDisk {
                    name: String::from("DH1"),
                    high_cyl: 700,
                    last_cylinder: 639,
                },
                "partition \"DH1\" ends on cylinder 700, past the disk's last cylinder 639",
            ),
            (
                BuildError::RdbAreaTooSmall {
                    needed: 6,
                    available: 4,
                },
                "the RDB area holds 4 blocks but the layout needs 6",
            ),
            (
                BuildError::PartitionTooSmall {
                    name: String::from("DH0"),
                    bytes: 100,
                    cylinder_bytes: 16384,
                },
                "partition \"DH0\" asks for 100 bytes, less than the 16384-byte cylinder \
                 that is the smallest partition",
            ),
        ];
        for (err, expected) in cases {
            assert_eq!(alloc::format!("{err}"), expected);
        }
    }

    /// The differential smoke test: build an image here, and let
    /// `rdbtool` — the tool whose conventions every default in
    /// [`envec_defaults`] was read out of — say what it sees. Agreement
    /// on the partition extents is the claim; anything more is the
    /// round-trip suite's job.
    ///
    /// Gated on `AMIGA_RDB_DIFFERENTIAL=1` because it shells out to a
    /// tool that is not a build dependency of this crate. It is not
    /// wired into CI yet; that is the next plan item.
    #[cfg(feature = "std")]
    #[test]
    fn rdbtool_reads_an_image_this_crate_built() {
        if std::env::var_os("AMIGA_RDB_DIFFERENTIAL").is_none() {
            return;
        }

        let mut disk = blank_disk(TEN_MIB_BLOCKS, 512);
        RdbBuilder::for_size(TEN_MIB, 512)
            .unwrap()
            .partition(PartitionSpec::by_size(4 * 1024 * 1024).bootable(0))
            .partition(PartitionSpec::by_size(2 * 1024 * 1024).named("WORK"))
            .build(&mut disk)
            .expect("build");

        let path = std::env::temp_dir().join("amiga-rdb-differential.hdf");
        std::fs::write(&path, &disk.data).expect("write image");

        let out = std::process::Command::new("rdbtool")
            .arg(&path)
            .arg("list")
            .output()
            .expect("run rdbtool");
        let stdout = String::from_utf8_lossy(&out.stdout);
        assert!(out.status.success(), "rdbtool failed: {stdout}");

        // `Partition: #0 'DH0'   1   256   8192  4.0Mi ...`
        let extents: Vec<(String, u32, u32)> = stdout
            .lines()
            .filter(|l| l.starts_with("Partition:"))
            .map(|l| {
                let f: Vec<&str> = l.split_whitespace().collect();
                (
                    f[2].trim_matches('\'').to_string(),
                    f[3].parse().unwrap(),
                    f[4].parse().unwrap(),
                )
            })
            .collect();
        assert_eq!(
            extents,
            [
                (String::from("DH0"), 1, 256),
                (String::from("WORK"), 257, 384)
            ]
        );

        let _ = std::fs::remove_file(&path);
    }
}
