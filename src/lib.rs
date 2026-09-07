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
//! [`BlockSource`], "read me 512-byte block N" — so the same code
//! serves an emulator holding an image file, a tool holding a raw
//! device, and a test holding a `Vec<u8>`. What is *inside* a partition
//! is out of scope by design: one filesystem family per crate, composed
//! through an adapter that offsets a partition's LBAs into the parent
//! device.
//!
//! Everything on disk is big-endian; all multi-byte reads go through
//! [`be32`]/[`be16`] rather than any `#[repr(C)]` overlay, so the crate
//! is byte-order- and alignment-safe on any host.

#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;

/// Anything that can produce 512-byte blocks by LBA.
///
/// The one seam between this crate and the world. Implementations are
/// expected to be cheap to call repeatedly with the same LBA; the crate
/// does not cache.
pub trait BlockSource {
    type Error;

    /// Read block `lba` into `buf`.
    fn read_block(&mut self, lba: u64, buf: &mut [u8; BLOCK_SIZE]) -> Result<(), Self::Error>;

    /// Total number of blocks, if known. `None` is legitimate (a raw
    /// character device may not know); only operations that need the
    /// disk's end require it.
    fn block_count(&self) -> Option<u64> {
        None
    }
}

/// The block size this crate speaks.
///
/// `rdb_BlockBytes` can in principle name other sizes, but 512 is what
/// every shipped tool writes and what the initial version supports;
/// [`Rdb::parse`] reports anything else as
/// [`RdbError::UnsupportedBlockBytes`] rather than misreading geometry.
pub const BLOCK_SIZE: usize = 512;

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

/// The chain terminator used by every block-pointer field
/// (`rdb_PartitionList`, `pb_Next`, ...): `0xFFFFFFFF`, i.e. `-1`, not
/// `0` — block 0 is a valid block address on a disk whose RDSK sits
/// later in the first sixteen.
pub const CHAIN_END: u32 = 0xFFFF_FFFF;

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

/// Verify an RDB-family block checksum.
///
/// Every RDB-family block carries `SummedLongs` (longword count, at
/// byte 4) and `ChkSum` (at byte 8) such that the first `SummedLongs`
/// big-endian longwords of the block sum to zero with 32-bit wrapping
/// arithmetic. Returns `false` for a `SummedLongs` that doesn't fit the
/// block — a malformed count must fail the check, not panic the host.
pub fn checksum_ok(block: &[u8; BLOCK_SIZE]) -> bool {
    let longs = be32(block, 4) as usize;
    if longs == 0 || longs > BLOCK_SIZE / 4 {
        return false;
    }
    let mut sum: u32 = 0;
    for i in 0..longs {
        sum = sum.wrapping_add(be32(block, i * 4));
    }
    sum == 0
}

/// Errors from parsing an RDB.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RdbError<E> {
    /// The underlying [`BlockSource`] failed.
    Io(E),
    /// No valid `RDSK` block in the first [`RDB_LOCATION_LIMIT`] blocks.
    ///
    /// Not necessarily damage: RDB-less images (a bare filesystem from
    /// block 0) are a real, if less common, layout — this is how a
    /// caller detects one.
    NoRdsk,
    /// A block in a chain had the wrong ID. `expected`/`found` are the
    /// magic numbers; `lba` is where.
    WrongId { lba: u64, expected: u32, found: u32 },
    /// A block's checksum failed. The chain is reported broken rather
    /// than the block trusted: a bad checksum on this platform usually
    /// means a bug wrote it, and silently accepting it is how images
    /// get corrupted further.
    BadChecksum { lba: u64 },
    /// A chain pointer walked past the end of the disk (only detectable
    /// when [`BlockSource::block_count`] is `Some`).
    ChainOutOfRange { lba: u64 },
    /// A chain revisited a block — a cycle. Without this check a
    /// crafted or corrupted image loops the parser forever.
    ChainCycle { lba: u64 },
    /// `rdb_BlockBytes` was not 512.
    UnsupportedBlockBytes { block_bytes: u32 },
    /// A `PART` block's `DosEnvec` was too short to contain the fields
    /// this crate needs (`de_TableSize` below `DE_DOSTYPE`).
    EnvecTooShort { lba: u64, table_size: u32 },
}

/// One partition, as read from a `PART` block.
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
    /// First block of the partition, in disk LBAs.
    pub start_lba: u64,
    /// Number of blocks in the partition.
    pub block_len: u64,
    /// `de_DosType` — e.g. `0x444F5303` (`DOS\x03`).
    pub dos_type: u32,
    /// `de_BootPri`.
    pub boot_pri: i32,
    /// `de_MaxTransfer`.
    pub max_transfer: u32,
    /// `de_Mask`.
    pub mask: u32,
    /// Blocks per cylinder (`de_Surfaces * de_BlocksPerTrack`), kept
    /// because filesystems and repartitioners both need it.
    pub cylinder_blocks: u64,
    /// `de_LowCyl`/`de_HighCyl`, inclusive.
    pub low_cyl: u32,
    pub high_cyl: u32,
    /// `de_NumBuffers`, `de_BufMemType` — mount parameters a handler
    /// needs even though they say nothing about the disk itself.
    pub num_buffers: u32,
    pub buf_mem_type: u32,
    /// `de_SizeBlock` in longwords (128 == 512-byte filesystem blocks).
    pub size_block_longs: u32,
}

/// A parsed RDB: the disk-level header plus its partitions.
///
/// Filesystem headers (`FSHD`) and bad-block lists are recorded as
/// chain heads for now and will grow their own types with the
/// `FSHD`/`LSEG` read support.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Rdb {
    /// LBA the `RDSK` block was found at (0..16).
    pub rdsk_block: u64,
    /// `rdb_Flags`.
    pub flags: u32,
    /// Disk geometry as the RDB declares it.
    pub cylinders: u32,
    pub heads: u32,
    pub sectors: u32,
    /// The block range the RDB structures themselves occupy
    /// (`rdb_RDBBlocksLo..=rdb_RDBBlocksHi`) — the area a repartitioner
    /// may rewrite and a filesystem must never touch.
    pub rdb_blocks_lo: u32,
    pub rdb_blocks_hi: u32,
    /// Head of the `FSHD` chain ([`CHAIN_END`] if none).
    pub filesys_header_list: u32,
    /// Head of the `BADB` chain ([`CHAIN_END`] if none).
    pub bad_block_list: u32,
    /// Partitions in on-disk chain order.
    pub partitions: Vec<Partition>,
}

/// Byte offsets into a `RDSK` block (NDK `RigidDiskBlock`).
mod rdsk {
    pub const BLOCK_BYTES: usize = 16;
    pub const FLAGS: usize = 20;
    pub const BAD_BLOCK_LIST: usize = 24;
    pub const PARTITION_LIST: usize = 28;
    pub const FILESYS_HEADER_LIST: usize = 32;
    pub const CYLINDERS: usize = 64;
    pub const SECTORS: usize = 68;
    pub const HEADS: usize = 72;
    pub const RDB_BLOCKS_LO: usize = 128;
    pub const RDB_BLOCKS_HI: usize = 132;
}

/// Byte offsets into a `PART` block (NDK `PartitionBlock`).
mod part {
    pub const NEXT: usize = 16;
    pub const FLAGS: usize = 20;
    pub const DRIVE_NAME: usize = 36; // BCPL: length byte, then chars (32 bytes total)
    pub const ENVIRONMENT: usize = 128; // DosEnvec, longwords
}

/// `DosEnvec` field indices, in longwords from its start
/// (NDK `dos/filehandler.h`).
mod de {
    pub const TABLE_SIZE: usize = 0;
    pub const SIZE_BLOCK: usize = 1;
    pub const SURFACES: usize = 3;
    pub const BLOCKS_PER_TRACK: usize = 5;
    pub const LOW_CYL: usize = 9;
    pub const HIGH_CYL: usize = 10;
    pub const NUM_BUFFERS: usize = 11;
    pub const BUF_MEM_TYPE: usize = 12;
    pub const MAX_TRANSFER: usize = 13;
    pub const MASK: usize = 14;
    pub const BOOT_PRI: usize = 15;
    pub const DOS_TYPE: usize = 16;
}

impl Rdb {
    /// Find and parse the RDB on `disk`.
    ///
    /// Scans the first [`RDB_LOCATION_LIMIT`] blocks for a `RDSK` block
    /// whose checksum passes (both conditions: an `RDSK` ID with a bad
    /// sum is skipped, matching what the ROM does, so a stale copy at a
    /// lower LBA cannot shadow the live RDB), then walks the `PART`
    /// chain.
    pub fn parse<S: BlockSource>(disk: &mut S) -> Result<Self, RdbError<S::Error>> {
        let mut buf = [0u8; BLOCK_SIZE];

        let mut rdsk_at = None;
        let scan_end = match disk.block_count() {
            Some(n) => n.min(RDB_LOCATION_LIMIT),
            None => RDB_LOCATION_LIMIT,
        };
        for lba in 0..scan_end {
            disk.read_block(lba, &mut buf).map_err(RdbError::Io)?;
            if be32(&buf, 0) == id::RDSK && checksum_ok(&buf) {
                rdsk_at = Some(lba);
                break;
            }
        }
        let rdsk_at = rdsk_at.ok_or(RdbError::NoRdsk)?;

        let block_bytes = be32(&buf, rdsk::BLOCK_BYTES);
        if block_bytes as usize != BLOCK_SIZE {
            return Err(RdbError::UnsupportedBlockBytes { block_bytes });
        }

        let mut rdb = Rdb {
            rdsk_block: rdsk_at,
            flags: be32(&buf, rdsk::FLAGS),
            cylinders: be32(&buf, rdsk::CYLINDERS),
            heads: be32(&buf, rdsk::HEADS),
            sectors: be32(&buf, rdsk::SECTORS),
            rdb_blocks_lo: be32(&buf, rdsk::RDB_BLOCKS_LO),
            rdb_blocks_hi: be32(&buf, rdsk::RDB_BLOCKS_HI),
            filesys_header_list: be32(&buf, rdsk::FILESYS_HEADER_LIST),
            bad_block_list: be32(&buf, rdsk::BAD_BLOCK_LIST),
            partitions: Vec::new(),
        };

        // Walk the PART chain. Bounded by visited-set rather than a
        // magic count: a cycle is the failure mode, not length.
        let mut next = be32(&buf, rdsk::PARTITION_LIST);
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

            disk.read_block(lba, &mut buf).map_err(RdbError::Io)?;
            let found = be32(&buf, 0);
            if found != id::PART {
                return Err(RdbError::WrongId { lba, expected: id::PART, found });
            }
            if !checksum_ok(&buf) {
                return Err(RdbError::BadChecksum { lba });
            }

            rdb.partitions.push(parse_part(&buf, lba)?);
            next = be32(&buf, part::NEXT);
        }

        Ok(rdb)
    }
}

fn parse_part<E>(buf: &[u8; BLOCK_SIZE], lba: u64) -> Result<Partition, RdbError<E>> {
    let envec = |i: usize| be32(buf, part::ENVIRONMENT + i * 4);

    // de_TableSize counts longwords *after itself*; DOS_TYPE is the
    // last field this crate requires. Enforced before reading past it.
    let table_size = envec(de::TABLE_SIZE);
    if (table_size as usize) < de::DOS_TYPE {
        return Err(RdbError::EnvecTooShort { lba, table_size });
    }

    // BCPL string: length byte then bytes, no terminator. The name is
    // ASCII in every image ever seen, but bytes are passed through
    // as-is (lossy) rather than trusted to be UTF-8.
    let name_len = (buf[part::DRIVE_NAME] as usize).min(31);
    let name_bytes = &buf[part::DRIVE_NAME + 1..part::DRIVE_NAME + 1 + name_len];
    let name: String = name_bytes.iter().map(|&b| b as char).collect();

    let surfaces = envec(de::SURFACES) as u64;
    let blocks_per_track = envec(de::BLOCKS_PER_TRACK) as u64;
    let cylinder_blocks = surfaces * blocks_per_track;
    let low_cyl = envec(de::LOW_CYL);
    let high_cyl = envec(de::HIGH_CYL);
    let flags = be32(buf, part::FLAGS);

    Ok(Partition {
        part_block: lba,
        name,
        bootable: flags & 1 != 0,
        no_automount: flags & 2 != 0,
        start_lba: low_cyl as u64 * cylinder_blocks,
        block_len: (high_cyl as u64 - low_cyl as u64 + 1) * cylinder_blocks,
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
    })
}

/// A [`BlockSource`] view of one partition: LBA 0 here is
/// `partition.start_lba` on the parent. This is the composition seam
/// with filesystem crates — they mount one of these, never the disk.
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
    Parent(E),
    OutOfRange { lba: u64, len: u64 },
}

impl<'a, S: BlockSource> PartitionSource<'a, S> {
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

    fn read_block(&mut self, lba: u64, buf: &mut [u8; BLOCK_SIZE]) -> Result<(), Self::Error> {
        if lba >= self.block_len {
            return Err(PartitionSourceError::OutOfRange { lba, len: self.block_len });
        }
        self.parent
            .read_block(self.start_lba + lba, buf)
            .map_err(PartitionSourceError::Parent)
    }

    fn block_count(&self) -> Option<u64> {
        Some(self.block_len)
    }
}

#[cfg(feature = "std")]
mod std_support {
    use super::{BlockSource, BLOCK_SIZE};
    use std::io::{Read, Seek, SeekFrom};

    /// A [`BlockSource`] over anything `Read + Seek` — a `File`, a
    /// `Cursor<Vec<u8>>`. The convenience the `std` feature exists for.
    pub struct SeekBlockSource<T: Read + Seek> {
        inner: T,
        blocks: Option<u64>,
    }

    impl<T: Read + Seek> SeekBlockSource<T> {
        /// `blocks` from the stream length; a stream whose length is
        /// not a block multiple keeps its trailing fragment invisible,
        /// the same as a real disk with a partial final sector.
        pub fn new(mut inner: T) -> std::io::Result<Self> {
            let len = inner.seek(SeekFrom::End(0))?;
            Ok(Self {
                inner,
                blocks: Some(len / BLOCK_SIZE as u64),
            })
        }
    }

    impl<T: Read + Seek> BlockSource for SeekBlockSource<T> {
        type Error = std::io::Error;

        fn read_block(&mut self, lba: u64, buf: &mut [u8; BLOCK_SIZE]) -> Result<(), Self::Error> {
            self.inner.seek(SeekFrom::Start(lba * BLOCK_SIZE as u64))?;
            self.inner.read_exact(buf)
        }

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

    /// In-memory disk for tests.
    struct MemDisk(Vec<u8>);

    impl BlockSource for MemDisk {
        type Error = ();

        fn read_block(&mut self, lba: u64, buf: &mut [u8; BLOCK_SIZE]) -> Result<(), ()> {
            let off = lba as usize * BLOCK_SIZE;
            if off + BLOCK_SIZE > self.0.len() {
                return Err(());
            }
            buf.copy_from_slice(&self.0[off..off + BLOCK_SIZE]);
            Ok(())
        }

        fn block_count(&self) -> Option<u64> {
            Some((self.0.len() / BLOCK_SIZE) as u64)
        }
    }

    fn put32(disk: &mut [u8], block: usize, off: usize, v: u32) {
        let o = block * BLOCK_SIZE + off;
        disk[o..o + 4].copy_from_slice(&v.to_be_bytes());
    }

    /// Compute and store a valid checksum over `longs` longwords.
    fn seal(disk: &mut [u8], block: usize, longs: u32) {
        put32(disk, block, 4, longs);
        put32(disk, block, 8, 0);
        let base = block * BLOCK_SIZE;
        let mut sum: u32 = 0;
        for i in 0..longs as usize {
            sum = sum.wrapping_add(u32::from_be_bytes(
                disk[base + i * 4..base + i * 4 + 4].try_into().unwrap(),
            ));
        }
        put32(disk, block, 8, sum.wrapping_neg());
    }

    /// Build a minimal valid image: RDSK at `rdsk_block`, one PART.
    fn one_partition_image(rdsk_block: usize) -> Vec<u8> {
        // Sized to the geometry it declares: 10 cylinders of 32 blocks.
        let mut d = vec![0u8; 320 * BLOCK_SIZE];
        let part_block = rdsk_block + 1;

        put32(&mut d, rdsk_block, 0, id::RDSK);
        put32(&mut d, rdsk_block, rdsk::BLOCK_BYTES, 512);
        put32(&mut d, rdsk_block, rdsk::BAD_BLOCK_LIST, CHAIN_END);
        put32(&mut d, rdsk_block, rdsk::PARTITION_LIST, part_block as u32);
        put32(&mut d, rdsk_block, rdsk::FILESYS_HEADER_LIST, CHAIN_END);
        put32(&mut d, rdsk_block, rdsk::CYLINDERS, 10);
        put32(&mut d, rdsk_block, rdsk::SECTORS, 32);
        put32(&mut d, rdsk_block, rdsk::HEADS, 1);
        seal(&mut d, rdsk_block, 64);

        put32(&mut d, part_block, 0, id::PART);
        put32(&mut d, part_block, part::NEXT, CHAIN_END);
        put32(&mut d, part_block, part::FLAGS, 1); // bootable
        let name = b"DH0";
        d[part_block * BLOCK_SIZE + part::DRIVE_NAME] = name.len() as u8;
        d[part_block * BLOCK_SIZE + part::DRIVE_NAME + 1
            ..part_block * BLOCK_SIZE + part::DRIVE_NAME + 1 + name.len()]
            .copy_from_slice(name);
        let e = part::ENVIRONMENT;
        put32(&mut d, part_block, e + de::TABLE_SIZE * 4, 16);
        put32(&mut d, part_block, e + de::SIZE_BLOCK * 4, 128);
        put32(&mut d, part_block, e + de::SURFACES * 4, 1);
        put32(&mut d, part_block, e + de::BLOCKS_PER_TRACK * 4, 32);
        put32(&mut d, part_block, e + de::LOW_CYL * 4, 2);
        put32(&mut d, part_block, e + de::HIGH_CYL * 4, 9);
        put32(&mut d, part_block, e + de::BOOT_PRI * 4, 0);
        put32(&mut d, part_block, e + de::DOS_TYPE * 4, 0x444F_5303);
        seal(&mut d, part_block, 64);

        d
    }

    #[test]
    fn parses_a_minimal_image() {
        let mut disk = MemDisk(one_partition_image(2));
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.rdsk_block, 2);
        assert_eq!(rdb.partitions.len(), 1);
        let p = &rdb.partitions[0];
        assert_eq!(p.name, "DH0");
        assert!(p.bootable);
        assert_eq!(p.dos_type, 0x444F_5303);
        assert_eq!(p.cylinder_blocks, 32);
        assert_eq!(p.start_lba, 64); // LowCyl 2 * 32
        assert_eq!(p.block_len, 256); // cyls 2..=9
    }

    #[test]
    fn no_rdsk_is_reported_not_invented() {
        let mut disk = MemDisk(vec![0u8; 32 * BLOCK_SIZE]);
        assert_eq!(Rdb::parse(&mut disk).unwrap_err(), RdbError::NoRdsk);
    }

    /// An `RDSK` ID with a bad checksum must be skipped, not trusted —
    /// this is the stale-copy-shadows-live-RDB case.
    #[test]
    fn bad_checksum_rdsk_is_skipped_in_the_scan() {
        let mut img = one_partition_image(3);
        // Plant a checksummed-wrong RDSK *earlier* than the real one.
        put32(&mut img, 1, 0, id::RDSK);
        put32(&mut img, 1, 4, 64);
        put32(&mut img, 1, 8, 0xDEAD_BEEF);
        let mut disk = MemDisk(img);
        let rdb = Rdb::parse(&mut disk).unwrap();
        assert_eq!(rdb.rdsk_block, 3);
    }

    #[test]
    fn part_chain_cycle_is_an_error_not_a_hang() {
        let mut img = one_partition_image(0);
        // PART at 1 points to itself.
        put32(&mut img, 1, part::NEXT, 1);
        seal(&mut img, 1, 64);
        let mut disk = MemDisk(img);
        assert_eq!(
            Rdb::parse(&mut disk).unwrap_err(),
            RdbError::ChainCycle { lba: 1 }
        );
    }

    #[test]
    fn part_chain_past_disk_end_is_an_error() {
        let mut img = one_partition_image(0);
        put32(&mut img, 0, rdsk::PARTITION_LIST, 1000);
        seal(&mut img, 0, 64);
        let mut disk = MemDisk(img);
        assert_eq!(
            Rdb::parse(&mut disk).unwrap_err(),
            RdbError::ChainOutOfRange { lba: 1000 }
        );
    }

    #[test]
    fn partition_source_offsets_and_bounds() {
        let mut disk = MemDisk(one_partition_image(2));
        // Stamp a marker at the partition's first block (LBA 64).
        disk.0[64 * BLOCK_SIZE] = 0xAB;
        let rdb = Rdb::parse(&mut disk).unwrap();
        let p = rdb.partitions[0].clone();
        let mut ps = PartitionSource::new(&mut disk, &p);
        assert_eq!(ps.block_count(), Some(256));
        let mut buf = [0u8; BLOCK_SIZE];
        ps.read_block(0, &mut buf).unwrap();
        assert_eq!(buf[0], 0xAB);
        assert!(matches!(
            ps.read_block(256, &mut buf),
            Err(PartitionSourceError::OutOfRange { lba: 256, len: 256 })
        ));
    }

    #[test]
    fn checksum_rejects_hostile_summed_longs() {
        let mut b = [0u8; BLOCK_SIZE];
        b[4..8].copy_from_slice(&0xFFFF_FFFFu32.to_be_bytes());
        assert!(!checksum_ok(&b));
        b[4..8].copy_from_slice(&0u32.to_be_bytes());
        assert!(!checksum_ok(&b));
    }
}
