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
//! test holding a `Vec<u8>`. What is *inside* a partition is out of
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

#![cfg_attr(not(feature = "std"), no_std)]

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
/// ([`DISK_ID`]/[`CTRLR_ID`]) gate whether the RDSK's identification
/// strings hold anything real, so a consumer printing those must check.
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
/// Note the ordering trap: [`SEG_LIST`] is bit 7 and [`GLOBAL_VEC`] bit
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
    /// unconditionally as [`FileSysHeader::seg_list_blocks`] because the
    /// chain has to be walkable either way; this bit only records
    /// whether the FSHD asked for it to be patched in.
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

/// Verify an RDB-family block checksum.
///
/// Every RDB-family block carries `SummedLongs` (longword count, at
/// byte 4) and `ChkSum` (at byte 8) such that the first `SummedLongs`
/// big-endian longwords of the block sum to zero with 32-bit wrapping
/// arithmetic. Returns `false` for a `SummedLongs` that doesn't fit the
/// block — a malformed count must fail the check, not panic the host.
pub fn checksum_ok(block: &[u8]) -> bool {
    let longs = be32(block, 4) as usize;
    if longs == 0 || longs > block.len() / 4 {
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
    /// The source's [`block_size`](BlockSource::block_size) is not a
    /// power of two in [`MIN_BLOCK_SIZE`]`..=`[`MAX_BLOCK_SIZE`].
    UnsupportedBlockSize { block_size: usize },
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
    /// `rdb_BlockBytes` disagrees with the source's
    /// [`block_size`](BlockSource::block_size). Every LBA in the RDB is
    /// in `rdb_BlockBytes` units; reading them through a differently
    /// sized source would silently address the wrong bytes, so the
    /// mismatch is an error, not a guess. (An image of a 4 KB-sector
    /// disk must be presented by a source that says 4096.)
    BlockBytesMismatch { block_bytes: u32, block_size: usize },
    /// A `PART` block's `DosEnvec` was too short to contain the fields
    /// this crate needs (`de_TableSize` below `DE_DOSTYPE`).
    EnvecTooShort { lba: u64, table_size: u32 },
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
    pub start_lba: u64,
    /// Number of device blocks in the partition.
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
    /// `de_LowCyl`/`de_HighCyl`, inclusive.
    pub low_cyl: u32,
    pub high_cyl: u32,
    /// `de_NumBuffers`, `de_BufMemType` — mount parameters a handler
    /// needs even though they say nothing about the disk itself.
    pub num_buffers: u32,
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
    /// Disk geometry as the RDB declares it.
    pub cylinders: u32,
    pub heads: u32,
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
    /// The block range the RDB structures themselves occupy
    /// (`rdb_RDBBlocksLo..=rdb_RDBBlocksHi`) — the area a repartitioner
    /// may rewrite and a filesystem must never touch.
    pub rdb_blocks_lo: u32,
    pub rdb_blocks_hi: u32,
    /// `rdb_LoCylinder`/`rdb_HiCylinder` — the cylinder range available
    /// to partitions. Distinct from `cylinders`: the RDB area itself
    /// normally sits below `lo_cylinder`, so this is the range a
    /// partitioner may actually hand out.
    pub lo_cylinder: u32,
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
    /// `rdb_DiskVendor`/`Product`/`Revision` — SCSI INQUIRY identity of
    /// the drive, space-padded ASCII (*not* BCPL, unlike
    /// `pb_DriveName`). Only meaningful when `flags` has
    /// [`rdb_flags::DISK_ID`]; otherwise these are whatever bytes
    /// happened to be there, so they are parsed unconditionally but must
    /// not be displayed without checking the bit.
    pub disk_vendor: String,
    pub disk_product: String,
    pub disk_revision: String,
    /// `rdb_ControllerVendor`/`Product`/`Revision`, gated the same way
    /// by [`rdb_flags::CTRLR_ID`].
    pub controller_vendor: String,
    pub controller_product: String,
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
/// single line fit to show a user; it is available in `no_std` because
/// it is `core::fmt`, unlike the error types, which are still waiting on
/// the cross-cutting error work.
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
        /// `rdb_RDBBlocksLo` and `rdb_RDBBlocksHi` as they were read.
        lo: u32,
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
        /// The reserved area it should have been inside, inclusive.
        lo: u64,
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
        /// The partition's extent, `start_lba..start_lba + block_len`.
        start_lba: u64,
        block_len: u64,
        /// The reserved area it collides with, inclusive.
        lo: u64,
        hi: u64,
    },
    /// Two partitions' extents intersect. Beyond the letter of the
    /// "overlap validation" plan item, but the same failure family and
    /// the same consequence — two filesystems mounting the same blocks,
    /// each destroying the other — for one extra comparison.
    PartitionsOverlap {
        /// Indices into [`Rdb::partitions`], `a` always the lower.
        a: usize,
        b: usize,
        /// Their `pb_DriveName`s.
        a_name: String,
        b_name: String,
        /// The blocks both claim: `start..start + len`.
        start: u64,
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
        let found = be32(buf, 0);
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
            if be32(&buf, 0) == id::RDSK && checksum_ok(&buf) {
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
    /// 3. every pair of partitions whose extents intersect.
    ///
    /// Check 3 goes beyond the RDB-versus-partition case, but it is the
    /// same failure — two owners, both writing — and costs one pass over
    /// the partition pairs.
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
    let summed_longs = be32(buf, 4) as usize;
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
        baud: present(de::BAUD),
        control: present(de::CONTROL),
        boot_blocks: present(de::BOOT_BLOCKS),
        envec_raw,
    })
}

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

    fn block_size(&self) -> usize {
        self.parent.block_size()
    }

    fn read_block(&mut self, lba: u64, buf: &mut [u8]) -> Result<(), Self::Error> {
        if lba >= self.block_len {
            return Err(PartitionSourceError::OutOfRange {
                lba,
                len: self.block_len,
            });
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
    use super::{block_size_ok, BlockSource, MIN_BLOCK_SIZE};
    use std::io::{Read, Seek, SeekFrom};

    /// A [`BlockSource`] over anything `Read + Seek` — a `File`, a
    /// `Cursor<Vec<u8>>`. The convenience the `std` feature exists for.
    ///
    /// A byte stream carries no sector size of its own, so the caller
    /// supplies it: [`new`](Self::new) assumes the classic 512, and
    /// [`with_block_size`](Self::with_block_size) takes the size of the
    /// device the image was taken from.
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

    fn put32(disk: &mut [u8], bs: usize, block: usize, off: usize, v: u32) {
        let o = block * bs + off;
        disk[o..o + 4].copy_from_slice(&v.to_be_bytes());
    }

    /// Compute and store a valid checksum over `longs` longwords.
    fn seal(disk: &mut [u8], bs: usize, block: usize, longs: u32) {
        put32(disk, bs, block, 4, longs);
        put32(disk, bs, block, 8, 0);
        let base = block * bs;
        let mut sum: u32 = 0;
        for i in 0..longs as usize {
            sum = sum.wrapping_add(u32::from_be_bytes(
                disk[base + i * 4..base + i * 4 + 4].try_into().unwrap(),
            ));
        }
        put32(disk, bs, block, 8, sum.wrapping_neg());
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

    #[test]
    fn checksum_rejects_hostile_summed_longs() {
        let mut b = [0u8; 512];
        b[4..8].copy_from_slice(&0xFFFF_FFFFu32.to_be_bytes());
        assert!(!checksum_ok(&b));
        b[4..8].copy_from_slice(&0u32.to_be_bytes());
        assert!(!checksum_ok(&b));
    }
}
