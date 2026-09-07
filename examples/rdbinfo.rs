//! Print an image's RDB: `cargo run --example rdbinfo -- disk.hdf [block-size]`
//!
//! `block-size` is the *device* block size the image was taken from
//! (default 512); it must match the RDB's `rdb_BlockBytes`.

use amiga_rdb::{rdb_flags, Rdb, SeekBlockSource, CHAIN_END};
use std::fs::File;

fn main() {
    let path = std::env::args()
        .nth(1)
        .expect("usage: rdbinfo <image> [block-size]");
    let block_size: usize = std::env::args()
        .nth(2)
        .map(|s| s.parse().expect("block-size must be a number"))
        .unwrap_or(512);
    let file = File::open(&path).expect("open image");
    let mut disk = SeekBlockSource::with_block_size(file, block_size).expect("stat image");
    let rdb = match Rdb::parse(&mut disk) {
        Ok(rdb) => rdb,
        Err(e) => {
            eprintln!("{path}: {e:?}");
            std::process::exit(1);
        }
    };

    println!(
        "RDSK at block {}  {} B/block  geometry {}/{}/{}  rdb blocks {}..={}",
        rdb.rdsk_block,
        rdb.block_bytes,
        rdb.cylinders,
        rdb.heads,
        rdb.sectors,
        rdb.rdb_blocks_lo,
        rdb.rdb_blocks_hi
    );
    // Only printed when the flag says the bytes mean anything: without
    // DISKID/CTRLRID these fields are uninitialised, and showing them
    // would be inventing a drive identity.
    if rdb.flags & rdb_flags::DISK_ID != 0 {
        println!(
            "disk: {} {} rev {}",
            rdb.disk_vendor, rdb.disk_product, rdb.disk_revision
        );
    }
    if rdb.flags & rdb_flags::CTRLR_ID != 0 {
        println!(
            "controller: {} {} rev {}",
            rdb.controller_vendor, rdb.controller_product, rdb.controller_revision
        );
    }
    if rdb.filesys_header_list != CHAIN_END {
        println!("FSHD chain head: block {}", rdb.filesys_header_list);
    }
    for p in &rdb.partitions {
        let ds = p.dos_type.to_be_bytes();
        let dos = format!(
            "{}{}{}\\{}",
            ds[0] as char, ds[1] as char, ds[2] as char, ds[3]
        );
        println!(
            "  {:<8} {}  cyls {:>5}..={:<5}  lba {:>8} +{:<8} ({} MB)  bootpri {}{}{}",
            p.name,
            dos,
            p.low_cyl,
            p.high_cyl,
            p.start_lba,
            p.block_len,
            p.block_len * rdb.block_bytes as u64 / (1024 * 1024),
            p.boot_pri,
            if p.bootable { "  bootable" } else { "" },
            if p.no_automount { "  noautomount" } else { "" },
        );
    }
}
