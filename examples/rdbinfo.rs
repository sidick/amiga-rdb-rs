//! Print an image's RDB: `cargo run --example rdbinfo -- disk.hdf`

use amiga_rdb::{Rdb, SeekBlockSource, CHAIN_END};
use std::fs::File;

fn main() {
    let path = std::env::args().nth(1).expect("usage: rdbinfo <image>");
    let file = File::open(&path).expect("open image");
    let mut disk = SeekBlockSource::new(file).expect("stat image");
    let rdb = match Rdb::parse(&mut disk) {
        Ok(rdb) => rdb,
        Err(e) => {
            eprintln!("{path}: {e:?}");
            std::process::exit(1);
        }
    };

    println!(
        "RDSK at block {}  geometry {}/{}/{}  rdb blocks {}..={}",
        rdb.rdsk_block, rdb.cylinders, rdb.heads, rdb.sectors, rdb.rdb_blocks_lo, rdb.rdb_blocks_hi
    );
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
            p.block_len * 512 / (1024 * 1024),
            p.boot_pri,
            if p.bootable { "  bootable" } else { "" },
            if p.no_automount { "  noautomount" } else { "" },
        );
    }
}
