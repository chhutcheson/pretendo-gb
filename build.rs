use std::{fs, io, path::{Path, PathBuf}};
use serde_json::{self, Value};
fn main() -> Result<(), io::Error> {
    let data = fs::read_to_string("data/Opcodes.json").unwrap();
    let opcodes: serde_json::Value = serde_json::from_str(&data).unwrap();
    println!("{opcodes}");
    let mut table = String::from("use super::Opcode;\npub const OPCODES: [Opcode; 256] = [\n");
    for i in 0..=0xFF {
        let op = &opcodes["unprefixed"][format!("{i:#04X}")];
        let mnemonic = op["mnemonic"].as_str().unwrap();
        let bytes = op["bytes"].as_u64().unwrap();
        let cycles = get_cycles(op);

        table.push_str(&format!(
            "Opcode {{ mnemonic: \"{mnemonic}\", bytes: {bytes}, cycles: {cycles:?} }},\n"
        ));
    }
    table.push_str("];\n");

    let op_table_path = Path::new("src/opcode/gen/opcode_table.rs");
    fs::create_dir_all(op_table_path.parent().expect("ERROR: unable to get parent dir for opcode table"))?;
    let out = PathBuf::from(op_table_path);
    println!("(BLD) Generated file: {out:?}");
    fs::write(out, table).unwrap();

    Ok(())
}

fn get_cycles(op: &Value) -> Vec<u64> {
    let mut cycles = op["cycles"].as_array().unwrap()
            .iter()
            .map(|v| v.as_u64().unwrap())
            .collect::<Vec<_>>();
    while cycles.len() < 2 { // fill if not max cap
            cycles.push(0);
        }
        cycles
}