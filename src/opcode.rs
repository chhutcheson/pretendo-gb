mod opcode_table;

struct Opcode {
    mnemonic: &'static str, bytes: u8, cycles: [u8; 2]
}