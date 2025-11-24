// https://gbdev.io/pandocs/Memory_Map.html

const MMAP_SIZE: u16 = u16::MAX;
const ROM_BANK_ONE_SIZE: u16 = 0x4000; // fixed rom bank
const ROM_BANK_ONE_START: u16 = 0x0000;
const ROM_BANK_TWO_SIZE: u16 = 0x4000; // used by some maps
const ROM_BANK_TWO_START: u16 = 0x4000;
const VRAM_SIZE: u16 = 0x2000; // video ram
const VRAM_START: u16 = 0x8000;
const EXTERNAL_RAM_SIZE: u16 = 0x2000; // ram on the cartridge
const EXTERNAL_RAM_START: u16 = 0xA000;
const WORKING_RAM_BANK_ONE_SIZE: u16 = 0x1000;
const WORKING_RAM_BANK_ONE_START: u16 = 0xC000;
const WORKING_RAM_BANK_TWO_SIZE: u16 = 0x1000;
const WORKING_RAM_BANK_TWO_START: u16 = 0xD000;

pub enum MMapError {
    OutOfBounds,
}

#[derive(Debug, Default, Clone)]
pub struct Mmu {
    mmap: Vec<u8>,
}

impl Mmu {
    pub fn new() -> Self {
        Self {
            mmap: vec![0; MMAP_SIZE as usize],
        }
    }
    
    pub fn read_u8(&self, addr: u16) -> Result<u8, MMapError> {
        self.mmap.get(addr as usize).cloned().ok_or(MMapError::OutOfBounds)
    }
    
    pub fn read_u16(&self, addr: u16) -> Result<u16, MMapError> {
        let bytes = self.mmap.get(addr as usize..=(addr as usize + 1)).ok_or(MMapError::OutOfBounds)?;
        // reconstruct u16 from byte array
        let n = (bytes[1] as u16) | ((bytes[0] as u16) << 8);
        Ok(n)
    }
    
    pub fn read_vram_u8(&self, addr: u16) -> Result<u8, MMapError> {
        self.read_u8(addr + VRAM_START)
    }
    
    pub fn read_vram_u16(&self, addr: u16) -> Result<u16, MMapError> {
        self.read_u16(addr + VRAM_START)
    }
}

// impl Default for Mmu {
//     fn default() -> Self {
//         Self::new()
//     }
// }