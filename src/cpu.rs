use std::rc::Rc;
use crate::mmu;

// clock speed, see page 12: https://gekkio.fi/files/gb-docs/gbctr.pdf
const CPU_CLOCK_SPEED: u32 = 4_194_304;

/// this hold this registers, stack pointer and program counter of the SM83 cpu
/// see ch 4 of https://gekkio.fi/files/gb-docs/gbctr.pdf
#[derive(Default, Debug, Clone)]
pub struct RegisterFile {
    af: u16, // should only be addressable in 8 bit mode
    bc: u16,
    de: u16,
    hl: u16,
    sp: u16, // stack pointer
    pc: u16, // program counter
    ir: u16,
    ie: u16,
}

const UPPER_BITS_MASK: u16 = !0xff00; // upper half of a u16
const LOWER_BITS_MASK: u16 = !0x00ff; // lower half of a u16
const HRAM_BASE_POINTER: u16 = 0xff00; // base pointer to HRAM

/// Possible states of the F register, see source
/// https://gbdev.io/pandocs/CPU_Registers_and_Flags.html
pub mod flags_bits {
    /// Code C in datasheet
    /// Is set in these cases:
    /// - When the result of an 8-bit addition is higher than $FF.
    /// - When the result of a 16-bit addition is higher than $FFFF.
    /// - When the result of a subtraction or comparison is lower than zero (like in Z80 and x86 CPUs,
    ///     but unlike in 65XX and ARM CPUs).
    /// - When a rotate/shift operation shifts out a “1” bit.
    ///
    /// Used by conditional jumps and instructions such as ADC, SBC, RL, RLA, etc.
    pub(crate) const CARRY: u8               = 0b0001_0000;

    /// Code H in datasheet
    /// This flag is used by the DAA instruction only.
    /// Indicates carry for the lower 4 bits of the result. DAA also uses the C flag, which must
    /// indicate carry for the upper 4 bits. After adding/subtracting two BCD numbers, DAA is used to
    /// convert the result to BCD format. BCD numbers range from $00 to $99 rather than $00 to $FF.
    /// Because only two flags (C and H) exist to indicate carry-outs of BCD digits, DAA is ineffective
    /// for 16-bit operations (which have 4 digits), and use for INC/DEC operations (which do not affect
    /// C-flag) has limits.
    pub(crate) const HALF_CARRY: u8           = 0b0010_0000;

    /// Code N in the datasheet
    /// These flags are used by the DAA instruction only. N indicates whether the previous instruction
    /// has been a subtraction. DAA also uses the C flag, which must indicate carry for the upper 4 bits.
    /// After adding/subtracting two BCD numbers, DAA is used to convert the result to BCD format. BCD
    /// numbers range from $00 to $99 rather than $00 to $FF. Because only two flags (C and H) exist to
    /// indicate carry-outs of BCD digits, DAA is ineffective for 16-bit operations (which have 4 digits),
    /// and use for INC/DEC operations (which do not affect C-flag) has limits.
    pub(crate) const SUBTRACTION_CARRY: u8              = 0b0100_0000;

    /// Code Z in the datasheet
    /// This bit is set if and only if the result of an operation is zero. Used by conditional jumps.
    pub(crate) const ZERO: u8                = 0b1000_0000;
}

#[repr(u8)]
enum Flags {
    Zero = flags_bits::ZERO,
    Subtraction = flags_bits::SUBTRACTION_CARRY,
    HalfCarry = flags_bits::HALF_CARRY,
    Carry = flags_bits::CARRY,
}

impl RegisterFile {
    // note - getting the top half of a u16 -> (n >> 8) as u8
    // conversion to u8 truncates the u16 so bottom half -> n as u8
    // this SHOULD be equivalent to the usual (n & 0xff) as u8

    pub fn new() -> Self {
        Default::default()
    }

    //--- AF registers

    #[inline]
    pub fn af(&self) -> u16 {
        self.af
    }

    #[inline]
    pub fn a(&self) -> u8 {
        (self.af >> 8) as u8
    }

    #[inline]
    pub fn f(&self) -> u8 {
        self.af as u8
    }

    #[inline]
    pub fn set_a(&mut self, value: u8) {
        self.af &= UPPER_BITS_MASK;
        self.af |= (value as u16) << 8;
    }

    #[inline]
    pub fn set_f(&mut self, value: u8) {
        self.af &= LOWER_BITS_MASK;
        self.af |= value as u16;
    }
    //--- AF registers

    //--- BC registers

    #[inline]
    pub fn bc(&self) -> u16 {
        self.bc
    }
    #[inline]
    pub fn b(&self) -> u8 {
        (self.bc >> 8) as u8
    }

    #[inline]
    pub fn c(&self) -> u8 {
        self.bc as u8
    }

    #[inline]
    pub fn set_b(&mut self, value: u8) {
        self.bc &= UPPER_BITS_MASK;
        self.bc |= (value as u16) << 8;
    }

    #[inline]
    pub fn set_c(&mut self, value: u8) {
        self.bc &= LOWER_BITS_MASK;
        self.bc |= value as u16;
    }

    #[inline]
    pub fn set_bc(&mut self, value: u16) {
        self.bc = value;
    }
    //--- BC registers

    //--- DE registers

    #[inline]
    pub fn de(&self) -> u16 {
        self.de
    }

    #[inline]
    pub fn d(&self) -> u8 {
        (self.de >> 8) as u8
    }

    #[inline]
    pub fn e(&self) -> u8 {
        self.de as u8
    }

    #[inline]
    pub fn set_d(&mut self, value: u8) {
        self.de &= UPPER_BITS_MASK;
        self.de |= (value as u16) << 8;
    }

    #[inline]
    pub fn set_e(&mut self, value: u8) {
        self.de &= LOWER_BITS_MASK;
        self.de |= value as u16;
    }

    #[inline]
    pub fn set_de(&mut self, value: u16) {
        self.de = value;
    }
    //--- DE registers

    //--- HL registers

    #[inline]
    pub fn hl(&self) -> u16 {
        self.hl
    }

    #[inline]
    pub fn h(&self) -> u8 {
        (self.hl >> 8) as u8
    }

    #[inline]
    pub fn l(&self) -> u8 {
        self.hl as u8
    }

    #[inline]
    pub fn set_h(&mut self, value: u8) {
        self.hl &= UPPER_BITS_MASK;
        self.hl |= (value as u16) << 8;
    }

    #[inline]
    pub fn set_l(&mut self, value: u8) {
        self.hl &= LOWER_BITS_MASK;
        self.hl |= value as u16;
    }

    #[inline]
    pub fn set_hl(&mut self, value: u16) {
        self.hl = value;
    }
    //--- HL registers

    #[inline]
    pub fn sp(&self) -> u16 {
        self.sp
    }

    #[inline]
    pub fn advance_sp(&mut self, value: u16) {
        self.sp += value;
    }

    #[inline]
    pub fn reduce_sp(&mut self, value: u16) {
        self.sp -= value;
    }

    #[inline]
    pub fn pc(&self) -> u16 {
        self.pc
    }

    #[inline]
    pub fn advance_pc(&mut self, value: u16) {
        self.pc += value;
    }
    
    pub fn set_carry<F>(&mut self, f: F) where F: FnOnce() -> bool {
        
    }
}

enum Instruction {
    // Add
    Add8Bit(ArithmeticTarget),
    // Add with carry
    Adc8Bit(ArithmeticTarget),
    // Sub
    Sub8Bit(ArithmeticTarget),
    // Sub with carry
    Sbc8Bit(ArithmeticTarget),
    // And
    And8Bit(ArithmeticTarget),
    // Xor
    Xor8Bit(ArithmeticTarget),
    // Or
    Or8Bit(ArithmeticTarget),
    // ComPare
    Cp8Bit(ArithmeticTarget),
}

enum ArithmeticTarget {
    A,
    F,
    B,
    C,
    D,
    E,
    H,
    L,
    HLPtr,
    Const(u8),
}

#[derive(Default, Debug, Clone)]
pub struct Cpu {
    registers: RegisterFile,
    mmu: Rc<mmu::Mmu>
}

/// - to_value is from the register that was added into
/// 
/// - from_value is from the register that added into to_value
/// 
/// - new_value is the value from the operation of from_value into to_value
/// 
/// - overflow is true if overflow occured in the operation
/// 
/// Returns the resulting flags
fn calculate_flags(to_value: u8, from_value: u8, new_value: u8, overflow: bool) -> u8 {
    ((new_value == 0) as u8 * flags_bits::ZERO)
        | (overflow as u8 * flags_bits::CARRY)
        | (to_value.is_half_carry(from_value) as u8 * flags_bits::HALF_CARRY)
}



impl Cpu {
    pub fn new(mmu: Rc<mmu::Mmu>) -> Self {
        Self { registers: Default::default(), mmu }
    }

    fn set_zero_flag(&mut self, value: u8) -> u8 {
        ((value == 0) as u8 * flags_bits::ZERO)
    }
    
    fn execute(&mut self, instruction: Instruction) {
        // getting num of cycles to advance from the op
        let _cycles = match instruction {
            Instruction::Add8Bit(r) => self.add_a_u8(r),
            Instruction::Adc8Bit(r) => self.adc_a_u8(r),
            Instruction::Sub8Bit(r) => self.sub_a_u8(r),
            Instruction::Sbc8Bit(r) => self.sbc_a_u8(r),
            Instruction::And8Bit(r) => self.and_a_u8(r), 
            Instruction::Xor8Bit(r) => self.xor_a_u8(r),
            Instruction::Or8Bit(r) => self.or_a_u8(r),
            Instruction::Cp8Bit(r) => self.cp_a_u8(r),
        };
        
        // wait for duration of instruction
        
    }
    
    // ----- Ops - https://gbdev.io/gb-opcodes/optables/octal
    /// 8 bit mode add to register A - 20x row
    /// 
    /// handles register addition (r8) and const addition (n8)
    /// corresponds to the `add A r8`, `add A n8` and `add A [HL]` opcodes
    ///
    /// returns number of cycles it took to execute
    fn add_a_u8(&mut self, r: ArithmeticTarget) -> u8 {
        let (value, cycles) = match r {
            // r8
            ArithmeticTarget::A => (self.registers.a(), 1),
            ArithmeticTarget::B => (self.registers.b(), 1),
            ArithmeticTarget::C => (self.registers.c(), 1),
            ArithmeticTarget::D => (self.registers.d(), 1),
            ArithmeticTarget::E => (self.registers.e(), 1),
            ArithmeticTarget::H => (self.registers.h(), 1),
            ArithmeticTarget::L => (self.registers.l(), 1),
            // n8
            ArithmeticTarget::Const(v) => (v, 2),
            // address of HL
            ArithmeticTarget::HLPtr => {
                // get value from pointer
                let ptr = self.registers.hl();
                let value = match self.mmu.read_u8(ptr) {
                    Ok(value) => value,
                    Err(_) => panic!("HL pointer is invalid"),
                };
                (value, 2)
            }
            _ => panic!("ADD op to register F is invalid"),
        };

        // add operation
        let (new_value, overflow) = self.registers.a().overflowing_add(value);
        // set flags
        self.registers.set_f(calculate_flags(self.registers.a(), value, new_value, overflow));
        // set register
        self.registers.set_a(new_value);

        cycles
    }
    
    /// 8 bit mode add with carry to register A - 21x row
    ///
    /// handles register addition (r8) and const addition (n8)
    /// corresponds to the `adc A r8`, `adc A n8` and `adc A [HL]` opcodes
    ///
    /// returns number of cycles it took to execute
    fn adc_a_u8(&mut self, r: ArithmeticTarget) -> u8 {
        // check for carry and add that to value
        if self.registers.f() & flags_bits::CARRY != 0 {
            self.registers.set_a(self.registers.a() + 1);
        }
        self.add_a_u8(r)
    }

    /// 8 bit mode subtraction to register A - 22x row
    ///
    /// handles register subtraction (r8) and const subtraction (n8)
    /// corresponds to the `sub A r8`, `sub A n8` and `sub A [HL]` opcodes
    ///
    /// returns number of cycles it took to execute
    fn sub_a_u8(&mut self, r: ArithmeticTarget) -> u8 {
        let (value, cycles) = match r {
            // r8
            ArithmeticTarget::A => (self.registers.a(), 1),
            ArithmeticTarget::B => (self.registers.b(), 1),
            ArithmeticTarget::C => (self.registers.c(), 1),
            ArithmeticTarget::D => (self.registers.d(), 1),
            ArithmeticTarget::E => (self.registers.e(), 1),
            ArithmeticTarget::H => (self.registers.h(), 1),
            ArithmeticTarget::L => (self.registers.l(), 1),
            // n8
            ArithmeticTarget::Const(v) => (v, 2),
            // address of HL
            ArithmeticTarget::HLPtr => {
                // get value from pointer
                let ptr = self.registers.hl();
                let value = match self.mmu.read_u8(ptr) {
                    Ok(value) => value,
                    Err(_) => panic!("HL pointer is invalid"),
                };
                (value, 2)
            }
            _ => panic!("SUB op to register F is invalid"),
        };
        
        // sub operation
        let (new_value, overflow) = self.registers.a().overflowing_sub(value);
        // set flags
        self.registers.set_f(calculate_flags(self.registers.a(), value, new_value, overflow));
        // set register
        self.registers.set_a(new_value);
        
        cycles
    }

    /// 8 bit mode subtraction with carry to register A - 23x row
    ///
    /// handles register subtraction (r8) and const subtraction (n8)
    /// corresponds to the `sbc A r8`, `sbc A n8` and `sbc A [HL]` opcodes
    ///
    /// returns number of cycles it took to execute
    fn sbc_a_u8(&mut self, r: ArithmeticTarget) -> u8 {
        // check for carry and add that to value
        if self.registers.f() & flags_bits::CARRY != 0 {
            self.registers.set_a(self.registers.a() + 1);
        }
        self.sub_a_u8(r)
    }

    /// 8 bit mode AND to register A - 24x row
    ///
    /// handles register and (r8) and const and (n8)
    /// corresponds to the `and A r8`, `and A n8` and `and A [HL]` opcodes
    ///
    /// returns number of cycles it took to execute
    fn and_a_u8(&mut self, r: ArithmeticTarget) -> u8 {
        let (value, cycles) = match r {
            // r8
            ArithmeticTarget::A => (self.registers.a(), 1),
            ArithmeticTarget::B => (self.registers.b(), 1),
            ArithmeticTarget::C => (self.registers.c(), 1),
            ArithmeticTarget::D => (self.registers.d(), 1),
            ArithmeticTarget::E => (self.registers.e(), 1),
            ArithmeticTarget::H => (self.registers.h(), 1),
            ArithmeticTarget::L => (self.registers.l(), 1),
            // n8
            ArithmeticTarget::Const(v) => (v, 2),
            // address of HL
            ArithmeticTarget::HLPtr => {
                // get value from pointer
                let ptr = self.registers.hl();
                let value = match self.mmu.read_u8(ptr) {
                    Ok(value) => value,
                    Err(_) => panic!("HL pointer is invalid"),
                };
                (value, 2)
            }
            _ => panic!("AND op to register F is invalid"),
        };
        
        let new_value = self.registers.a() & value;
        // always set the half carry flag
        self.registers.set_f(((new_value == 0) as u8 * flags_bits::ZERO) | flags_bits::HALF_CARRY);
        // set register
        self.registers.set_a(new_value);
        
        cycles
    }

    /// 8 bit mode XOR to register A - 25x row
    ///
    /// handles register xor (r8) and const xor (n8)
    /// corresponds to the `xor A r8`, `xor A n8` and `xor A [HL]` opcodes
    ///
    /// returns number of cycles it took to execute
    fn xor_a_u8(&mut self, r: ArithmeticTarget) -> u8 {
        let (value, cycles) = match r {
            // r8
            ArithmeticTarget::A => (self.registers.a(), 1),
            ArithmeticTarget::B => (self.registers.b(), 1),
            ArithmeticTarget::C => (self.registers.c(), 1),
            ArithmeticTarget::D => (self.registers.d(), 1),
            ArithmeticTarget::E => (self.registers.e(), 1),
            ArithmeticTarget::H => (self.registers.h(), 1),
            ArithmeticTarget::L => (self.registers.l(), 1),
            // n8
            ArithmeticTarget::Const(v) => (v, 2),
            // address of HL
            ArithmeticTarget::HLPtr => {
                // get value from pointer
                let ptr = self.registers.hl();
                let value = match self.mmu.read_u8(ptr) {
                    Ok(value) => value,
                    Err(_) => panic!("HL pointer is invalid"),
                };
                (value, 2)
            }
            _ => panic!("XOR op to register F is invalid"),
        };

        let new_value = self.registers.a() ^ value;
        // always set the half carry flag
        self.registers.set_f(((new_value == 0) as u8 * flags_bits::ZERO) | flags_bits::HALF_CARRY);
        // set register
        self.registers.set_a(new_value);

        cycles
    }

    /// 8 bit mode OR to register A - 25x row
    ///
    /// handles register or (r8) and const or (n8)
    /// corresponds to the `or A r8`, `or A n8` and `or A [HL]` opcodes
    ///
    /// returns number of cycles it took to execute
    fn or_a_u8(&mut self, r: ArithmeticTarget) -> u8 {
        let (value, cycles) = match r {
            // r8
            ArithmeticTarget::A => (self.registers.a(), 1),
            ArithmeticTarget::B => (self.registers.b(), 1),
            ArithmeticTarget::C => (self.registers.c(), 1),
            ArithmeticTarget::D => (self.registers.d(), 1),
            ArithmeticTarget::E => (self.registers.e(), 1),
            ArithmeticTarget::H => (self.registers.h(), 1),
            ArithmeticTarget::L => (self.registers.l(), 1),
            // n8
            ArithmeticTarget::Const(v) => (v, 2),
            // address of HL
            ArithmeticTarget::HLPtr => {
                // get value from pointer
                let ptr = self.registers.hl();
                let value = match self.mmu.read_u8(ptr) {
                    Ok(value) => value,
                    Err(_) => panic!("HL pointer is invalid"),
                };
                (value, 2)
            }
            _ => panic!("OR op to register F is invalid"),
        };

        let new_value = self.registers.a() | value;
        // always set the half carry flag
        self.registers.set_f(0x00);
        // set register
        self.registers.set_a(new_value);

        cycles
    }
    
    fn cp_a_u8(&mut self, r: ArithmeticTarget) -> u8 {
        let (value, cycles) = match r {
            // r8
            ArithmeticTarget::A => (self.registers.a(), 1),
            ArithmeticTarget::B => (self.registers.b(), 1),
            ArithmeticTarget::C => (self.registers.c(), 1),
            ArithmeticTarget::D => (self.registers.d(), 1),
            ArithmeticTarget::E => (self.registers.e(), 1),
            ArithmeticTarget::H => (self.registers.h(), 1),
            ArithmeticTarget::L => (self.registers.l(), 1),
            // n8
            ArithmeticTarget::Const(v) => (v, 2),
            // address of HL
            ArithmeticTarget::HLPtr => {
                // get value from pointer
                let ptr = self.registers.hl();
                let value = match self.mmu.read_u8(ptr) {
                    Ok(value) => value,
                    Err(_) => panic!("HL pointer is invalid"),
                };
                (value, 2)
            }
            _ => panic!("CP op to register F is invalid"),
        };
        
        let (value, overflow) = self.registers.a().overflowing_sub(value);
        
        self.registers.set_f(calculate_flags(self.registers.a(), value, value, overflow));
        
        cycles
    }
}

trait HalfCarry {
    /// Determines if the operation had a half-carry
    ///
    /// returns true if it was a half-carry operation
    fn is_half_carry(&self, r: Self) -> bool;
}

impl HalfCarry for u8 {
    fn is_half_carry(&self, r: Self) -> bool {
        (self & 0xF) + (r & 0xF) > 0xF
    }
}

#[cfg(test)]
mod tests {
    use crate::cpu::RegisterFile;

    #[test]
    pub fn test_registers() {
        let mut registers = RegisterFile::new();
        registers.set_f(0xff);
        assert_eq!(registers.af(), 0xff);
        registers.set_f(0x0);
        assert_eq!(registers.af(), 0x0);
        registers.set_a(0xff);
        assert_eq!(registers.af(), 0xff00);
        registers.set_a(0x0);
        assert_eq!(registers.af(), 0x0);
        registers.set_a(0xff);
        registers.set_f(0xff);
        assert_eq!(registers.af(), 0xffff);
    }
}