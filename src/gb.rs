use std::rc::Rc;
use crate::cpu::Cpu;
use crate::mmu::Mmu;

struct GB {
    cpu: Cpu,
    mmu: Rc<Mmu>,
}

impl GB {
    fn new() -> Self {
        let mmu = Rc::new(Mmu::new());
        Self {
            cpu: Cpu::new(mmu.clone()),
            mmu: mmu.clone(),
        }
    }
}