use crate::controller::{Controller, FdmState, PilotInput};
use crate::fdm::Fdm;

pub struct Sim {
    pub fdm: Fdm,
    pub controller: Box<dyn Controller>,
}

impl Sim {
    pub fn new(fdm: Fdm, controller: Box<dyn Controller>) -> Sim {
        Sim { fdm, controller }
    }

    pub fn set_controller(&mut self, controller: Box<dyn Controller>) {
        self.controller = controller;
    }

    /// Advance one step: controller reads state, produces motor commands, plant integrates.
    pub fn step(&mut self, dt: f64, pilot: &PilotInput) -> FdmState {
        let s = self.fdm.snapshot();
        let motors = self.controller.update(dt, &s, pilot);
        self.fdm.step(dt, &motors);
        self.fdm.snapshot()
    }

    /// Bypass the controller and drive the 4 motor throttle commands directly.
    pub fn step_motors(&mut self, dt: f64, motors: &[f64; 4]) -> FdmState {
        self.fdm.step(dt, motors);
        self.fdm.snapshot()
    }

    pub fn snapshot(&self) -> FdmState {
        self.fdm.snapshot()
    }
}
