use crate::aero::Aero;
use crate::ground::{Contact, Ground};
use crate::propulsion::{Motor, Prop, Quad, Rotor};
use crate::math::v3;

pub struct Aircraft {
    pub mass: f64,
    pub inertia: [f64; 3],
    pub quad: Quad,
    pub aero: Aero,
    pub ground: Ground,
}

pub fn f450() -> Aircraft {
    let motor = Motor {
        kv: 960.0,
        resistance: 0.117,
        i0: 0.45,
        vmax: 14.63,
    };
    let prop = Prop {
        diam: 0.23876,
        ixx: 6.05e-5,
        sense_multiplier: -1.0,
    };
    let rotors = vec![
        Rotor { motor, prop, pos: v3(0.1651, 0.1651, -0.025), sense: 1.0, rps: 0.0 },
        Rotor { motor, prop, pos: v3(-0.1651, -0.1651, -0.025), sense: 1.0, rps: 0.0 },
        Rotor { motor, prop, pos: v3(0.1651, -0.1651, -0.025), sense: -1.0, rps: 0.0 },
        Rotor { motor, prop, pos: v3(-0.1651, 0.1651, -0.025), sense: -1.0, rps: 0.0 },
    ];

    let contact = |x: f64, y: f64| Contact {
        pos: v3(x, y, 0.1524),
        k: 3000.0,
        c: 100.0,
        mu_s: 0.8,
        mu_d: 0.5,
    };
    let ground = Ground {
        contacts: vec![
            contact(0.1651, 0.1651),
            contact(-0.1651, 0.1651),
            contact(0.1651, -0.1651),
            contact(-0.1651, -0.1651),
        ],
    };

    Aircraft {
        mass: 1.4,
        inertia: [0.0190, 0.0190, 0.0252],
        quad: Quad { rotors },
        aero: Aero { s: 0.016129, cd: 1.0 },
        ground,
    }
}
