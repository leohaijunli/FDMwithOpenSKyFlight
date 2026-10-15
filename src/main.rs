use rfdm::autopilot::Autopilot;
use rfdm::config::f450;
use rfdm::controller::PilotInput;
use rfdm::fdm::{Fdm, State};
use rfdm::math::q_from_euler;
use rfdm::sim::Sim;
use std::f64::consts::FRAC_PI_2;
use std::io::Write;

fn main() {
    let aircraft = f450();
    let state = State {
        p_ned: [0.0, 0.0, -0.15],
        v_ned: [0.0; 3],
        w_b: [0.0; 3],
        q: q_from_euler(0.0, 0.0, FRAC_PI_2),
    };
    let fdm = Fdm::new(aircraft, state);
    let ap = Autopilot::new(5.0, FRAC_PI_2);
    let mut sim = Sim::new(fdm, Box::new(ap));

    let dt = 0.001;
    let t_end = 15.0;
    let pilot = PilotInput::default();

    let mut out = String::from("t,alt,north,east,u,v,w,phi,theta,psi,rpm0,rpm1,rpm2,rpm3\n");
    let mut step = 0;
    while sim.fdm.time() <= t_end {
        if step % 50 == 0 {
            let s = sim.snapshot();
            let rpm = |i: usize| sim.fdm.aircraft.quad.rotors[i].rps * 60.0;
            out.push_str(&format!(
                "{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.3},{:.4},{:.4},{:.4},{:.0},{:.0},{:.0},{:.0}\n",
                s.time,
                s.alt,
                s.pos_ned[0],
                s.pos_ned[1],
                s.vel_body[0],
                s.vel_body[1],
                s.vel_body[2],
                s.euler[0],
                s.euler[1],
                s.euler[2],
                rpm(0),
                rpm(1),
                rpm(2),
                rpm(3),
            ));
        }
        sim.step(dt, &pilot);
        step += 1;
    }

    let mut f = std::fs::File::create("fdm_out.csv").unwrap();
    f.write_all(out.as_bytes()).unwrap();
    let lines: Vec<&str> = out.lines().collect();
    for l in &lines[lines.len().saturating_sub(8)..] {
        println!("{}", l);
    }
}
