use rfdm::config::f450;
use rfdm::fdm::{Fdm, State};
use rfdm::math::q_from_euler;
use rfdm::px4::{SitlLink, TcpServerTransport};
use rfdm::sensors::{GeoOrigin, SensorSuite};
use std::time::Duration;

fn main() {
    let origin = GeoOrigin::new(47.397742, 8.545594, 488.0);
    let state = State {
        p_ned: [0.0, 0.0, 0.0],
        v_ned: [0.0; 3],
        w_b: [0.0; 3],
        q: q_from_euler(0.0, 0.0, 0.0),
    };
    let mut fdm = Fdm::new(f450(), state);
    let mut sensors = SensorSuite::with_noise(origin);

    let transport = TcpServerTransport::bind("0.0.0.0:4560").expect("bind tcp 4560");
    let mut link = SitlLink::new(transport);

    let dt = 0.004;
    println!("PX4 SITL simulator listening on tcp 0.0.0.0:4560, waiting for PX4...");

    loop {
        let s = fdm.snapshot();
        let frame = sensors.sample(dt, &s);
        if let Err(e) = link.send_sensors(&frame, &s, &origin) {
            eprintln!("send error: {e}");
        }

        let mut waited = 0;
        loop {
            if let Err(e) = link.poll_actuators() {
                eprintln!("recv error: {e}");
            }
            if link.has_pending() {
                break;
            }
            std::thread::sleep(Duration::from_micros(100));
            waited += 1;
            if waited > 10_000 {
                break;
            }
        }

        let motors = link.take_motors().unwrap_or([0.0; 4]);
        fdm.step(dt, &motors);
    }
}
