use rfdm::config::f450;
use rfdm::fdm::{Fdm, State};
use rfdm::math::q_from_euler;
use rfdm::px4::{SitlLink, TcpServerTransport};
use rfdm::sensors::{GeoOrigin, SensorSuite};
use std::time::{Duration, Instant};

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
    let step_period = Duration::from_micros(4_000);
    println!("PX4 SITL simulator listening on tcp 0.0.0.0:4560, waiting for PX4...");

    let mut last_hb = -1.0;
    let mut last_connected = false;
    let mut last_stats = Instant::now();
    let mut step_count = 0u64;
    let mut actuator_count = 0u64;
    let mut motors = [0.0; 4];
    loop {
        let step_start = Instant::now();
        let s = fdm.snapshot();
        if s.time - last_hb >= 1.0 {
            if let Err(e) = link.send_heartbeat() {
                eprintln!("heartbeat send error: {e}");
            }
            last_hb = s.time;
        }
        let frame = sensors.sample(dt, &s);
        if let Err(e) = link.send_sensors(&frame, &s, &origin) {
            eprintln!("sensor send error: {e}");
        }
        match link.poll_actuators() {
            Ok(true) => {
                actuator_count += 1;
                if let Some(outputs) = link.take_motors() {
                    motors = outputs;
                }
            }
            Ok(false) => {}
            Err(e) => eprintln!("actuator receive error: {e}"),
        }

        let connected = link.transport.is_connected();
        if connected != last_connected {
            println!(
                "PX4 TCP {}",
                if connected {
                    "connected"
                } else {
                    "disconnected"
                }
            );
            last_connected = connected;
            if !connected {
                // Do not keep driving the aircraft with stale outputs after PX4 disconnects.
                motors = [0.0; 4];
            }
        }

        fdm.step(dt, &motors);
        step_count += 1;
        if last_stats.elapsed() >= Duration::from_secs(1) {
            println!(
                "SITL {:.3}s: {:.0} sensor steps/s, {} actuator frames/s, connected={}",
                fdm.time(),
                step_count as f64 / last_stats.elapsed().as_secs_f64(),
                actuator_count,
                connected
            );
            step_count = 0;
            actuator_count = 0;
            last_stats = Instant::now();
        }

        let elapsed = step_start.elapsed();
        if elapsed < step_period {
            std::thread::sleep(step_period - elapsed);
        }
    }
}
