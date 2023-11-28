use std::time::{Duration, Instant};

use rand::Rng;
use tokio::time::interval;
use vehicle_dynamics::{Car, Gear, HandBrake};

#[tokio::main]
async fn main() {
    let mut rng = rand::thread_rng();
    let mut distance_travelled = 0.0;
    let mut avg_speed = 0.0;

    let mut car = Car::new(rng.gen_range(0.0..1.0));
    car.set_handbrake_position(HandBrake::Disengaged);
    car.set_clutch_position(1.0);
    car.shift_gear(Gear::First);
    car.set_clutch_position(0.5);
    car.set_accelerator_position(0.5);
    car.set_clutch_position(0.0);

    let mut interval = interval(Duration::from_secs(1));
    let mut refuelling = None;

    loop {
        car.update();
        display(&car);
        distance_travelled += car.speed() / 3600.0;
        avg_speed = (avg_speed + car.speed()) * 0.5;
        println!("Distance travelled: {}", distance_travelled);
        println!("Average speed: {}", avg_speed);
        interval.tick().await;

        // Stop for refuelling, slowly get into the gas station
        if car.fuel_level() < 0.25 && car.speed() != 0.0 {
            car.set_clutch_position(rng.gen_range(0.3..0.9));
            car.set_brake_position(rng.gen_range(0.3..0.9));
            continue;
        }
        // Start refuelling
        if car.fuel_level() < 0.25 && car.speed() == 0.0 && refuelling.is_none() {
            car.set_handbrake_position(HandBrake::Full);
            car.shift_gear(Gear::Neutral);
            car.set_clutch_position(0.0);
            car.set_brake_position(0.0);

            refuelling = Some(
                // Time during which car is stationary at the refuelling point: between 7.5-17.5 minutes
                Instant::now() + Duration::from_secs_f32(300.0 + 60.0 * rng.gen_range(2.5..12.5)),
            );
        }

        if let Some(till) = refuelling {
            if till < Instant::now() {
                refuelling.take();
                car.set_handbrake_position(HandBrake::Disengaged);
                continue;
            }
            car.refuel(0.001);
            continue;
        }

        if rng.gen_bool(0.05) && car.rpm() > 2500 || car.rpm() > 3500 || car.rpm() < 1250 {
            shift_gears(&mut car, rng.gen_range(0.25..1.0));
        } else {
            car.set_clutch_position(0.0);
        }

        // very few times, press the brake to slow down, else remove
        if rng.gen_bool(0.05) || car.brake_position() > 0.5 {
            car.set_brake_position(rng.gen_range(0.3..1.0));
            continue;
        } else {
            car.set_brake_position(0.0);
        }

        // even fewer times, engage hand brake to slow down instantly, or else do the opposite
        if rng.gen_bool(0.005) {
            if rng.gen_bool(0.25) || car.hand_brake() == &HandBrake::Half {
                car.set_handbrake_position(HandBrake::Full);
                continue;
            } else {
                car.set_handbrake_position(HandBrake::Half);
            }
        } else if car.hand_brake() != &HandBrake::Disengaged {
            car.set_handbrake_position(
                if rng.gen_bool(0.25) || car.hand_brake() == &HandBrake::Full {
                    HandBrake::Half
                } else {
                    HandBrake::Disengaged
                },
            );
        }

        car.set_accelerator_position(rng.gen_range(0.25..1.0));
    }
}

fn shift_gears(car: &mut Car, clutch_position: f64) {
    let clutch_gear_combo = |car: &mut Car, gear| {
        car.set_clutch_position(clutch_position);
        car.shift_gear(gear);
    };
    match car.gear() {
        Gear::Reverse => clutch_gear_combo(car, Gear::Neutral),
        Gear::Neutral => {
            if car.clutch_position() > 0.5 {
                clutch_gear_combo(car, Gear::First)
            }
        }
        Gear::First => {
            if car.rpm() > 2500 && car.speed() > 10.0 {
                clutch_gear_combo(car, Gear::Second)
            }
        }
        Gear::Second => match car.speed() as u8 {
            0..=10 => clutch_gear_combo(car, Gear::First),
            s if s > 25 && car.rpm() > 3000 => clutch_gear_combo(car, Gear::Third),
            _ => {}
        },
        Gear::Third => match car.speed() as u8 {
            0..=10 => clutch_gear_combo(car, Gear::First),
            11..=20 => clutch_gear_combo(car, Gear::Second),
            s if s > 50 && car.rpm() > 3500 => clutch_gear_combo(car, Gear::Fourth),
            _ => {}
        },
        Gear::Fourth => match car.speed() as u8 {
            0..=10 => clutch_gear_combo(car, Gear::First),
            11..=20 => clutch_gear_combo(car, Gear::Second),
            21..=40 => clutch_gear_combo(car, Gear::Third),
            s if s > 80 && car.rpm() > 4000 => clutch_gear_combo(car, Gear::Fifth),
            _ => {}
        },
        Gear::Fifth => match car.speed() as u8 {
            0..=10 => clutch_gear_combo(car, Gear::First),
            11..=20 => clutch_gear_combo(car, Gear::Second),
            21..=40 => clutch_gear_combo(car, Gear::Third),
            41..=70 => clutch_gear_combo(car, Gear::Fourth),
            _ => {}
        },
    }
}

fn display(car: &Car) {
    println!("\t----");
    println!("Speed: {}", car.speed());
    println!("Fuel: {:?}", car.fuel_level() * 40.0);
    println!("Gear: {:?}", car.gear());
    println!("RPM: {}", car.rpm());
    println!("Accelerator: {}", car.accelerator_position());
    println!("Brake: {:0.2}", car.brake_position());
    println!("Clutch: {:0.2}", car.clutch_position());
    println!("Hand brake: {:?}", car.hand_brake());
}
