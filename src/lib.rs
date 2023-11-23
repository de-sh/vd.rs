use std::f64::consts::PI;

const BASE_RPM: f64 = 750.0;
const MAX_RPM: f64 = 5000.0;
const WHEEL_RADIUS: f64 = 0.4; // in m
const WHEEL_CIRCUMFERENCE: f64 = 2.0 * PI * WHEEL_RADIUS; // in m
const SPEED_FACTOR: f64 = WHEEL_CIRCUMFERENCE * 0.006; // RPM to kmph formulation
const SPEED_ALPHA: f64 = 0.7;

#[derive(Debug, Default)]
pub enum Gear {
    #[default]
    Neutral,
    First,
    Second,
    Third,
    Fourth,
    Fifth,
    Reverse,
}

#[derive(Debug, Default)]
pub enum HandBrake {
    Disengaged,
    Half,
    #[default]
    Full,
}

#[derive(Debug, Default)]
pub struct Car {
    instantaneous_speeds: Vec<f64>,
    speed: f64,
    engine_rpm: u32,
    transmission_rpm: f64,
    gear: Gear,
    accelerator_position: f64,
    brake_position: f64,
    clutch_position: f64,
    hand_brake: HandBrake,
}

impl Car {
    pub fn shift_gear(&mut self, gear: Gear) {
        self.gear = gear;
    }

    pub fn gear(&self) -> &Gear {
        &self.gear
    }

    pub fn set_accelerator_position(&mut self, position: f64) {
        self.accelerator_position = position;
    }

    pub fn accelerator_position(&self) -> f64 {
        self.accelerator_position
    }

    pub fn set_clutch_position(&mut self, position: f64) {
        self.clutch_position = position;
    }

    pub fn clutch_position(&self) -> f64 {
        self.clutch_position
    }

    pub fn set_brake_position(&mut self, position: f64) {
        self.brake_position = position;
    }

    pub fn brake_position(&self) -> f64 {
        self.brake_position
    }

    pub fn set_handbrake_position(&mut self, position: HandBrake) {
        self.hand_brake = position;
    }

    pub fn hand_brake(&self) -> &HandBrake {
        &self.hand_brake
    }

    fn transmission_ratio(&self) -> f64 {
        match self.gear {
            Gear::Reverse => -0.75,
            Gear::Neutral => 0.0,
            Gear::First => 0.75,
            Gear::Second => 1.25,
            Gear::Third => 1.75,
            Gear::Fourth => 2.25,
            Gear::Fifth => 3.0,
        }
    }

    fn update_rpm(&mut self) {
        let rpm = BASE_RPM + (MAX_RPM - BASE_RPM) * self.accelerator_position;
        self.engine_rpm = rpm as u32;
        self.transmission_rpm = if self.clutch_position <= 0.5 {
            rpm / self.transmission_ratio() // above biting point
        } else {
            0.0 // Transmission is disconnected
        };
    }

    fn update_instataneous_speed(&mut self) {
        let speed = self.transmission_rpm * SPEED_FACTOR;
        self.instantaneous_speeds.push(speed);
    }

    pub fn update(&mut self) {
        self.update_rpm();
        self.update_instataneous_speed();

        let initial_speed = self.instantaneous_speeds[0];

        let speed = exponential_moving_average(&self.instantaneous_speeds, SPEED_ALPHA);

        // To ensure we are working with only a small window of values. Here that is 2 values,
        // so we also reverse and store the ema value at the start to give us better result with the next round
        self.instantaneous_speeds.resize_with(2, || initial_speed);
        self.instantaneous_speeds.reverse();
        self.instantaneous_speeds[0] = speed;

        self.speed = speed;
    }

    pub fn speed(&self) -> f64 {
        self.speed
    }
}

// Consider the vehicle's instantaneous speeds were: [15.2, 60.4]
// We need to ensure that the instantaneous speeds are a bit more realistic,
// so we use the exponential moving average(alpha = 0.7): 46.84
fn exponential_moving_average(instantaneous_values: &[f64], alpha: f64) -> f64 {
    let mut instantaneous_values = instantaneous_values.iter();
    let mut last_value = *instantaneous_values.next().unwrap();
    for value in instantaneous_values {
        last_value = alpha * value + (1.0 - alpha) * last_value;
    }

    last_value
}
