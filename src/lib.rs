use std::f64::consts::PI;

use serde::Serialize;

const BASE_RPM: f64 = 750.0;
const MAX_RPM: f64 = 5000.0;
const WHEEL_RADIUS: f64 = 0.4; // in m
const WHEEL_CIRCUMFERENCE: f64 = 2.0 * PI * WHEEL_RADIUS; // in m
const SPEED_FACTOR: f64 = WHEEL_CIRCUMFERENCE * 0.006; // RPM to kmph formulation
const SPEED_ALPHA: f64 = 0.5;
const BRAKING_ALPHA: f64 = 0.5;
const MAX_POWER: f64 = 100.0; // kW
const MAX_TORQUE: f64 = 200.0; // Nm
const BSFC: f64 = 180.0; // g/kWh

#[derive(Debug, Default, PartialEq, Serialize)]
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

#[derive(Debug, Default, PartialEq, Serialize)]
pub enum HandBrake {
    Disengaged,
    Half,
    #[default]
    Full,
}

impl HandBrake {
    fn effect(&self) -> Option<f64> {
        match self {
            HandBrake::Disengaged => None,
            HandBrake::Half => Some(0.75),
            HandBrake::Full => Some(1.0),
        }
    }
}

#[derive(Debug, Default)]
pub struct Car {
    instantaneous_speeds: Vec<f64>,
    instantaneous_braking: Vec<f64>,
    /// effective value after brake has been applied
    effective_braking: f64,
    speed: f64,
    engine_rpm: u32,
    transmission_rpm: f64,
    gear: Gear,
    accelerator_position: f64,
    brake_position: f64,
    clutch_position: f64,
    hand_brake: HandBrake,
    fuel_level: f64,
    ignition: bool,
}

impl Car {
    pub fn new(fuel_level: f64) -> Self {
        Self { fuel_level, ..Default::default() }
    }

    pub fn shift_gear(&mut self, gear: Gear) {
        self.gear = gear;
    }

    pub fn gear(&self) -> &Gear {
        &self.gear
    }

    pub fn set_accelerator_position(&mut self, position: f64) {
        self.accelerator_position = position;
        self.brake_position = 0.0;
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

    pub fn smooth_braking(&mut self) -> f64 {
        self.instantaneous_braking.reverse();
        self.instantaneous_braking.resize_with(2, || 0.0);
        self.instantaneous_braking =
            exponential_moving_average(&self.instantaneous_braking, BRAKING_ALPHA);
        self.instantaneous_braking.reverse();

        self.instantaneous_braking[0]
    }

    pub fn set_brake_position(&mut self, position: f64) {
        self.brake_position = position;
        self.accelerator_position = 0.0;
    }

    pub fn update_braking(&mut self) {
        let mut braking = self.brake_position;

        // Take into account effect of handbrake
        if let Some(effect) = self.hand_brake.effect() {
            braking = effect.max(braking)
        }

        if braking > 0.1 {
            self.instantaneous_braking.push(braking);
        }

        self.effective_braking = if braking > 0.0 {
            self.smooth_braking()
        } else {
            self.instantaneous_braking = vec![0.0];
            0.0
        };
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
            Gear::Reverse => -0.10,
            Gear::Neutral => 0.0,
            Gear::First => 0.30,
            Gear::Second => 0.50,
            Gear::Third => 0.80,
            Gear::Fourth => 1.0,
            Gear::Fifth => 1.40,
        }
    }

    fn update_rpm(&mut self) {
        let rpm = if self.fuel_level > 0.0 && self.ignition {
            BASE_RPM + (MAX_RPM - BASE_RPM) * self.accelerator_position
        } else {
            0.0 // Car has no fuel to burn or ignition is off
        };
        self.engine_rpm = rpm as u32;
        self.transmission_rpm = if self.clutch_position <= 0.5 {
            rpm * self.transmission_ratio() // above biting point
        } else {
            0.0 // Transmission is disconnected
        };
    }

    pub fn rpm(&self) -> u32 {
        self.engine_rpm
    }

    fn smooth_speed(&mut self) -> f64 {
        let initial_speed = self.instantaneous_speeds[0];

        let speeds = exponential_moving_average(&self.instantaneous_speeds, SPEED_ALPHA);
        let speed = speeds.last().unwrap();

        // To ensure we are working with only a small window of values. Here that is 2 values,
        // so we also reverse and store the ema value at the start to give us better result with the next round
        self.instantaneous_speeds.resize_with(2, || initial_speed);
        self.instantaneous_speeds.reverse();
        self.instantaneous_speeds[0] = *speed;
        *speed
    }

    fn update_speed(&mut self) {
        // Don't change speed much if clutch engaged or ignition turned off
        if self.clutch_position > 0.5 || !self.ignition {
            self.speed *= 0.97 - self.effective_braking; // decrease speed by a small factor(0.03) anyways to emulate road resistence
            return;
        }
        self.speed = if self.accelerator_position == 0.0
            && (self.speed < 3.0 || self.effective_braking > 0.75)
        {
            0.0
        } else {
            let speed = self.transmission_rpm * SPEED_FACTOR * (1.0 - self.effective_braking);

            self.instantaneous_speeds.push(speed);
            self.smooth_speed()
        };
    }

    pub fn speed(&self) -> f64 {
        self.speed
    }

    pub fn update_fuel(&mut self) {
        let power_output = self.engine_rpm as f64 * MAX_TORQUE * (2.0 * PI) / (60.0 * 1000.0);
        let power_output = power_output.min(MAX_POWER) * 5.0 / self.transmission_ratio();
        let fuel_consumption = power_output * BSFC;
        self.fuel_level -= fuel_consumption * 10_f64.powi(-10);
        self.fuel_level = self.fuel_level.max(0.0);
    }

    pub fn refuel(&mut self, fuel_level: f64) {
        self.fuel_level += fuel_level;
        self.fuel_level %= 1.0; // Max fuel level is 100%, i.e. 1.0
    }

    pub fn fuel_level(&self) -> f64 {
        self.fuel_level
    }

    pub fn update(&mut self) {
        self.update_rpm();
        self.update_braking();
        self.update_speed();
        self.update_fuel();
    }

    pub fn turn_key(&mut self, ignition: bool) {
        self.ignition = ignition;
    }

    pub fn ignition(&self) -> bool {
        self.ignition
    }
}

// Consider the vehicle's instantaneous speeds were: [15.2, 60.4]
// We need to ensure that the instantaneous speeds are a bit more realistic,
// so we use the exponential moving average(alpha = 0.7): 46.84
fn exponential_moving_average(instantaneous_values: &[f64], alpha: f64) -> Vec<f64> {
    let mut instantaneous_values = instantaneous_values.iter();
    let mut last_value = *instantaneous_values.next().unwrap();
    let mut ema = vec![last_value];
    for value in instantaneous_values {
        last_value = alpha * value + (1.0 - alpha) * last_value;
        ema.push(last_value);
    }

    ema
}
