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
    motor_rpm: u32,
    transmission_rpm: f64,
    accelerator_position: f64,
    brake_position: f64,
    hand_brake: HandBrake,
    distance_travelled: f64,
    energy_consumed: f64,
    soc: f64,
    soh: f64,
    ignition: bool,
    status: String,
}

impl Car {
    pub fn new(soc: f64, soh: f64) -> Self {
        Self { soc, soh, ..Default::default() }
    }

    pub fn set_status(&mut self, status: &str) {
        self.status = status.to_owned();
    }

    pub fn get_status(&self) -> &str {
        &self.status
    }

    pub fn soh(&self) -> f64 {
        self.soh
    }

    pub fn distance_travelled(&self) -> f64 {
        self.distance_travelled
    }

    pub fn energy_consumed(&self) -> f64 {
        self.energy_consumed
    }

    pub fn set_accelerator_position(&mut self, position: f64) {
        self.accelerator_position = position;
        self.brake_position = 0.0;
    }

    pub fn accelerator_position(&self) -> f64 {
        self.accelerator_position
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

    fn update_rpm(&mut self) {
        let rpm = if self.soc > 0.0 && self.ignition {
            BASE_RPM + (MAX_RPM - BASE_RPM) * self.accelerator_position
        } else {
            0.0 // Car has no fuel to burn or ignition is off
        };
        self.motor_rpm = rpm as u32;
        self.transmission_rpm = rpm;
    }

    pub fn rpm(&self) -> u32 {
        self.motor_rpm
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
        // Don't change speed much if ignition turned off
        if !self.ignition {
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

        self.distance_travelled += self.speed / 3600.0;

        // low charge driving affects health
        if self.soc() < 0.1 {
            self.soh -= 2.0_f64.powi(-8);
        }
    }

    pub fn speed(&self) -> f64 {
        self.speed
    }

    pub fn update_charge(&mut self) {
        let power_output = self.motor_rpm as f64 * MAX_TORQUE * (2.0 * PI) / (60.0 * 1000.0);
        let charge_consumption = power_output.min(MAX_POWER) * 5.0 / 3600.0;
        self.energy_consumed += charge_consumption;

        self.soc -= charge_consumption * 10_f64.powi(-10);
        self.soc = self.soc.max(0.0);
    }

    pub fn charge(&mut self, charge: f64) {
        // fast charging can also ruin health
        if charge > 0.02 {
            self.soh -= 2.0_f64.powi(-8);
        }
        self.soc += charge;
        self.soc %= self.soh; // Max fuel level is only upto SoH
    }

    pub fn soc(&self) -> f64 {
        self.soc
    }

    pub fn update(&mut self) {
        self.update_rpm();
        self.update_braking();
        self.update_speed();
        self.update_charge();
    }

    pub fn turn_key(&mut self, ignition: bool) {
        self.status = if ignition { "Running" } else { "Stopped" }.to_owned();
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
