//! Interfaces for Pulse Width Modulation output.

use returncode::ReturnCode;

/// PWM control for a single pin.
pub trait Pwm {
    /// The chip-dependent type of a PWM pin.
    type Pin;

    /// Generate a PWM single on the given pin at the given frequency and duty
    /// cycle.
    ///
    /// - `frequency_hz` is specified in Hertz.
    /// - `duty_cycle` is specified in hundredths of a percent. So setting
    ///   `duty_cycle` to 1 would mean a PWM output with a 0.01% duty cycle.
    ///   Passing 5000 would result in a 50% duty cycle.
    fn start(&self, pin: &Self::Pin, frequency_hz: usize, duty_cycle: usize) -> ReturnCode;

    /// Stop a PWM pin output.
    fn stop(&self, pin: &Self::Pin) -> ReturnCode;

    /// Return the maximum PWM frequency supported by the PWM implementation.
    /// The frequency will be specified in Hertz.
    fn get_maximum_frequency_hz(&self) -> usize;
}

/// Higher-level PWM interface that restricts the user to a specific PWM pin.
/// This is particularly useful for passing to capsules that need to control
/// only a specific pin.
pub trait PwmPin {
    /// Start a PWM output. Same as the `start` function in the `Pwm` trait.
    fn start(&self, frequency_hz: usize, duty_cycle: usize) -> ReturnCode;

    /// Stop a PWM output. Same as the `stop` function in the `Pwm` trait.
    fn stop(&self) -> ReturnCode;

    /// Return the maximum PWM frequency supported by the PWM implementation.
    /// Same as the `get_maximum_frequency_hz` function in the `Pwm` trait.
    fn get_maximum_frequency_hz(&self) -> usize;
}
