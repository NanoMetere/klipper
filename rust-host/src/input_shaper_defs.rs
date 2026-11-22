// SPDX-License-Identifier: GPL-3.0-only
//! Built-in input shaper definitions adapted from klippy/extras/shaper_defs.py.
//! Upstream copyright (C) 2020-2026 Dmitry Butyugin <dmbutyugin@google.com>.

use crate::{
    config::Config,
    input_shaper::{Error, Pulses, Shaper},
};
use std::f64::consts::PI;

const SECTION: &str = "input_shaper";

pub fn from_config(config: &Config) -> Result<Option<Shaper>, &'static str> {
    if config.values(SECTION).is_none() {
        return Ok(None);
    }
    let default = config.get(SECTION, "shaper_type").unwrap_or("mzv");
    let mut any = false;
    let mut axes: [Pulses; 3] = std::array::from_fn(|_| Pulses::default());
    for (slot, (t, f, d)) in axes.iter_mut().zip([
        ("shaper_type_x", "shaper_freq_x", "damping_ratio_x"),
        ("shaper_type_y", "shaper_freq_y", "damping_ratio_y"),
        ("shaper_type_z", "shaper_freq_z", "damping_ratio_z"),
    ]) {
        let name = config.get(SECTION, t).unwrap_or(default);
        let max_damping = match name {
            "zv" | "zvd" | "mzv" => 0.99,
            "ei" => 0.4,
            "2hump_ei" => 0.3,
            "3hump_ei" => 0.2,
            _ => return Err(t),
        };
        let read = |key, minimum, maximum, fallback| {
            config
                .get(SECTION, key)
                .map_or(Some(fallback), |x| x.parse::<f64>().ok())
                .filter(|x| (minimum..=maximum).contains(x))
                .ok_or(key)
        };
        let freq = read(f, 0.0, 1e6, 0.0)?;
        let damping = read(d, 0.0, max_damping, 0.1)?;
        if freq > 0.0 {
            *slot = builtin(name, freq, damping).map_err(|_| f)?;
            any = true;
        }
    }
    Ok(any.then_some(Shaper::new(axes)))
}

fn builtin(name: &str, frequency: f64, dr: f64) -> Result<Pulses, Error> {
    let df = (1.0 - dr * dr).sqrt();
    let td = 1.0 / (frequency * df);
    match name {
        "zv" => {
            let k = (-dr * PI / df).exp();
            Pulses::new(&[(0.0, 1.0), (0.5 * td, k)])
        }
        "zvd" => {
            let k = (-dr * PI / df).exp();
            Pulses::new(&[(0.0, 1.0), (0.5 * td, 2.0 * k), (td, k * k)])
        }
        "mzv" => {
            // Closed solution for stock's default n=3, t=0.75. Parameterized
            // forms are outside this static configuration subset.
            let k = (-2.0 * 0.75 * dr * PI / (2.0 * df)).exp();
            let side = 0.292_893_218_813_452_4;
            let middle = 0.414_213_562_373_095_15;
            Pulses::new(&[
                (0.0, side),
                (0.375 * td, middle * k),
                (0.75 * td, side * (k * k)),
            ])
        }
        "ei" => {
            let tol = 1.0 / 20.0;
            let a1 = (0.24968 + 0.24961 * tol)
                + ((0.80008 + 1.23328 * tol) + (0.49599 + 3.17316 * tol) * dr) * dr;
            let a3 = (0.25149 + 0.21474 * tol)
                + ((-0.83249 + 1.41498 * tol) + (0.85181 - 4.90094 * tol) * dr) * dr;
            let a2 = 1.0 - a1 - a3;
            let t2 = 0.4999
                + (((0.46159 + 8.57843 * tol) * tol)
                    + (((4.26169 - 108.644 * tol) * tol) + ((1.75601 + 336.989 * tol) * tol) * dr)
                        * dr)
                    * dr;
            Pulses::new(&[(0.0, a1), (t2 * td, a2), (td, a3)])
        }
        "2hump_ei" => hump(
            frequency,
            dr,
            [
                [0., 0., 0., 0.],
                [0.49890, 0.16270, -0.54262, 6.16180],
                [0.99748, 0.18382, -1.58270, 8.17120],
                [1.49920, -0.09297, -0.28338, 1.85710],
            ],
            [
                [0.16054, 0.76699, 2.26560, -1.22750],
                [0.33911, 0.45081, -2.58080, 1.73650],
                [0.34089, -0.61533, -0.68765, 0.42261],
                [0.15997, -0.60246, 1.00280, -0.93145],
            ],
        ),
        "3hump_ei" => hump(
            frequency,
            dr,
            [
                [0., 0., 0., 0.],
                [0.49974, 0.23834, 0.44559, 12.4720],
                [0.99849, 0.29808, -2.36460, 23.3990],
                [1.49870, 0.10306, -2.01390, 17.0320],
                [1.99960, -0.28231, 0.61536, 5.40450],
            ],
            [
                [0.11275, 0.76632, 3.29160, -1.44380],
                [0.23698, 0.61164, -2.57850, 4.85220],
                [0.30008, -0.19062, -2.14560, 0.13744],
                [0.23775, -0.73297, 0.46885, -2.08650],
                [0.11244, -0.45439, 0.96382, -1.46000],
            ],
        ),
        _ => Err(Error::Parameters),
    }
}

fn hump<const N: usize>(
    frequency: f64,
    damping: f64,
    times: [[f64; 4]; N],
    amps: [[f64; 4]; N],
) -> Result<Pulses, Error> {
    let tau = 1.0 / frequency;
    let poly = |[a, b, c, d]: [f64; 4]| ((d * damping + c) * damping + b) * damping + a;
    Pulses::new(&std::array::from_fn::<_, N, _>(|i| {
        (tau * poly(times[i]), poly(amps[i]))
    }))
}
