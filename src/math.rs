use core::fmt;
use std::iter;

use crate::{Capacitance, Current, Inductance, Resistance, T, Voltage};

pub trait Func: fmt::Debug {
    type Derivetive: Func;
    type Integral: Func;

    fn f(&self, x: T) -> T;
    fn derivative(&self) -> Self::Derivetive;
    fn integral(&self, a0: T) -> Self::Integral;

    fn intersects_at(&self, rhs: impl Func, guess: T) -> Option<T> {
        let f = |x| x - (self.f(x) - rhs.f(x)) / (self.derivative().f(x) - rhs.derivative().f(x));

        if guess.is_nan() {
            panic!("bad guess, f: {self:?} rhs: {rhs:?}");
        }
        let mut x = guess;
        let mut last = guess;
        for i in 0..10 {
            //println!("i: {i}, x: {x}");

            x = f(last);

            if x.is_nan() {
                //panic!("f: {self:?} rhs: {rhs:?}, i: {i}, x: {x}");
                println!("f: {self:?} rhs: {rhs:?}, i: {i}, x: {x}");
                return Some(last)
            }

            last = x;
        }

        Some(x)
    }
}

/// f(x) = k1*e^(s1*x) + k2*e^(s2*x)
pub struct WonkyF {
    k1: T,
    k2: T,

    s1: T,
    s2: T,
}

impl fmt::Debug for WonkyF {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "f(x) = {:?}*e^({:?}*x) + {:?}*e^({:?}*x)",
            self.k1, self.s1, self.k2, self.s2
        )
    }
}

// https://www.youtube.com/watch?v=m27OkXwBbuk
pub fn rlc(
    v_in: Voltage,
    v_cout_old: Voltage,
    i_old: Current,
    l: Inductance,
    c: Capacitance,
    r: Resistance,
) -> WonkyF {
    let q0 = c.0 * v_cout_old.0;

    // v_in = r * i + l * di/dt + q/c
    // d_q/d_t = i
    //
    // Differentiating w.r.t. `t` where v_in is constant:
    // r * d_i/d_t + l * d2_i / d2_t + i / c = 0
    // <=> (divide by `l`)
    // d2_i / d2_t + (r * di)/(l * dt) + i / (l * c) = 0
    //
    // Characteristic equation:
    // s^2 + r/l*s + 1 / (l*c) = 0
    //
    // s1 and s2 are the roots
    //
    // sq_root = sqrt((r/2l)^2 - 1/(l*c))
    // s1 = -r/(2*l) + sqrt((r/2l)^2 - 1/(l*c))
    // s2 = -r/(2*l) - sqrt((r/2l)^2 - 1/(l*c))
    //
    // s1 = -a + sqrt(a^2-ohmega^2) = -a + sq_root
    // s2 = -a - sqrt(a^2-ohmega^2) = -a - sq_root
    //
    let a = r.0 / (2.0 * l.0);
    let ohmega = 1.0 / T::sqrt(l.0 * c.0);
    let b = T::sqrt(a * a - ohmega * ohmega);
    //
    // i(t) = k1 * T::exp(s1*t) + k2 * T::exp(s2*t)

    match a.partial_cmp(&ohmega).unwrap() {
        std::cmp::Ordering::Less => {
            // Underdamped response
            // t > 0
            let s1 = -a + T::sqrt(a * a + ohmega * ohmega);
            let s2 = -a - T::sqrt(a * a + ohmega * ohmega);
            //panic!("Underdamped response: {s1} {s2}");
            //let i = k1 * T::exp(s1 * t.0) + k2 * T::exp(s2 * t.0);
            //k1 + k2 = old_i;

            // u = r*i + l * di_dt + q0; // Adderar man initial spänning av cappen här?
            let di_dt0 = (v_in.0 - r.0 * i_old.0 - q0) / l.0; // Adderar man initial spänning av cappen här?
            //r*di_dt + l * d2i_d2t + i / c;

            //-----

            //let di_dt = s1*k1 * T::exp(s1 * t) + s2*k2 * T::exp(s2 * t);
            //k1 + k2 = i_old;
            //k1 = i_old - k2;

            //let di_dt0 = s1*(i_old - k2) * T::exp(s1 * t) + s2*k2 * T::exp(s2 * t);
            //let di_dt0 = s1*i_old - s1*k2 + s2 * k2;
            //let di_dt0 = s1*i_old + (s2 - s1) * k2;
            //let di_dt0 - s1*i_old =  (s2 - s1) * k2;
            let k2 = (di_dt0 - s1 * i_old.0) / (s2 - s1);
            let k1 = i_old.0 - k2;

            // k1 * T::exp(s1 * t) + k2 * T::exp(s2 * t) = k*t + m;

            // this seem to be the one we have...
            //let ohmega_d = T::sqrt(ohmega * ohmega - a * a);
            //let i = T::exp(-a * t) * ((k1 + k2) * T::cos(ohmega_d * t) + j(k1 - k2) * sin(ohmega_d * t));

            WonkyF { k1, k2, s1, s2 }
        }
        std::cmp::Ordering::Equal => {
            todo!("Critically damped")
            // Critically damped
            // t > 0
            // let i =  T::exp(-a * t) * (k1 + k2 * t);
        }
        std::cmp::Ordering::Greater => {
            todo!("Overdamped response")
            // Overdamped
            // t > 0
            // let i = k1 * T::exp(s1 * t) + k2 * T::exp(s2 * t);
        }
    }
}

impl Func for WonkyF {
    type Derivetive = Self;
    type Integral = Self;

    fn f(&self, x: T) -> T {
        self.k1 * T::exp(self.s1 * x) + self.k2 * T::exp(self.s2 * x)
    }

    fn derivative(&self) -> WonkyF {
        WonkyF {
            k1: self.s1 * self.k1,
            k2: self.s2 * self.k2,

            s1: self.s1,
            s2: self.s2,
        }
    }

    fn integral(&self, a0: T) -> Self::Integral {
        assert_eq!(a0, 0.0, "Not implemented");
        WonkyF {
            k1: self.k1 / self.s1,
            k2: self.k2 / self.s2,

            s1: self.s1,
            s2: self.s2,
        }
    }
}

pub struct Line {
    pub k: T,
    pub m: T,
}

impl fmt::Debug for Line {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "f(t) = {}x + {}", self.k, self.m)
    }
}

impl Func for Line {
    type Derivetive = Self;
    type Integral = Polynomial;

    fn f(&self, x: T) -> T {
        self.k * x + self.m
    }

    fn derivative(&self) -> Self {
        Self { k: 0.0, m: self.k }
    }

    fn integral(&self, a0: T) -> Self::Integral {
        Polynomial {
            factors: vec![a0, self.m, self.k / 2.0],
        }
    }
}

/// f(x) = a0 + a1 * x + a2 * x^2 + ...
pub struct Polynomial {
    factors: Vec<T>,
}

impl fmt::Debug for Polynomial {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "f(x) = ")?;
        if let Some(a0) = self.factors.get(0) {
            write!(f, "{a0}")?;
        } else {
            write!(f, "0")?;
        }

        if let Some(a1) = self.factors.get(1) {
            write!(f, "{a1} * x")?;
        }

        for (i, a) in self
            .factors
            .iter()
            .enumerate()
            .skip(2)
            .filter(|(_, a)| **a != 0.0)
        {
            write!(f, "+ {a} * x^{i}")?;
        }

        Ok(())
    }
}

impl Func for Polynomial {
    type Derivetive = Self;
    type Integral = Self;

    fn f(&self, x: T) -> T {
        self.factors
            .iter()
            .enumerate()
            .map(|(i, &a)| a * x.powi(i as i32))
            .sum()
    }

    fn derivative(&self) -> Self::Derivetive {
        Self {
            factors: self
                .factors
                .iter()
                .skip(1)
                .enumerate()
                .map(|(i, &a)| a * (i + 1) as T)
                .collect(),
        }
    }

    fn integral(&self, a0: T) -> Self::Integral {
        Self {
            factors: iter::once(a0)
                .chain(
                    self.factors
                        .iter()
                        .copied()
                        .enumerate()
                        .map(|(i, a)| a / (i + 1) as T),
                )
                .collect(),
        }
    }
}
