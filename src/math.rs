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

/// f(t) = e^(-a*t) * (amp_cos*cos(wd*t) + amp_sin*sin(wd*t)) + c0
///
/// Models the underdamped RLC step response where `a = R/(2L)` is the
/// damping coefficient and `wd = sqrt(1/(LC) - a²)` is the damped natural
/// frequency.  `c0` is only non-zero in the antiderivative returned by
/// `integral()`; the primary current function always has c0 = 0.
pub struct DampedSineF {
    a: T,
    wd: T,
    amp_cos: T,
    amp_sin: T,
    c0: T,
}

impl fmt::Debug for DampedSineF {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "f(t) = e^(-{:.4}*t) * ({:.4}*cos({:.4}*t) + {:.4}*sin({:.4}*t)) + {:.4}",
            self.a, self.amp_cos, self.wd, self.amp_sin, self.wd, self.c0
        )
    }
}

impl Func for DampedSineF {
    type Derivetive = Self;
    type Integral = Self;

    fn f(&self, x: T) -> T {
        T::exp(-self.a * x)
            * (self.amp_cos * T::cos(self.wd * x) + self.amp_sin * T::sin(self.wd * x))
            + self.c0
    }

    /// d/dt [e^(-a*t)*(A*cos(wd*t) + B*sin(wd*t))]
    ///   = e^(-a*t) * [(-a*A + wd*B)*cos(wd*t) + (-a*B - wd*A)*sin(wd*t)]
    fn derivative(&self) -> Self {
        Self {
            a: self.a,
            wd: self.wd,
            amp_cos: -self.a * self.amp_cos + self.wd * self.amp_sin,
            amp_sin: -self.a * self.amp_sin - self.wd * self.amp_cos,
            c0: 0.0,
        }
    }

    /// Returns the antiderivative F such that F(0) = a0.
    ///
    /// ∫ e^(-a*t)*(A*cos(wd*t) + B*sin(wd*t)) dt
    ///   = e^(-a*t)/denom * [(-a*A - wd*B)*cos(wd*t) + (wd*A - a*B)*sin(wd*t)] + C
    /// where denom = a²+wd².  C is chosen so that F(0) = a0.
    fn integral(&self, a0: T) -> Self {
        assert_eq!(
            self.c0, 0.0,
            "integral() only supported on the primary current function (c0 must be 0)"
        );
        let denom = self.a * self.a + self.wd * self.wd;
        let new_amp_cos = (-self.a * self.amp_cos - self.wd * self.amp_sin) / denom;
        let new_amp_sin = (self.wd * self.amp_cos - self.a * self.amp_sin) / denom;
        // F(0) = new_amp_cos + c0 = a0  =>  c0 = a0 - new_amp_cos
        Self {
            a: self.a,
            wd: self.wd,
            amp_cos: new_amp_cos,
            amp_sin: new_amp_sin,
            c0: a0 - new_amp_cos,
        }
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
) -> DampedSineF {
    // KVL: v_in = R*i + L*di/dt + v_c
    // Differentiating w.r.t. t (v_in constant):
    //   L*d²i/dt² + R*di/dt + i/C = 0
    // Characteristic equation: s² + (R/L)*s + 1/(LC) = 0
    //   s = -α ± sqrt(α² - ω₀²),  α = R/(2L),  ω₀ = 1/sqrt(LC)
    let a = r.0 / (2.0 * l.0);
    let ohmega = 1.0 / T::sqrt(l.0 * c.0);

    match a.partial_cmp(&ohmega).unwrap() {
        std::cmp::Ordering::Less => {
            // Underdamped: roots are complex  s = -α ± j·ωd
            // where ωd = sqrt(ω₀² - α²)
            let wd = T::sqrt(ohmega * ohmega - a * a);

            // KVL at t=0: di/dt(0) = (v_in - R*i(0) - v_c(0)) / L
            let di_dt0 = (v_in.0 - r.0 * i_old.0 - v_cout_old.0) / l.0;

            // i(t) = e^(-α*t) * (A*cos(ωd*t) + B*sin(ωd*t))
            // i(0)  = A             = i_old
            // i'(0) = -α*A + ωd*B  = di_dt0  =>  B = (di_dt0 + α*i_old) / ωd
            let amp_cos = i_old.0;
            let amp_sin = (di_dt0 + a * i_old.0) / wd;

            DampedSineF { a, wd, amp_cos, amp_sin, c0: 0.0 }
        }
        std::cmp::Ordering::Equal => todo!("Critically damped response"),
        std::cmp::Ordering::Greater => todo!("Overdamped response"),
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
