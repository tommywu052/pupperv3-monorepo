use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Expression {
    Normal,
    Happy,
}

pub struct ExpressionState {
    current: Expression,
    /// Current animated squint value (0.0 = normal open, target = smile)
    squint: f32,
    target_squint: f32,
    last_switch: Instant,
    hold_duration: Duration,
}

impl ExpressionState {
    pub fn new() -> Self {
        Self {
            current: Expression::Normal,
            squint: 0.0,
            target_squint: 0.0,
            last_switch: Instant::now(),
            hold_duration: Self::random_hold(),
        }
    }

    pub fn update(&mut self) {
        if self.last_switch.elapsed() >= self.hold_duration {
            self.current = match self.current {
                Expression::Normal => Expression::Happy,
                Expression::Happy => Expression::Normal,
            };
            self.target_squint = match self.current {
                Expression::Normal => 0.0,
                Expression::Happy => 0.6,
            };
            self.last_switch = Instant::now();
            self.hold_duration = Self::random_hold();
        }

        let speed: f32 = 4.0;
        let dt: f32 = 1.0 / 60.0;
        self.squint += (self.target_squint - self.squint) * (speed * dt).min(1.0);
    }

    pub fn squint(&self) -> f32 {
        self.squint
    }

    /// Random hold time between 4–10 seconds
    fn random_hold() -> Duration {
        let secs = 4.0 + Self::pseudo_random() * 6.0;
        Duration::from_secs_f64(secs)
    }

    /// Simple pseudo-random using system clock nanoseconds (no extra dependency)
    fn pseudo_random() -> f64 {
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .subsec_nanos();
        (nanos % 10000) as f64 / 10000.0
    }
}
