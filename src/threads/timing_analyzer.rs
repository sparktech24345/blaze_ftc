use std::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;
use crate::MovingAverage;

pub struct TimingAnalyzer  {
    min: AtomicU32,
    avg: AtomicU32,
    max: AtomicU32,
    rolling_avg: Mutex<MovingAverage<u32>>
}
impl TimingAnalyzer {
    pub fn new() -> TimingAnalyzer {
        TimingAnalyzer {
            min: AtomicU32::new(u32::MAX),
            avg: AtomicU32::new(0),
            max: AtomicU32::new(0),
            rolling_avg: Mutex::new(MovingAverage::new(500)),
        }
    }
    ///everything is stored in micros (to_text prints millis). the moving average holds 500 samples in a u32,
    ///so each sample is capped at 1 s to keep the sum from overflowing
    pub fn update(&self, dur: Duration) {
        let micros = u32::try_from(dur.as_micros()).unwrap_or(u32::MAX);
        let mut ravg = self.rolling_avg.lock().unwrap();
        ravg.update(micros.min(1_000_000));
        self.avg.store(ravg.avg(), Ordering::SeqCst);
        self.min.fetch_min(micros, Ordering::SeqCst);
        self.max.fetch_max(micros, Ordering::SeqCst);
    }
    pub fn to_text(&self) -> String {
        let min = (self.min.load(Ordering::SeqCst) as f64) / 1000.0;
        let max = (self.max.load(Ordering::SeqCst) as f64) / 1000.0;
        let avg = (self.avg.load(Ordering::SeqCst) as f64) / 1000.0;
        format!("min: {}, avg: {}, max: {}", min, avg, max)
    }
}
