//! Time primitives. Re-exports of `std::time` for now, but kept in a
//! single module so we can swap in a deterministic monotonic clock for
//! simulation later without touching call sites.

pub use std::time::{Duration, Instant};
