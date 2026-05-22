//! # robotics-vision
//!
//! Object detection abstraction. The platform uses it to answer "is
//! there a cube at (x, y, z) ready to pick?" without committing to a
//! specific sensor or detection technique.
//!
//! ## What ships
//!
//! [`VirtualWorldDetector`] — reads object positions out of the
//! simulation backend. Trivially correct, no perception needed. This
//! is what the sim mode uses and what tests assert against.
//!
//! ## What slots in later
//!
//! * **OpenCV blob detection** — bind `opencv-rust`, expose a
//!   `CameraDetector`. Calibrate intrinsics + extrinsics so pixel
//!   blobs become 3D rays; intersect with the table plane to get
//!   object positions in base frame.
//! * **YOLO / Detectron2** — `tract` or `ort` for ONNX inference in
//!   Rust. Same `ObjectDetector` trait, different implementation.
//! * **Depth cameras** (RealSense, Kinect) — pull point clouds and
//!   run cluster segmentation. Trait stays the same.
//!
//! The trait commits to *base-frame coordinates*. That keeps the
//! camera calibration concern inside the detector and out of every
//! consumer.

use async_trait::async_trait;
use robotics_core::{Result, SensorReading, Vec3};
use robotics_simulation::SimulationBackend;
use tracing::debug;

/// Object detector trait. Anything that produces "there is an
/// object at (x, y, z)" results implements this.
#[async_trait]
pub trait ObjectDetector: Send + Sync {
    async fn detect(&mut self) -> Result<Vec<SensorReading>>;
}

/// Reads the simulator's ground-truth object list and returns it as
/// detections. Equivalent to a perfect camera. Used in sim mode and
/// in tests to isolate motion/planning bugs from perception bugs.
pub struct VirtualWorldDetector {
    sim: SimulationBackend,
}

impl VirtualWorldDetector {
    pub fn new(sim: SimulationBackend) -> Self {
        Self { sim }
    }
}

#[async_trait]
impl ObjectDetector for VirtualWorldDetector {
    async fn detect(&mut self) -> Result<Vec<SensorReading>> {
        let objects = self.sim.objects().await;
        let readings: Vec<SensorReading> = objects
            .into_iter()
            .map(|o| SensorReading::Object {
                id: o.id,
                position: Vec3 { x: o.position.x, y: o.position.y, z: o.position.z },
            })
            .collect();
        debug!(count = readings.len(), "virtual detector");
        Ok(readings)
    }
}
