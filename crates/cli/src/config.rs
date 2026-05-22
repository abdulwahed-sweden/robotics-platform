//! Config loading. Each config file is its own TOML document; we
//! don't merge them into one giant struct because they belong to
//! different concerns (kinematic geometry vs. sim parameters vs.
//! hardware wiring) and you typically version-control them
//! separately.

use std::fs;
use std::path::Path;

use anyhow::{Context, Result};
use robotics_core::Vec3;
use robotics_hardware::HardwareConfig;
use robotics_kinematics::ArmModel;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct SimConfig {
    pub objects: Vec<SimObject>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct SimObject {
    pub id: String,
    pub position: Vec3,
    pub grasp_radius: f64,
}

pub fn load_arm(path: &str) -> Result<ArmModel> {
    if !Path::new(path).exists() {
        // Use defaults so `cargo run -- simulate` works in a fresh
        // checkout. The CLI logs that defaults are in use.
        tracing::warn!(path, "arm config not found; using SG90 defaults");
        return Ok(ArmModel::sg90_default());
    }
    let raw = fs::read_to_string(path).with_context(|| format!("reading {path}"))?;
    let m: ArmModel = toml::from_str(&raw).with_context(|| format!("parsing {path}"))?;
    Ok(m)
}

pub fn load_hardware(path: &str) -> Result<HardwareConfig> {
    if !Path::new(path).exists() {
        tracing::warn!(path, "hardware config not found; using Pi4 defaults");
        return Ok(HardwareConfig::pi4_default());
    }
    let raw = fs::read_to_string(path).with_context(|| format!("reading {path}"))?;
    let c: HardwareConfig = toml::from_str(&raw).with_context(|| format!("parsing {path}"))?;
    Ok(c)
}

pub fn load_sim(path: &str) -> Result<SimConfig> {
    if !Path::new(path).exists() {
        tracing::warn!(path, "sim config not found; using defaults with one demo cube");
        return Ok(SimConfig {
            objects: vec![SimObject {
                id: "cube_a".into(),
                position: Vec3::new(0.12, 0.0, 0.05),
                grasp_radius: 0.03,
            }],
        });
    }
    let raw = fs::read_to_string(path).with_context(|| format!("reading {path}"))?;
    let c: SimConfig = toml::from_str(&raw).with_context(|| format!("parsing {path}"))?;
    Ok(c)
}
