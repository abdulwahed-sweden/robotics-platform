//! # robotics-audit
//!
//! Every command issued through a remote surface (dashboard, MQTT
//! bridge, future REST API) lands here before it's allowed to affect
//! the world. The log is:
//!
//! * **Append-only.** Once written, an entry is not rewritten.
//! * **Self-describing.** Each line is a full JSON object including
//!   the command payload and the outcome — no foreign keys to resolve.
//! * **Deterministic-replay ready.** The `t` field is wall-clock UTC;
//!   the replay driver uses inter-entry gaps to reproduce the original
//!   cadence.
//!
//! ## Format
//!
//! ```jsonl
//! {"t":"2026-05-22T10:00:00.123Z","op":"dashboard","cmd":{"kind":"move","target":{"x":0.1,"y":0.0,"z":0.1},"approach_pitch":-1.57},"out":"accepted"}
//! {"t":"2026-05-22T10:00:03.456Z","op":"dashboard","cmd":{"kind":"emergency_stop"},"out":"estop"}
//! ```
//!
//! ## Durability trade-off
//!
//! Writes do *not* fsync. The OS buffers are flushed on close. If the
//! process is SIGKILLed mid-flight we may lose the last few entries.
//! For a true legal-grade audit log you would fsync per write (and
//! pay ~10 ms each) — that lands when this crate grows a Postgres
//! backend with proper transactional guarantees.

use std::path::Path;
use std::sync::Arc;

use anyhow::Result;
use chrono::{DateTime, Utc};
use robotics_protocols::Command;
use serde::{Deserialize, Serialize};
use tokio::fs::{File, OpenOptions};
use tokio::io::AsyncWriteExt;
use tokio::sync::Mutex;
use tracing::warn;

/// One row of the audit log. Field names are deliberately short — the
/// file gets long.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuditEntry {
    /// Wall-clock UTC at the moment the dashboard accepted the command.
    pub t: DateTime<Utc>,
    /// Who issued it. With auth, the operator's username; today, the
    /// transport ("dashboard", "cli", "mqtt").
    pub op: String,
    pub cmd: Command,
    /// "accepted" | "estop" | "rejected:<reason>".
    pub out: String,
}

impl AuditEntry {
    pub fn new(op: impl Into<String>, cmd: Command, out: impl Into<String>) -> Self {
        Self { t: Utc::now(), op: op.into(), cmd, out: out.into() }
    }
}

/// Append-only writer. Cheap to clone (it's an `Arc` around the file).
pub struct AuditRecorder {
    file: Mutex<File>,
}

impl AuditRecorder {
    /// Open `path` in append mode, creating it if absent. The returned
    /// `Arc` lets the recorder be shared across handlers without a
    /// second wrapping.
    pub async fn open<P: AsRef<Path>>(path: P) -> Result<Arc<Self>> {
        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .await?;
        Ok(Arc::new(Self { file: Mutex::new(file) }))
    }

    /// Record one entry. Locks the file briefly (sub-ms on SSD) and
    /// writes one line. Errors are logged but never propagated — an
    /// audit failure must not block the command path.
    pub async fn record(&self, entry: &AuditEntry) {
        let mut line = match serde_json::to_string(entry) {
            Ok(s) => s,
            Err(e) => {
                warn!(error = %e, "audit: serialize failed");
                return;
            }
        };
        line.push('\n');
        let mut f = self.file.lock().await;
        if let Err(e) = f.write_all(line.as_bytes()).await {
            warn!(error = %e, "audit: write failed");
        }
    }
}
