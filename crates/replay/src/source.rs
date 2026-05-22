//! Pluggable sources of audit entries.
//!
//! Today: read a JSONL file. Tomorrow: stream from Postgres, an HTTP
//! endpoint, a Kafka topic — same trait, different impl.

use std::path::Path;

use async_trait::async_trait;
use robotics_audit::AuditEntry;
use tokio::fs::File;
use tokio::io::{AsyncBufReadExt, BufReader, Lines};
use tracing::warn;

/// Pull entries in chronological order.
#[async_trait]
pub trait ReplaySource: Send {
    /// Returns the next entry, or `None` when the source is exhausted.
    /// Implementations should skip malformed records rather than error,
    /// so a single bad line doesn't tank a long replay.
    async fn next(&mut self) -> std::io::Result<Option<AuditEntry>>;
}

/// JSONL file source. Streams one line at a time; the file is never
/// fully materialised in memory.
pub struct JsonlSource {
    lines: Lines<BufReader<File>>,
}

impl JsonlSource {
    pub async fn open<P: AsRef<Path>>(path: P) -> std::io::Result<Self> {
        let file = File::open(path).await?;
        let reader = BufReader::new(file);
        Ok(Self { lines: reader.lines() })
    }
}

#[async_trait]
impl ReplaySource for JsonlSource {
    async fn next(&mut self) -> std::io::Result<Option<AuditEntry>> {
        loop {
            match self.lines.next_line().await? {
                Some(line) if line.trim().is_empty() => continue,
                Some(line) => match serde_json::from_str::<AuditEntry>(&line) {
                    Ok(e) => return Ok(Some(e)),
                    Err(err) => {
                        warn!(error = %err, "replay: skipping malformed audit line");
                        continue;
                    }
                },
                None => return Ok(None),
            }
        }
    }
}

/// In-memory source. Useful for unit tests that build a fixed log.
pub struct VecSource(std::vec::IntoIter<AuditEntry>);

impl VecSource {
    pub fn new(entries: Vec<AuditEntry>) -> Self {
        Self(entries.into_iter())
    }
}

#[async_trait]
impl ReplaySource for VecSource {
    async fn next(&mut self) -> std::io::Result<Option<AuditEntry>> {
        Ok(self.0.next())
    }
}
