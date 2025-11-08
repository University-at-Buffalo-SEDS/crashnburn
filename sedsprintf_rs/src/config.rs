//! Telemetry configuration and schema description.
//!
//! This module defines:
//! - Device-/build-time constants (identifiers, limits, retries).
//! - The `DataEndpoint` and `DataType` enums.
//! - Functions that describe per-type schema metadata:
//!   - [`get_message_data_type`]
//!   - [`get_message_info_types`]
//!   - [`get_message_meta`]

#[allow(unused_imports)]
use crate::{MessageDataType, MessageElementCount, MessageMeta, MessageType, STRING_VALUE_ELEMENT};
use strum_macros::EnumCount;

// -----------------------------------------------------------------------------
// User-editable configuration
// -----------------------------------------------------------------------------

/// Device identifier string.
///
/// This string is attached to every telemetry packet and is used to identify
/// the platform in downstream tools. It should be **unique per platform**.
pub const DEVICE_IDENTIFIER: &str = "CrashNBurn";

/// Maximum length, in bytes, of any **static** UTF-8 string payload.
///
/// Dynamic string messages may be longer, but many tests and error paths
/// assume this bound when generating placeholder data.
pub const MAX_STATIC_STRING_LENGTH: usize = 1024;

/// Maximum length, in bytes, of any **static** hex payload.
pub const MAX_STATIC_HEX_LENGTH: usize = 1024;

/// Maximum number of fractional digits when converting floating-point values
/// to strings (e.g., for human-readable error payloads).
///
/// Higher values increase both payload size and formatting cost.
pub const MAX_PRECISION_IN_STRINGS: usize = 8; // 12 is expensive; tune as needed

/// Maximum payload size (in bytes) that is allowed to be allocated on the
/// stack before the implementation switches to heap allocation.
pub const MAX_STACK_PAYLOAD_SIZE: usize = 256;

/// Maximum number of times a handler is retried before giving up and
/// surfacing a [`TelemetryError::HandlerError`](crate::TelemetryError::HandlerError).
pub const MAX_HANDLER_RETRIES: usize = 3;

// -----------------------------------------------------------------------------
// Endpoints
// -----------------------------------------------------------------------------

/// The different destinations where telemetry packets can be sent.
///
/// When adding new endpoints:
/// - Keep the discriminants sequential from `0` with no gaps (if you assign
///   explicit values).
/// - Update any tests that iterate over `0..=MAX_VALUE_DATA_ENDPOINT`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd, EnumCount)]
#[repr(u32)]
pub enum DataEndpoint {
    Serial,
}

impl DataEndpoint {
    /// Return a stable string representation used in logs and in
    /// `TelemetryPacket::to_string()` output.
    ///
    /// This should remain stable over time for compatibility with tests and
    /// external tooling.
    pub fn as_str(self) -> &'static str {
        match self {
            DataEndpoint::Serial => "Serial",
        }
    }
}

// -----------------------------------------------------------------------------
// Data types
// -----------------------------------------------------------------------------

/// Logical telemetry message kinds.
///
/// Each variant corresponds to:
/// - a concrete payload element type (via [`get_message_data_type`]),
/// - a message severity/role (via [`get_message_info_types`]),
/// - schema metadata (via [`get_message_meta`]).
///
/// When adding new variants:
/// - Keep discriminants sequential from `0` with no gaps (if assigning
///   explicit values).
/// - Update all mapping functions and any tests that iterate over the enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd, EnumCount)]
#[repr(u32)]
pub enum DataType {
    /// Encoded telemetry error text (string payload).
    TelemetryError,
    GyroscopeData,
    AccelerometerData,
    BarometerData,
}

impl DataType {
    /// Return a stable string representation used in logs, headers, and in
    /// `TelemetryPacket::to_string()` formatting.
    ///
    /// This must be kept up to date when adding new variants.
    pub fn as_str(&self) -> &'static str {
        match self {
            DataType::TelemetryError => "TELEMETRY_ERROR",
            DataType::GyroscopeData => "GYROSCOPE_DATA",
            DataType::AccelerometerData => "ACCELEROMETER_DATA",
            DataType::BarometerData => "BAROMETER_DATA",
        }
    }
}

// -----------------------------------------------------------------------------
// Schema helpers: element type, message kind, and metadata
// -----------------------------------------------------------------------------

/// Return the element type for the payload of a given [`DataType`].
///
/// The order and mapping must stay in lock-step with [`DataType`], and with the
/// schema used by `TelemetryPacket` validation. Available element types are:
///
/// - `String`
/// - `Float32`
/// - `UInt8`, `UInt16`, `UInt32`, `UInt64`, `UInt128`
/// - `Int8`, `Int16`, `Int32`, `Int64`, `Int128`
pub const fn get_message_data_type(data_type: DataType) -> MessageDataType {
    match data_type {
        DataType::TelemetryError => MessageDataType::String,
        DataType::GyroscopeData => MessageDataType::Float32,
        DataType::AccelerometerData => MessageDataType::Float32,
        DataType::BarometerData => MessageDataType::Float32,
    }
}

/// Return the logical message type (severity/category) for a given [`DataType`].
///
/// This affects how messages may be surfaced or filtered in the higher-level
/// API (e.g. errors vs informational telemetry).
pub const fn get_message_info_types(message_type: DataType) -> MessageType {
    match message_type {
        DataType::TelemetryError => MessageType::Error,
        DataType::GyroscopeData => MessageType::Info,
        DataType::AccelerometerData => MessageType::Info,
        DataType::BarometerData => MessageType::Info,
    }
}

/// Return the full schema metadata for a given [`DataType`].
///
/// Each variant specifies:
/// - `element_count`: either `Static(n)` (fixed number of elements) or
///   `Dynamic` (variable-length payload—size validated at runtime).
/// - `endpoints`: default destination list for packets of that type.
///
/// The element count is interpreted relative to the element type returned by
/// [`get_message_data_type`].
pub const fn get_message_meta(data_type: DataType) -> MessageMeta {
    match data_type {
        DataType::TelemetryError => {
            MessageMeta {
                // Telemetry Error
                element_count: MessageElementCount::Dynamic,
                endpoints: &[DataEndpoint::Serial],
            }
        }
        DataType::AccelerometerData => {
            MessageMeta {
                // System Status
                element_count: MessageElementCount::Static(3),
                endpoints: &[DataEndpoint::Serial],
            }
        }
        DataType::GyroscopeData => {
            MessageMeta {
                // Barometer Data
                element_count: MessageElementCount::Static(3),
                endpoints: &[DataEndpoint::Serial],
            }
        }
        DataType::BarometerData => {
            MessageMeta {
                // Message Data
                element_count: MessageElementCount::Static(3),
                endpoints: &[DataEndpoint::Serial],
            }
        }
    }
}
