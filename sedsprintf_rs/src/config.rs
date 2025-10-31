// src/config.rs
#[allow(unused_imports)]
pub(crate) use crate::{
    get_needed_message_size, MessageDataType, MessageMeta, MessageSizeType, MessageType,
    DYNAMIC_ELEMENT, STRING_VALUE_ELEMENTS,
};
use strum::EnumCount;
use strum_macros::EnumCount;


//----------------------User Editable----------------------
pub const DEVICE_IDENTIFIER: &str = "CrashNBurn";
pub const MAX_STATIC_STRING_LENGTH: usize = 1024;
pub const MAX_STATIC_HEX_LENGTH: usize = 1024;
pub const MAX_PRECISION_IN_STRINGS: usize = 8; // 12 is expensive; tune as needed

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd, EnumCount)]
#[repr(u32)]
pub enum DataEndpoint {
    Serial,
}

impl DataEndpoint {
    pub fn as_str(self) -> &'static str {
        match self {
            DataEndpoint::Serial => "Serial",
        }
    }
}

/// All data types that can be logged.
/// Each data type corresponds to a specific message format.
/// /// When adding new data types, ensure to update MESSAGE_DATA_TYPES,
/// MESSAGE_INFO_TYPES, MESSAGE_ELEMENTS, and MESSAGE_TYPES accordingly.
/// These must increase sequentially from 0 without gaps.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd, EnumCount)]
#[repr(u32)]
pub enum DataType {
    TelemetryError,
    GyroscopeData,
    AccelerometerData,
    BarometerData,
}

impl DataType {
    pub fn as_str(&self) -> &'static str {
        match self {
            DataType::TelemetryError => "TELEMETRY_ERROR",
            DataType::GyroscopeData => "GYROSCOPE_DATA",
            DataType::AccelerometerData => "ACCELEROMETER_DATA",
            DataType::BarometerData => "BAROMETER_DATA",
        }
    }
}

pub const MESSAGE_DATA_TYPES: [MessageDataType; DataType::COUNT] = [
    MessageDataType::String,
    MessageDataType::Float32,
    MessageDataType::Float32,
    MessageDataType::Float32,
];

pub const MESSAGE_INFO_TYPES: [MessageType; DataType::COUNT] = [
    MessageType::Error,
    MessageType::Info,
    MessageType::Info,
    MessageType::Info,
];

/// how many elements each message carries
pub const MESSAGE_ELEMENTS: [usize; DataType::COUNT] = [
    DYNAMIC_ELEMENT,       // elements int he Telemetry Error data
    3,                     // elements in the Gyroscope data (gyro x,y,z)
    3,                     // elements in the Accelerometer data (accel x,y,z)
    3,                     // elements in the Barometer data (pressure, temperature, altitude)
];
/// All message types with their metadata.
pub const MESSAGE_TYPES: [MessageMeta; DataType::COUNT] = [
    MessageMeta {
        data_size: MessageSizeType::Dynamic,
        endpoints: &[DataEndpoint::Serial],
    },
    MessageMeta {
        data_size: MessageSizeType::Static(get_needed_message_size(DataType::GyroscopeData)),
        endpoints: &[DataEndpoint::Serial],
    },
    MessageMeta {
        data_size: MessageSizeType::Static(get_needed_message_size(DataType::AccelerometerData)),
        endpoints: &[DataEndpoint::Serial],
    },
    MessageMeta {
        data_size: MessageSizeType::Static(get_needed_message_size(DataType::BarometerData)),
        endpoints: &[DataEndpoint::Serial],
    },
];
// -------------------------------------------------------------
