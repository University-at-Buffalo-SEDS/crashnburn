// src/config.rs
use crate::{data_type_size, MessageDataType, MessageType};

//----------------------User Editable----------------------
pub const DEVICE_IDENTIFIER: &str = "CrashNBurn";
pub const MAX_STATIC_STRING_LENGTH: usize = 1024;
pub const MAX_STATIC_HEX_LENGTH: usize = 1024;

pub const MAX_PRECISION_IN_STRINGS: usize = 8; // 12 is expensive; tune as needed

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
#[repr(u32)]
pub enum DataEndpoint {
    Serial = 0,
}

pub const MAX_VALUE_DATA_ENDPOINT: u32 = DataEndpoint::Serial as u32;

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
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
#[repr(u32)]
pub enum DataType {
    TelemetryError = 0,
    GyroscopeData = 1,
    AccelerometerData = 2,
    BarometerData = 3,
}

pub const MAX_VALUE_DATA_TYPE: u32 = DataType::BarometerData as u32;

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
    1,                     // elements int he Telemetry Error data
    3,                     // elements in the Gyroscope data (gyro x,y,z)
    3,                     // elements in the Accelerometer data (accel x,y,z)
    3,                     // elements in the Barometer data (pressure, temperature, altitude)
];
/// All message types with their metadata.
pub const MESSAGE_TYPES: [MessageMeta; DataType::COUNT] = [
    MessageMeta {
        data_size: get_needed_message_size(DataType::TelemetryError),
        endpoints: &[DataEndpoint::Serial],
    },
    MessageMeta {
        data_size: get_needed_message_size(DataType::GyroscopeData),
        endpoints: &[DataEndpoint::Serial],
    },
    MessageMeta {
        data_size: get_needed_message_size(DataType::AccelerometerData),
        endpoints: &[DataEndpoint::Serial],
    },
    MessageMeta {
        data_size: get_needed_message_size(DataType::BarometerData),
        endpoints: &[DataEndpoint::Serial],
    },
];
// -------------------------------------------------------------

// ----------------------Not User Editable----------------------
#[allow(dead_code)]
pub const STRING_VALUE_ELEMENTS: usize = 1;
impl DataType {
    pub const COUNT: usize = (MAX_VALUE_DATA_TYPE + 1) as usize;
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
pub enum MessageSizeType {
    Static(usize),
    Dynamic,

}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
pub struct MessageMeta {
    pub data_size: MessageSizeType,
    pub endpoints: &'static [DataEndpoint],
}

#[inline(always)]
pub fn message_meta(ty: DataType) -> &'static MessageMeta {
    &MESSAGE_TYPES[ty as usize]
}

#[inline(always)]
pub const fn get_needed_message_size(ty: DataType) -> usize {
    data_type_size(get_data_type(ty)) * MESSAGE_ELEMENTS[ty as usize]
}

#[inline(always)]
pub const fn get_info_type(ty: DataType) -> MessageType {
    MESSAGE_INFO_TYPES[ty as usize]
}

#[inline(always)]
pub const fn get_data_type(ty: DataType) -> MessageDataType {
    MESSAGE_DATA_TYPES[ty as usize]
}
