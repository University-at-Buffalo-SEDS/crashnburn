// src/config.rs
#[allow(dead_code)]
use core::mem::size_of;


#[allow(dead_code)]
pub const STRING_VALUE_ELEMENTS: usize = 1;

//----------------------User Editable----------------------
pub const DEVICE_IDENTIFIER: &str = "CrashNBurn";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
#[repr(u32)]
pub enum DataEndpoint {
    Serial = 0,
}

pub const MAX_VALUE_DATA_ENDPOINT: u32 = DataEndpoint::Serial as u32;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
#[allow(dead_code)]
pub enum MessageDataType {
    Float32,
    UInt8,
    UInt32,
    String,
    Hex,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
#[allow(dead_code)]
pub enum MessageType {
    Info,
    Error,
}

impl DataEndpoint {
    #[inline]
    pub fn as_str(self) -> &'static str {
        match self {
            DataEndpoint::Serial => "Serial",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Ord, PartialOrd)]
#[repr(u32)]
/// All data types that can be logged.
/// Each data type corresponds to a specific message format.
/// /// When adding new data types, ensure to update MESSAGE_DATA_TYPES,
/// MESSAGE_INFO_TYPES, MESSAGE_ELEMENTS, and MESSAGE_TYPES accordingly.
/// These must increase sequentially from 0 without gaps.
pub enum DataType {
    TelemetryError = 0,
    GyroscopeData = 1,
    AccelerometerData = 2,
    BarometerData = 3,
}

pub const MAX_VALUE_DATA_TYPE: u32 = DataType::BarometerData as u32;

impl DataType {
    pub const COUNT: usize = (MAX_VALUE_DATA_TYPE + 1) as usize;

    #[inline]
    pub fn as_str(self) -> &'static str {
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

pub const fn data_type_size(dt: MessageDataType) -> usize {
    match dt {
        MessageDataType::Float32 => size_of::<f32>(),
        MessageDataType::UInt8 => size_of::<u8>(),
        MessageDataType::UInt32 => size_of::<u32>(),
        MessageDataType::String => MAX_STRING_LENGTH,
        MessageDataType::Hex => MAX_HEX_LENGTH,
    }
}

/// how many elements each message carries
pub const MESSAGE_ELEMENTS: [usize; DataType::COUNT] = [
    1,                     // elements int he Telemetry Error data
    3,                     // elements in the Gyroscope data (gyro x,y,z)
    3,                     // elements in the Accelerometer data (accel x,y,z)
    3,                     // elements in the Barometer data (pressure, temperature, altitude)
];
/// Fixed maximum length for the TelemetryError message (bytes, UTF-8).
pub const MAX_STRING_LENGTH: usize = 1024;
pub const MAX_HEX_LENGTH: usize = 1024;

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
#[derive(Debug, Clone, Copy)]
pub struct MessageMeta {
    pub data_size: usize,
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
