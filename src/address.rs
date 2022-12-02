use crate::MessageTransmitter;
use crate::Write;
use crate::{Command, ServoId};

pub struct Address(pub u8);
impl Into<u8> for Address {
    fn into(self) -> u8 {
        self.0
    }
}

#[repr(u8)]
pub enum AddressU8 {
    Id = 0,
    AckPolicy = 1,
    AlarmLEDPolicy = 2,
    TorquePolicy = 3,
    // Reserved = 4,
    MaxTemperature = 5,
    MinVoltage = 6,
    MaxVoltage = 7,
    AccelerationRatio = 8,
    MaxAcceleration = 9,
    DeadZone = 10,
    SaturaterOffset = 11,
    PWMOffset = 14,
    MinPWM = 15,
    LEDBlinkPeriod = 38,
    ADCFaultDetectionPeriod = 39,
    PacketGarbageDetectionPeriod = 40,
    StopDetectionPeriod = 41,
    OverloadDetectionPeriod = 42,
    StopThreshold = 43,
    InpositionMargin = 44,
    // Reserved = 45,
    // Reserved = 46,
    CalibrationDifference = 47,
    StatusError = 48,
    StatusDetail = 49,
    // Reserved  = 50,
    // Reserved = 51,
    TorqueControl = 52,
    LEDControl = 53,
}
impl Into<u8> for AddressU8 {
    fn into(self) -> u8 {
        self as u8
    }
}
impl Into<Address> for AddressU8 {
    fn into(self) -> Address{
        Address(self as u8)
    }
}


#[repr(u8)]
pub enum AddressU16 {
    SaturatorSlope = 12,
    MaxPWM = 16,
    OverloadPWMThreshold = 18,
    MinPosition = 20,
    MaxPosition = 22,
    PositionKp = 24,
    PositionKd = 26,
    PositionKi = 28,
    PositionFFGain1 = 30,
    PositionFFGain2 = 32,
    Voltage = 54,
    Temperature = 55,
    CurrentControlMode = 56,
    Tick = 57,
    CalibratedPosition = 58,
    AbsolutePosition = 60,
    DifferentialPosition = 62,
    Pwm = 64,
    AbsoluteGoalPosition = 68
}

impl Into<u8> for AddressU16 {
    fn into(self) -> u8 {
        self as u8
    }
}
impl Into<Address> for AddressU16 {
    fn into(self) -> Address{
        Address(self as u8)
    }
}


#[repr(u8)]
pub enum ReadOnlyAddressU8 {
    // u8
    DesiredVelocity = 72,    
}
impl Into<u8> for ReadOnlyAddressU8 {
    fn into(self) -> u8 {
        self as u8
    }
}
impl Into<Address> for ReadOnlyAddressU8 {
    fn into(self) -> Address{
        Address(self as u8)
    }
}


pub enum ReadOnlyAddressU16 {
    // u16
    Voltage = 54,
    Temperature = 55,
    CurrentControlMode = 56,
    Tick = 57,
    CalibratedPosition = 58,
    AbsolutePosition = 60,
    DifferentialPosition = 62,
    Pwm = 64,
    AbsoluteGoalPosition = 68,
    AbsoluteDesiredTrajectoryPosition = 70
}
impl Into<u8> for ReadOnlyAddressU16 {
    fn into(self) -> u8 {
        self as u8
    }
}
impl Into<Address> for ReadOnlyAddressU16 {
    fn into(self) -> Address{
        Address(self as u8)
    }
}


pub trait ReadableAddress : Into<Address> { 
    type Value;
}
impl ReadableAddress for ReadOnlyAddressU8 {
    type Value = u8;
}
impl ReadableAddress for ReadOnlyAddressU16 {
    type Value = u16;
}
impl ReadableAddress for AddressU8 {
    type Value = u8;
}
impl ReadableAddress for AddressU16 {
    type Value = u16;
}

pub trait WritableAddress<V> : Into<u8> { 
    fn send_write_message<S>( self, tx: &mut MessageTransmitter<S>, servo_id: ServoId, val: V ) -> Result<(),S::Error>
    where S: Write<u8>;
}
impl WritableAddress<u8> for AddressU8 {
    fn send_write_message<S>( self, tx: &mut MessageTransmitter<S>,servo_id:ServoId, val: u8 ) -> Result<(),S::Error>
    where S: Write<u8> {
        const DATA_LEN:usize = 3;
        tx.packet_header[MessageTransmitter::<S>::CMD_INDEX] = Command::RAMWrite as u8;
        tx.packet_header[MessageTransmitter::<S>::PID_INDEX] = servo_id.into();
        tx.packet_header[MessageTransmitter::<S>::LEN_INDEX] = (MessageTransmitter::<S>::HEADER_LEN + DATA_LEN) as u8;

        tx.data[0] = self.into();
        tx.data[1] = 1;
        tx.data[2] = val;

        tx.send::<DATA_LEN>()

    }    
}
impl WritableAddress<u16> for AddressU16 {
    fn send_write_message<S>( self, tx: &mut MessageTransmitter<S>,servo_id:ServoId, val: u16 ) -> Result<(),S::Error>
    where S: Write<u8> {
        const DATA_LEN:usize = 4;
        tx.packet_header[MessageTransmitter::<S>::CMD_INDEX] = Command::RAMWrite as u8;
        tx.packet_header[MessageTransmitter::<S>::PID_INDEX] = servo_id.into();
        tx.packet_header[MessageTransmitter::<S>::LEN_INDEX] = (MessageTransmitter::<S>::HEADER_LEN + DATA_LEN) as u8;

        tx.data[0] = self.into();
        tx.data[1] = 1;

        let le_val = val.to_le_bytes();
        tx.data[2] = le_val[0];
        tx.data[3] = le_val[1];

        tx.send::<DATA_LEN>()

    }    
}

pub struct EEPAddress(u8);
impl Into<u8> for EEPAddress {
    fn into(self) -> u8 {
        self.0
    }
}


#[repr(u8)]
pub enum EEPAddressU8 {
    BaudRate = 4,
    // Reserved = 5,
    Id = 6,
    AckPolicy = 7,
    AlarmLEDPolicy = 8,
    TorquePolicy = 9,
    // Reserved = 10,
    MaxTemperature = 11,
    MinVoltage = 12,
    MaxVoltage = 13,
    AccelerationRatio = 14,
    MaxAcceleration = 15,
    DeadZone = 16,
    SaturaterOffset = 17,
    LEDBlinkPeriod = 44,
    ADCFaultDetectionPeriod = 45,
    PacketGarbageDetectionPeriod = 46,
    StopDetectionPeriod = 47,
    OverloadDetectionPeriod = 48,
    StopThreshold = 49,
    InpositionMargin = 50,
    CalibrationDifference = 53
}
impl Into<u8> for EEPAddressU8 {
    fn into(self) -> u8 {
        self as u8
    }
}
impl Into<EEPAddress> for EEPAddressU8 {
    fn into(self) -> EEPAddress {
        EEPAddress(self as u8)
    }
}

pub enum EEPAddressU16 {
    SaturatorSlope = 18,
    MaxPWM = 22,
    OverloadPWMThreshold = 24,
    MinPosition = 26,
    MaxPosition = 28,
    PositionKp = 30,
    PositionKd = 32,
    PositionKi = 34,
    PositionFFGain1 = 36,
    PositionFFGain2 = 38,
}
impl Into<u8> for EEPAddressU16 {
    fn into(self) -> u8 {
        self as u8
    }
}
impl Into<EEPAddress> for EEPAddressU16 {
    fn into(self) -> EEPAddress {
        EEPAddress(self as u8)
    }
}


#[repr(u8)]
pub enum ReadOnlyEEPAddressU8 {
    ModelNumber1 = 0,
    ModelNumber2 = 1,
    Version1 = 2,
    Version2 = 3
}
impl Into<u8> for ReadOnlyEEPAddressU8 {
    fn into(self) -> u8 {
        self as u8
    }
}
impl Into<EEPAddress> for ReadOnlyEEPAddressU8 {
    fn into(self) -> EEPAddress {
        EEPAddress( self as u8 )
    }
}

pub trait ReadableEEPAddress : Into<EEPAddress> { 
    type Value;
}
impl ReadableEEPAddress for ReadOnlyEEPAddressU8 {
    type Value = u8;
}
impl ReadableEEPAddress for EEPAddressU8 {
    type Value = u8;
}
impl ReadableEEPAddress for EEPAddressU16 {
    type Value = u16;
}

pub trait WritableEEPAddress<V> { 
    fn send_write_message<S>( self, tx: &mut MessageTransmitter<S>, servo_id: ServoId, val: V ) -> Result<(),S::Error>
    where S: Write<u8>;
}
impl WritableEEPAddress<u8> for EEPAddressU8 {
    fn send_write_message<S>( self, tx: &mut MessageTransmitter<S>,servo_id:ServoId, val: u8 ) -> Result<(),S::Error>
    where S: Write<u8> {
        const DATA_LEN:usize = 3;
        tx.packet_header[MessageTransmitter::<S>::CMD_INDEX] = Command::EEPWrite as u8;
        tx.packet_header[MessageTransmitter::<S>::PID_INDEX] = servo_id.into();
        tx.packet_header[MessageTransmitter::<S>::LEN_INDEX] = (MessageTransmitter::<S>::HEADER_LEN + DATA_LEN) as u8;

        tx.data[0] = self.into();
        tx.data[1] = 1;
        tx.data[2] = val;

        tx.send::<DATA_LEN>()

    }    
}
impl WritableEEPAddress<u16> for EEPAddressU16 {
    fn send_write_message<S>( self, tx: &mut MessageTransmitter<S>,servo_id:ServoId, val: u16 ) -> Result<(),S::Error>
    where S: Write<u8> {
        const DATA_LEN:usize = 4;
        tx.packet_header[MessageTransmitter::<S>::CMD_INDEX] = Command::EEPWrite as u8;
        tx.packet_header[MessageTransmitter::<S>::PID_INDEX] = servo_id.into();
        tx.packet_header[MessageTransmitter::<S>::LEN_INDEX] = (MessageTransmitter::<S>::HEADER_LEN + DATA_LEN) as u8;

        tx.data[0] = self.into();
        tx.data[1] = 1;

        let le_val = val.to_le_bytes();
        tx.data[2] = le_val[0];
        tx.data[3] = le_val[1];

        tx.send::<DATA_LEN>()

    }    
}

