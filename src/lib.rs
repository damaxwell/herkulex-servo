#![cfg_attr(not(test), no_std)]
use embedded_hal::serial::{Read,Write};
// use core::fmt::Debug;
// use core::fmt::Formatter;

mod address;
pub use address::{Address,AddressU8, AddressU16, ReadOnlyAddressU8, ReadOnlyAddressU16};
pub use address::{ReadableAddress, WritableAddress};
pub use address::{EEPAddress,EEPAddressU8, EEPAddressU16, ReadOnlyEEPAddressU8};
pub use address::{ReadableEEPAddress, WritableEEPAddress};

// #[derive(Debug)]
pub enum Error {
    InvalidColorValue,
}

pub enum AckReaderError {
    Overflow,
    Corrupt,
    Incomplete
}



#[repr(u8)]
#[derive(PartialEq,Copy,Clone,Eq,Ord,PartialOrd,Hash)]
enum Command {
    EEPWrite  = 0x01,
    EEPRead   = 0x02,
    RAMWrite  = 0x03,
    RAMRead   = 0x04,
    IJog      = 0x05,
    SJog      = 0x06,
    Stat      = 0x07,
    Rollback  = 0x08,
    Reboot    = 0x09
}
impl Into<u8> for Command {
    fn into(self) -> u8 {
        self as u8
    }
}


#[repr(u8)]
#[derive(PartialEq,Copy,Clone,Eq,Ord,PartialOrd,Hash)]
pub enum Color {
    Black = 0,
    Green = 1,
    Blue = 2,
    Yellow = 3,
    Red = 4,
    Magenta = 5,
    Cyan = 6,
    White = 7
}
impl TryInto<Color> for u8 {

    type Error = crate::Error;

    fn try_into( self ) -> Result<Color, Error> {
        match self {
            1 => Ok( Color::Black ),
            _ => Err( Error::InvalidColorValue )
        }
    }

}
impl Into<u8> for Color {
    fn into(self) -> u8 {
        self as u8
    }
}

#[repr(u8)]
#[derive(PartialEq,Copy,Clone,Eq,Ord,PartialOrd,Hash)]
pub enum Torque {
    FreeMoving = 0x00,
    BrakeEnabled = 0x40,
    Powered = 0x60
}
impl Into<u8> for Torque {
    fn into(self) -> u8 {
        self as u8
    }
}


#[repr(u8)]
#[derive(PartialEq,Copy,Clone,Eq,Ord,PartialOrd,Hash)]
pub enum JogMode {
    Position,
    Velocity
}

struct JogFlags(u8);
impl JogFlags {
    fn new( mode: JogMode, color: Color ) -> Self {
        JogFlags( ( (mode as u8) << 1 ) | (color as u8) << 2 )
    }

    // fn color(&self) -> Color {
    //     ((self.0 & 0b00011100) >> 2).try_into().unwrap()
    // }
}
impl Into<u8> for JogFlags {
    fn into(self) -> u8 {
        self.0
    }
}



#[derive(PartialEq,Copy,Clone,Eq,Ord,PartialOrd,Hash)]
pub struct ServoId(pub u8);
impl ServoId {
    const DEFAULT_SERVO_ID:u8 = 0xFD;
    const ALL_SERVOS:u8 = 0xFE;

    pub const fn default() -> Self {
        ServoId( Self::DEFAULT_SERVO_ID )
    }

    pub const fn all() -> Self {
        ServoId( Self::ALL_SERVOS )
    }
}
impl From<ServoId> for u8 {
    fn from( other: ServoId ) -> u8 {
        other.0
    }
}


#[derive(PartialEq,Copy,Clone,Eq,Ord,PartialOrd,Hash)]
pub struct IDSkip(pub bool);
impl Into<u8> for IDSkip {
    fn into(self) -> u8 {
        self.0 as u8
    }
}

#[derive(PartialEq,Copy,Clone,Eq,Ord,PartialOrd,Hash)]
pub struct BaudSkip(pub bool);
impl Into<u8> for BaudSkip {
    fn into(self) -> u8 {
        self.0 as u8
    }
}

#[derive(PartialEq,Copy,Clone,Eq,Ord,PartialOrd,Hash)]
pub struct Ticks(pub u8);
impl Into<u8> for Ticks {
    fn into(self) -> u8 {
        self.0 as u8
    }
}


const HEADER_LEN:usize = 7;
pub struct MessageTransmitter<S> 
where S: Write<u8> {
    serial: S,
    packet_header: [u8; HEADER_LEN],
    data: [u8; 5],
}

impl<S> MessageTransmitter<S> 
where S: Write<u8> {

    const HEADER_LEN:usize = 7;
    const LEN_INDEX:usize = 2;
    const PID_INDEX:usize = 3;
    const CMD_INDEX:usize = 4;
    const CS1_INDEX:usize = 5;
    const CS2_INDEX:usize = 6;


    pub fn new( serial: S ) -> Self {
        let mut rv = MessageTransmitter {
            serial: serial,
            packet_header: [0; HEADER_LEN],
            data: [0; 5],
            // rx_message: HerkMessage::new()
        };

        // The first two bytes of a packet are always 0xFF.
        rv.packet_header[0] = 0xFF;
        rv.packet_header[1] = 0xFF;

        rv
    }

    #[inline]
    pub fn send_eep_write<V,A:WritableEEPAddress<V>>(&mut self, servo_id: ServoId, addr: A, data: V) -> Result<(),S::Error> 
    {
        addr.send_write_message( self, servo_id, data )
    }

    #[inline]
    pub fn send_eep_write_bytes<A>(&mut self, servo_id: ServoId, addr: A, data: &[u8]) -> Result<(),S::Error> 
    where A: Into<EEPAddress> {
        self.send_eep_write_bytes_raw(servo_id, addr.into(), data)        
    }

    pub fn send_eep_write_bytes_raw(&mut self, servo_id: ServoId, addr: EEPAddress, data: &[u8]) -> Result<(),S::Error> {
        self.send_message_write_bytes( servo_id, Command::EEPWrite, addr.into(), data )
    }

    #[inline]
    pub fn send_eep_read<T: ReadableEEPAddress>(&mut self, servo_id: ServoId, addr: T ) -> Result<(), S::Error>  {
        self.send_eep_read_raw(servo_id, addr.into(), core::mem::size_of::<T::Value>() as u8)
    }

    pub fn send_eep_read_raw(&mut self, servo_id: ServoId, addr: EEPAddress, byte_count: u8) -> Result<(),S::Error> {
        self.send_message_9( servo_id, Command::EEPRead, addr.into(), byte_count )
    }

    #[inline]
    pub fn send_ram_write<V,A:WritableAddress<V>>(&mut self, servo_id: ServoId, addr: A, data: V) -> Result<(),S::Error> 
    {
        addr.send_write_message( self, servo_id, data )
    }

    #[inline]
    pub fn send_ram_write_bytes<A>(&mut self, servo_id: ServoId, addr: A, data: &[u8]) -> Result<(),S::Error> 
    where A:Into<Address> {
        self.send_ram_write_bytes_raw(servo_id, addr.into(), data)
    }

    pub fn send_ram_write_bytes_raw(&mut self, servo_id: ServoId, addr: Address, data: &[u8]) -> Result<(),S::Error> {
        self.send_message_write_bytes( servo_id, Command::RAMWrite, addr.into(), data)
    }


    #[inline]
    pub fn send_ram_read<T: ReadableAddress>(&mut self, servo_id: ServoId, addr: T ) -> Result<(), S::Error>  {
        self.send_ram_read_raw(servo_id, addr.into(), core::mem::size_of::<T::Value>() as u8)
    }

    #[inline]
    pub fn send_ram_read_raw(&mut self, servo_id: ServoId, addr: Address, byte_count: u8) -> Result<(),S::Error> {
        self.send_message_9( servo_id, Command::RAMRead, addr.into(), byte_count )
    }

    pub fn send_ijog_position(&mut self, servo_id: ServoId, position: u16, playtime: Ticks, color: Color )  -> Result<(),S::Error> {
        let position_bytes = position.to_le_bytes();
        let flags = JogFlags::new( JogMode::Position, color).into();
        self.send_message_13( servo_id, Command::IJog, position_bytes[0], position_bytes[1], flags, servo_id.into(), playtime.into() )
    }

    pub fn send_ijog_continuous(&mut self, servo_id: ServoId, pwm: u16, playtime: Ticks, color: Color )  -> Result<(),S::Error> {
        let pwm_bytes = pwm.to_le_bytes();
        let flags = JogFlags::new( JogMode::Velocity, color).into();
        self.send_message_13( servo_id, Command::IJog, pwm_bytes[0], pwm_bytes[1], flags, servo_id.into(), playtime.into() )
    }

    pub fn send_sjog_position(&mut self, servo_id: ServoId, position: u16, playtime: Ticks, color: Color )  -> Result<(),S::Error> {
        let position_bytes = position.to_le_bytes();
        let flags = JogFlags::new( JogMode::Position, color).into();
        self.send_message_13( servo_id, Command::SJog, playtime.into(), position_bytes[0], position_bytes[1], flags, servo_id.into() )
    }

    // TODO
    // pub fn send_sjog_positions(&mut self, servo_id: ServoId, position: u16, playtime: Ticks, color: Color )  -> Result<(),S::Error> {
    // }

    pub fn send_sjog_continuous(&mut self, servo_id: ServoId, pwm: u16, playtime: Ticks, color: Color )  -> Result<(),S::Error> {
        let pwm_bytes = pwm.to_le_bytes();
        let flags = JogFlags::new( JogMode::Velocity, color).into();
        self.send_message_13( servo_id, Command::SJog, playtime.into(), pwm_bytes[0], pwm_bytes[1], flags, servo_id.into() )
    }

    pub fn send_stat(&mut self, servo_id: ServoId) -> Result<(),S::Error> {
        self.send_message_7( servo_id, Command::Stat )
    }

    pub fn send_rollback(&mut self, servo_id: ServoId, id_skip: IDSkip, baud_skip: BaudSkip ) -> Result<(),S::Error> {
        self.send_message_9(servo_id, Command::Rollback, id_skip.into(), baud_skip.into() )
    }

    #[inline]
    pub fn send_reboot(&mut self, servo_id: ServoId) -> Result<(),S::Error>{
        self.send_message_7( servo_id, Command::Reboot )
    }

    fn send_message_7(&mut self, servo_id: ServoId, command: Command) -> Result<(),S::Error> {
        self.packet_header[Self::CMD_INDEX] = command.into();
        self.packet_header[Self::PID_INDEX] = servo_id.into();
        self.packet_header[Self::LEN_INDEX] = Self::HEADER_LEN as u8;

        self.send::<0>()
    }


    fn send_message_9(&mut self, servo_id: ServoId, command: Command, data0: u8, data1: u8 ) -> Result<(),S::Error>

    {
        self.packet_header[Self::CMD_INDEX] = command.into();
        self.packet_header[Self::PID_INDEX] = servo_id.into();
        self.packet_header[Self::LEN_INDEX] = (Self::HEADER_LEN as u8) + 2;
        self.data[0] = data0;
        self.data[1] = data1;

        self.send::<2>()
    }

    fn send_message_13(&mut self, servo_id: ServoId, command: Command, data0: u8, data1: u8, data2: u8, data3: u8, data4: u8 )
        -> Result<(),S::Error>
     {
        self.packet_header[Self::CMD_INDEX] = command.into();
        self.packet_header[Self::PID_INDEX] = servo_id.into();
        self.packet_header[Self::LEN_INDEX] = (Self::HEADER_LEN as u8) + 5;
        self.data[0] = data0;
        self.data[1] = data1;
        self.data[2] = data2;
        self.data[3] = data3;
        self.data[4] = data4;

        self.send::<5>()
    }

    fn send_message_write_bytes(&mut self, servo_id: ServoId, command: Command, addr: u8, bytes: &[u8]) 
        ->Result<(),S::Error>
      {
        let data_len:u8 = bytes.len() as u8;

        self.packet_header[Self::CMD_INDEX] = command.into();
        self.packet_header[Self::PID_INDEX] = servo_id.into();
        self.packet_header[Self::LEN_INDEX] = (Self::HEADER_LEN as u8) + 2 + data_len;

        self.data[0] = addr.into();
        self.data[1] = data_len;

        self.encode_checksum_write_bytes(bytes);

        for b in self.packet_header {
            nb::block!( self.serial.write(b) )?;
        }
        nb::block!( self.serial.write(self.data[0]) )?;
        nb::block!( self.serial.write(self.data[1]) )?;
        for &b in bytes {
            nb::block!( self.serial.write(b) )?;
        }

        nb::block!( self.serial.flush() )?;

        Ok( () )
    }

    fn encode_checksum_write_bytes(&mut self, bytes: &[u8]) {
        let mut cs1 = self.packet_header[2]^self.packet_header[3]^self.packet_header[4];
        // The first two bytes of self.data contain the address and data length.
        for k in 0..2_usize {
            cs1 = cs1 ^ self.data[ usize::from(k) ];
        }
        for b in bytes {
            cs1 = cs1 ^ b;
        }

        cs1 = cs1 & 0xFE;
        let cs2 = (!(cs1)) & 0xFE;
        self.packet_header[Self::CS1_INDEX] = cs1;
        self.packet_header[Self::CS2_INDEX] = cs2;
    }

    fn send<const N: usize>(&mut self) -> Result<(),S::Error> {
        // Compute checksum
        let mut cs1 = self.packet_header[2]^self.packet_header[3]^self.packet_header[4];
        for k in 0..N {
            cs1 = cs1 ^ self.data[ usize::from(k) ];
        }
        cs1 = cs1 & 0xFE;
        let cs2 = (!(cs1)) & 0xFE;
        self.packet_header[Self::CS1_INDEX] = cs1;
        self.packet_header[Self::CS2_INDEX] = cs2;

        // Send the packet.
        for b in self.packet_header {
            nb::block!( self.serial.write(b) )?;
        }
        for k in 0..N {
            nb::block!( self.serial.write(self.data[k]) )?;
        }
        nb::block!( self.serial.flush() )?;

        Ok( () )
    }

    pub fn set_led_color(&mut self, servo: ServoId, color: Color) -> Result<(),S::Error> {
        self.send_ram_write( servo, AddressU8::LEDControl, color.into() )
    }

    pub fn set_torque(&mut self, servo: ServoId, torque: Torque) -> Result<(),S::Error>{
        self.send_ram_write( servo, AddressU8::TorqueControl, torque.into())
    }

    pub fn set_position(&mut self, servo_id: ServoId, position: u16, playtime: Ticks, color: Color) -> Result<(),S::Error> {
        self.send_sjog_position( servo_id, position, playtime, color )
    }

    pub fn release(self) -> S {
        self.serial
    }

}

pub struct Status {
    error: u8,
    detail: u8
}
impl Status {
    pub fn new(error: u8, detail: u8) -> Self {
        Status{ error: error, detail: detail }
    }

    pub fn has_error(&self) -> bool {
        self.error != 0
    }

    pub fn moving(&self) -> bool {
        self.detail & 0x01 != 0
    }

    pub fn in_position(&self) -> bool {
        self.detail & 0x02 != 0
    }

    pub fn torque_on(&self) -> bool {
        self.detail & 0x40 != 0
    }    
}


#[derive(PartialEq,Copy,Clone,Eq,Ord,PartialOrd,Hash)]
pub enum AckMessageState {
    Pending,
    Corrupt,
    Overflow,
    Complete
}
use AckMessageState::{Pending, Corrupt, Overflow, Complete};

pub enum AckMessage<'a> {
    Pending,
    Malformed,
    // Rollback(AckMessageRollback),
    // Rewind(AckMessageReboot),
    RAMReadU8(AckMessageRAMReadU8<'a>),
    RAMReadU16(AckMessageRAMReadU16<'a>),
    RAMReadOther(AckMessageRAMReadOther<'a>),
    Stat(AckMessageStat<'a>),
    Other(AckMessageOther<'a>),
    // RAMWrite(AckMessageRAMWrite),
    // EEPRead(AckMessageStat),
    // EEPWrite(AckMessageStat),
}
// impl<'a> Debug for AckMessage<'a> {
//    fn fmt(&self, f: &mut Formatter<'_>) -> Result<(), core::fmt::Error> {
//      match self {
//         AckMessage::Malformed => write!(f,"Malformed"),
//         AckMessage::RAMReadU8(_) => write!(f,"RAMReadU8"),
//         AckMessage::RAMReadU16(_) => write!(f,"RAMReadU16"),
//         AckMessage::RAMReadOther(_) => write!(f,"RAMReadOther"),
//         AckMessage::Other(_) => write!(f, "Other"),
//      }
//    }
// }

pub struct AckMessageOther<'a> {
    core: &'a AckMessageReader
}

impl<'a> AckMessageOther<'a> {
    pub fn servo_id(&self) -> ServoId {
        ServoId( self.core.buf[3] )
    }

    pub fn command(&self) -> u8 {
        self.core.buf[4]
    }

    pub fn data(&self) -> &[u8] {
        &self.core.buf[7..self.core.p]
    }

    pub fn status(&self) -> Status {
        Status::new(self.core.buf[self.core.p-2], self.core.buf[self.core.p-1])
    }

}

pub struct AckMessageRAMReadU8<'a> {
    core: &'a AckMessageReader
}

impl<'a> AckMessageRAMReadU8<'a> {
    pub fn servo_id(&self) -> ServoId {
        ServoId( self.core.buf[3] )
    }

    pub fn address(&self) -> Address {
        Address(self.core.buf[7+0])
    }

    pub fn value(&self) -> u8 {
        self.core.buf[7+2]
    }

    pub fn status(&self) -> Status {
        Status::new(self.core.buf[7+3], self.core.buf[7+4])
    }
}

pub struct AckMessageRAMReadU16<'a> {
    core: &'a AckMessageReader
}

impl<'a> AckMessageRAMReadU16<'a> {
    pub fn servo_id(&self) -> ServoId {
        ServoId( self.core.buf[3] )
    }

    pub fn address(&self) -> Address {
        Address(self.core.buf[7])
    }

    pub fn value(&self) -> u16 {
        ((self.core.buf[7+3] as u16)  | ((self.core.buf[7+4] as u16) << 8)).to_le()
    }

    pub fn byte0(&self) -> u8 {
      self.core.buf[7+3]
    }

    pub fn byte1(&self) -> u8 {
      self.core.buf[7+4]
    }

    pub fn status(&self) -> Status {
        Status::new(self.core.buf[7+4], self.core.buf[7+5])
    }
}

pub struct AckMessageRAMReadOther<'a> {
    core: &'a AckMessageReader
}

impl<'a> AckMessageRAMReadOther<'a> {
    pub fn servo_id(&self) -> ServoId {
        ServoId( self.core.buf[3] )
    }

    pub fn address(&self) -> Address {
        Address(self.core.buf[7])
    }

    pub fn data(&self) -> &[u8] {
        &self.core.buf[7+2..(self.core.p-2)]
    }

    pub fn status(&self) -> Status {
        Status::new(self.core.buf[self.core.p-2], self.core.buf[self.core.p-1])
    }
}
pub struct AckMessageStat<'a> {
    core: &'a AckMessageReader    
}
impl<'a> AckMessageStat<'a> {
    pub fn servo_id(&self) -> ServoId {
        ServoId( self.core.buf[3] )
    }

    pub fn status(&self) -> Status {
        Status::new(self.core.buf[7], self.core.buf[8])
    }
}



// TODO: pick the buffer size, up to 256 bytes
const LEN:usize = 32;
pub struct AckMessageReader {
    buf:[u8; 32],
    p: usize,
    state: AckMessageState
}
impl AckMessageReader {

    pub const fn new() -> Self {
        let mut rv = AckMessageReader {
            buf: [0;32],
            p:0,
            state: Pending
        };
        rv.buf[0] = 0xFF;
        rv.buf[1] = 0xFF;
        rv
    }

    pub fn clear(&mut self) {
        self.p = 0;
        self.state = Pending;
    }

    pub fn is_pending(&self) -> bool {
        self.state == Pending
    }

    pub fn is_corrupt(&self) -> bool {
        self.state == Corrupt
    }

    pub fn is_overflow(&self) -> bool {
        self.state == Overflow
    }

    pub fn is_complete(&self) -> bool {
        self.state == Complete
    }

    pub fn push(&mut self, b: u8) -> bool {
        if self.state != Pending {
            return false;
        }

        if self.p < 2 {
            if b != 0xFF {
                self.p = 0;
            } else {
                self.p += 1;
            }
            return true;
        }

        self.buf[self.p] = b;
        
        self.p += 1;
        if self.p == self.buf[2].into() {

            let mut cs1 = self.buf[2]^self.buf[3]^self.buf[4];

            for b in  &self.buf[7..(self.p as usize)] {
                cs1 = cs1^b;
            }
            cs1 = cs1 & 0xFE;
            let cs2 = !cs1 & 0xFE;

            if (cs1 != self.buf[5]) || (cs2 != self.buf[6]) {
                self.state = Corrupt;
            } else {
                self.state = Complete;
            }

        } else if self.p == LEN {
            self.state = Overflow;
        }
        true
    }

    pub fn push_from<S>(&mut self, serial: &mut S) -> Result<(),S::Error>
    where S: Read<u8> {
        while self.state == Pending {
            match serial.read() {
                Err(nb::Error::WouldBlock) => break,
                Err(nb::Error::Other(e)) => return Err(e),
                Ok(b) => {
                    self.push(b);
                }
            }
        }
        Ok(())
    }

    pub fn message(&self) -> AckMessage {
        if self.state == Pending {
            return AckMessage::Pending;
        } else if self.state != Complete {
            return AckMessage::Malformed;
        } else if self.buf[4] == (Command::RAMRead as u8) | 0x40 {
            match self.buf[2] {
                12 => return AckMessage::RAMReadU8(AckMessageRAMReadU8{ core: self}),
                13 => return AckMessage::RAMReadU16( AckMessageRAMReadU16{core: self}),
                _ => return AckMessage::RAMReadOther( AckMessageRAMReadOther{ core: self } )
            }
        } else if self.buf[4] == (Command::Stat as u8) | 0x40 {
            return AckMessage::Stat(AckMessageStat{ core: self } );
        }
        return AckMessage::Other(AckMessageOther{ core: self } )
    }
}




#[cfg(test)]
mod tests {
    use super::*;
    use AddressU8::{LEDControl,TorqueControl,StatusError};

    struct FakeSerial(Vec<u8>);

    impl FakeSerial {
        fn new() -> Self {
            FakeSerial( vec![] )
        }
    }

    impl Write<u8> for FakeSerial {
        type Error = ();

        // type Error = ();
        fn write( &mut self, b: u8 ) -> nb::Result<(), ()> {
            self.0.push( b );
            Ok( () )
        }

        fn flush( &mut self ) -> nb::Result<(), ()> 
        {
            Ok( () )
        }
    }


    #[test]
    fn stat() {

        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );

        let servo = ServoId::default();
        tx.send_stat( servo ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x07, 0xFD, 0x07, 0xFC, 0x02 ];

        assert_eq!( serial.0, packet );
    }

    #[test]
    fn reboot() {

        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );
        let servo = ServoId::default();

        tx.send_reboot( servo ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x07, 0xFD, 0x09, 0xF2, 0x0C ];

        assert_eq!( serial.0, packet );
    }

    #[test]
    fn ram_write_u8_led() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );
        let servo = ServoId::default();

        tx.send_ram_write( servo, LEDControl, Color::Green.into() ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x0A, 0xFD, 0x03, 0xC0, 0x3E, 0x35, 0x01, 0x01 ];

        assert_eq!( serial.0, packet );        
    }

    #[test]
    fn ram_write_led_easy() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );
        let servo = ServoId::default();

        tx.set_led_color( servo, Color::Green ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x0A, 0xFD, 0x03, 0xC0, 0x3E, 0x35, 0x01, 0x01 ];

        assert_eq!( serial.0, packet );        
    }


    #[test]
    fn ram_write_u8_torque() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );
        let servo = ServoId::default();

        tx.send_ram_write( servo, TorqueControl, Torque::Powered.into() ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x0A, 0xFD, 0x03, 0xA0, 0x5E, 0x34, 0x01, 0x60 ];

        assert_eq!( serial.0, packet );        
    }

    #[test]
    fn ram_write_torque_easy() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );
        let servo = ServoId::default();

        tx.set_torque( servo, Torque::Powered ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x0A, 0xFD, 0x03, 0xA0, 0x5E, 0x34, 0x01, 0x60 ];

        assert_eq!( serial.0, packet );        
    }

    #[test]
    fn ram_write_raw() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );
        let servo = ServoId::default();

        tx.send_ram_write_bytes( servo, StatusError, &0_u16.to_le_bytes() ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x0B, 0xFD, 0x03, 0xC6, 0x38, 0x30, 0x02, 0x00, 0x00 ];

        assert_eq!( serial.0, packet );        
    }

    #[test]
    fn ram_read_u8_led() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );
        let servo = ServoId::default();

        tx.send_ram_read( servo, LEDControl ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x09, 0xFD, 0x04, 0xC4, 0x3A, 0x35, 0x01];

        assert_eq!( serial.0, packet );        
    }

    #[test]
    fn rollback() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );
        let servo = ServoId::default();

        tx.send_rollback( servo, IDSkip(true), BaudSkip(true)).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x09, 0xFD, 0x08, 0xFC, 0x02, 0x01, 0x01];

        assert_eq!( serial.0, packet );        
    }

    #[test]
    fn eep_read_raw() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );
        let servo = ServoId::default();
        use EEPAddressU16::PositionKp;

        tx.send_eep_read_raw( servo, PositionKp.into(), 4).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x09, 0xFD, 0x02, 0xEC, 0x12, 0x1E, 0x04];

        assert_eq!( serial.0, packet );                
    }

    #[test]
    fn eep_write_bytes() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );
        let servo = ServoId::default();
        use EEPAddressU16::PositionKp;

        let data = [ 0xC8, 0x00, 0xE8, 0x03 ];
        tx.send_eep_write_bytes( servo, PositionKp, &data).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x0D, 0xFD, 0x01, 0xC8, 0x36, 0x1E, 0x04, 0xC8, 0x00, 0xE8, 0x03 ];

        assert_eq!( serial.0, packet );                
    }

    #[test]
    fn sjog_position() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );

        let servo = ServoId::default();
        let position = 512;
        tx.send_sjog_position( servo, position, Ticks(60), Color::Green ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x0C, 0xFD, 0x06, 0x30, 0xCE, 0x3C, 0x00, 0x02, 0x04, 0xFD ];

        assert_eq!( serial.0, packet );                
        
    }

    #[test]
    fn sjog_cts() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );

        let pwm = 320;
        tx.send_sjog_continuous( ServoId::default(), pwm, Ticks(60), Color::Blue ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x0C, 0xFD, 0x06, 0x7C, 0x82, 0x3C, 0x40, 0x01, 0x0A, 0xFD ];

        assert_eq!( serial.0, packet );        
    }

    #[test]
    fn ijog_position() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );

        let servo = ServoId::default();
        let position = 512;
        tx.send_ijog_position( servo, position, Ticks(60), Color::Green ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x0C, 0xFD, 0x05, 0x32, 0xCC, 0x00, 0x02, 0x04, 0xFD, 0x3C  ];

        assert_eq!( serial.0, packet );                
        
    }

    #[test]
    fn ijog_cts() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );

        let pwm = 320;
        tx.send_ijog_continuous( ServoId::default(), pwm, Ticks(60), Color::Blue ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x0C, 0xFD, 0x05, 0x7E, 0x80, 0x40, 0x01, 0x0A, 0xFD, 0x3C ];

        assert_eq!( serial.0, packet );        
    }

    #[test]
    fn ack_message() {
        let packet:[u8;15] = [0xFF, 0xFF, 0x0F, 0xFD, 0x42, 0x4C, 0xB2, 0x1E, 0x04, 0xB8, 0x01, 0x40, 0x1F, 0x00, 0x00];
        let mut r = AckMessageReader::new();
        for (k,&c) in packet.iter().enumerate() {
            r.push(c);
            if k+1 < packet.len() {
                assert!(r.is_pending());
            } else {
                assert!(r.is_complete());
            }
        }
        let m = r.message();

        if let AckMessage::Other(m) = m{
            assert!(m.servo_id() == ServoId::default());
            assert!(!m.status().has_error());
            assert!(!m.status().moving());
        } else {
            unreachable!()
        }
    }

    #[test]
    fn ack_message_leading_garbage() {
        let packet:[u8;17] = [0xC1, 0x1F, 0xFF, 0xFF, 0x0F, 0xFD, 0x42, 0x4C, 0xB2, 0x1E, 0x04, 0xB8, 0x01, 0x40, 0x1F, 0x00, 0x00];
        let mut r = AckMessageReader::new();
        for (k,&c) in packet.iter().enumerate() {
            r.push(c);
            if k+1 < packet.len() {
                assert!(r.is_pending());
            } else {
                assert!(r.is_complete());
            }
        }
        let m = r.message();

        if let AckMessage::Other(m) = m{
            assert!(m.servo_id() == ServoId::default());
            assert!(!m.status().has_error());
            assert!(!m.status().moving());
        } else {
            unreachable!()
        }
    }

    #[test]
    fn ack_message_checksum() {
        let packet:[u8;17] = [0xC1, 0x1F, 0xFF, 0xFF, 0x0F, 0xFD, 0x40, 0x4C, 0xB2, 0x1E, 0x04, 0xB8, 0x01, 0x40, 0x1F, 0x00, 0x00];
        let mut r = AckMessageReader::new();
        for (k,&c) in packet.iter().enumerate() {
            r.push(c);
            if k+1 < packet.len() {
                assert!(r.is_pending());
            } else {
                assert!(r.is_corrupt());
            }
        }        
    }

    #[test]
    fn ack_message_overflow() {
        let mut packet:[u8;33] = [0;33];
        packet[0] = 0xFF;
        packet[1] = 0xFF;
        let mut r = AckMessageReader::new();
        for c in packet {
            r.push(c);
        }
        assert!(r.is_overflow());
    }

    #[test]
    fn ack_message_ramreadu8() {
        let packet:[u8;12] = [ 0xFF, 0xFF, 0x0C, 0xFD, 0x44, 0xC2, 0x3C, 0x035, 0x01, 0x01, 0x00, 0x42];
        let mut r = AckMessageReader::new();
        for c in packet {
            r.push(c);
        }
        assert!(r.is_complete());

        let m = match r.message() {
            AckMessage::RAMReadU8(m) => m,
            _ => unreachable!()
        };
        assert!( m.servo_id() == ServoId::default() );
        assert!( m.address() == LEDControl.into() );
        assert!( m.value() == Color::Green.into() );
        let s = m.status();
        assert!(!s.has_error());
        assert!(s.in_position());
        assert!(s.torque_on());
    }

    #[test]
    fn ack_message_stat() {
        let packet:[u8;9] = [ 0xFF, 0xFF, 0x09, 0xFD, 0x47, 0xF2, 0x0C, 0x00, 0x40];
        let mut r = AckMessageReader::new();
        for c in packet {
            r.push(c);
        }
        assert!(r.is_complete());

        let m = match r.message() {
            AckMessage::Stat(m) => m,
            _ => unreachable!()
        };
        assert!( m.servo_id() == ServoId::default() );
        let s = m.status();
        assert!(!s.has_error());
        assert!(!s.in_position());
        assert!(!s.moving());
        assert!(s.torque_on());
    }

}
