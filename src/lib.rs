use embedded_hal::serial::Write;

mod address;
pub use address::{Address,AddressU8, AddressU16, ReadOnlyAddressU8, ReadOnlyAddressU16};
pub use address::{ReadableAddress, WritableAddress};
pub use address::{EEPAddress,EEPAddressU8, EEPAddressU16, ReadOnlyEEPAddressU8};
pub use address::{ReadableEEPAddress, WritableEEPAddress};

#[derive(Debug)]
pub enum Error {
    InvalidColorValue
}



#[repr(u8)]
enum Command {
    EEPWrite  = 0x01,
    EEPRead   = 0x02,
    RAMWrite  = 0x03,
    RAMRead   = 0x04,
    // IJog      = 0x05,
    SJog      = 0x06,
    Stat      = 0x07,
    Rollback  = 0x08,
    Reboot    = 0x09
}

#[repr(u8)]
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
pub enum JogMode {
    Position,
    Velocity
}

struct SJogFlags(u8);
impl SJogFlags {
    fn new( mode: JogMode, color: Color ) -> Self {
        SJogFlags( ( (mode as u8) << 1 ) | (color as u8) << 2 )
    }

    // fn color(&self) -> Color {
    //     ((self.0 & 0b00011100) >> 2).try_into().unwrap()
    // }
}
impl Into<u8> for SJogFlags {
    fn into(self) -> u8 {
        self.0
    }
}



pub struct ServoId(u8);
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




pub struct IDSkip(bool);
pub struct BaudSkip(bool);


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
        // // It's a bug to try to write more than 214 bytes of data.
        // assert!(data.len() < 215);

        self.send_eep_write_bytes_raw(servo_id, addr.into(), data)        
    }

    pub fn send_eep_write_bytes_raw(&mut self, servo_id: ServoId, addr: EEPAddress, data: &[u8]) -> Result<(),S::Error> {

        let data_len:u8 = data.len() as u8;

        self.packet_header[Self::CMD_INDEX] = Command::EEPWrite as u8;
        self.packet_header[Self::PID_INDEX] = servo_id.into();
        self.packet_header[Self::LEN_INDEX] = (Self::HEADER_LEN as u8) + 2 + data_len;

        self.data[0] = addr.into();
        self.data[1] = data_len;

        self.encode_checksum_extra(2,data);

        for b in self.packet_header {
            nb::block!( self.serial.write(b) )?;
        }
        nb::block!( self.serial.write(self.data[0]) )?;
        nb::block!( self.serial.write(self.data[1]) )?;
        for &b in data {
            nb::block!( self.serial.write(b) )?;
        }

        nb::block!( self.serial.flush() )?;

        Ok( () )
    }

    #[inline]
    pub fn send_eep_read<T: ReadableEEPAddress>(&mut self, servo_id: ServoId, addr: T ) -> Result<(), S::Error>  {
        self.send_eep_read_raw(servo_id, addr.into(), core::mem::size_of::<T::Value>() as u8)
    }

    pub fn send_eep_read_raw(&mut self, servo_id: ServoId, addr: EEPAddress, byte_count: u8) -> Result<(),S::Error> {
        const DATA_LEN:usize = 2;

        self.packet_header[Self::CMD_INDEX] = Command::EEPRead as u8;
        self.packet_header[Self::PID_INDEX] = servo_id.into();
        self.packet_header[Self::LEN_INDEX] = (Self::HEADER_LEN + DATA_LEN) as u8;

        self.data[0] = addr.into();
        self.data[1] = byte_count;

        self.send::<DATA_LEN>()
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

        // // It's a bug to try to write more than 214 bytes of data.
        // assert!(data.len() < 215);

        let data_len:u8 = data.len() as u8;

        self.packet_header[Self::CMD_INDEX] = Command::RAMWrite as u8;
        self.packet_header[Self::PID_INDEX] = servo_id.into();
        self.packet_header[Self::LEN_INDEX] = (Self::HEADER_LEN as u8) + 2 + data_len;

        self.data[0] = addr.into();
        self.data[1] = data_len;

        self.encode_checksum_extra(2,data);

        for b in self.packet_header {
            nb::block!( self.serial.write(b) )?;
        }
        nb::block!( self.serial.write(self.data[0]) )?;
        nb::block!( self.serial.write(self.data[1]) )?;
        for &b in data {
            nb::block!( self.serial.write(b) )?;
        }

        nb::block!( self.serial.flush() )?;

        Ok( () )
    }


    #[inline]
    pub fn send_ram_read<T: ReadableAddress>(&mut self, servo_id: ServoId, addr: T ) -> Result<(), S::Error>  {
        self.send_ram_read_raw(servo_id, addr.into(), core::mem::size_of::<T::Value>() as u8)
    }

    pub fn send_ram_read_raw(&mut self, servo_id: ServoId, addr: Address, byte_count: u8) -> Result<(),S::Error> {
        const DATA_LEN:usize = 2;

        self.packet_header[Self::CMD_INDEX] = Command::RAMRead as u8;
        self.packet_header[Self::PID_INDEX] = servo_id.into();
        self.packet_header[Self::LEN_INDEX] = (Self::HEADER_LEN + DATA_LEN) as u8;

        self.data[0] = addr.into();
        self.data[1] = byte_count;

        self.send::<DATA_LEN>()
    }

    pub fn send_sjog_position(&mut self, servo_id: ServoId, position: u16, playtime: u8, color: Color )  -> Result<(),S::Error> {
        let servo_id:u8 = servo_id.into();
        const DATA_LEN:usize = 5;

        self.packet_header[Self::CMD_INDEX] = Command::SJog as u8;
        self.packet_header[Self::PID_INDEX] = servo_id;
        self.packet_header[Self::LEN_INDEX] = (Self::HEADER_LEN + DATA_LEN) as u8;

        self.data[0] = playtime as u8;
        self.data[1] = (position & 0xFF) as u8;
        self.data[2] = (position >> 8) as u8;
        self.data[3] = SJogFlags::new( JogMode::Position, color).into();
        self.data[4] = servo_id;

        self.send::<DATA_LEN>()
    }

    pub fn send_sjog_continuous(&mut self, servo_id: ServoId, pwm: u16, playtime: u8, color: Color )  -> Result<(),S::Error> {
        const DATA_LEN:usize = 5;

        let servo_id:u8 = servo_id.into();

        self.packet_header[Self::CMD_INDEX] = Command::SJog as u8;
        self.packet_header[Self::PID_INDEX] = servo_id;
        self.packet_header[Self::LEN_INDEX] = (Self::HEADER_LEN  + DATA_LEN) as u8;

        self.data[0] = playtime as u8;
        self.data[1] = (pwm & 0xFF) as u8;
        self.data[2] = (pwm >> 8) as u8;
        self.data[3] = SJogFlags::new( JogMode::Velocity, color).into();
        self.data[4] = servo_id;

        self.send::<DATA_LEN>( )
    }

    pub fn send_stat(&mut self, servo_id: ServoId) -> Result<(),S::Error> {
        self.packet_header[Self::CMD_INDEX] = Command::Stat as u8;
        self.packet_header[Self::PID_INDEX] = servo_id.into();
        self.packet_header[Self::LEN_INDEX] = Self::HEADER_LEN as u8;

        self.send::<0>()
    }

    pub fn send_rollback(&mut self, servo_id: ServoId, id_skip: IDSkip, baud_skip: BaudSkip ) -> Result<(),S::Error> {
        const DATA_LEN:usize = 2;

        self.packet_header[Self::CMD_INDEX] = Command::Rollback as u8;
        self.packet_header[Self::PID_INDEX] = servo_id.into();
        self.packet_header[Self::LEN_INDEX] = (Self::HEADER_LEN + DATA_LEN) as u8;

        self.data[0] = id_skip.0 as u8;
        self.data[1] = baud_skip.0 as u8;

        self.send::<DATA_LEN>()
    }

    pub fn send_reboot(&mut self, servo_id: ServoId) -> Result<(),S::Error>{
        self.packet_header[Self::CMD_INDEX] = Command::Reboot as u8;
        self.packet_header[Self::PID_INDEX] = servo_id.into();
        self.packet_header[Self::LEN_INDEX] = Self::HEADER_LEN as u8;

        self.send::<0>()
    }

    fn encode_checksum<const N: usize>( &mut self ) {
        let mut cs1 = self.packet_header[2]^self.packet_header[3]^self.packet_header[4];
        for k in 0..N {
            cs1 = cs1 ^ self.data[ usize::from(k) ];
        }
        cs1 = cs1 & 0xFE;
        let cs2 = (!(cs1)) & 0xFE;
        self.packet_header[Self::CS1_INDEX] = cs1;
        self.packet_header[Self::CS2_INDEX] = cs2;
    }

    fn encode_checksum_extra(&mut self, p: u8, extra: &[u8]) {
        let mut cs1 = self.packet_header[2]^self.packet_header[3]^self.packet_header[4];
        for k in 0..p {
            cs1 = cs1 ^ self.data[ usize::from(k) ];
        }
        for b in extra {
            cs1 = cs1 ^ b;
        }

        cs1 = cs1 & 0xFE;
        let cs2 = (!(cs1)) & 0xFE;
        self.packet_header[Self::CS1_INDEX] = cs1;
        self.packet_header[Self::CS2_INDEX] = cs2;
    }

    fn send<const N: usize>(&mut self) -> Result<(),S::Error> {
        self.encode_checksum::<N>();

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

    pub fn set_torque_on(&mut self, servo: ServoId) -> Result<(),S::Error>{
        self.send_ram_write( servo, AddressU8::TorqueControl, 0x60)
    }

    // pub fn set_position(&mut self, servo_id: ServoId, pos: u16) -> Result<(),S::Error> {
    //     let servo_id:u8 = servo_id.into();
    //     self.packet_header[Self::CMD_INDEX] = Command::SJog as u8;
    //     self.packet_header[Self::PID_INDEX] = servo_id;
    //     self.packet_header[Self::LEN_INDEX] = (Self::HEADER_LEN as u8) + 5;

    //     let playtime = 60;

    //     let jog_lsb = (pos & 0xFF) as u8;
    //     let jog_msb = ((pos >> 8) & 0xFF) as u8;

    //     let set = 0x04;

    //     self.data[0] = playtime;
    //     self.data[1] = jog_lsb;
    //     self.data[2] = jog_msb;
    //     self.data[3] = set;
    //     self.data[4] = servo_id;

    //     self.encode_checksum(5);

    //     for b in self.packet_header {
    //         nb::block!( self.serial.write(b) )?;
    //     }
    //     nb::block!( self.serial.write(self.data[0]) )?;
    //     nb::block!( self.serial.write(self.data[1]) )?;
    //     nb::block!( self.serial.write(self.data[2]) )?;
    //     nb::block!( self.serial.write(self.data[3]) )?;
    //     nb::block!( self.serial.write(self.data[4]) )?;

    //     Ok( () )
    // }

    pub fn release(self) -> S {
        self.serial
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
    fn ram_write_u8_led_easy() {
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

        tx.send_ram_write( servo, TorqueControl, 0x60).unwrap();

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
        let playtime = 60;
        tx.send_sjog_position( servo, position, playtime, Color::Green.into() ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x0C, 0xFD, 0x06, 0x30, 0xCE, 0x3C, 0x00, 0x02, 0x04, 0xFD ];

        assert_eq!( serial.0, packet );                
        
    }

    #[test]
    fn sjog_cts() {
        let serial = FakeSerial::new();

        let mut tx = MessageTransmitter::new( serial );

        let pwm = 320;
        let color = Color::Blue;
        let playtime = 60;
        tx.send_sjog_continuous( ServoId::default(), pwm, playtime, color ).unwrap();

        let serial = tx.release();

        let packet = [0xFF, 0xFF, 0x0C, 0xFD, 0x06, 0x7C, 0x82, 0x3C, 0x40, 0x01, 0x0A, 0xFD ];

        assert_eq!( serial.0, packet );        
    }

}
