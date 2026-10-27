use std::cmp::PartialEq;
use std::panic::UnwindSafe;
use crate::control::hardware::LynxHub;
use crate::serialization::i2c_comms::i2c_device::I2CDevice;
use crate::serialization::i2c_comms::octoquad_i2c::OctoQuadRegisters::*;
use crate::serialization::i2c_comms::octoquad_i2c::RegisterType::{Int16, Int32, UInt16, UInt8};
use crate::serialization::i2c_comms::pinpoint_i2c::{PinpointI2C, PinpointSnapshot};
use crate::serialization::lynx_commands::base_lynx_command::LynxCommand;
use crate::serialization::lynx_commands::lynx_commands::LynxI2CWriteReadMultipleBytesCommandData;
use crossbeam_channel::Sender;
use num_enum::{IntoPrimitive, TryFromPrimitive};
use std::sync::{Arc, Mutex};
use std::time::Instant;
use bitflags::{bitflags, Flags};
use strum_macros::EnumIter;

pub struct OctoQuadI2C {
	hub: &'static LynxHub,
	bus: u8,
	i2c_addr: u8,
	packets_in_flight: Arc<Mutex<Vec<u8>>>,
	pub mistake_alert_sender: Option<Sender<Instant>>,
	read_mode: ReadMode
}
impl I2CDevice<OctoQuadSnapshot, OctoQuadRegisters> for OctoQuadI2C {
	fn get_location(&self) -> (&'static LynxHub, u8, u8) {
		(self.hub, self.bus, self.i2c_addr)
	}

	fn get_read_mode(&self) -> u8 {
		self.read_mode.bits()
	}

	fn expected_read_len(&self) -> usize {
		let (start, end) = OctoQuadRegisters::get_from_readmode(self.read_mode);
		(end.addr() + end.get_len() - start.addr()) as usize
	}

	fn get_packet_utils(&self) -> (&Arc<Mutex<Vec<u8>>>, &Option<Sender<Instant>>) {
		(&self.packets_in_flight, &self.mistake_alert_sender)
	}
}
impl OctoQuadI2C {
	pub fn new(hub: &'static LynxHub, bus: u8, i2c_addr: u8, read_mode: ReadMode) -> Self {
		let read_mode = ReadMode::from_bits_truncate(read_mode.bits());
		let read_mode = if read_mode.is_empty() {
			log::error!("octoquad read mode has no data selected, reading the localizer");
			ReadMode::LOCALIZER
		} else { read_mode };
		Self {
			hub,
			bus,
			i2c_addr,
			packets_in_flight: Arc::new(Mutex::new(vec![])),
			mistake_alert_sender: None,
			read_mode
		}
	}
	pub fn fire_bulk_read_request(&self) {
		log::trace!("firing octoquad bulk read req");
		let cmd = self.get_read_cmd();
		let pif = self.hub.send_lynx_packet(cmd);
		self.add_pif(pif);
	}
	pub fn fire_bulk_read_req_func(&self) -> Box<dyn FnMut() + 'static + Send + Sync + UnwindSafe> {
		let quad = OctoQuadI2C {
			hub: self.hub,
			bus: self.bus,
			i2c_addr: self.i2c_addr,
			packets_in_flight: self.packets_in_flight.clone(),
			mistake_alert_sender: None,
			read_mode: self.read_mode
		};
		Box::new(move || {
			quad.fire_bulk_read_request();
		})
	}
	///reads every register the read mode needs in one contiguous transaction
	pub const fn get_read_cmd(&self) -> LynxCommand {
		let (start, end) = OctoQuadRegisters::get_from_readmode(self.read_mode);
		self.get_read_cmd_for_registers(start, end)
	}
	pub const fn get_read_cmd_for_registers(&self, start: OctoQuadRegisters, end: OctoQuadRegisters) -> LynxCommand {
		let start_addr = start.addr();
		let addr_end = end.addr() + end.get_len();
		let bytes = addr_end - start_addr;
		self.get_read_lynx_command(start_addr, bytes)
	}
	pub const fn get_read_lynx_command(&self, i2c_start_addr: u8, bytes_to_read: u8) -> LynxCommand {
		LynxCommand::LynxI2CWriteReadMultipleBytesCommand(LynxI2CWriteReadMultipleBytesCommandData {
			i2c_bus: self.bus,
			i2c_addr_7bit: self.i2c_addr,
			bytes_to_read,
			i2c_start_addr
		})
	}
}


enum RegisterType {
	UInt8,
	Int32,
	Int16,
	UInt16,
	Float32
}
impl RegisterType {
	fn get_len(&self) -> u8 {
		match self {
			UInt8 => {1}
			Int32 => {4}
			Int16 => {2}
			UInt16 => {2}
			RegisterType::Float32 => {4}
		}
	}
}
#[derive(Copy, Clone, Debug)]
pub struct OctoQuadSnapshot {
	pub localizer: Option<OctoQuadLocalizerData>,
	pub positions: Option<EncoderPosSnapshot>,
	pub velocities: Option<EncoderVelSnapshot>,
}
impl From<(u8, Vec<u8>)> for OctoQuadSnapshot {
	fn from((rm, value): (u8, Vec<u8>)) -> Self {
		let rm = ReadMode::from_bits_truncate(rm);
		//get_read_cmd reads one contiguous block, so every register is at its address minus the first one's.
		//(this also handles LOCALIZER | VELOCITY, where the positions in between are read but ignored)
		let (start, _) = OctoQuadRegisters::get_from_readmode(rm);
		let block = |register: OctoQuadRegisters, len: usize| {
			let offset = (register.addr() - start.addr()) as usize;
			value.get(offset..offset + len)
		};
		let localizer = if rm.contains(ReadMode::LOCALIZER) {
			block(LOCALIZER_STATUS, OctoQuadLocalizerData::LEN).map(OctoQuadLocalizerData::from)
		} else {None};
		let positions = if rm.contains(ReadMode::POSITIONS) {
			block(ENCODER_0_POSITION, 8 * Int32.get_len() as usize).map(|bytes| {
				let mut pos = [0; 8];
				for (pos, bytes) in pos.iter_mut().zip(bytes.chunks_exact(4)) {
					*pos = i32::from_le_bytes(bytes.try_into().unwrap());
				}
				EncoderPosSnapshot {pos}
			})
		} else {None};
		let velocities = if rm.contains(ReadMode::VELOCITY) {
			block(ENCODER_0_VELOCITY, 8 * Int16.get_len() as usize).map(|bytes| {
				let mut vel = [0; 8];
				for (vel, bytes) in vel.iter_mut().zip(bytes.chunks_exact(2)) {
					*vel = i16::from_le_bytes(bytes.try_into().unwrap());
				}
				EncoderVelSnapshot {vel}
			})
		} else {None};
		Self {localizer, positions, velocities}
	}
}
impl Into<Vec<u8>> for OctoQuadSnapshot {
	fn into(self) -> Vec<u8> {
		let mut total = vec![];
		if let Some(it) = &self.localizer {
			total.extend_from_slice(&it.into());
		}
		if let Some(it) = &self.positions {
			it.pos.iter().for_each(|it| total.extend_from_slice(&it.to_le_bytes()));
		}
		if let Some(it) = &self.velocities {
			it.vel.iter().for_each(|it| total.extend_from_slice(&it.to_le_bytes()));
		}
		total
	}
}
#[derive(Clone, Copy, Debug)]
pub struct OctoQuadLocalizerData {
	pub localizer_status: LocalizerStatus,
	pub vel_x_mm_s: i16,
	pub vel_y_mm_s: i16,
	pub vel_heading_rad_s: f32,
	pub pos_x_mm: i16,
	pub pos_y_mm: i16,
	pub heading_rad: f32,
	pub crc: u16,
}
const SCALAR_LOCALIZER_HEADING_VELOCITY: f32 = 1.0/600.0;
const SCALAR_LOCALIZER_HEADING: f32 = 1.0/5000.0;
impl OctoQuadLocalizerData {
	///LOCALIZER_STATUS through LOCALIZER_CRC16, inclusive
	const LEN: usize = 15;
	///value must be at least LEN bytes
	fn from(value: &[u8]) -> Self {
		let localizer_status = value[0].try_into().unwrap_or(LocalizerStatus::INVALID);
		let vel_x_mm_s = i16::from_le_bytes(value[1..3].try_into().unwrap());
		let vel_y_mm_s = i16::from_le_bytes(value[3..5].try_into().unwrap());
		let vel_heading_rad_s = i16::from_le_bytes(value[5..7].try_into().unwrap()) as f32 * SCALAR_LOCALIZER_HEADING_VELOCITY;
		let pos_x_mm = i16::from_le_bytes(value[7..9].try_into().unwrap());
		let pos_y_mm = i16::from_le_bytes(value[9..11].try_into().unwrap());
		let heading_rad = i16::from_le_bytes(value[11..13].try_into().unwrap()) as f32 * SCALAR_LOCALIZER_HEADING;
		let crc = u16::from_le_bytes(value[13..15].try_into().unwrap());
		Self {localizer_status, vel_x_mm_s, vel_y_mm_s, vel_heading_rad_s, pos_x_mm, pos_y_mm, heading_rad, crc}
	}
	///note: this leaves out the crc
	fn into(&self) -> Vec<u8> {
		let mut start = vec![];
		start.extend_from_slice(&[self.localizer_status as u8]);
		start.extend_from_slice(&self.vel_x_mm_s.to_le_bytes());
		start.extend_from_slice(&self.vel_y_mm_s.to_le_bytes());
		start.extend_from_slice(&((self.vel_heading_rad_s / SCALAR_LOCALIZER_HEADING_VELOCITY).round() as i16).to_le_bytes());
		start.extend_from_slice(&self.pos_x_mm.to_le_bytes());
		start.extend_from_slice(&self.pos_y_mm.to_le_bytes());
		start.extend_from_slice(&((self.heading_rad / SCALAR_LOCALIZER_HEADING).round() as i16).to_le_bytes());
		start
	}
}
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable, Debug)]
#[repr(C)]
pub struct EncoderPosSnapshot {
	pub pos: [i32; 8]
}
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable, Debug)]
#[repr(C)]
pub struct EncoderVelSnapshot {
	pub vel: [i16; 8]
}
#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, TryFromPrimitive, IntoPrimitive)]
enum LocalizerStatus
{
	INVALID = 0,
	NOT_INITIALIZED = 1,
	WARMING_UP_IMU = 2,
	CALIBRATING_IMU = 3,
	RUNNING = 4,
	FAULT_NO_IMU = 5
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, TryFromPrimitive, IntoPrimitive)]
#[derive(EnumIter)]
pub enum OctoQuadRegisters {
	CHIP_ID = 0x00,
	FIRMWARE_VERSION_MAJOR = 0x01,
	FIRMWARE_VERSION_MINOR = 0x02,
	FIRMWARE_VERSION_ENGINEERING = 0x03,
	COMMAND = 0x04,
	COMMAND_DAT_0 = 0x05,
	COMMAND_DAT_1 = 0x06,
	COMMAND_DAT_2 = 0x07,
	COMMAND_DAT_3 = 0x08,
	COMMAND_DAT_4 = 0x09,
	COMMAND_DAT_5 = 0x0A,
	COMMAND_DAT_6 = 0x0B,

	LOCALIZER_YAW_AXIS = 0x0C,
	LOCALIZER_STATUS = 0x0D,
	LOCALIZER_VX = 0x0E,
	LOCALIZER_VY = 0x10,
	LOCALIZER_VH = 0x12,
	LOCALIZER_X = 0x14,
	LOCALIZER_Y = 0x16,
	LOCALIZER_H = 0x18,
	LOCALIZER_CRC16 = 0x1A,

	ENCODER_0_POSITION = 0x1C,
	ENCODER_1_POSITION = 0x20,
	ENCODER_2_POSITION = 0x24,
	ENCODER_3_POSITION = 0x28,
	ENCODER_4_POSITION = 0x2C,
	ENCODER_5_POSITION = 0x30,
	ENCODER_6_POSITION = 0x34,
	ENCODER_7_POSITION = 0x38,

	ENCODER_0_VELOCITY = 0x3C,
	ENCODER_1_VELOCITY = 0x3E,
	ENCODER_2_VELOCITY = 0x40,
	ENCODER_3_VELOCITY = 0x42,
	ENCODER_4_VELOCITY = 0x44,
	ENCODER_5_VELOCITY = 0x46,
	ENCODER_6_VELOCITY = 0x48,
	ENCODER_7_VELOCITY = 0x4A,

	ENCODER_DATA_CRC16 = 0x4C,
	UNKNOWN = 0x4E
}
#[derive(PartialEq, Eq, Copy, Clone)]
pub struct ReadMode(pub u8);
bitflags! {
    impl ReadMode: u8 {
        const LOCALIZER = 0b00000001;
        const POSITIONS = 0b00000010;
        const VELOCITY  = 0b00000100;
    }
}

impl OctoQuadRegisters {
	const fn min(&mut self, other: Self) {
		if other.addr() < self.addr() {
			*self = other
		}
	}
	const fn max(&mut self, other: Self) {
		if other.addr() > self.addr() {
			*self = other
		}
	}
	///the first and last register a read mode needs. an empty read mode reads the localizer
	const fn get_from_readmode(rm: ReadMode) -> (Self, Self) {
		if rm.is_empty() {
			return (LOCALIZER_STATUS, LOCALIZER_CRC16)
		}
		let mut start = UNKNOWN;
		let mut end = CHIP_ID;
		if rm.contains(ReadMode::LOCALIZER) {
			start.min(LOCALIZER_STATUS);
			end.max(LOCALIZER_CRC16);
		}
		if rm.contains(ReadMode::POSITIONS) {
			start.min(ENCODER_0_POSITION);
			end.max(ENCODER_7_POSITION);
		}
		if rm.contains(ReadMode::VELOCITY) {
			start.min(ENCODER_0_VELOCITY);
			end.max(ENCODER_7_VELOCITY);
		}
		(start, end)
	}
	const fn addr(&self) -> u8 {*self as u8}
	const fn get_len(&self) -> u8 {
		let mut iter = [CHIP_ID, FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_ENGINEERING, COMMAND, COMMAND_DAT_0, COMMAND_DAT_1, COMMAND_DAT_2, COMMAND_DAT_3, COMMAND_DAT_4, COMMAND_DAT_5, COMMAND_DAT_6,
			LOCALIZER_YAW_AXIS, LOCALIZER_STATUS, LOCALIZER_VX, LOCALIZER_VY, LOCALIZER_VH, LOCALIZER_X, LOCALIZER_Y, LOCALIZER_H, LOCALIZER_CRC16,
			ENCODER_0_POSITION, ENCODER_1_POSITION, ENCODER_2_POSITION, ENCODER_3_POSITION, ENCODER_4_POSITION, ENCODER_5_POSITION, ENCODER_6_POSITION, ENCODER_7_POSITION,
			ENCODER_0_VELOCITY, ENCODER_1_VELOCITY, ENCODER_2_VELOCITY, ENCODER_3_VELOCITY, ENCODER_4_VELOCITY, ENCODER_5_VELOCITY, ENCODER_6_VELOCITY, ENCODER_7_VELOCITY,
			ENCODER_DATA_CRC16, UNKNOWN];
		let mut i = 0;
		loop {
			let it = iter[i];
			if it as u8 == UNKNOWN as u8 {
				return 0
			}
			if it as u8 == *self as u8 {
				let next = iter[i + 1];
				return (next as u8) - (it as u8)
			}
			i += 1;
		}
	}
}
#[cfg(test)]
mod tests {
	use super::*;

	fn read_len(rm: ReadMode) -> usize {
		let (start, end) = OctoQuadRegisters::get_from_readmode(rm);
		(end.addr() + end.get_len() - start.addr()) as usize
	}
	///fake register contents: every byte is its own register address
	fn fake_read(rm: ReadMode) -> Vec<u8> {
		let (start, _) = OctoQuadRegisters::get_from_readmode(rm);
		(0..read_len(rm)).map(|i| start.addr() + i as u8).collect()
	}

	#[test]
	fn register_lengths() {
		assert_eq!(LOCALIZER_STATUS.get_len(), 1);
		assert_eq!(LOCALIZER_CRC16.get_len(), 2);
		assert_eq!(ENCODER_7_POSITION.get_len(), 4);
		assert_eq!(ENCODER_DATA_CRC16.get_len(), 2);
		assert_eq!(UNKNOWN.get_len(), 0);
		assert_eq!(read_len(ReadMode::LOCALIZER), OctoQuadLocalizerData::LEN);
		assert_eq!(read_len(ReadMode::POSITIONS), 32);
		assert_eq!(read_len(ReadMode::VELOCITY), 16);
		assert_eq!(read_len(ReadMode::all()), 15 + 32 + 16);
	}

	#[test]
	fn every_read_mode_parses() {
		for bits in 1..8u8 {
			let rm = ReadMode::from_bits_truncate(bits);
			let snapshot = OctoQuadSnapshot::from((bits, fake_read(rm)));
			assert_eq!(snapshot.localizer.is_some(), rm.contains(ReadMode::LOCALIZER), "{:03b}", bits);
			assert_eq!(snapshot.positions.is_some(), rm.contains(ReadMode::POSITIONS), "{:03b}", bits);
			assert_eq!(snapshot.velocities.is_some(), rm.contains(ReadMode::VELOCITY), "{:03b}", bits);
			if let Some(it) = snapshot.localizer {
				assert_eq!(it.vel_x_mm_s.to_le_bytes(), [LOCALIZER_VX.addr(), LOCALIZER_VX.addr() + 1]);
				assert_eq!(it.crc.to_le_bytes(), [LOCALIZER_CRC16.addr(), LOCALIZER_CRC16.addr() + 1]);
			}
			if let Some(it) = snapshot.positions {
				assert_eq!(it.pos[0].to_le_bytes()[0], ENCODER_0_POSITION.addr());
				assert_eq!(it.pos[7].to_le_bytes()[0], ENCODER_7_POSITION.addr());
			}
			if let Some(it) = snapshot.velocities {
				assert_eq!(it.vel[0].to_le_bytes()[0], ENCODER_0_VELOCITY.addr());
				assert_eq!(it.vel[7].to_le_bytes()[0], ENCODER_7_VELOCITY.addr());
			}
		}
	}

	#[test]
	fn short_reads_do_not_panic() {
		let snapshot = OctoQuadSnapshot::from((ReadMode::all().bits(), vec![0; 3]));
		assert!(snapshot.localizer.is_none() && snapshot.positions.is_none() && snapshot.velocities.is_none());
	}

	#[test]
	fn localizer_reencodes_like_the_device() {
		let mut raw = fake_read(ReadMode::LOCALIZER);
		raw[0] = LocalizerStatus::RUNNING as u8;
		let parsed = OctoQuadLocalizerData::from(&raw);
		//everything but the crc
		assert_eq!(OctoQuadLocalizerData::into(&parsed), raw[..OctoQuadLocalizerData::LEN - 2].to_vec());
	}
}
