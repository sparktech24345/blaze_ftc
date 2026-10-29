use std::sync::{Arc, Mutex, MutexGuard};
use std::sync::atomic::{AtomicBool, Ordering};
use std::{panic, thread, vec};
use std::time::{Duration, Instant};
use crossbeam_channel::{select, unbounded, Receiver, Sender};
use crate::sdk_proxy::proxy::IdTransform;
use crate::serialization::command::Command;
use crate::serialization::lynx_commands::base_lynx_command::LynxCommand;
use crate::serialization::packet::{Packet, Packets};
use crate::telemetry::telemetry::get_allowed_to_send_dangerous_packets;

pub(crate) type MessageList = Mutex<Vec<Instant>>;
///message number 0 is reserved by the lynx protocol (the SDK only uses 1..=255), so slot 0 is never handed out
fn register_packet_0(current_things: &mut MutexGuard<Vec<Instant>>) -> u8 {
    let mut x = 1usize;
    let mut old = current_things[1];
    for i in 1..current_things.len() {
        if current_things[i] < old {
            x = i;
            old = current_things[i];
        }
    }
    log::trace!("got oldest id: {}", x);
    if current_things[x].elapsed() < Duration::from_millis(4) {
        log::info!("used id {} just {} ns ago!", x, current_things[x].elapsed().as_nanos());
    }
    //just use the oldest one. this will... probably work
    current_things[x] = Instant::now();
    x as u8
}
pub(crate) fn register_packet(mg: &MessageList) -> u8 {
    let mut current_things = mg.lock().expect("could not lock MessageList");
    register_packet_0(&mut current_things)
}
pub(crate) fn register_packets(mg: &MessageList, num: u8) -> Vec<u8> {
    let mut current_things = mg.lock().expect("could not lock MessageList");
    register_packets_from(&mut current_things, num, Instant::now)
}

fn register_packets_from(
    current_things: &mut [Instant],
    num: u8,
    mut now: impl FnMut() -> Instant,
) -> Vec<u8> {
    let mut numbers = Vec::with_capacity(num as usize);
    let mut selected = vec![false; current_things.len()];
    for _ in 0..num {
        let index = current_things
            .iter()
            .enumerate()
            .skip(1)
            .filter(|(index, _)| !selected[*index])
            .min_by_key(|(_, last_used)| **last_used)
            .map(|(index, _)| index)
            .expect("not enough message numbers for packet batch");
        selected[index] = true;
        current_things[index] = now();
        numbers.push(index as u8);
    }
    numbers
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn batch_message_numbers_are_unique() {
        let timestamp = Instant::now();
        let mut messages = vec![timestamp; u8::MAX as usize + 1];
        let numbers = register_packets_from(&mut messages, 4, || timestamp);

        assert_eq!(numbers.len(), 4);
        assert!(numbers.iter().all(|number| *number != 0));
        let mut unique = numbers.clone();
        unique.sort_unstable();
        unique.dedup();
        assert_eq!(unique.len(), numbers.len());
    }
}
/**
 * ok so the first return is the normal one, the second is the wierd one (for ftc sdk).
 */
pub fn generate_write_sdk_proxy(
    to_write: Sender<Packets>,
    packets_to_watch: Arc<Mutex<Vec<IdTransform>>>,
    running: &'static AtomicBool,
    id: u8
) -> (Sender<Packets>, Sender<Packet>, Arc<MessageList>) {
    let (regular_tx, regular_rx) = unbounded::<Packets>();
    let (ftcsdk_tx, ftcsdk_rx) = unbounded::<Packet>();
    //one slot per possible message number. slot 0 exists but is never used, see register_packet_0
    let current_things_old = Arc::new(Mutex::new(vec![Instant::now(); u8::MAX as usize + 1]));
    let current_things = current_things_old.clone();
    thread::spawn(move || {
        match panic::catch_unwind(move || {
            while running.load(Ordering::SeqCst) {
                do_proxy(&regular_rx, &ftcsdk_rx, &current_things, &to_write, &packets_to_watch)
            }
        }) {
            Ok(_) => {}
            Err(it) => {
                log::info!("ERROR IN WRITE SDK PROXY {}", id);
                if let Some(s) = it.downcast_ref::<&str>() {
                    log::info!("Caught panic: {}", s);
                } else if let Some(s) = it.downcast_ref::<String>() {
                    log::info!("Caught panic: {}", s);
                } else {
                    log::info!("Caught unknown panic type");
                }
            }
        };
        println!("write sdk proxy exiting");
    });
    //note to self. put MessageList in the proxy impl, and make a LynxHub method that turns LynxCommands into packets and sends them, returning the message id.
    (regular_tx, ftcsdk_tx, current_things_old)
}

fn do_proxy(regular_rx: &Receiver<Packets>, ftcsdk_rx: &Receiver<Packet>, current_things: &Arc<Mutex<Vec<Instant>>>, to_write: &Sender<Packets>, packets_to_watch: &Arc<Mutex<Vec<IdTransform>>>) {
    select! {
        recv(regular_rx) -> msg => {
            log::trace!("ftc write proxy: got our packet");
            let mut packets = msg.expect("could not receive from regular rx");
            for packet in &mut packets {
                packet.checksum = packet.checksum();
            }
            if packets.is_empty() {
                return;
            }
            to_write.send(packets).unwrap();
        }
        recv(ftcsdk_rx) -> msg => {
            let mut packet = msg.expect("could not receive from ftcsdk tx");
            log::trace!("ftc write proxy: got FTC packet, msg num:{}, ref num:{}", packet.message_number, packet.reference_number);
            let x = register_packet(&current_things);
            let transform = IdTransform {
                old_id: packet.message_number,
                new_id: x,
                old_pack_id: packet.packet_id,
                old_pack: packet.clone(),
                sent_time: Instant::now()
            }; //TODO: ok. this should be ref num but we're testing with msg num
            log::trace!(
                "rewriting ftc packet: old ref:{}, old id:{}, new ref:{}",
                packet.reference_number,
                packet.message_number,
                transform.new_id
            );
            packet.reference_number = transform.new_id;
            packet.message_number = transform.new_id; //TODO: determine if this is... a good idea
            packet.checksum = packet.checksum(); //set checksum whoops
            let mut lock = packets_to_watch.lock().unwrap();
            //this id is being reused, so whatever was mapped to it before is stale (see Proxy::forget_transforms)
            lock.retain(|it| it.new_id != transform.new_id);
            lock.push(transform);
            to_write.send(packet.into()).unwrap();
        }
    };
}

fn is_legal(pack: &mut Packet) -> bool {
    if get_allowed_to_send_dangerous_packets() {
        return true
    }
    match &mut pack.payload_data {
        Command::LynxCommand(it) => {
            match &mut it.command {
                LynxCommand::LynxSetMotorPowerCommand(it) => {
                    it.power = 0;
                    true
                },
                LynxCommand::LynxSetServoPulseWidthCommand(_) => true,
                _ => true
            }
        }
        _ => {true}
    }
}
