use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::Result;
use base64::Engine as _;

use libconcentratord::jitqueue::TxPacket;
use libconcentratord::signals::Signal;
use libconcentratord::{jitqueue, stats};
use libloragw_2g4::hal;

use super::super::wrapper;

pub fn jit_loop(
    queue: Arc<Mutex<jitqueue::Queue<wrapper::TxPacket>>>,
    antenna_gain_dbi: i8,
    stop_receive: Receiver<Signal>,
) -> Result<()> {
    debug!("Start JIT queue loop");

    loop {
        // Instead of a 10ms sleep, we receive from the stop channel with a
        // timeout of 10ms.
        if let Ok(v) = stop_receive.recv_timeout(Duration::from_millis(10)) {
            debug!("Received stop signal, signal: {}", v);
            return Ok(());
        }

        let tx_packet = match get_tx_packet(&queue)? {
            Some(v) => v,
            None => continue,
        };

        let downlink_id = tx_packet.get_id();
        let mut tx_packet = tx_packet.tx_packet();
        tx_packet.rf_power -= antenna_gain_dbi;

        match hal::send(&tx_packet) {
            Ok(_) => {
                if log::log_enabled!(log::Level::Info) {
                    let payload_size = (tx_packet.size as usize).min(tx_packet.payload.len());
                    let payload = base64::engine::general_purpose::STANDARD
                        .encode(&tx_packet.payload[..payload_size]);
                    info!(
                        "Scheduled packet for TX, downlink_id: {}, count_us: {}, freq: {}, bw: {}, mod: {:?}, dr: {:?}, coderate: {:?}, rf_power: {}, size: {}, payload: {}",
                        downlink_id,
                        tx_packet.count_us,
                        tx_packet.freq_hz,
                        tx_packet.bandwidth,
                        hal::Modulation::LoRa,
                        tx_packet.datarate,
                        tx_packet.coderate,
                        tx_packet.rf_power,
                        tx_packet.size,
                        payload
                    );
                }

                if let Ok(tx_info) = wrapper::downlink_to_tx_info_proto(&tx_packet) {
                    stats::inc_tx_counts(&tx_info);
                }
            }
            Err(err) => {
                error!("Schedule packet for tx error, error: {}", err);
            }
        }
    }
}

fn get_tx_packet(
    queue: &Arc<Mutex<jitqueue::Queue<wrapper::TxPacket>>>,
) -> Result<Option<wrapper::TxPacket>> {
    let mut queue = queue.lock().map_err(|_| anyhow!("Lock queue error"))?;
    let concentrator_count = hal::get_instcnt()?;
    Ok(queue.pop(concentrator_count))
}
