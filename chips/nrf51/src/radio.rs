//! Radio/BLE Driver for nrf51dk
//!
//! Sending BLE advertisement packets
//! Possible payload is 30 bytes
//!
//! Author: Niklas Adolfsson <niklasadolfsson1@gmail.com>
//! Author: Fredrik Nilsson <frednils@student.chalmers.se>
//! Date: June 3, 2017

use chip;
use core::cell::Cell;
use kernel::hil::time::Client;
use kernel::returncode::ReturnCode;
use nvic;
use peripheral_interrupts::NvicIdx;
use peripheral_registers::{RADIO_REGS, RADIO_BASE};


pub const PACKET0_S1_SIZE: u32 = 0;
pub const PACKET0_S0_SIZE: u32 = 0;

pub const RADIO_PCNF0_S0LEN_POS: u32 = 8;
pub const RADIO_PCNF0_S1LEN_POS: u32 = 16;
pub const RADIO_PCNF0_LFLEN_POS: u32 = 0;

pub const RADIO_PCNF1_WHITEEN_DISABLED: u32 = 0;
pub const RADIO_PCNF1_WHITEEN_ENABLED: u32 = 1;
pub const RADIO_PCNF1_WHITEEN_POS: u32 = 25;

pub const RADIO_PCNF1_BALEN_POS: u32 = 16;
pub const RADIO_PCNF1_STATLEN_POS: u32 = 8;
pub const RADIO_PCNF1_MAXLEN_POS: u32 = 0;

pub const RADIO_PCNF1_ENDIAN_POS: u32 = 24;
pub const RADIO_PCNF1_ENDIAN_BIG: u32 = 1;

pub const PACKET_LENGTH_FIELD_SIZE: u32 = 0;
pub const PACKET_PAYLOAD_MAXSIZE: u32 = 64;
pub const PACKET_BASE_ADDRESS_LENGTH: u32 = 4;
pub const PACKET_STATIC_LENGTH: u32 = 64;


// internal Radio State
pub const RADIO_STATE_DISABLE: u32 = 0;
pub const RADIO_STATE_RXRU: u32 = 1;
pub const RADIO_STATE_RXIDLE: u32 = 2;
pub const RADIO_STATE_RX: u32 = 3;
pub const RADIO_STATE_RXDISABLE: u32 = 4;
pub const RADIO_STATE_TXRU: u32 = 9;
pub const RADIO_STATE_TXIDLE: u32 = 10;
pub const RADIO_STATE_TX: u32 = 11;
pub const RADIO_STATE_TXDISABLE: u32 = 12;


// static mut TX_BUF: [u8; 16] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
static mut RX_BUF: [u8; 12] = [0x00; 12];



// FROM LEFT
// ADVTYPE      ;;      4 bits
// RFU          ;;      2 bits
// TxAdd        ;;      1 bit
// RxAdd        ;;      1 bit
// Length       ;;       6 bits
// RFU          ;;      2 bits
// AdvD         ;;      6 bytes
// AdvData      ;;      31 bytes

// Header (2 bytes) || Address (6 bytes) || Payload 31 bytes
static mut PAYLOAD: [u8; 39] = [// ADV_IND, public addr  [HEADER]
                                0x02,
                                0xE,
                                0x00,
                                // Address          [ADV ADDRESS]
                                0x90,
                                0xD8,
                                0x7A,
                                0xBD,
                                0xA3,
                                0xED,
                                // [LEN, AD-TYPE, LEN-1 bytes of data ...]
                                // 0x09 - Local name
                                // 0x54 0x6f 0x63 0x6b 0x4f 0x54 - TockOs
                                0x7,
                                0x09,
                                0x54,
                                0x6f,
                                0x63,
                                0x6b,
                                0x4f,
                                0x53,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00]; //[DATA]

pub struct Radio {
    regs: *const RADIO_REGS,
    txpower: Cell<usize>,
    client: Cell<Option<&'static Client>>,
}

pub static mut RADIO: Radio = Radio::new();


impl Radio {
    pub const fn new() -> Radio {
        Radio {
            regs: RADIO_BASE as *const RADIO_REGS,
            txpower: Cell::new(0),
            client: Cell::new(None),
        }
    }

    pub fn set_client<C: Client>(&self, client: &'static C) {
        self.client.set(Some(client));
    }

    fn init_radio_ble(&self) {
        let regs = unsafe { &*self.regs };

        self.radio_on();

        // TX Power acc. twpower variable in the struct
        self.set_txpower();

        // BLE MODE
        self.set_channel_rate(0x03);

        self.set_channel_freq(37);
        self.set_data_white_iv(37);

        // Set PREFIX | BASE Address
        regs.PREFIX0.set(0x0000008e);
        regs.BASE0.set(0x89bed600);

        self.set_tx_address(0x00);
        self.set_rx_address(0x01);
        // regs.RXMATCH.set(0x00);

        // Set Packet Config
        self.set_packet_config(0x00);

        // CRC Config
        self.set_crc_config();

        // Buffer configuration
        self.set_tx_buffer();

        self.enable_interrupts();
        self.enable_nvic();

        regs.READY.set(0);
        regs.TXEN.set(1);
    }

    fn set_crc_config(&self) {
        let regs = unsafe { &*self.regs };
        // CRC Config
        regs.CRCCNF.set(0x103); // 3 bytes CRC and don't include Address field in the CRC
        regs.CRCINIT.set(0x555555); // INIT CRC Value
        // CRC Polynomial  x24 + x10 + x9 + x6 + x4 + x3 + x + 1
        regs.CRCPOLY.set(0x00065B);
    }


    // Packet configuration
    // Argument unsed atm
    fn set_packet_config(&self, _: u32) {
        let regs = unsafe { &*self.regs };

        // This initialization have to do with the header in the PDU it is 2 bytes
        // ADVTYPE      ;;      4 bits
        // RFU          ;;      2 bits
        // TxAdd        ;;      1 bit
        // RxAdd        ;;      1 bit
        // Length       ;;      6 bits
        // RFU          ;;      2 bits

        regs.PCNF0
            .set(// set S0 to 1 byte
                 (1 << RADIO_PCNF0_S0LEN_POS) |
                     // set S1 to 2 bits
                     (2 << RADIO_PCNF0_S1LEN_POS) |
                     // set length to 6 bits
                     (6 << RADIO_PCNF0_LFLEN_POS));


        regs.PCNF1
            .set((RADIO_PCNF1_WHITEEN_ENABLED << RADIO_PCNF1_WHITEEN_POS) |
                // set little-endian
                (0 << RADIO_PCNF1_ENDIAN_POS) |
                // Set BASE + PREFIX address to 4 bytes
                (3 << RADIO_PCNF1_BALEN_POS) |
                // don't extend packet length
                (0 << RADIO_PCNF1_STATLEN_POS) |
                // max payload size 37
                (37 << RADIO_PCNF1_MAXLEN_POS));
    }

    // TODO set from capsules?!
    fn set_rx_address(&self, _: u32) {
        let regs = unsafe { &*self.regs };
        regs.RXADDRESSES.set(0x01);
    }

    // TODO set from capsules?!
    fn set_tx_address(&self, _: u32) {
        let regs = unsafe { &*self.regs };
        regs.TXADDRESS.set(0x00);
    }

    // should not be configured from the capsule i.e.
    // assume always BLE
    fn set_channel_rate(&self, rate: u32) {
        let regs = unsafe { &*self.regs };
        // set channel rate,  3 - BLE 1MBIT/s
        regs.MODE.set(rate);
    }

    fn set_data_white_iv(&self, val: u32) {
        let regs = unsafe { &*self.regs };
        // DATAIV
        regs.DATAWHITEIV.set(val);
    }

    fn set_channel_freq(&self, val: u32) {
        let regs = unsafe { &*self.regs };
        //37, 38 and 39 for adv.
        match val {
            37 => regs.FREQEUNCY.set(2),
            38 => regs.FREQEUNCY.set(26),
            39 => regs.FREQEUNCY.set(80),
            _ => regs.FREQEUNCY.set(7),
        }
    }

    fn radio_on(&self) {
        let regs = unsafe { &*self.regs };
        // reset and enable power
        regs.POWER.set(0);
        regs.POWER.set(1);
    }

    fn radio_off(&self) {
        let regs = unsafe { &*self.regs };
        regs.POWER.set(0);
    }

    // pre-condition validated before arrving here
    // argue where the put the returncode
    fn set_txpower(&self) {
        let regs = unsafe { &*self.regs };
        regs.TXPOWER.set(self.txpower.get() as u32);
    }

    fn set_tx_buffer(&self) {
        let regs = unsafe { &*self.regs };
        unsafe {
            regs.PACKETPTR.set((&PAYLOAD as *const u8) as u32);
        }
    }

    fn set_rx_buffer(&self) {
        let regs = unsafe { &*self.regs };
        unsafe {
            regs.PACKETPTR.set((&RX_BUF as *const u8) as u32);
        }
    }

    #[inline(never)]
    #[no_mangle]
    pub fn handle_interrupt(&self) {
        let regs = unsafe { &*self.regs };
        self.disable_nvic();
        self.disable_interrupts();
        nvic::clear_pending(NvicIdx::RADIO);

        if regs.READY.get() == 1 {
            if regs.STATE.get() <= 4 {
                self.set_rx_buffer();
            } else {
                self.set_tx_buffer();
            }
            regs.READY.set(0);
            regs.END.set(0);
            regs.START.set(1);
        }

        if regs.PAYLOAD.get() == 1 {
            regs.PAYLOAD.set(0);
        }

        if regs.ADDRESS.get() == 1 {
            regs.ADDRESS.set(0);
        }

        if regs.END.get() == 1 {
            regs.END.set(0);
            regs.DISABLE.set(1);
            // this state only verifies that END is received in TX-mode
            // which means that the transmission is finished
            match regs.STATE.get() {
                RADIO_STATE_TXRU |
                RADIO_STATE_TXIDLE |
                RADIO_STATE_TXDISABLE |
                RADIO_STATE_TX => {
                    match regs.FREQEUNCY.get() {
                        // frequency 39
                        80 => {
                            self.radio_off();
                        }
                        // frequency 38
                        26 => {
                            self.set_channel_freq(39);
                            self.set_data_white_iv(39);
                            regs.READY.set(0);
                            regs.TXEN.set(1);
                        }
                        // frequency 37
                        2 => {
                            self.set_channel_freq(38);
                            self.set_data_white_iv(38);
                            regs.READY.set(0);
                            regs.TXEN.set(1);
                        }
                        // don't care as we only support advertisements at the moment
                        _ => {
                            self.set_channel_freq(37);
                            self.set_data_white_iv(37)
                        }
                    }
                }
                _ => (),
            }
        }
        self.enable_nvic();
        self.enable_interrupts();
    }


    pub fn enable_interrupts(&self) {
        // INTENSET REG
        let regs = unsafe { &*self.regs };
        // 15 i.e set 4 LSB
        regs.INTENSET.set(1 | 1 << 1 | 1 << 2 | 1 << 3);
    }

    pub fn disable_interrupts(&self) {
        let regs = unsafe { &*self.regs };
        // disable all possible interrupts
        regs.INTENCLR.set(0x4ff);
    }

    pub fn enable_nvic(&self) {
        nvic::enable(NvicIdx::RADIO);
    }

    pub fn disable_nvic(&self) {
        nvic::disable(NvicIdx::RADIO);
    }

    pub fn reset_payload(&self) {
        // reset contents except header || address
        for i in 9..39 {
            unsafe {
                PAYLOAD[i] = 0;
            }
        }
        unsafe {
            PAYLOAD[1] = 6;
        }
    }

    // these are used by ble_advertising_driver and are therefore public
    // FIXME: should be moved to the HIL later

    pub fn start_adv(&self) {
        self.init_radio_ble();
    }

    pub fn set_adv_data(&self,
                        ad_type: usize,
                        data: &'static mut [u8],
                        len: usize,
                        offset: usize)
                        -> &'static mut [u8] {
        if offset == 9 {
            //FIXME: move call to the capsule!?
            self.reset_payload();
        }
        unsafe {
            PAYLOAD[offset] = (len + 1) as u8;
            PAYLOAD[offset + 1] = ad_type as u8;
        }

        for (i, c) in data.as_ref()[0..len].iter().enumerate() {
            unsafe {
                PAYLOAD[i + offset + 2] = *c;
            }
        }
        unsafe {
            PAYLOAD[1] = (offset - 1 + len) as u8;
        }
        data
    }

    pub fn clear_adv_data(&self) {
        self.reset_payload();
    }

    // FIXME: added a temporary variable in struct that keeps
    // track of the twpower because we turn off the radio
    // between advertisements,
    // it is configured to 0 by default or the latest conifigured value
    pub fn set_adv_txpower(&self, dbm: usize) -> ReturnCode {
        match dbm {
            // +4 dBm, 0 dBm, -4 dBm, -8 dBm, -12 dBm, -16 dBm, -20 dBm, -30 dBm
            e @ 0x04 | e @ 0x00 | e @ 0xF4 | e @ 0xFC | e @ 0xF8 | e @ 0xF0 | e @ 0xEC |
            e @ 0xD8 => {
                self.txpower.set(e);
                ReturnCode::SUCCESS
            }
            _ => ReturnCode::ENOSUPPORT,
        }
    }
}

#[no_mangle]
#[allow(non_snake_case)]
pub unsafe extern "C" fn RADIO_Handler() {
    use kernel::common::Queue;
    nvic::disable(NvicIdx::RADIO);
    chip::INTERRUPT_QUEUE.as_mut()
        .unwrap()
        .enqueue(NvicIdx::RADIO);
}
