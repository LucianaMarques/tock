//! RTC driver, sensortag family

use core::cell::Cell;
use kernel::common::registers::{ReadOnly, ReadWrite};
use kernel::common::StaticRef;
use kernel::hil::time::{self, Alarm, Frequency, Time};

#[repr(C)]
struct RtcRegisters {
    ctl: ReadWrite<u32, Control::Register>,

    // Event flags
    evflags: ReadWrite<u32, EvFlags::Register>,

    // Integer part
    sec: ReadWrite<u32>,
    // Fractional part (1/32kHz parts of a second)
    subsec: ReadOnly<u32>,

    _subsec_inc: ReadOnly<u32>,
    channel_ctl: ReadWrite<u32, ChannelControl::Register>,
    _channel0_cmp: ReadOnly<u32>,
    channel1_cmp: ReadWrite<u32>,
    _channel2_cmp: ReadOnly<u32>,
    _channel2_cmp_inc: ReadOnly<u32>,
    _channel1_capture: ReadOnly<u32>,

    // A read request to the sync register will not return
    // until all outstanding writes have properly propagated to the RTC domain
    sync: ReadOnly<u32>,
}

register_bitfields![
    u32,
    Control [
        COMB_EV_MASK OFFSET(16) NUMBITS(3) [
            NoEvent = 0b00,
            Channel0 = 0b01,
            Channel1 = 0b10,
            Channel2 = 0b11
        ],
        RESET       OFFSET(7) NUMBITS(1) [],
        RTC_UPD_EN  OFFSET(1) NUMBITS(1) [],
        ENABLE      OFFSET(0) NUMBITS(1) []
    ],
    EvFlags [
        CH2 OFFSET(16) NUMBITS(1) [],
        CH1 OFFSET(8)  NUMBITS(1) [],
        CH0 OFFSET(0)  NUMBITS(1) []
    ],
    ChannelControl [
        CH2_CONT_EN OFFSET(18)  NUMBITS(1) [],
        CH2_EN      OFFSET(16)  NUMBITS(1) [],
        CH1_CAPT_EN OFFSET(9)   NUMBITS(1) [],
        CH1_EN      OFFSET(8)   NUMBITS(1) [],
        CH0_EN      OFFSET(0)   NUMBITS(1) []
    ]
];

const RTC_BASE: StaticRef<RtcRegisters> =
    unsafe { StaticRef::new(0x40092000 as *const RtcRegisters) };

pub struct Rtc {
    registers: StaticRef<RtcRegisters>,
    callback: Cell<Option<&'static time::Client>>,
}

pub static mut RTC: Rtc = Rtc::new();

impl Rtc {
    const fn new() -> Rtc {
        Rtc {
            registers: RTC_BASE,
            callback: Cell::new(None),
        }
    }

    pub fn start(&self) {
        let registers = &*self.registers;
        registers.ctl.write(Control::ENABLE::SET);

        registers.sync.get();
    }

    pub fn stop(&self) {
        let registers = &*self.registers;
        registers.ctl.write(Control::ENABLE::CLEAR);

        registers.sync.get();
    }

    fn read_counter(&self) -> u32 {
        let registers = &*self.registers;

        /*
            SEC can change during the SUBSEC read, so we need to be certain
            that the SUBSEC we read belong to the correct SEC counterpart.
        */
        let mut current_sec: u32 = 0;
        let mut current_subsec: u32 = 0;
        let mut after_subsec_read: u32 = 1;
        while current_sec != after_subsec_read {
            current_sec = registers.sec.get();
            current_subsec = registers.subsec.get();
            after_subsec_read = registers.sec.get();
        }

        return (current_sec << 16) | (current_subsec >> 16);
    }

    pub fn is_running(&self) -> bool {
        let registers = &*self.registers;
        registers.channel_ctl.read(ChannelControl::CH1_EN) != 0
    }

    pub fn handle_interrupt(&self) {
        let registers = &*self.registers;

        // Event flag is cleared when you set it
        registers.evflags.write(EvFlags::CH1::SET);
        registers.ctl.modify(Control::COMB_EV_MASK::NoEvent);
        registers.channel_ctl.modify(ChannelControl::CH1_EN::CLEAR);

        registers.sync.get();

        self.callback.get().map(|cb| cb.fired());
    }

    pub fn set_client(&self, client: &'static time::Client) {
        self.callback.set(Some(client));
    }
}

pub struct RtcFreq(());

impl Frequency for RtcFreq {
    // The RTC Frequency is tuned, as there is exactly 0xFFFF (64kHz)
    // subsec increments to reach a second, this yields the correct
    // `tics` to set the comparator correctly.
    fn frequency() -> u32 {
        0xFFFF
    }
}

impl Time for Rtc {
    type Frequency = RtcFreq;

    fn disable(&self) {
        let registers = &*self.registers;

        registers.ctl.modify(Control::COMB_EV_MASK::NoEvent);
        registers.channel_ctl.modify(ChannelControl::CH1_EN::CLEAR);

        registers.sync.get();
    }

    fn is_armed(&self) -> bool {
        self.is_running()
    }
}

impl Alarm for Rtc {
    fn now(&self) -> u32 {
        self.read_counter()
    }

    fn set_alarm(&self, tics: u32) {
        let registers = &*self.registers;

        registers.ctl.modify(Control::COMB_EV_MASK::Channel1);
        registers.channel1_cmp.set(tics);
        registers.channel_ctl.modify(ChannelControl::CH1_EN::SET);

        registers.sync.get();
    }

    fn get_alarm(&self) -> u32 {
        let registers = &*self.registers;
        registers.channel1_cmp.get()
    }
}
