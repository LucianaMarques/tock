//! Shared setup for nrf52dk boards.

#![no_std]

extern crate capsules;
#[allow(unused_imports)]
#[macro_use(debug, debug_verbose, debug_gpio, static_init)]
extern crate kernel;
extern crate nrf52;
extern crate nrf5x;

use capsules::virtual_alarm::VirtualMuxAlarm;
use capsules::virtual_spi::MuxSpiMaster;
use kernel::hil;
use nrf5x::rtc::Rtc;

/// Pins for SPI for the flash chip MX25R6435F
#[derive(Debug)]
pub struct SpiMX25R6435FPins {
    chip_select: usize,
    write_protect_pin: usize,
    hold_pin: usize,
}

impl SpiMX25R6435FPins {
    pub fn new(chip_select: usize, write_protect_pin: usize, hold_pin: usize) -> Self {
        Self {
            chip_select,
            write_protect_pin,
            hold_pin,
        }
    }
}

/// Pins for the SPI driver
#[derive(Debug)]
pub struct SpiPins {
    mosi: usize,
    miso: usize,
    clk: usize,
}

impl SpiPins {
    pub fn new(mosi: usize, miso: usize, clk: usize) -> Self {
        Self { mosi, miso, clk }
    }
}

/// Pins for the UART
#[derive(Debug)]
pub struct UartPins {
    rts: usize,
    txd: usize,
    cts: usize,
    rxd: usize,
}

impl UartPins {
    pub fn new(rts: usize, txd: usize, cts: usize, rxd: usize) -> Self {
        Self { rts, txd, cts, rxd }
    }
}

/// Supported drivers by the platform
pub struct Platform {
    ble_radio: &'static capsules::ble_advertising_driver::BLE<
        'static,
        nrf52::radio::Radio,
        VirtualMuxAlarm<'static, Rtc>,
    >,
    button: &'static capsules::button::Button<'static, nrf5x::gpio::GPIOPin>,
    console: &'static capsules::console::Console<'static, nrf52::uart::Uarte>,
    gpio: &'static capsules::gpio::GPIO<'static, nrf5x::gpio::GPIOPin>,
    led: &'static capsules::led::LED<'static, nrf5x::gpio::GPIOPin>,
    rng: &'static capsules::rng::SimpleRng<'static, nrf5x::trng::Trng<'static>>,
    temp: &'static capsules::temperature::TemperatureSensor<'static>,
    ipc: kernel::ipc::IPC,
    alarm: &'static capsules::alarm::AlarmDriver<
        'static,
        capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>,
    >,
    // The nRF52dk does not have the flash chip on it, so we make this optional.
    nonvolatile_storage:
        Option<&'static capsules::nonvolatile_storage_driver::NonvolatileStorage<'static>>,
}

impl kernel::Platform for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&kernel::Driver>) -> R,
    {
        match driver_num {
            capsules::console::DRIVER_NUM => f(Some(self.console)),
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
            capsules::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules::led::DRIVER_NUM => f(Some(self.led)),
            capsules::button::DRIVER_NUM => f(Some(self.button)),
            capsules::rng::DRIVER_NUM => f(Some(self.rng)),
            capsules::ble_advertising_driver::DRIVER_NUM => f(Some(self.ble_radio)),
            capsules::temperature::DRIVER_NUM => f(Some(self.temp)),
            capsules::nonvolatile_storage_driver::DRIVER_NUM => {
                f(self.nonvolatile_storage.map_or(None, |nv| Some(nv)))
            }
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            _ => f(None),
        }
    }
}

/// Generic function for starting an nrf52dk board.
#[inline]
pub unsafe fn setup_board(
    button_rst_pin: usize,
    gpio_pins: &'static mut [&'static nrf5x::gpio::GPIOPin],
    debug_pin1_index: usize,
    debug_pin2_index: usize,
    debug_pin3_index: usize,
    led_pins: &'static mut [(&'static nrf5x::gpio::GPIOPin, capsules::led::ActivationMode)],
    uart_pins: &UartPins,
    spi_pins: &SpiPins,
    mx25r6435f: &Option<SpiMX25R6435FPins>,
    button_pins: &'static mut [(&'static nrf5x::gpio::GPIOPin, capsules::button::GpioMode)],
    app_memory: &mut [u8],
    process_pointers: &'static mut [core::option::Option<
        &'static mut kernel::procs::Process<'static>,
    >],
    app_fault_response: kernel::procs::FaultResponse,
) {
    // Make non-volatile memory writable and activate the reset button
    let uicr = nrf52::uicr::Uicr::new();
    nrf52::nvmc::NVMC.erase_uicr();
    nrf52::nvmc::NVMC.configure_writeable();
    while !nrf52::nvmc::NVMC.is_ready() {}
    uicr.set_psel0_reset_pin(button_rst_pin);
    while !nrf52::nvmc::NVMC.is_ready() {}
    uicr.set_psel1_reset_pin(button_rst_pin);

    // Configure kernel debug gpios as early as possible
    kernel::debug::assign_gpios(
        Some(&nrf5x::gpio::PORT[debug_pin1_index]),
        Some(&nrf5x::gpio::PORT[debug_pin2_index]),
        Some(&nrf5x::gpio::PORT[debug_pin3_index]),
    );

    let gpio = static_init!(
        capsules::gpio::GPIO<'static, nrf5x::gpio::GPIOPin>,
        capsules::gpio::GPIO::new(gpio_pins)
    );
    for pin in gpio_pins.iter() {
        pin.set_client(gpio);
    }

    // LEDs
    let led = static_init!(
        capsules::led::LED<'static, nrf5x::gpio::GPIOPin>,
        capsules::led::LED::new(led_pins)
    );

    // Buttons
    let button = static_init!(
        capsules::button::Button<'static, nrf5x::gpio::GPIOPin>,
        capsules::button::Button::new(button_pins, kernel::Grant::create())
    );
    for &(btn, _) in button_pins.iter() {
        use kernel::hil::gpio::PinCtl;
        btn.set_input_mode(kernel::hil::gpio::InputMode::PullUp);
        btn.set_client(button);
    }

    let rtc = &nrf5x::rtc::RTC;
    rtc.start();
    let mux_alarm = static_init!(
        capsules::virtual_alarm::MuxAlarm<'static, nrf5x::rtc::Rtc>,
        capsules::virtual_alarm::MuxAlarm::new(&nrf5x::rtc::RTC)
    );
    rtc.set_client(mux_alarm);

    let virtual_alarm1 = static_init!(
        capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>,
        capsules::virtual_alarm::VirtualMuxAlarm::new(mux_alarm)
    );
    let alarm = static_init!(
        capsules::alarm::AlarmDriver<
            'static,
            capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>,
        >,
        capsules::alarm::AlarmDriver::new(virtual_alarm1, kernel::Grant::create())
    );
    virtual_alarm1.set_client(alarm);
    let ble_radio_virtual_alarm = static_init!(
        capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>,
        capsules::virtual_alarm::VirtualMuxAlarm::new(mux_alarm)
    );

    nrf52::uart::UARTE0.configure(
        nrf5x::pinmux::Pinmux::new(uart_pins.txd as u32),
        nrf5x::pinmux::Pinmux::new(uart_pins.rxd as u32),
        nrf5x::pinmux::Pinmux::new(uart_pins.cts as u32),
        nrf5x::pinmux::Pinmux::new(uart_pins.rts as u32),
    );
    let console = static_init!(
        capsules::console::Console<nrf52::uart::Uarte>,
        capsules::console::Console::new(
            &nrf52::uart::UARTE0,
            115200,
            &mut capsules::console::WRITE_BUF,
            &mut capsules::console::READ_BUF,
            kernel::Grant::create()
        )
    );
    kernel::hil::uart::UART::set_client(&nrf52::uart::UARTE0, console);
    console.initialize();

    // Attach the kernel debug interface to this console
    let kc = static_init!(capsules::console::App, capsules::console::App::default());
    kernel::debug::assign_console_driver(Some(console), kc);

    let ble_radio = static_init!(
        capsules::ble_advertising_driver::BLE<
            'static,
            nrf52::radio::Radio,
            VirtualMuxAlarm<'static, Rtc>,
        >,
        capsules::ble_advertising_driver::BLE::new(
            &mut nrf52::radio::RADIO,
            kernel::Grant::create(),
            &mut capsules::ble_advertising_driver::BUF,
            ble_radio_virtual_alarm
        )
    );
    kernel::hil::ble_advertising::BleAdvertisementDriver::set_receive_client(
        &nrf52::radio::RADIO,
        ble_radio,
    );
    kernel::hil::ble_advertising::BleAdvertisementDriver::set_transmit_client(
        &nrf52::radio::RADIO,
        ble_radio,
    );
    ble_radio_virtual_alarm.set_client(ble_radio);

    let temp = static_init!(
        capsules::temperature::TemperatureSensor<'static>,
        capsules::temperature::TemperatureSensor::new(
            &mut nrf5x::temperature::TEMP,
            kernel::Grant::create()
        )
    );
    kernel::hil::sensors::TemperatureDriver::set_client(&nrf5x::temperature::TEMP, temp);

    let rng = static_init!(
        capsules::rng::SimpleRng<'static, nrf5x::trng::Trng>,
        capsules::rng::SimpleRng::new(&mut nrf5x::trng::TRNG, kernel::Grant::create())
    );
    nrf5x::trng::TRNG.set_client(rng);

    // SPI
    let mux_spi = static_init!(
        MuxSpiMaster<'static, nrf52::spi::SPIM>,
        MuxSpiMaster::new(&nrf52::spi::SPIM0)
    );
    hil::spi::SpiMaster::set_client(&nrf52::spi::SPIM0, mux_spi);
    hil::spi::SpiMaster::init(&nrf52::spi::SPIM0);
    nrf52::spi::SPIM0.configure(
        nrf5x::pinmux::Pinmux::new(spi_pins.mosi as u32),
        nrf5x::pinmux::Pinmux::new(spi_pins.miso as u32),
        nrf5x::pinmux::Pinmux::new(spi_pins.clk as u32),
    );

    let nonvolatile_storage: Option<
        &'static capsules::nonvolatile_storage_driver::NonvolatileStorage<'static>,
    > = if let Some(driver) = mx25r6435f {
        // Create a SPI device for the mx25r6435f flash chip.
        let mx25r6435f_spi = static_init!(
            capsules::virtual_spi::VirtualSpiMasterDevice<'static, nrf52::spi::SPIM>,
            capsules::virtual_spi::VirtualSpiMasterDevice::new(
                mux_spi,
                &nrf5x::gpio::PORT[driver.chip_select]
            )
        );
        // Create an alarm for this chip.
        let mx25r6435f_virtual_alarm = static_init!(
            VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>,
            VirtualMuxAlarm::new(mux_alarm)
        );
        // Setup the actual MX25R6435F driver.
        let mx25r6435f = static_init!(
            capsules::mx25r6435f::MX25R6435F<
                'static,
                capsules::virtual_spi::VirtualSpiMasterDevice<'static, nrf52::spi::SPIM>,
                nrf5x::gpio::GPIOPin,
                VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>,
            >,
            capsules::mx25r6435f::MX25R6435F::new(
                mx25r6435f_spi,
                mx25r6435f_virtual_alarm,
                &mut capsules::mx25r6435f::TXBUFFER,
                &mut capsules::mx25r6435f::RXBUFFER,
                Some(&nrf5x::gpio::PORT[driver.write_protect_pin]),
                Some(&nrf5x::gpio::PORT[driver.hold_pin])
            )
        );
        mx25r6435f_spi.set_client(mx25r6435f);
        mx25r6435f_virtual_alarm.set_client(mx25r6435f);

        pub static mut FLASH_PAGEBUFFER: capsules::mx25r6435f::Mx25r6435fSector =
            capsules::mx25r6435f::Mx25r6435fSector::new();
        let nv_to_page = static_init!(
            capsules::nonvolatile_to_pages::NonvolatileToPages<
                'static,
                capsules::mx25r6435f::MX25R6435F<
                    'static,
                    capsules::virtual_spi::VirtualSpiMasterDevice<'static, nrf52::spi::SPIM>,
                    nrf5x::gpio::GPIOPin,
                    VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>,
                >,
            >,
            capsules::nonvolatile_to_pages::NonvolatileToPages::new(
                mx25r6435f,
                &mut FLASH_PAGEBUFFER
            )
        );
        hil::flash::HasClient::set_client(mx25r6435f, nv_to_page);

        let nonvolatile_storage = static_init!(
            capsules::nonvolatile_storage_driver::NonvolatileStorage<'static>,
            capsules::nonvolatile_storage_driver::NonvolatileStorage::new(
                nv_to_page,
                kernel::Grant::create(),
                0x60000, // Start address for userspace accessible region
                0x20000, // Length of userspace accessible region
                0,       // Start address of kernel accessible region
                0x60000, // Length of kernel accessible region
                &mut capsules::nonvolatile_storage_driver::BUFFER
            )
        );
        hil::nonvolatile_storage::NonvolatileStorage::set_client(nv_to_page, nonvolatile_storage);
        Some(nonvolatile_storage)
    } else {
        None
    };

    // Start all of the clocks. Low power operation will require a better
    // approach than this.
    nrf52::clock::CLOCK.low_stop();
    nrf52::clock::CLOCK.high_stop();

    nrf52::clock::CLOCK.low_set_source(nrf52::clock::LowClockSource::XTAL);
    nrf52::clock::CLOCK.low_start();
    nrf52::clock::CLOCK.high_set_source(nrf52::clock::HighClockSource::XTAL);
    nrf52::clock::CLOCK.high_start();
    while !nrf52::clock::CLOCK.low_started() {}
    while !nrf52::clock::CLOCK.high_started() {}

    let platform = Platform {
        button: button,
        ble_radio: ble_radio,
        console: console,
        led: led,
        gpio: gpio,
        rng: rng,
        temp: temp,
        alarm: alarm,
        nonvolatile_storage: nonvolatile_storage,
        ipc: kernel::ipc::IPC::new(),
    };

    let mut chip = nrf52::chip::NRF52::new();

    debug!("Initialization complete. Entering main loop\r");
    debug!("{}", &nrf52::ficr::FICR_INSTANCE);

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new());

    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
    }
    kernel::procs::load_processes(
        board_kernel,
        &_sapps as *const u8,
        app_memory,
        process_pointers,
        app_fault_response,
    );

    board_kernel.kernel_loop(&platform, &mut chip, process_pointers, Some(&platform.ipc));
}
