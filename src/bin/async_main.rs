#![no_std]
#![no_main]

use core::cell::RefCell;

use alloc::boxed::Box;
use critical_section::Mutex;
use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{
    clock::CpuClock,
    gpio::{
        interconnect::{PeripheralInput, PeripheralOutput},
        Level, Output, OutputPin,
    },
    peripheral::Peripheral,
    peripherals::{Peripherals, TWAI0},
    prelude::*,
    rng::Rng,
    timer::timg::TimerGroup,
    twai::{
        filter::{Filter, SingleExtendedFilter},
        BaudRate, Twai, TwaiConfiguration, TwaiMode,
    },
    Async,
};
use esp_wifi::esp_now::{EspNow, EspNowReceiver};
use robocon_rs::{
    components::gamepad::Gamepad,
    node::{command::Command, message::EspNowMessage},
    util::sized_slice,
};
use static_cell::StaticCell;
use {defmt_rtt as _, esp_backtrace as _};

extern crate alloc;

const NODE_ID: u8 = 0x00;
const LED_FLASH_TIME_MS: u64 = 20;

static GAMEPAD: Mutex<RefCell<Option<Gamepad>>> = Mutex::new(RefCell::new(None));

fn peripherals_init(cpu_clock: CpuClock) -> Peripherals {
    let mut config = esp_hal::Config::default();
    config.cpu_clock = cpu_clock;
    esp_hal::init(config)
}

fn twai_init<'a, RX: PeripheralInput, TX: PeripheralOutput>(
    twai: TWAI0,
    filter: impl Filter,
    baud_rate: BaudRate,
    mode: TwaiMode,
    rx_pin: impl Peripheral<P = RX> + 'a,
    tx_pin: impl Peripheral<P = TX> + 'a,
) -> Twai<'a, Async> {
    let mut config = TwaiConfiguration::new(twai, rx_pin, tx_pin, baud_rate, mode).into_async();
    config.set_filter(filter);
    let twai = config.start();
    twai
}

fn led_init(
    spawner: &Spawner,
    led_pin: impl Peripheral<P = impl OutputPin> + 'static,
) -> &Signal<CriticalSectionRawMutex, Instant> {
    static LED_CTRL: StaticCell<Signal<CriticalSectionRawMutex, Instant>> = StaticCell::new();
    let led_ctrl = &*LED_CTRL.init(Signal::new());
    let led = Output::new(led_pin, Level::Low);
    spawner.spawn(led_flash(led, led_ctrl)).unwrap();
    led_ctrl
}

#[embassy_executor::task]
async fn led_flash(
    mut led: Output<'static>,
    control: &'static Signal<CriticalSectionRawMutex, Instant>,
) {
    let mut signal_time = Instant::now();
    loop {
        if control.signaled() {
            signal_time = control.wait().await;
        }
        if Instant::now().duration_since(signal_time) > Duration::from_millis(LED_FLASH_TIME_MS) {
            led.set_low();
        } else {
            led.set_high();
        }
        Timer::after(Duration::from_millis(1)).await;
    }
}

#[embassy_executor::task]
async fn esp_now_recv(mut esp_now_rx: EspNowReceiver<'static>) {
    loop {
        let recv = esp_now_rx.receive_async().await;
        if let Some(message) = EspNowMessage::from_slice(recv.data()) {
            let (_, to, command, payload) = message.split();
            if to == NODE_ID.into() {
                match command {
                    Command::NotifyGamepadState => {
                        if payload.len() == 9 {
                            critical_section::with(|cs| {
                                let mut gamepad = GAMEPAD.borrow_ref_mut(cs);
                                let gamepad = gamepad.as_mut().unwrap();
                                let new_state = Gamepad::from(sized_slice::<9>(&payload).unwrap());
                                gamepad.update(&new_state);
                            });
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

#[main]
async fn main(spawner: Spawner) {
    let peripherals = peripherals_init(CpuClock::max());
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    esp_alloc::heap_allocator!(72 * 1024);

    esp_hal_embassy::init(
        esp_hal::timer::systimer::SystemTimer::new(peripherals.SYSTIMER)
            .split::<esp_hal::timer::systimer::Target>()
            .alarm0,
    );
    info!("Embassy initialized.");

    let wifi_ctrl = Box::new(
        esp_wifi::init(
            timg0.timer0,
            Rng::new(peripherals.RNG),
            peripherals.RADIO_CLK,
        )
        .unwrap(),
    );
    let esp_now = EspNow::new(Box::leak(wifi_ctrl), peripherals.WIFI).unwrap();
    info!(
        "ESP Now initialized. version: {}",
        esp_now.version().unwrap()
    );
    let (_, _esp_now_tx, esp_now_rx) = esp_now.split();

    let mut _twai = twai_init(
        peripherals.TWAI0,
        SingleExtendedFilter::new(b"xxxxxxxxxxxxxxxxxxxxxxxxxxxxx", b"x"),
        BaudRate::B1000K,
        TwaiMode::Normal,
        peripherals.GPIO1,
        peripherals.GPIO2,
    );
    info!("TWAI initialized.");

    let _led_ctrl = led_init(&spawner, peripherals.GPIO7);
    info!("LED Indicator initialized.");

    critical_section::with(|cs| {
        GAMEPAD.borrow_ref_mut(cs).replace(Gamepad::default());
    });

    let _ = spawner.spawn(esp_now_recv(esp_now_rx)).unwrap();
    info!("ESP Now receive task spawn.");

    loop {
        critical_section::with(|cs| {
            let mut gamepad = GAMEPAD.borrow_ref_mut(cs);
            let gamepad = gamepad.as_mut().unwrap();
            println!("");
            println!(
                "Joystick: left {}, right {}",
                gamepad.left_joystick(),
                gamepad.right_joystick()
            );
            println!(
                "Trigger: left {}, right {}",
                gamepad.l2_value(),
                gamepad.r2_value()
            );
            println!(
                "A: {}, B: {}, X: {}, Y: {}",
                gamepad.is_circle_pushed(),
                gamepad.is_cross_pushed(),
                gamepad.is_triangle_pushed(),
                gamepad.is_square_pushed(),
            );
            println!(
                "Up: {}, Down: {}, Left: {}, Right: {}",
                gamepad.is_up_pushed(),
                gamepad.is_down_pushed(),
                gamepad.is_left_pushed(),
                gamepad.is_right_pushed(),
            );
            println!(
                "L1: {}, R1: {}, L2: {}, R2: {}, L3: {}, R3: {}",
                gamepad.is_l1_pushed(),
                gamepad.is_r1_pushed(),
                gamepad.is_l2_pushed(),
                gamepad.is_r2_pushed(),
                gamepad.is_l3_pushed(),
                gamepad.is_r3_pushed(),
            );
            println!(
                "Select: {}, Start: {}, Share: {}, Home: {}",
                gamepad.is_select_pushed(),
                gamepad.is_start_pushed(),
                gamepad.is_share_pushed(),
                gamepad.is_home_pushed(),
            );
        });
        Timer::after(Duration::from_millis(100)).await;
    }
}
