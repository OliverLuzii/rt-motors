#![no_std]
#![no_main]
#![feature(ascii_char)]

use esp_backtrace as _;
use circular_buffer::CircularBuffer;
use static_cell::StaticCell;

use embassy_executor::Spawner;
use embassy_futures::select::{Either};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::watch::Watch;
use esp_hal::Async;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::{Input, Level};
use esp_hal::interrupt::Priority;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::mcpwm::operator::PwmPinConfig;
use esp_hal::mcpwm::timer::PwmWorkingMode;
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig};
use esp_hal::peripherals::MCPWM0;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::uart::{RxConfig, Uart, UartRx};
use esp_rtos::embassy::InterruptExecutor;

esp_bootloader_esp_idf::esp_app_desc!();

const MICROS_TO_SECS: f32 = 0.000001;
const HZ_TO_RPM: f32 = 60.0;
const NEWLINE: u8 = 13;

const INPUT_WATCH_SIZE: usize = 2;
const MOTOR_WATCH_SIZE: usize = 2;

const UART_RBUF_SIZE: usize = 8;

const ENCODER_TIMEOUT_MICROS: u64 = 50_000;
const POSEDGE_HZ_RATIO: f32 = 6.4634;
const GEAR_REDUCTION: f32 = 1.0 / 100.0;

const PWM_PERIOD: u16 = 3000;
const RPM_BUFFER_SIZE: usize = 6;
const RPM_DEADZONE: f32 = 3.0;
const K_P: f32 = 0.015 * PWM_PERIOD as f32;
const K_I: f32 = 0.02 * PWM_PERIOD as f32;

#[derive(Debug, Default, Clone)]
pub struct MotorState {
    fall: Option<embassy_time::Instant>,
    rise: Option<embassy_time::Instant>,
    positive_edge: Option<embassy_time::Duration>,
    hall_state: u8,
    speed_buffer: CircularBuffer<RPM_BUFFER_SIZE, f32>
}

impl MotorState {
    // Gives unstable readings due to sensor innacuracies, filtering necessary.
    fn get_speed_reading(&self) -> Option<f32> {
        let (fall, rise, pos_edge) = match (self.fall, self.rise, self.positive_edge) {
            (Some(f), Some(r), Some(p_e)) => (f, r, p_e),
            _ => return None,
        };

        let direction: f32 = match self.hall_state {
            0b0010 | 0b1011 | 0b1101 | 0b0100 => 1.0,
            0b0001 | 0b0111 | 0b1110 | 0b1000 => -1.0,
            _ => {
                log::warn!("Invalid state transition for motor.");
                return None;
            }
        };

        let instant = embassy_time::Instant::now();

        let rise_delta = instant - rise;
        let fall_delta = instant - fall;

        if rise_delta.as_micros() >= ENCODER_TIMEOUT_MICROS
            || fall_delta.as_micros() >= ENCODER_TIMEOUT_MICROS
        {
            return Some(0.0);
        }

        return Some(
            (1.0 / ((pos_edge.as_micros() as f32 * MICROS_TO_SECS) * POSEDGE_HZ_RATIO))
                * HZ_TO_RPM
                * GEAR_REDUCTION
                * direction,
        );
    }

    fn get_speed(&self) -> f32 {
        self.speed_buffer.iter().sum::<f32>() / RPM_BUFFER_SIZE as f32
    }

    fn add_to_buffer(&mut self, speed: Option<f32>) {
        match speed {
            Some(spd) => {self.speed_buffer.push_back(spd);},
            None => {}
        }
    }
}

#[embassy_executor::task]
async fn uart_reader(
    mut rx: UartRx<'static, Async>,
    input_watch: &'static Watch<NoopRawMutex, Option<f32>, INPUT_WATCH_SIZE>,
) {
    let mut rbuf: [u8; UART_RBUF_SIZE] = [0u8; UART_RBUF_SIZE];
    let mut offset = 0;

    let input_sender = input_watch.sender();
    loop {
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf[offset..]).await;

        match r {
            Ok(len) => {
                if offset < UART_RBUF_SIZE {
                    offset += len;
                } else {
                    log::warn!("Overflowed message buffer, emptying...");
                    rbuf.fill(0);
                    offset = 0;
                }
            }
            _ => {},
        };

        if !rbuf.contains(&NEWLINE) {
            continue;
        }

        let _rbuf = rbuf.clone();
        let string: Option<&[core::ascii::Char; UART_RBUF_SIZE]> = _rbuf.as_ascii();

        rbuf.fill(0);
        offset = 0;

        let value = match string {
            Some(str) => str::parse::<f32>(str.as_str().trim_matches(|c| c == '\0' || c == '\r')),
            None => {
                log::warn!("Couldn't convert message to ASCII string, ignoring...");
                continue;
            }
        }
        .ok();

        input_sender.send(value);
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn rpm_interrupt(
    xor_pin: AnyPin<'static>,
    h1_pin: AnyPin<'static>,
    h2_pin: AnyPin<'static>,
    motor_watch: &'static Watch<CriticalSectionRawMutex, Option<f32>, MOTOR_WATCH_SIZE>,
) {
    let input_config = esp_hal::gpio::InputConfig::default();

    let mut xor = Input::new(xor_pin, input_config);
    let h1 = Input::new(h1_pin, input_config);
    let h2 = Input::new(h2_pin, input_config);

    let mut motor_state = MotorState::default();
    let motor_sender = motor_watch.sender();

    loop {
        xor.wait_for_any_edge().await;
        let time = embassy_time::Instant::now();

        let xor_level = xor.level();
        let h1_level = h1.level();
        let h2_level = h2.level();

        let (fall, rise) = match xor_level {
            Level::Low => (Some(time), None),
            Level::High => (None, Some(time)),
        };

        let hall_state = match (h1_level, h2_level) {
            (Level::Low, Level::Low) => 0b00,
            (Level::High, Level::Low) => 0b10,
            (Level::High, Level::High) => 0b11,
            (Level::Low, Level::High) => 0b01,
        };

        let fall = fall.or(motor_state.fall);
        let rise = rise.or(motor_state.rise);

        let positive_edge = match (fall, rise) {
            (Some(f), Some(r)) => {
                if f > r {
                    Some(f - r)
                } else {
                    None
                }
            }

            _ => None,
        }
        .or(motor_state.positive_edge);

        motor_state.fall = fall;
        motor_state.rise = rise;
        motor_state.positive_edge = positive_edge;
        motor_state.hall_state = 0b1111 & (motor_state.hall_state << 2 | hall_state);
        motor_state.add_to_buffer(motor_state.get_speed_reading());

        motor_sender.send(Some(motor_state.get_speed()));
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn pid_controller(
    mcpwm: MCPWM0<'static>,
    pin_a: AnyPin<'static>,
    pin_b: AnyPin<'static>,
    input_watch: &'static Watch<NoopRawMutex, Option<f32>, INPUT_WATCH_SIZE>,
    motor_watch: &'static Watch<CriticalSectionRawMutex, Option<f32>, MOTOR_WATCH_SIZE>,
) {
    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(80)).unwrap();
    let mut mcpwm = McPwm::new(mcpwm, clock_cfg);

    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer1);
    let mut pin_a = mcpwm
        .operator0
        .with_pin_a(pin_a, PwmPinConfig::UP_ACTIVE_HIGH);

    let mut pin_b = mcpwm
        .operator1
        .with_pin_b(pin_b, PwmPinConfig::UP_ACTIVE_HIGH);

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(PWM_PERIOD - 1, PwmWorkingMode::Increase, Rate::from_khz(20))
        .unwrap();

    mcpwm.timer0.start(timer_clock_cfg.clone());
    mcpwm.timer1.start(timer_clock_cfg.clone());

    let mut reference_rpm = 0f32;
    let mut motor_rpm = 0f32;

    let mut accum_error = 0f32;
    let mut previous_time = embassy_time::Instant::now();

    let mut input_receiver = input_watch.receiver().unwrap();
    let mut motor_receiver = motor_watch.receiver().unwrap();

    loop {
        match embassy_futures::select::select(input_receiver.changed(), motor_receiver.changed())
            .await
        {
            Either::First(ref_rpm) => {
                if let Some(rpm) = ref_rpm {
                    reference_rpm = rpm;
                }
            }
            Either::Second(m_rpm) => {
                if let Some(rpm) = m_rpm {
                    motor_rpm = rpm;
                }
            }
        };

        let error = reference_rpm - motor_rpm;
        let delta = embassy_time::Instant::now() - previous_time;

        if reference_rpm.abs() > RPM_DEADZONE {
            accum_error += error * (delta.as_micros() as f32 * MICROS_TO_SECS);
        } else {
            accum_error = 0.0;
        }

        // if accum_error * error <= 0.0 {
        //    accum_error = 0.0; 
        // }

        let control =
            (K_P * error + K_I * accum_error).clamp(-(PWM_PERIOD as f32), PWM_PERIOD as f32);

        match control.signum() {
            -1.0 => {
                pin_b.set_timestamp(0);
                pin_a.set_timestamp(libm::roundf(control.abs()) as u16);
            }
            1.0 => {
                pin_a.set_timestamp(0);
                pin_b.set_timestamp(libm::roundf(control.abs()) as u16);
            }
            _ => {
                log::warn!("Control signal isn't a number.");
            }
        }

        // log::info!("Control signal applied: {} from {}", libm::roundf(control.abs()) as u16, PWM_PERIOD);
        previous_time = embassy_time::Instant::now();
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    esp_rtos::start(timg0.timer0);

    let uart_config = esp_hal::uart::Config::default()
        .with_rx(RxConfig::default().with_fifo_full_threshold(UART_RBUF_SIZE as u16));

    let uart = Uart::new(peripherals.UART0, uart_config)
        .unwrap()
        .with_tx(peripherals.GPIO1)
        .with_rx(peripherals.GPIO3)
        .into_async();

    let (rx, _) = uart.split();

    static INPUT_WATCH: StaticCell<Watch<NoopRawMutex, Option<f32>, INPUT_WATCH_SIZE>> =
        StaticCell::new();
    let input_watch = INPUT_WATCH.init(Watch::new());

    static LM_WATCH: StaticCell<Watch<CriticalSectionRawMutex, Option<f32>, MOTOR_WATCH_SIZE>> =
        StaticCell::new();
    let lm_watch = LM_WATCH.init(Watch::new());

    static INTERRUPT_EXECUTOR: StaticCell<InterruptExecutor<0>> = StaticCell::new();
    let interrupt_executor = INTERRUPT_EXECUTOR.init(InterruptExecutor::new(
        software_interrupt.software_interrupt0,
    ));

    let interrupt_spawner = interrupt_executor.start(Priority::Priority3);

    spawner.must_spawn(uart_reader(rx, input_watch));
    log::info!("UART reader initialized.");

    interrupt_spawner.must_spawn(rpm_interrupt(
        peripherals.GPIO4.into(),
        peripherals.GPIO16.into(),
        peripherals.GPIO17.into(),
        lm_watch,
    ));
    log::info!("RPM sensor initialized.");

    spawner.must_spawn(pid_controller(
        peripherals.MCPWM0,
        peripherals.GPIO22.into(),
        peripherals.GPIO23.into(),
        input_watch,
        lm_watch,
    ));
    log::info!("PID controller initialized.\n");
    
    let mut input_receiver = input_watch.receiver().unwrap();
    let mut lm_receiver = lm_watch.receiver().unwrap();

    loop {
        let lm_poll = embassy_futures::join::join(
            lm_receiver.changed(),
            embassy_time::Timer::after_millis(200),
        );

        match embassy_futures::select::select(input_receiver.changed(), lm_poll).await {
            Either::First(Some(input_rpm)) => {
                log::info!("Received input, new set speed: {} rpm", input_rpm);
            }
            Either::Second((Some(motor_rpm), _)) => {
                log::info!("Motor speed polled, currently at: {} rpm", motor_rpm);
            }
            _ => {}
        };
    }
}
