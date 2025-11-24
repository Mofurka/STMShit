#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f1xx_hal::{pac, prelude::*, serial::{Config, Serial}};
use rtt_target::{rtt_init_print, rprintln};
use core::fmt::Write;
use bxcan::{filter::Mask32, Frame, StandardId, Fifo};

macro_rules! dual_println {
    ($tx:expr, $($arg:tt)*) => {{
        rprintln!($($arg)*);
        writeln!($tx, $($arg)*).ok();
    }};
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut afio = dp.AFIO.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Настройка UART
    let mut gpioa = dp.GPIOA.split();


    let tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let rx = gpioa.pa10;
    let serial = Serial::new(
        dp.USART1,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(115200.bps()),
        &clocks,
    );

    let (mut tx, _rx) = serial.split();

    let mut gpiob = dp.GPIOB.split();

    // Настройка CAN1 на пинах PA11 (RX) и PA12 (TX)
    let _can1_rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
    let _can1_tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);

    // Настройка CAN2 на пинах PB12 (RX) и PB13 (TX) - требует remap
   gpiob.pb12.into_floating_input(&mut gpiob.crh);
   gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh);

    // Инициализация CAN1 через HAL
    let can1_peripheral = stm32f1xx_hal::can::Can::new(dp.CAN1);
    let mut can1 = bxcan::Can::builder(can1_peripheral)
        .set_bit_timing(0x001c_0003) // 500 kbit/s при 36 MHz APB1
        .enable();

    // Инициализация CAN2 (зависит от CAN1)
    let can2_peripheral = stm32f1xx_hal::can::Can::new(dp.CAN2);
    let mut can2 = bxcan::Can::builder(can2_peripheral)
        .set_bit_timing(0x001c_0003) // 500 kbit/s при 36 MHz APB1
        .enable();


    can1.modify_filters()
        .enable_bank(0, Fifo::Fifo0, Mask32::accept_all())
        .slave_filters()
        .enable_bank(14, Fifo::Fifo0, Mask32::accept_all());

    let mut led = gpiob.pb10.into_push_pull_output(&mut gpiob.crh);

    let mut counter: u8 = 0;

    loop {
        dual_println!(tx, "LED ON - Sending CAN frame #{}", counter);
        led.set_high();

        let frame1 = Frame::new_data(StandardId::new(0x123).unwrap(), [counter, 0x01, 0x02, 0x03]);
        if let Err(_) = can1.transmit(&frame1) {
            dual_println!(tx, "CAN1 TX error");
        }

        delay(clocks.sysclk().raw() / 20);

        if let Ok(rx_frame) = can2.receive() {
            let id = match rx_frame.id() {
                bxcan::Id::Standard(sid) => sid.as_raw() as u32,
                bxcan::Id::Extended(eid) => eid.as_raw(),
            };
            dual_println!(tx, "CAN2 received: ID=0x{:03X}, Data={:?}", id, rx_frame.data());
        }

        dual_println!(tx, "LED OFF - Sending CAN frame #{}", counter);
        led.set_low();

        let frame2 = Frame::new_data(StandardId::new(0x456).unwrap(), [counter, 0x04, 0x05, 0x06]);
        if let Err(_) = can2.transmit(&frame2) {
            dual_println!(tx, "CAN2 TX error");
        }

        delay(clocks.sysclk().raw() / 20);

        if let Ok(rx_frame) = can1.receive() {
            let id = match rx_frame.id() {
                bxcan::Id::Standard(sid) => sid.as_raw() as u32,
                bxcan::Id::Extended(eid) => eid.as_raw(),
            };
            dual_println!(tx, "CAN1 received: ID=0x{:03X}, Data={:?}", id, rx_frame.data());
        }

        counter = counter.wrapping_add(1);
        delay(clocks.sysclk().raw() / 10);
    }
}



fn delay(mut cycles: u32) {
    while cycles != 0 {
        cortex_m::asm::nop();
        cycles -= 1;
    }
}
