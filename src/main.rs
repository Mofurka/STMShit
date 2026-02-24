#![deny(warnings)]
#![no_main]
#![no_std]


use panic_halt as _;
use rtt_target::{rtt_init_print, rprintln};
use stm32f4xx_hal::{pac, prelude::*, serial::{Config, Serial}};
use stm32f4xx_hal::rcc::Config as RccConfig;
use cortex_m_rt::entry;
use core::fmt::Write;
use bxcan::{filter::Mask32, Fifo, Frame, StandardId};

macro_rules! dual_println {
    ($tx:expr, $($arg:tt)*) => {{
        rprintln!($($arg)*);
        writeln!($tx, $($arg)*).ok();
    }};
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Starting STM32F407 RGB LED blinky...");

    let dp = pac::Peripherals::take().unwrap();

    let mut rcc = dp
        .RCC
        .freeze(RccConfig::default());

    // Настройка UART и GPIO - делаем split до constrain
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioe = dp.GPIOE.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);



    rprintln!("Clock configured: SYSCLK = {} Hz", &rcc.clocks.sysclk().raw());


    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();
    let serial: Serial<_, u8> = Serial::new(
        dp.USART1,
        (tx_pin, rx_pin),
        Config::default().baudrate(115200.bps()),
        &mut rcc,
    ).unwrap();
    
    let (mut tx, _rx) = serial.split();

    // CAN1 на пинах PA11 (RX) и PA12 (TX) - стандартные пины для CAN1
    let can1 = {
        let can_rx = gpioa.pa11.into_alternate();
        let can_tx = gpioa.pa12.into_alternate();

        let can = stm32f4xx_hal::can::Can::new(dp.CAN1, (can_tx, can_rx), &mut rcc);

        bxcan::Can::builder(can)
            // APB1 (PCLK1): 8MHz, Bit rate: 500kBit/s, Sample Point 87.5%
            // Value was calculated with http://www.bittiming.can-wiki.info/
            .set_bit_timing(0x001c_0000)
            .set_loopback(true)  // Loopback - CAN принимает свои сообщения
            .set_silent(true)    // Silent - не требуется ACK от других узлов
    };
    dual_println!(tx, "CAN1 initialized with loopback+silent mode.");

    // CAN2 на пинах PB5 (RX) и PB6 (TX)
    let can2 = {
        let can_rx = gpiob.pb5.into_alternate();
        let can_tx = gpiob.pb6.into_alternate();

        let can = stm32f4xx_hal::can::Can::new(dp.CAN2, (can_tx, can_rx), &mut rcc);

        bxcan::Can::builder(can)
            // APB1 (PCLK1): 8MHz, Bit rate: 500kBit/s, Sample Point 87.5%
            // Value was calculated with http://www.bittiming.can-wiki.info/
            .set_bit_timing(0x001c_0000)
            .set_loopback(true)  // Loopback - CAN принимает свои сообщения
            .set_silent(true)    // Silent - не требуется ACK от других узлов
    };
    dual_println!(tx, "CAN2 initialized with loopback+silent mode.");

    // Configure filters so that can frames can be received.
    let mut can1 = can1.enable();
    let mut filters = can1.modify_filters();
    filters.enable_bank(0, Fifo::Fifo0, Mask32::accept_all());

    let mut can2 = {
        let can2_enabled = can2.enable();

        // A total of 28 filters are shared between the two CAN instances.
        // Split them equally between CAN1 and CAN2.
        filters.set_split(14);
        let mut slave_filters = filters.slave_filters();
        slave_filters.enable_bank(14, Fifo::Fifo0, Mask32::accept_all());
        can2_enabled
    };

    // Drop filters to leave filter configuration mode.
    drop(filters);

    dual_println!(tx, "CAN1 and CAN2 enabled, ready for communication");

    let mut red = gpioe.pe3.into_push_pull_output();
    let mut green = gpioe.pe4.into_push_pull_output();
    let mut blue = gpioe.pe5.into_push_pull_output();

    dual_println!(tx, "GPIO configured, starting LED cycle...");

    let mut cycle_count: u32 = 0;

    loop {
        red.set_low();
        green.set_low();
        blue.set_low();
        dual_println!(tx, "LED OFF");

        delay();
        

        red.set_high();
        green.set_low();
        blue.set_low();
        dual_println!(tx, "RED");
        delay();

        red.set_low();
        green.set_high();
        blue.set_low();
        dual_println!(tx, "GREEN");
        delay();

        red.set_low();
        green.set_low();
        blue.set_high();
        dual_println!(tx, "BLUE");
        delay();

        red.set_high();
        green.set_high();
        blue.set_low();
        dual_println!(tx, "YELLOW");
        delay();

        red.set_high();
        green.set_low();
        blue.set_high();
        dual_println!(tx, "MAGENTA");
        delay();

        red.set_low();
        green.set_high();
        blue.set_high();
        dual_println!(tx, "CYAN");
        delay();

        red.set_high();
        green.set_high();
        blue.set_high();
        dual_println!(tx, "WHITE");
        delay();

        // Выводим статистику CAN
        cycle_count += 1;

        // Отправляем CAN сообщение из CAN1 в конце каждого цикла
        // CAN1 -> CAN2 (используя данные счетчика цикла)
        let data = [
            (cycle_count >> 24) as u8,
            (cycle_count >> 16) as u8,
            (cycle_count >> 8) as u8,
            cycle_count as u8,
            0xAA,
            0xBB,
            0xCC,
            0xDD,
        ];
        let frame = Frame::new_data(StandardId::new(0x100).unwrap(), data);

        // Простая синхронная отправка
        match can1.transmit(&frame) {
            Ok(_status) => {
                dual_println!(tx, "CAN1 TX: Sent cycle #{}", cycle_count);
            }
            Err(nb::Error::WouldBlock) => {
                dual_println!(tx, "CAN1 TX: Mailbox full (WouldBlock)");
            }
            Err(_) => {
                dual_println!(tx, "CAN1 TX: Error");
            }
        }

        // Небольшая задержка для передачи сообщения по шине
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }

        // CAN2 принимает сообщение от CAN1
        match can2.receive() {
            Ok(rx_frame) => {
                let id = match rx_frame.id() {
                    bxcan::Id::Standard(sid) => sid.as_raw() as u32,
                    bxcan::Id::Extended(eid) => eid.as_raw(),
                };
                dual_println!(tx, "CAN2 RX from CAN1: ID=0x{:03X}, Data={:?}", id, rx_frame.data());
            }
            Err(nb::Error::WouldBlock) => {
                dual_println!(tx, "CAN2 RX: No messages from CAN1");
            }
            Err(_) => {
                dual_println!(tx, "CAN2 RX: Error");
            }
        }

        // Отправляем с CAN2 ответ обратно в CAN1
        let frame2 = Frame::new_data(StandardId::new(0x200).unwrap(), [cycle_count as u8, 0x11, 0x22, 0x33]);
        match can2.transmit(&frame2) {
            Ok(_) => dual_println!(tx, "CAN2 TX: Sent reply cycle #{}", cycle_count),
            Err(nb::Error::WouldBlock) => {
                dual_println!(tx, "CAN2 TX: Mailbox full (WouldBlock)");
            }
            Err(_) => dual_println!(tx, "CAN2 TX: Error"),
        }

        // Небольшая задержка
        for _ in 0..10000 {
            cortex_m::asm::nop();
        }

        // CAN1 принимает ответ от CAN2
        match can1.receive() {
            Ok(rx_frame) => {
                let id = match rx_frame.id() {
                    bxcan::Id::Standard(sid) => sid.as_raw() as u32,
                    bxcan::Id::Extended(eid) => eid.as_raw(),
                };
                dual_println!(tx, "CAN1 RX from CAN2: ID=0x{:03X}, Data={:?}", id, rx_frame.data());
            }
            Err(nb::Error::WouldBlock) => {
                dual_println!(tx, "CAN1 RX: No messages from CAN2");
            }
            Err(_) => {
                dual_println!(tx, "CAN1 RX: Error");
            }
        }
    }
}

fn delay() {
    for _ in 0..1_000_000 {
        cortex_m::asm::nop();
    }
  
}