#![deny(unsafe_code)]
#![deny(warnings)]
#![no_main]
#![no_std]


use panic_halt as _;
use rtt_target::{rtt_init_print, rprintln};
use stm32f4xx_hal::{pac, prelude::*, serial::{Config, Serial}};
use cortex_m_rt::entry;
use core::fmt::Write;

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

    let p = pac::Peripherals::take().unwrap();

    let rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.freeze();
    rprintln!("Clock configured: SYSCLK = {} Hz", clocks.sysclk().raw());



    // Настройка UART
    let gpioa = p.GPIOA.split();
    let gpioe = p.GPIOE.split();


    let tx = gpioa.pa9.into_alternate();
    let rx = gpioa.pa10.into_alternate();
    let serial: Serial<_, u8> = Serial::new(
        p.USART1,
        (tx, rx),
        Config::default().baudrate(115200.bps()),
        &clocks,
    ).unwrap();
    
    let (mut tx, _rx) = serial.split();
    let mut red = gpioe.pe3.into_push_pull_output();
    let mut green = gpioe.pe4.into_push_pull_output();
    let mut blue = gpioe.pe5.into_push_pull_output();

    dual_println!(tx, "GPIO configured, starting LED cycle...");

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
    }
}

fn delay() {
    for _ in 0..1_000_000 {
        cortex_m::asm::nop();
    }
  
}
