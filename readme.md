Сначала установите:
cargo install cargo-binutils
rustup component add llvm-tools-preview
Затем создайте бинарник:
cargo objcopy --release -- -O binary firmware.bin
Вариант 2: Используйте arm-none-eabi-objcopy напрямую
cargo build --release
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/stm32f105-blinky.exe firmware.bin
