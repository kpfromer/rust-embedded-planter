rm program.bin program.uf2
cargo build --release
cargo objcopy --release -- -O binary program.bin
./uf2conv.py program.bin -c -f 0xADA52840 -b 0x26000 -o program.uf2
cp program.uf2 /Volumes/CLUEBOOT
