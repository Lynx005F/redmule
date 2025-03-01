export BENDER_DIR=$(pwd)/hw/bender
echo "Exporting bender path to $BENDER_DIR"
export PATH=$BENDER_DIR:$PATH
unset BENDER_DIR
echo "Exporting SDK and GCC Toolchain paths"
export PATH=/usr/pack/riscv-1.0-kgf/pulp-gcc-2.6.0/bin:$PATH
export PULP_RISCV_GCC_TOOLCHAIN=/usr/pack/riscv-1.0-kgf/pulp-gcc-2.6.0
export PULP_CC=riscv32-unknown-elf-gcc
export PULP_LD=riscv32-unknown-elf-gcc
export PATH=/usr/pack/gcc-5.2.0-af/x86_64-rhe6-linux/bin:$PATH
