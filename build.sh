#!/bin/sh
export ARCH=arm
export CROSS_COMPILE=~/android/platforms/carbon-kk/prebuilts/gcc/linux-x86/arm/arm-eabi-4.7/bin/arm-eabi-
make f6_mpcs_tmo_defconfig
make -j8
