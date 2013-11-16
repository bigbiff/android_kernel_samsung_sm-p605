HOW TO BUILD KERNEL 3.4.39 FOR SM-P605

1. How to Build
	- get Toolchain
			From Android Source Download site( http://source.android.com/source/downloading.html )
			Toolchain is included in Android source code.

	- edit Makefile
			edit "CROSS_COMPILE" to right toolchain path(You downloaded).
			ex) CROSS_COMPILE= $(android platform directory you download)/android/prebuilt/linux-x86/toolchain/arm-eabi-4.7/bin/arm-eabi-
			ex) CROSS_COMPILE=/usr/local/toolchain/arm-eabi-4.7/bin/arm-eabi-

	- make
			$ export ARCH=arm
			$ make VARIANT_DEFCONFIG=msm8974_sec_lt03eur_defconfig msm8974_sec_defconfig SELINUX_DEFCONFIG=selinux_defconfig
			$ make

2. Output files
	- Kernel : Kernel/arch/arm/boot/zImage
	- module : Kernel/drivers/*/*.ko

3. How to make .tar binary for downloading into target.
	- change current directory to Kernel/arch/arm/boot
	- type following command
	$ tar cvf SM-P605_Kernel.tar zImage
