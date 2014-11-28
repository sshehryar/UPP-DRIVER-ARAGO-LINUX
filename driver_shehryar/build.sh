#export KERNEL_DIR=/home/hassan/Applications/omapL138Sdk/psp/linux-2.6.37-psp03.21.00.04.sdk/
#export KERNEL_DIR=/home/hassan/Applications/omapL138Sdk/psp/linux-2.6.33-rc4-psp03.20.00.14.sdk/
#export TOOLCHAIN_PATH=/home/build/toolchains/arm-2009q1/bin/
#export PATH=${PATH}:${TOOLCHAIN_PATH}
#export COMPILER_PREFIX=arm-none-linux-gnueabi-
#make -C ${KERNEL_DIR} M=`pwd` modules ARCH=arm CROSS_COMPILE=${COMPILER_PREFIX}

make -C ${KERNEL_DIR} M=`pwd` modules ARCH=arm CROSS_COMPILE=${COMPILER_PREFIX}
