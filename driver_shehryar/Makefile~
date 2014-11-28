#CC=arm-linux-gcc
#EXTRA_CFLAGS:=-DUpp_STATISTICS_ENABLE
INCLUDEDIR=${KERNEL_DIR}/include
#upp-objs :=  csl/prototype.o csl/Upp_ioctl.o csl/Upp.o upp_driver.o 
ifneq ($(KERNELRELEASE), )
obj-m:=upp.o 
endif
# Use this makefile by calling: make -C <path to kernel> M=<path to this makefile>
