export ARCH=arm64
EXTRA_CFLAGS   += -O0
obj-m+=s5p_mmc.o

pwd:=`pwd`
kdir:=/root/nanopi_m3/linux

all:
	make -C $(kdir) M=$(pwd) modules
	cp s5p_mmc.ko /nfs/ -f
clean:
	rm -rf *.o *.ko Module.symvers modules.order *.mod.c
