deb:
	arm-unknown-linux-gnu-gcc -Wall -g -o sb-001 sb-001.c

rel:
	arm-unknown-linux-gnu-gcc -Wall -O2 -s -o sb-001 sb-001.c

002:
	mipsel-openwrt-linux-uclibc-gcc -Wall -O2 -s -o sb-002 sb-002.c -lm

send/arm:
	scp sb-001 sb-001.c root@192.168.0.111:

send/mips:
	scp sb-002 root@192.168.0.222:/tmp

tag:
	ctags -R . ~/toolchain-mipsel_24kec+dsp_gcc-4.8-linaro_uClibc-0.9.33.2/include

clean:
	rm -f sb-001
	rm -f sb-002
	rm -f tags

