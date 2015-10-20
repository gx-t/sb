deb:
	arm-unknown-linux-gnu-gcc -Wall -g -o sb-001 sb-001.c
	mipsel-openwrt-linux-gcc -Wall -g -o sb-002 sb-002.c

rel:
	arm-unknown-linux-gnu-gcc -Wall -O2 -s -o sb-001 sb-001.c
	mipsel-openwrt-linux-gcc -Wall -O2 -s -o sb-002 sb-002.c

send/arm:
	scp sb-001 sb-001.c root@192.168.0.111:

send/mips:
	scp sb-002 root@192.168.0.115:

clean:
	rm -f sb-001
	rm -f sb-002

