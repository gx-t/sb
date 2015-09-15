deb:
	arm-unknown-linux-gnu-gcc -Wall -g -o sb-001 sb-001.c

rel:
	arm-unknown-linux-gnu-gcc -Wall -O2 -s -o sb-001 sb-001.c

clean:
	rm -f sb-001

