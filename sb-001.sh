#!/bin/sh

# PIN3  --- LED
# PIN4  --- LED
#-------------
# PIN9  --- Counter
# PIN14 --- Counter Test Clock
# PIN15 --- DS18B20
# PIN16 --- Strobe
# PIN17 --- SDA
# PIN18 --- SCL
# PIN19-34 --- Parallel
# PIN35 --- ADC0
# PIN36 --- ADC1
# PIN37 --- ADC2
# PIN38 --- ADC3

dbfile=test.db

data() {
		echo -e ".timeout 1000\n.mode insert data\nselect * from data;" |
		sqlite3 $dbfile
}

while sleep 10
do
	curl -X POST -d "$(data)" http://www.seismoinstruments.am/insert.php
	echo -e ".timeout 1000\ndelete from data;" | sqlite3 $dbfile
done &


./sb-001 `cat key` | sqlite3 -batch $dbfile


