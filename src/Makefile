CC=gcc
STRIP=strip

all: readtemp

clean:
	rm -f *.o libst6105j.so readtemp

readtemp: libst6105j.so readtemp.o
	$(CC) -fPIC readtemp.o -o readtemp -L. -lst6105j
	$(STRIP) --strip-all readtemp

libst6105j.so: st6105j.o
	$(CC) -fPIC st6105j.o -shared -Wl,-soname,libst6105j.so -o libst6105j.so
	$(STRIP) --strip-unneeded libst6105j.so

st6105j.o: st6105j.c
	$(CC) -fPIC -c st6105j.c -o st6105j.o

readtemp.o: readtemp.c
	$(CC) -fPIC -c readtemp.c -o readtemp.o

st6105j.c: st6105j.h
readtemp.c: st6105j.h

