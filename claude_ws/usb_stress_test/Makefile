CC = gcc
CFLAGS = -O2 -Wall -pthread
LDFLAGS = -lpthread

host_libusb: host_libusb.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS) -lusb-1.0

device_fs: device_fs.c
	$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

.PHONY: clean
clean:
	rm -f host_libusb device_fs
