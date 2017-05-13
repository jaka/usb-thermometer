OPTS	:= -O2 -s
CFLAGS	+= -Wall -Werror -Wextra -fdata-sections -ffunction-sections -fno-strict-aliasing -std=c90
LDFLAGS	+= -Wl,--gc-sections
SFLAGS	:= -R .comment -R .gnu.version -R .note -R .note.ABI-tag

CC	?= cc
STRIP	?= strip

#SOURCES	:= $(wildcard src/*.c)
#OBJECTS	:= $(patsubst %.c,%.o,$(SOURCES))
TARGET := usbtemp

all: $(OBJECTS) $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) $(OPTS) -c -o $@ $<

$(TARGET): $(OBJECTS) $(TARGET).c
	$(CC) $(CFLAGS) $(OPTS) -o $@ $^ $(LDFLAGS)
	$(STRIP) $(SFLAGS) $@

clean:
#	rm -fr $(OBJECTS)
	rm -f $(TARGET)

