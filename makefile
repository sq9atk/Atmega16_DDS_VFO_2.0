avr-gcc -mmcu=atmega8 -Os -Wall dds.c -o dds.o;
avr-objcopy -O ihex dds.o dds.hex;
sudo avrdude -p m8 -P avrdoper -c usbasp -U flash:w:dds.hex

