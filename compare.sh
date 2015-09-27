#!/bin/sh
# Compares baserom.gbc and pokecrystal.gbc

# create baserom.txt if necessary
if [ ! -f baserom.txt ]; then
    hexdump -C "GBA.ROM" > "baserom.txt"
fi

hexdump -C GBABios.gba > GBABios.txt

diff -u "baserom.txt" GBABios.txt | less
