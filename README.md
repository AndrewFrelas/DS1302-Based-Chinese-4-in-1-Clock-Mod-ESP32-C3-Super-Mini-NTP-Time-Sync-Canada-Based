# DS1302-Based-Chinese-4-in-1-Clock-Mod-ESP32-C3-Super-Mini-NTP-Time-Sync-Canada-Based-

<img src="https://andrewfrelas.com/images/2c648675-4923-4c1a-ac4f-e0c0c9073953.png" alt="Clock">

Firmware to drive a Chinese DS1302 based Clock with a charger using a ESP32 C3 Super Mini for NTP Time - Canadian Based API used for temp.

Please note that I have used Claude to quickly code this firmware and I have only been testing this for a few days, It was built mainly for myself and as such there are some limits to the firmware like that the api is Canadian so temperatures will only work in Canada and that the clock has no way of making negative values so everything will show as positive values and it was made using the official government of Canada api. Furthermore it only uses unmanned weather stations though i do plan to change this in the future.

I would also like to highlight that I do not have a complete build guide yet but i will post photos of it later including where I tapped
off the clock to get 5v from to power the esp32 C3 Super Mini. 

Furthermore this will require the removal of the DS1302 and the temperature probe and the temp probe and may need to have a offset applied to
work properly for the temperature and i cant guarantee they will not change the clock or boards in the future so make sure you know this is
a risky mod and you could end up with a non functional device therefor I take no responsibility for you performing this mod.
```
Bill of materials

1 x Temu "4-in-1" wireless charger alarm clock Boards marked YM-SZ010A V0.1 (green) and YM-SZ010-L V0.5 (white)
1 x ESP32-C3 SuperMini
3 x 10 kΩ resistor (one in series in each bus wire — CE, SCLK, I/O.)
1 x 220 Ω resistor (series in the temperature line)
1 x 2.2 µF capacitor 16 V or better; ceramic, film or electrolytic all fine
Hook-up wire for power and wrapping wire for bus wires for chip and temp probe pad
```
On the white board find the DS1302 chip and remove it along with the temperature probe.

The DS1302, and its pinout

The DS1302 is an 8-pin trickle-charge timekeeping chip with 31 bytes of static RAM, talking a 3-wire synchronous serial interface.

```
        +--\__/--+
 VCC2  1|        |8  VCC1
   X1  2|        |7  SCLK
   X2  3|        |6  I/O
  GND  4|        |5  CE  (also called RST)
        +--------+
```

Solder wires to 5,6,7

On the ESP32-C3 SuperMini side solder 3 10 kΩ resistor to the super mini and attach

| ESP32-C3 pin | Series resistor | Goes to | Notes |
|---|---|---|---|
| GPIO3 | 10 kΩ | DS1302 pad **5** (CE) | input only, rising-edge interrupt |
| GPIO4 | 10 kΩ | DS1302 pad **7** (SCLK) | input only |
| GPIO5 | 10 kΩ | DS1302 pad **6** (I/O) | high-Z except when answering a read |
| GPIO7 | 220 Ω | NTC pad (the ~2 V one) | plus 2.2 µF from that pad to ground |

now for the temperature probe its best to check with a multimeter while the clock is powered each pad as they are unlabeled one will likely 
be at 5v assuming you are powering the clock with a usbc to usb cable and a 5v usb power brick it is important to check as if you get the wrong
pad you may damage your ESP32-C3 SuperMini but the pad that is not at 5v is the one you are after.

```
| GPIO7 | 220 Ω | NTC pad (the ~2 V one) | plus 2.2 µF from that pad to ground |
```

for me as to where i got the ground from i used the ground from pin 4 of the now removed DS1302

Now how to power the ESP32-C3 SuperMini

my advice here is to look for a pad with ground there is a small one labled as such on the green board, having said that i would avoid the vcc for 5v
there is a surface mount resistor on the green board that when powered will give 5v as if you use vcc it may have too low a voltage and the esp will fail
to boot properly all the time.

Programming the ESP32-C3 SuperMini

you will need a computer with arduino studio and the esp32 boards loaded on forgive me as i do not have a tutorial for that It has been a long time since
i added the esp32 boards so i do not remember how its done. Anyhow you will need to edit the .ino file to have your ssid and wifi password then compile and
flash the file onto your ESP32-C3 SuperMini. After you have done that it should work. 

For now I know this is a real basic write up but I wanted to just get the information out and while I did use claude for coding i wanted to write this section
myself to ensure its accurate. Unlike most I want to be clear in my use of AI and Claude in this project so there is no misconceptions about this being coded
by myself and that I do have limited time to fix things or change things so likely what you see is what you get but if you want to make changes feel free to
fork the code and edit it to your own needs all i ask is that you do not use the code for commercial use.

