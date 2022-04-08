# Arduino + BluePill

## PlatformIO Project Configuration

- Platform: BlackPill F103C8 (128k)
- Framework: Arduino
- Upload Method: serial

## Uploading

The best way to upload the sketch is to use ST-Link or J-Link debuggers. But due to the additional hardware costs you may want to use a cheaper alternative method.

I have tried to upload the program using the DFU, HID, and Maple bootloaders but I couldn't get them to work.

[STM32 Maple DFU Bootloader 2.0](https://github.com/rogerclarkmelbourne/STM32duino-bootloader/blob/master/binaries/generic_boot20_pc13.bin)

[STM32 HID Bootloader](https://github.com/Serasidis/STM32_HID_Bootloader/releases)

You can find some Arduino-STM32 upload methods in this page:

[Program STM32 Blue Pill (STM32F103C8T6) with Arduino IDE](https://www.sgbotic.com/index.php?dispatch=pages.view&page_id=48)

STM32F103C8T6 has an embedded bootloader in its ROM that can be used to upload the sketch. This is the best alternative method I've found.

This method is implemented in PlatformIO under the name `upload_protocol` of `serial`.

All the hardware you need is a connection between UART1 of the board (PA9-TX1 and PA10-RX1) and a USB to serial dongle connected to your computer.

![](assets/bootloader.jpg)

> Before uploading the sketch, make sure that BOOT0 is connected to VCC and RESET button is pressed. This will start up the microcontroller in the bootloader mode.
 
> Each time you change the configuration of BOOT0, you should RESET the board.

![](assets/boot0.jpg)

### Boot Modes

A couple of special MCU pins has to be set-up to proper logical values to enter the bootloader. The pins are named BOOT0 and BOOT1 on the STM32 microcontroller. Boot pins can select several modes of bootloader operation:

| BOOT1  | BOOT0  | Boot Mode         | Aliasing                                    |
| ------ | ------ | ----------------- | ------------------------------------------- |
| X      | 0      | Main Flash Memory | Main flash memory is selected as boot space |
| 0      | 1      | System Memory     | System memory is selected as boot space     |
| 1      | 1      | Embedded SRAM     | Embedded SRAM is selected as boot space     |

## Pinout

![](assets\bluepill-pinout.gif)

## References

- [BlackPill F103C8 (128k)](https://docs.platformio.org/en/latest/boards/ststm32/blackpill_f103c8_128.html)
- [Program STM32F4 with UART](http://stm32f4-discovery.net/2014/09/program-stm32f4-with-uart/)