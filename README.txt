This is a firmware for the STM32F100RBT6B microcontroller on the STM32VLDiscovery board.
This firware inverts data consumed from an n_rf24l01 tranceiver and send them back. 

This project is created from a template located on
https://github.com/libopencm3/libopencm3-examples.git README.txt.

This project uses the n_rf24l01 library to communicate with the n_rf24l01 transceiver
and the libopencm3 library to commnicate with peripherals.

n_rf24l01 library is a git submodule.
libopencm3 library is a git submodule.

Related git projects:
 n_rf24l01_library - a set of files to communicate with the n_rf24l01 transceiver
 n_rf24l01_linux_driver - a kernel driver, for odrouid-u3, to communicate with the n_rf24l01 transceiver
 n_rf24l01_app - Linux application which communicates with this firmware on the STM32VLDiscovery board 
 
This project is a child dream...
