This is firmware for STM32F100RBT6B microcontroller on STM32VLDiscovery board.
This firware communicates with odroid-u3 (or other Linux-based sbc) via n_rf24l01 transceiver and controls engines on radiocar.

This is independent project. But it uses n_rf24l01 library to communicate with n_rf24l01 transceiver.

n_rf24l01 library is git submodule.

Related git projects:
 n_rf24l01 library - set of file to communicate with n_rf24l01 transceiver
 n_rf24l01-linux-driver - this is control panel side (kernel driver for odrouid-u3 and app uses this driver)

This project is child dream...