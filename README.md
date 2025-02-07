# embassy-npcx

[![no-std](https://github.com/OpenDevicePartnership/embassy-npcx/actions/workflows/nostd.yml/badge.svg)](https://github.com/OpenDevicePartnership/embassy-npcx/actions/workflows/nostd.yml)
[![check](https://github.com/OpenDevicePartnership/embassy-npcx/actions/workflows/check.yml/badge.svg)](https://github.com/OpenDevicePartnership/embassy-npcx/actions/workflows/check.yml)
[![LICENSE](https://img.shields.io/badge/License-MIT-blue)](./LICENSE)

## Introduction

This is the Embassy HAL for the Nuvoton NPCX family. For development, we will be
focused on supporting the NPCX490 family first using the 980521543-011 board with 
NPCX498F (Ramon EVB w/ 200-pin MECC) as the development platform. The plan is to 
eventually upstream this to Embassy official repository.

When using this HAL, to run code from the flash you need to use the `link_flash.x` linker script, instead of the usual cortex-m-rt `link.x` linker script. To do this, add `-C link-arg=-Tlink_flash.x` to your rust flags.

## Peripherals HALs

* Timer
* Clocks
* Interrupt
* GPIO
* I2C
* SPI
* eSPI
* Other...

## Plan

We will focus on Async HAL first and blocking HAL as optional.
