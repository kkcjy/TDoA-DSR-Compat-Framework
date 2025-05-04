
## How to install AdHocUWB on Crazyflie 2.1 w/o brushless


### Clone Crazyflie firmware and create branch

Clone and checkout the newest release. 
```bash
git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git crazyflie-firmware-adhocuwb
git tag -l
git checkout 2025.02
```

Make sure it can be Make and compile.
```bash
make defconfig
make -j
```

Create branch from the current commit.
```bash
git branch adhocuwb
git checkout adhocuwb
```

### Add Submodule libdw3000

Add libdw3000 as submodule.
```bash
git submodule add https://github.com/SEU-NetSI/libdw3000.git vendor/libdw3000
```

Double check the setting in `.gitmodules`.
```config
[submodule "vendor/libdw3000"]
        path = vendor/libdw3000
        url = https://github.com/SEU-NetSI/libdw3000.git
```

In `Makefile`, add 
```makefile
INCLUDES += -I$(srctree)/vendor/libdw3000/include
```

In `vendor/Kbuild`, add
```config
# libdw3000
obj-y += libdw3000/src/libdw3000.o
obj-y += libdw3000/src/libdw3000Spi.o
obj-y += libdw3000/src/mac_802_15_4.o
```

### Add Submodule AdHocUWB

Add AdHocUWB as submodule.
```bash
git submodule add https://github.com/SEU-NetSI/AdHocUWB.git src/deck/drivers/AdHocUWB
```

Double check the setting in `.gitmodules`.
```
[submodule "src/deck/drivers/AdHocUWB"]
        path = src/deck/drivers/AdHocUWB
        url = https://github.com/SEU-NetSI/AdHocUWB.git
```

In `Makefile`, add 
```
INCLUDES += -I$(srctree)/src/deck/drivers/AdHocUWB/Inc -I$(srctree)/src/deck/drivers/AdHocUWB/platform/Inc
```

Modify `src/deck/Kbuild`, see the next part.


### Add and modify Kconfig and Kbuild files

In `src/deck/Kbuild`, add
```
obj-y += drivers/AdHocUWB/
```

Add new file `src/deck/Kconfig`, an examply content is below.
```config
config DECK_ADHOC
    bool "Support the Adhoc deck"
    default y
    help
        The Adhoc deck is used for ranging, networking and communication within the
        swarm, based on the DW3000 UWB chip.

    config DECK_ADHOCDECK_USE_ALT_PINS
    bool "adhoc deck alternative IRQ and RESET pins"
    default name
    depends on DECK_ADHOC
    help
        ADHOC deck alternative IRQ and RESET pins(IO_2, IO_4) instead of
        default (RX1, TX1), leaving UART1 free for use.

    config DECK_ADHOCDECK_USE_UART2_PINS
    bool "adhoc deck use UART2 (TX2, RX2) pins"
    default name
    depends on DECK_ADHOC
    help
       ADHOC deck alternative IRQ and RESET pins(TX2, RX2) instead of
       default (RX1, TX1).
```

In `Kconfig`, change from 
```config
menu "Expansion deck configuration"

config DECK_FORCE
    string "Force load specified custom deck driver"
    default "none"
    help
        A colon seperated list of custom drivers to force load or "none".

source src/deck/drivers/src/Kconfig

endmenu
```
to 
```config
menu "Expansion deck configuration"

config DECK_FORCE
    string "Force load specified custom deck driver"
    default "none"
    help
        A colon seperated list of custom drivers to force load or "none".

source src/deck/drivers/src/Kconfig
source src/deck/Kconfig

endmenu
```

In folder `configs`, add new config files: 
* `adhoc_defconfig` 
* `adhoc_alt_defconfig`
* `adhoc_uart2_defconfig`

An examply content for `adhoc_uart2_defconfig` file is as follows.
```config
CONFIG_DECK_ACTIVE_MARKER=n
CONFIG_DECK_AI=n
CONFIG_DECK_LEDRING=n
CONFIG_DECK_BUZZ=n
CONFIG_DECK_LIGHTHOUSE=n
CONFIG_DECK_LOCO=n
CONFIG_DECK_MULTIRANGER=n
CONFIG_DECK_USD=n
CONFIG_DECK_RPM=n
CONFIG_DECK_ADHOC=y
CONFIG_DECK_ADHOCDECK_USE_UART2_PINS=y
```

### Other important settings.

In `Makefile`, change from 
```
LDFLAGS += --specs=nosys.specs --specs=nano.specs $(PROCESSOR) -nostdlib
```
to 
```
LDFLAGS += --specs=nosys.specs --specs=nano.specs $(PROCESSOR)
```

In `Makefile`, change from 
```
ARCH_CFLAGS += -Os -Werror
```
to 
```
ARCH_CFLAGS += -Os
```

