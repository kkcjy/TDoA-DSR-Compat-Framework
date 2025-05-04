
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
[submodule "vendor/libdw3000"]
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
````

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

In `src/deck/Kbuild`, add
```
obj-y += drivers/AdHocUWB/
````

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

In folder `configs`, add new config files: 
* `adhoc_defconfig` 
* `adhoc_alt_defconfig`
* `adhoc_uart2_defconfig`
