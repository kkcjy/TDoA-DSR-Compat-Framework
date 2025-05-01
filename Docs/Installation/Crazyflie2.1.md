
# How to install AdHocUWB on Crazyflie 2.1 w/o brushless

## Add Submodule libdw3000

In `.gitmodules`, add
```
[submodule "vendor/libdw3000"]
       path = vendor/libdw3000
       url = https://github.com/SEU-NetSI/libdw3000.git
```

In `Makefile`, add 
```
INCLUDES += -I$(srctree)/vendor/libdw3000/include
```

In `vendor/Kbuild`, add
```
# libdw3000
obj-y += libdw3000/src/libdw3000.o
obj-y += libdw3000/src/libdw3000Spi.o
obj-y += libdw3000/src/mac_802_15_4.o
````

## Add Submodule AdHocUWB

In `.gitmodules`, add
```
[submodule "src/modules/AdHocUWB"]
       path = src/modules/AdHocUWB
       url = https://github.com/SEU-NetSI/AdHocUWB.git
```

In `Makefile`, add 
```
INCLUDES += -I$(srctree)/src/modules/AdHocUWB/Inc
INCLUDES += -I$(srctree)/src/modules/AdHocUWB/platform/Inc
```

In `src/modules/Kbuild`, add
```
obj-y += AdHocUWB/
````

## Other important settings.

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