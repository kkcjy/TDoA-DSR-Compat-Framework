
## How to install AdHocUWB on Crazyflie 2.1 w/ or w/o brushless

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
obj-$(CONFIG_DECK_ADHOC) += drivers/AdHocUWB/
```

Add new file `src/deck/Kconfig`, an examply content is below.
```config
config DECK_ADHOC
    bool "Support the Adhoc deck"
    default n
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
    
    config UWB_LOCALIZATION_ENABLE
    bool "Enable UWB-based localization"
    default y
    depends on DECK_ADHOC
    help
        Enables UWB-based localization functionality.
        This feature depends on the Adhoc deck (DW3000 UWB chip) being enabled.

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

### Addtion to support Crazyflie 2.1 Brushless

In folder `configs`, add new config files: 
* `adhoc_c21b_defconfig` 
* `adhoc_c21b_alt_defconfig`
* `adhoc_c21b_uart2_defconfig`

An examply content for `adhoc_c21b_alt_defconfig` file is as follows.
```config
CONFIG_PLATFORM_CF21BL=y

CONFIG_MOTORS_REQUIRE_ARMING=y
CONFIG_MOTORS_ESC_PROTOCOL_DSHOT=y

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
CONFIG_DECK_ADHOCDECK_USE_ALT_PINS=y
```

In file `src/platform/src/platform_stm32f4.c`, change from 
```c
#define DEFAULT_PLATFORM_STRING "0;CF20"
```
to
```c
#define DEFAULT_PLATFORM_STRING "0;C21B"
```

### Add Relative Localization Function

In file `src/deck/Kconfig`, and a new config
```config
config UWB_LOCALIZATION_ENABLE
    bool "Enable UWB-based localization"
    default y
    depends on DECK_ADHOC
    help
        Enables UWB-based localization functionality.
        This feature depends on the Adhoc deck (DW3000 UWB chip) being enabled.
```

In file 'src/modules/src/estimator/estimator_kalman.c', add some code:

Add some definition:
```config
#ifdef CONFIG_UWB_LOCALIZATION_ENABLE
static float swarmVelocityXInWorld;
static float swarmVelocityYInWorld;
static float swarmGyroZ;
static float swarmPositionZ;
#endif
```

Add a function :
```
#ifdef CONFIG_UWB_LOCALIZATION_ENABLE
void estimatorKalmanGetSwarmInfo(short *vx, short *vy, float *gyroZ, uint16_t *height)
{
  *vx = (short)(swarmVelocityXInWorld * 100);
  *vy = (short)(swarmVelocityYInWorld * 100);
  *gyroZ = swarmGyroZ;
  *height = (uint16_t)(swarmPositionZ * 100);
}
#endif
```

In kalmanTask() find kalmanCoreFinalize(&coreData), add these code:
```
  if (kalmanCoreFinalize(&coreData))
    {
      #ifdef CONFIG_UWB_LOCALIZATION_ENABLE
      swarmVelocityXInWorld = coreData.R[0][0] * coreData.S[KC_STATE_PX] + coreData.R[0][1] * coreData.S[KC_STATE_PY] + coreData.R[0][2] * coreData.S[KC_STATE_PZ];
      swarmVelocityYInWorld = coreData.R[1][0] * coreData.S[KC_STATE_PX] + coreData.R[1][1] * coreData.S[KC_STATE_PY] + coreData.R[1][2] * coreData.S[KC_STATE_PZ];
      swarmGyroZ = gyroLatest.z * DEG_TO_RAD;
      swarmPositionZ = coreData.S[KC_STATE_Z];
      #endif
      STATS_CNT_RATE_EVENT(&finalizeCounter);
    }
```

In file 'src/modules/interface/estimator/estimator_kalman.h', add function declaration：
```
#ifdef CONFIG_UWB_LOCALIZATION_ENABLE
void estimatorKalmanGetSwarmInfo(short *vx, short *vy, float *gyroZ, uint16_t *height);
#endif
```
