
# Supported platform

The following platforms are supported by now (2025.04.23).

* Crazyflie 2.1 : STM32F405
* Crazyflie 2.1 brushless : STM32F405
* Athena-efficiency : STM32L496
* AdHoc UWB STM32H7 Module : STM32H743

The connection between the hardware platform and the configuration.

|   | Support platforms | Defined config variable name         | File name          | 
|---|-------------------|--------------------------------------|--------------------| 
| 1 | Crazyflie 2.1     | CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE   | platform_crazyflie | 
| 2 | Crazyflie 2.1 bl  | CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE   | platform_crazyflie | 
| 3 | Athena-efficiency | CONFIG_ADHOCUWB_PLATFORM_ATHENA      | platform_athena3.2 |
| 4 | AdHoc UWB         | CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7  | platform_adhocuwbH7|
| 5 | Linux             | CONFIG_ADHOCUWB_PLATFORM_LINUX       | platform_linux     |
| 6 | MacOS             | CONFIG_ADHOCUWB_PLATFORM_MACOS       | platform_macos     |


The following defination must be given in somewhere outside the AdHocUWB repo before compile.

```c
//one the following must be defined
#define CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE y
#define CONFIG_ADHOCUWB_PLATFORM_ATHENA y
#define CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7  y
#define CONFIG_ADHOCUWB_PLATFORM_LINUX y
#define CONFIG_ADHOCUWB_PLATFORM_MACOS y
```

For example, the following usage is provided for reference.

```c
#if defined(CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE)
    #include "platform_crazyflie.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_ATHENA)
    #include "platform_athena3.2.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7)
    #include "platform_adhocuwb.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_LINUX)
    #include "platform_linux.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_MACOS)
    #include "platform_macos.h"
#else
    //
#endif
```