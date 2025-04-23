
# Supported platform

The following platforms are supported by now (2025.04.23).

* Crazyflie 2.1 : STM32F405
* Crazyflie 2.1 brushless : STM32F405
* Athena-efficiency : STM32L496
* AdHoc UWB STM32H7 Module : STM32H743

The connection between the hardware platform and the configuration.

|   | Support platforms | Defined config variable name       | File name | 
|---|-------------------|------------------------------------|-----------| 
| 1 | Crazyflie 2.1     | CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE | platform_crazyflie | 
| 2 | Crazyflie 2.1 bl  | CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE | platform_crazyflie | 
| 3 | Athena-efficiency | CONFIG_ADHOCUWB_PLATFORM_ATHENAEFF | platform_athenaeff |
| 4 | AdHoc UWB H7M     | CONFIG_ADHOCUWB_PLATFORM_ADHOCUH7M | platform_adhocuh7m |
| 5 | Ubuntu            | CONFIG_ADHOCUWB_PLATFORM_UBUNTUMAC | platform_ubuntumac |
| 6 | MacOS             | CONFIG_ADHOCUWB_PLATFORM_UBUNTUMAC | platform_ubuntumac |


The following defination must be given in somewhere outside the AdHocUWB repo before compile.

```c
//one the following must be defined
#define CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE y
#define CONFIG_ADHOCUWB_PLATFORM_ATHENAEFF y
#define CONFIG_ADHOCUWB_PLATFORM_ADHOCUH7M y
#define CONFIG_ADHOCUWB_PLATFORM_UBUNTUMAC y
```

```python
def hello():
    print("Hello, world!")
```