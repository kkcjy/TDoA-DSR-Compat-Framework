#ifndef __ADHOCUWB_PLATFORM_H__
#define __ADHOCUWB_PLATFORM_H__

#define CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE y

#if defined(CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE)
    #include "adhocuwb_platform_crazyflie.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_ATHENA)
    #include "adhocuwb_platform_athena3.2.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_ADHOCUWBH7)
    #include "adhocuwb_platform_adhocuwbH7.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_LINUX)
    #include "adhocuwb_platform_linux.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_MACOS)
    #include "adhocuwb_platform_macos.h"
#else
    //
#endif

#endif
