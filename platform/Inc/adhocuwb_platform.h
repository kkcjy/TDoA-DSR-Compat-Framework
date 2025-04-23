#ifndef __ADHOCUWB_PLATFORM_H__
#define __ADHOCUWB_PLATFORM_H__

#define CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE y

#if defined(CONFIG_ADHOCUWB_PLATFORM_CRAZYFLIE)
    #include "platform_crazyflie.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_ATHENA)
    #include "platform_athena3.2.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_ADHOCUWB)
    #include "platform_adhocuwb.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_LINUX)
    #include "platform_linux.h"
#elif defined(CONFIG_ADHOCUWB_PLATFORM_MACOS)
    #include "platform_macos.h"
#else
    //
#endif

#endif