[33mcommit 552757f1c514682fd22da730421945c28d8e27c8[m[33m ([m[1;36mHEAD[m[33m -> [m[1;32mmain[m[33m, [m[1;31morigin/main[m[33m, [m[1;31morigin/HEAD[m[33m)[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Fri May 1 19:26:40 2026 -0500

    Add UART error callback to abort uart6 transaction when ESP and STM are desynched

[33mcommit e657860353b5161b2a6dbf096b921c8969aecd10[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Fri May 1 19:26:00 2026 -0500

    Init PD11 on CM7 core

[33mcommit b4a40c70744c81edac4bb403f8df1a1905c6d31e[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Wed Apr 29 22:21:57 2026 -0500

    Add GPIO PD11 as a motor relay and add motor override mode

[33mcommit 580e17126e4eff04cc116a244ef8839b487bc3e1[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Sun Mar 29 19:23:34 2026 -0500

    Add control logic for 4 motor speeds based on UI and sonar data

[33mcommit a238c3861190399f3dc94b3359cff9e2fc53aa1e[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Sun Mar 29 19:19:49 2026 -0500

    Increase motors to 4 and remove ADC that controlled PWM. Set up queues to send information to the motor control task.

[33mcommit 3cbaf82dc94fe5dedf839de946564ee36ea17965[m
Merge: ef95159 f45e694
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Fri Mar 13 16:58:33 2026 -0500

    Merge branch 'dev_jack'

[33mcommit ef95159174fc3be025f667f7d75161c223ba4f7b[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Fri Mar 13 16:53:26 2026 -0500

    Merge dev_jack manually :P

[33mcommit f45e694ffdf12acf03b4275c96cc595e544b70b1[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Fri Mar 13 15:41:12 2026 -0500

    Pre IOC Merge. Build successful

[33mcommit 103952fd3225d015843928952afaa565faf05870[m
Merge: 8cf4778 9a1a16e
Author: Chase G <gattusoc@msoe.edu>
Date:   Fri Mar 13 15:27:09 2026 -0500

    Merge pull request #1 from dev_chase
    
    Add tasks for sonar, UI, and motor control

[33mcommit 9a1a16ebfe1977eb6e0fb86ada0b3b73a6386368[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Fri Mar 13 15:21:07 2026 -0500

    Add GPS task and remove USB_OTG

[33mcommit f1e098978abd9de402a3aed6869a492c4956c9d1[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Sun Mar 1 16:18:24 2026 -0600

    Read UI data on USART6. Change User USB to print UI data.

[33mcommit 1bcde6271ef2bc8bf32e1bdfe68ffdd236d46bbe[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Fri Feb 27 22:23:51 2026 -0600

    Set up motor controller task for one motor

[33mcommit 824e5076ae42b97eebbe2e5ed90bcfd7858115e3[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Wed Feb 25 23:19:48 2026 -0600

    Transition to Jack's IOC file with inclusion of User USB setup. Move code out of main and into respective peripheral/subsystem files. Use User USB to print results of sonar distance.

[33mcommit 5570598af6083131943a3d53ae81d099e27068b6[m[33m ([m[1;31morigin/gps_demo[m[33m)[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Sun Feb 15 15:22:53 2026 -0600

    Completed GPS Test Plan Software

[33mcommit 4f6791a0c56d6ea69d0b425e49f39a63dbf7dda9[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Sat Feb 14 15:53:16 2026 -0600

    Added in ATT solution command and support for usb printing. Test demo code is implemented, but untested

[33mcommit e2367ef2d4b106bba5f26889a1dda7f2764cd224[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Wed Feb 11 02:34:40 2026 -0600

    Moved usb code to usb.c, usb.h

[33mcommit 0f625c6725899d5243618e7921079efcc18d1fc6[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Wed Feb 11 02:18:56 2026 -0600

    Abstracted message functions

[33mcommit b48fa77be32f522247669eef8759441d70fa8b76[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Tue Feb 10 22:58:08 2026 -0600

    Added USB-OTG Serial Debugging in preperation for GPS Test plan.

[33mcommit 21c54791dfc3dba2cfe4f9caa6f95013c496caca[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Tue Feb 10 14:58:13 2026 -0600

    Added main.html - User Interface webpage template

[33mcommit abe222f8391b2d8045afcd3cf2c47dfb4a0a466d[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Sat Feb 7 01:18:58 2026 -0600

    Added support for the HNR-NAV solution. All decoded data is now easily accessbile within GPS_Data struct, with the parsed data being seperated out. Integration to main system is ready once that is started.

[33mcommit 340bd407af7498a3f07ae4c5e65a1a8809ec20a7[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Wed Feb 4 15:42:56 2026 -0600

    Working UART RX/TX w/ DMA

[33mcommit 6a7acb3c0fa04ff51e3205a8a3fe9fd4a003a778[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Wed Feb 4 00:55:42 2026 -0600

    Fixed baud rate issues. Misconfigured HSE crystal assuming X3 comes provided. Switching to 8 MHz fixes all clocking issues that were being experinced before.

[33mcommit 6c0e70ef69bc68781a3999e49e607d5aac4d6533[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Tue Feb 3 21:54:33 2026 -0600

    Obtain distance from UART sonar channels

[33mcommit 9e6a7182b79057987d0e5495f5bdeee0ac88d062[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Fri Jan 30 23:06:24 2026 -0600

    Current progress: neither works again. No idea why....

[33mcommit 8e500646cab85c91640332d8a0a49f4fa724265c[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Fri Jan 30 18:40:04 2026 -0600

    Cleaned up main. Added in uart4, DMA1, and removed ETH temporarily to reduce bloat.

[33mcommit 958b20ae8327f2759593fff8251aaac19881a3fc[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Fri Jan 30 17:55:34 2026 -0600

    Port in-progress. Working main

[33mcommit 089a97fee962320413b78d54e3332ba952e19c08[m
Merge: fc04052 c21b2d5
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Fri Jan 30 17:03:28 2026 -0600

    Merge branch 'gps_demo'

[33mcommit fc04052f8ff5773027109f51c64feca34d93f398[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Fri Jan 30 16:30:01 2026 -0600

    Test run with dual core

[33mcommit c21b2d57042bc619e37bd13357331208c68e0247[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Fri Jan 30 15:40:10 2026 -0600

    IOC change

[33mcommit 8cf47789e146048e3d79b2781046a796f3d3d3cf[m
Author: jackrbauer <151313714+jackrbauer@users.noreply.github.com>
Date:   Thu Jan 22 02:25:57 2026 -0600

    Update .gitignore
    
    Includes more ignore definitions to avoid conflicts between developers

[33mcommit 281fbe64bc46aa084eae26125dda36a76d1ba7c4[m
Author: jackrbauer <151313714+jackrbauer@users.noreply.github.com>
Date:   Thu Jan 22 02:22:04 2026 -0600

    Create README.md
    
    Added README on Main

[33mcommit 73f6c8d5bb61146816fafe7adc5e31c2854f6a48[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Thu Jan 22 01:16:29 2026 -0600

    Removed unnecessary code, added template header

[33mcommit 4c8dbe288551071e21bdde5d156c7e32ce64c1f5[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Thu Jan 22 00:59:22 2026 -0600

    Working GPS Demo

[33mcommit b0e12e575118326d8f148b3e6f8ba18100e4ed78[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Wed Jan 21 18:53:11 2026 -0600

    PVT data command and parsing into the GPS_Data struct is now functional

[33mcommit 07576caef573be48517ad18c42bd9cc941e3153b[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Wed Jan 21 03:39:37 2026 -0600

    Working id decoder! We now can parse the data and easily retrieve the devices UID.

[33mcommit 2218f045ba722b180477238933cfbe5b1824ee94[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Tue Jan 20 20:27:48 2026 -0600

    Working UART-UBX comminucation pipeline. DMA in RX works, but not on TX which I anticipate will be required for the longer messages!

[33mcommit bd0907fe478dc71a6ac96155fa73540480b7b60e[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Mon Jan 19 16:40:16 2026 -0600

    Set up FreeRTOS with heartbeat timer and sonar task. Set up M4 core to toggle a gpio for now.

[33mcommit 35b3a3044ed22a51428edd6728414f96b925626e[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Sun Jan 18 21:30:59 2026 -0600

    Updated Readme

[33mcommit b5ff0ac3cf1ec3fc98191c4e4200a640268bf2f7[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Sun Jan 18 21:12:14 2026 -0600

    Able to retrieve data consisently using  DMA Channel 0, but it doesn't seem to be awfully consistent yet

[33mcommit 41dab16ddfc99decd07555cbdad1b3520346e5b9[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Sun Jan 18 16:47:47 2026 -0600

    Update STM32 project to latest version IDE v2.0 and added the UART HAL Driver

[33mcommit bd39631a1ec0d70939883f43d57d6f5ad4d9add1[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Sun Jan 18 15:22:39 2026 -0600

    Working & Tested UBX frame initializing functions.

[33mcommit 5228a489d2d0849c6d3b3522dc6f30bf69010c3a[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Sun Jan 18 04:52:29 2026 -0600

    checksum computation and ubx initial structuring

[33mcommit f4acb8aead4017716f808585f65b030771434ff4[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Sun Jan 18 00:47:06 2026 -0600

    main.* cleanup

[33mcommit f772758a4b3d0374308792b01969b0d642956224[m
Author: jackrbauer <jackrbauer02@gmail.com>
Date:   Sun Jan 18 00:17:41 2026 -0600

    Changed personal settings to work with local workspace (not pushed). Updated git ignore to exlude more directories

[33mcommit 2c063c32d9c33e8a56dfc632dd1f3ea78a5a1524[m
Author: Chase Gattuso <gattusoc@msoe.edu>
Date:   Sat Jan 17 17:02:58 2026 -0600

    Switch to using STM32H755 with FreeRTOS

[33mcommit e668c3d308cddb2b5b5f5ba1c3cd1f19734341fc[m
Author: Chase G <chasemanflash@gmail.com>
Date:   Tue Oct 21 00:08:05 2025 -0500

    Create README.md

[33mcommit 4cc17070df2eff1cf0db7b0c7dc78a3390ca7f75[m
Author: Flash42424 <chasemanflash@gmail.com>
Date:   Mon Oct 20 23:44:44 2025 -0500

    Inital commit STM32 project
