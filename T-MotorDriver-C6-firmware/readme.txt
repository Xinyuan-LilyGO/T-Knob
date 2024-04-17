firware.bin
默认先下载这个固件，下载完成后重新上电，可以打开串口，看下面模式电机现象是否对应

mode=0, Velocity Angle Ctrl    ；电机不会转，用手拧一下电机，会回弹
mode=1, Angle Ctrl                 ；电机不会转，用手拧一下电机，会回弹
mode=2, Velocity Ctrl              ；电机会低速转动
mode=3, Velocity Torque Ctrl  ；电机会低速转动，并且碰就停
mode=4, Torque Ctrl               ；电机不会转，用手拧一下电机，会有档位咔咔咔的手感
mode=5, Led & Buzzer ON      ；就灯和蜂鸣器响，电机没有任何效果

