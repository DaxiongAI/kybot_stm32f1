Kybot交流群：488295654   
目录结构：  
bin：自动化脚本，makefile  
cmake：Linux下cmake编译工程的文件  
inc：头文件  
keil：keil工程文件，在Windows下用keil编译的同学可以直接打开里面的工程文件即可  
lib：stm32f1的library文件  
matlab：MATLAB源码以及一些测试数据和结果  
out：Linux下编译的输出目录  
src：源码文件  
udev：stlink的Linux下udev文件  

我们在STM32F103C8T6上实现了   
1.AT命令，用户可以很方便的调整kybot的参数以及查看kybot的运行状态，具体参看用户手册   
2.类Linux终端，终端可tab键补全AT命令，历史记录功能   
3.可设置打印级别的底盘log输出，log有颜色区分，级别有debug, info, warn, error, fatal，替代printf打印   
4.两路蓝牙，一路用于和安卓APP通信，可在APP上控制kybot;一路用于和ROS通信，一些简单的应用无需两台电脑，用蓝牙即可通过ROS控制kybot  
5.陀螺仪数据上报ROS频率100HZ，其他信息上报频率50HZ  
6.由于两路串口的通信数据量比较大，为节省MCU资源，最大限度的发挥stm32的性能，两路串口通信全部采用DMA配置  
7.I2C通信采用硬件I2C  

底盘代码未采用其他开源的操作系统，我们自己实现了一个简单的前后台系统  
开源的代码中包括求艾伦方差的函数，算法在MATLAB中，stm32仅用于算法的数据输出， 

stm32代码中包含陀螺仪的卡尔曼滤波算法，目前卡尔曼滤波算法仅算法3(KALMAN_3宏打开)测试的效果可以，  
其他的方法测试不理想，不知道哪里写错了，看到的大神请指点下  

Linux下编译环境已经搭建好，目前暂时还不能在Linux下编译   
