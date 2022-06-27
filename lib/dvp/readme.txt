1.install: connect the usb camera-->
			enter the command 'sudo ./install.sh'-->
			reboot the linux OS
			
2.sdk introduction:
-------------------------------
-- Demo                         simple demo code, no UI
-- Demo_arm64                   simple demo executalbe program
-- basedcam2                    qt demo
-- libdvp.so             	sdk library
-- libhzd.so           		algorithm library
-- usb2_m_all.dscam.so          usb2 device driver library
-- usb3_m_all.dscam.so          u3s device driver library
-- usb3_v_all.dscam.so          u3v device driver library
-- usb3_m3s_all.dscam.so        M3S device driver library
-- BasicFuncation.tar.gz        qt demo code

NOTE:
libdvp.so libhzd.so usbxxx.dscam.so  must be placed in the same directory


1.安装：接上usb相机后输入命令 sudo ./install.sh,  重启机器后可以正常使用相机
2.sdk目录介绍：
------------------------------
-- Demo                        目录：简单的出图示例程序源码、无界面
-- Demo_arm64                  可执行程序demo
-- basedcam2                   qt可执行程序
-- libdvp.so                   sdk api库
-- libhzd.so                   图像库
-- usb2_m_all.dscam.so         usb2相机设备驱动库
-- usb3_m_all.dscam.so         u3s相机设备驱动库
-- usb3_v_all.dscam.so         u3v相机设备驱动库
-- usb3_m3s_all.dscam.so       M3S相机设备驱动库
-- BasicFuncation.tar.gz       qt示例源码

注意事项：
libdvp.so  libhzd.so usbxxx.dscam.so 库必须放在同一级目录 

