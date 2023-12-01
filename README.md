- 工程环境：Arduion IDE 2.2.1
- esp模组：esp32-C6-MINI-1U 
- 电机驱动：[TMC6300](https://docs.sparkfun.com/SparkFun_IoT_Brushless_Motor_Driver/assets/component_documentation/TMC6300.pdf)
- 磁性角度传感器：[MT6701](https://www.magntek.com.cn/upload/MT6701_Rev.1.0.pdf)

---
准备工作：拉取最新 Arduion 的esp32 开发包，[地址](https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/installing.html)

1. 点击上面的地址连接，复制下面的 Development release link：
~~~
https://espressif.github.io/arduino-esp32/package_esp32_dev_index.json
~~~
2. 打开 `Arduion IDE` -> `File` -> `Preferences`
3. 将复制的连接粘贴到 "Addition boards manager URLs" 中
4. 点击 OK， 等待下载完成

---
完成准备工作后开始一个例程：  

1. 使用 Arduion IDE 打开一个例程
2. 点击选择 `tool` -> `Board` -> `esp32` -> `ESP32C6 Dev Module` 开发板
3. 在 `tool` -> `Port` 选择串口后，就可以编译下载例程了
4. Note：使能 `tool` -> `USB CDC On Boot` 开启查看 Serial.print() 输出的功能
