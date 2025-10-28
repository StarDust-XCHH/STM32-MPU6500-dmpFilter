# STM32-MPU6500-dmpFilter
This is a very simple, ready-to-use project. It uses an STM32F4 development board to drive the MPU6500 sensor for IMU data acquisition, applies the official DMP library for filtering, and finally outputs roll, pitch, and yaw angles.

> **Author**: [StarDust 星辰涵], Beijing University of Posts and Telecommunications (BUPT)  
> **License**: MIT (see [LICENSE](LICENSE))

**Ideal for rapid prototyping and learning, this repository is tailored for developers and students who want to quickly integrate the MPU6500 gyroscope/accelerometer module into their projects. If you need more detailed guidance, please refer to the official library: https://github.com/libdriver/mpu6500.git**

---

Follow these steps exactly to ensure successful code generation and compilation:

1. Clone the project using Git
```bash
git clone https://github.com/StarDust-XCHH/MPU6500-DMP-Fusion.git
```


2. Install required tools
    - Install [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
    - Install [Keil MDK-ARM](https://www2.keil.com/mdk5/) (v5.38 or later recommended)


3. Generate the Keil project
    - Open MPU6500_DMP.ioc in STM32CubeMX
    - Click Project → Generate Code
    - Wait for generation to complete (proceed even if a project-generation warning appears)

4. Build the project
    - Open the generated MDK-ARM/MPU6500_DMP.uvprojx
    - Click Build — compilation should succeed with no errors






