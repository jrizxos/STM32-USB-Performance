# WORK IN PROGRESS

# STM32-USB-Performance
This is a project to measure the bandwidth of the USB2.0 periferal in STM32 microprocessors. We used the Continuous Transmit program to measure the bandwidth of Bulk USB Transfers for different message lenghts and different workloads on a Nucleo F767zi. We also use 2 gpio pins and an oscilloscope to measure the time left to the CPU for more work as the USB periferal is sending a message. Using the measurements we developed Continuous Transmit Optimized which is the optimal way to continually send messages through USB.

## Using the Code
It is recommended that you use [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) to initialize the code for your microprocessor with the configuration inclued in the CubeMx project file, then replace the files in the Inc/ and Src/ folders that were generated.
