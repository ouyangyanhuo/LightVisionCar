# Light Vision Car
A product born from the marriage of cutting-edge technological achievements like electronic computing and modern automotive engineering. This intelligent electric vehicle employs a computer vision system—though it currently lacks road recognition capabilities (a feature once implemented)—operating instead as a tracking system. It actively seeks and follows the infrared heat signature of vehicles ahead, rapidly accelerating to maximum speed to close the distance before maintaining a synchronized, relative standstill with the lead vehicle.

# ​​About My Car

​​- Front-wheel steering​​
- ​​Automotive-grade MCU​​(NOT STM32)
- ​​Dual-motor powertrain​​
​​- Fully self-designed control circuit hardware​​

## Code Other

1. Use Seekfree Library

2. Continuous updates and maintenance will be provided.

# About Run

To replicate this system, you must first fabricate the hardware components as follows:

​- ​Rear Drive:​​ Dual brushed motors (operating voltage: ​​10–12V​​)
​​- Front Steering:​​ U400 servo motor (operating voltage: ​​6–7V​​)
​​- Tires:​​ High-speed operation requires ​​slick or semi-slick tires​​ for optimal traction.
​​- Tracking Target:​​ The lead vehicle must have ​​two evenly spaced, symmetrical infrared heat sources​​ for reliable detection.

The recommended toolchain for software compilation is Aurix Development Studio.

# LICENSE
It is licensed under the [GPL-3.0](https://github.com/ouyangyanhuo/LightVisionCar/blob/main/LICENSE) .