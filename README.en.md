![Linar](imagenes/linar-dark.png#gh-dark-mode-only)
![Linar](imagenes/linar-light.png#gh-light-mode-only)
[![english](https://img.shields.io/badge/language-english-red)](https://github.com/udesa-ai/dyna1-quadruped/blob/readme/README.en.md)
[![español](https://img.shields.io/badge/idioma-español-blue.svg)](https://github.com/udesa-ai/dyna1-quadruped/blob/readme/README.md)

<h1 align="center">Dyna1</h3>
<h3 align="center">Quadruped platform for evaluation of autonomous locomotion algorithms</h3>

![Linar](imagenes/look.jpg#gh-dark-mode-only)
![Linar](imagenes/stand.jpg#gh-light-mode-only)

## Description
<p align="justify"> We present a new design of a 12-degree-of-freedom quadruped, focused on reducing manufacturing costs and complexity, and maximizing the use of regionally available components. To facilitate assembly, the legs are modular, and the motors are placed as close to the body as possible to reduce their inertia. Brushless motors are used along with Odrive controllers, allowing for simple control through their CAN interface. The body contains the shoulder motors and all the necessary electronics for its operation. Its structure consists of four bars and transversal acrylic plates. The ROS2 operating system is implemented to control the quadruped. All crucial processes for the robot's operation run on the onboard computer, with others on the external computer. It was experimentally demonstrated that the leg is capable of jumping up to 30 cm with a weight similar to a quarter of the body. A control period (12 ms) and measurement period (10 ms) are achieved with a standard deviation of 0.135 ms and 0.246 ms respectively, and the RMS current while walking is around 15 A in the most demanding motor, so it should not have overheating problems. A Pybullet simulator was created to efficiently evaluate autonomy algorithms. </p>

## Hardware

In the hardware folder, you can find:

- 3D models of the quadruped design in the following formats:
  - Fusion 360 (.f3d)
  - Step (.stp)
- Bill of Materials (BOM)

## Software

TODO
