# Wax Printer for Fabrication of Paper-based Microfluidic Devices

## Video demonstration:
[![Project demonstration](https://img.youtube.com/vi/NEMOkZihh2M/maxresdefault.jpg)](https://www.youtube.com/watch?v=NEMOkZihh2M)

## The Problem Statement:

>This project will entail the design, building and testing of a solid ink printer for wax printing of microfluidic paper based devices. The printer must be able to process and print a CAD design of a microfluidic channel using solid wax. The system should also allow for melting of the wax through the paper once the design is printed. This project involves the design and construction of the hardware required to implement a printer system, using solid wax as the ink, along with a controlled heating system for melting the wax. The design and implementation of accurate positioning and dispensing mechanisms will need to be applied for accurate and repeatable printing to be accomplished.

>The student will be expected to deliver a complete solid wax printing system to accurately and repeatably fabricate paper-based microfluidic devices. The student will design and build electronic circuitry to achieve this and will develop software to process the CAD design and control the printing and heating functionality.   

## About this repository 

In this repository you'll find my code for the STM32F103C8T6 microcontroller. Code functionality includes:
- Stepper Motor Control
- Serial Communications with the Raspberry Pi
- Closed loop heating systems
- IO control such as limit switches
- Bresenhams line and circle algorithms for interpreting G1 and G2 moves

The User-Interface code written in Python and running on the raspberry Pi can be found here:
Wax Printer UI [GitHub Repository](https://github.com/Blargian/EPR400-UI).

## Disclaimer: 

I've learnt quite a lot more about embedded systems and good coding practices so I am quite certain I commit a lot of cardinal embedded sins in this codebase. Don't judge too harshly! :) 
