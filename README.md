# SmartRoadSigns

## Aim of the project
In Ukraine there is a big problem of car accidents due to speeding on the roeds. 35% of car accidents injuries happened due to speeding (2020 ststistics)
More than 50% of road deaths happened du to speeding in Ukraine in 2020.
One of problems can be that drivers don't notice appropriate signs and as a result don't follow restrictions and rules on roads.
What is one of possible solutions?
We made a sign that shows both the sign with speed limiting and the current velocity of the vehicle approaching the sign. 
The sign consists of radar and LED matrices that display information to the driver.

## Details
WS2812B - LED matrix (6x)
HB100 - Microwawe Sensor, works on a Doppler effect.
STM32F103C8 (Blue pill) - microcontroller

### IDE's
STM32 Cube IDE (version 1.6.1)

## Download and use
Ensure to have all required components for the project

Download STM32 Cube IDE (version 1.6.1).
Clone this repository, must be cloned to ```~/STM32CubeIDE/workspace_1.6.1/```.
Go to *File/Open Project from File System/* choose already existing cloned folder.

## Usage
After succesful compilation of the project. Flash the microcontroller. Everything should work properly. On the upper four matrices the sign fill be displayed.
The bottom two matrices will show current speed of the object that is moving to the sign.
The speed limit can be set in a special function - ```set_speed_limit(int limit)```, which is called before the main while loop.
If target object is moving towards sign with velocity greater than set speed limit, then current speed will be dispalyed diferently: 
- color of digits changes to red
- two exclamation points appear on the right and left side of speed display
If target object is moving with velocity <= to set speed limit, then:
- color of digits is green

Additionlly the on matrices different signs can be displayed. 
In the project root is located folder with python code that recieves image, crops and pixelates it and writes to file RGB representation of the sign. 
In order to dispaly signs that are generated this way:
- comment out the call of ```WS_set_sign(avgVel)``` function
- uncomment ```WS_img_set(sign)```
After recompile the project and flash the microcontroller.

## Project scheme

## How everything works
### How matrices are set
All matrices are connected between each other. Data is sent using DMA. Firstly we write to LED_Data array all data for each LED, so it has its id (from 0 to 384) and RGB values (each value from 0 to 255). Then from this data we pass value to matrices using pwm and pwm_data array where all values for RGB are represented as duty cycles.
the size of ```pwm_data = 24 * 2``` so we can send data to two leds, this was done to decrease memory usage. Firstly we are passing data to first LED, after first LED is set, we get interrupt from DMA that transfer is half done, in the iterrupt we rewrite first 24 values in pwm_data for the next transfer, while data from \[24-48) array elemets is being passed to the next LED. After all data from pwm_data is transfered we get another interrupt from DMA, so in this interrrupt we set values in pwm_data from \[24-48) for the future transfer.

### How speed is measured

## Demo

## Team
- [Ustym Hanyk](https://github.com/UstymHanyk)
- [Ostap Dutka](https://github.com/Ostap2003)
- Dmytro Ryzhenkov (mentor)
