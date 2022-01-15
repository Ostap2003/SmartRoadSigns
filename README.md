# SmartRoadSigns

## Aim of the project
In Ukraine there is a big problem of car accidents due to speeding on the roads. 35% of car accidents injuries happened due to speeding (2020 ststistics)
More than 50% of road deaths happened due to speeding in Ukraine in 2020.
One of problems can be that drivers don't notice appropriate signs and as a result don't follow restrictions and rules on roads.
What is one of possible solutions?
We made a sign that shows both the sign with speed limiting and the current velocity of the vehicle approaching the sign. 
The sign consists of radar and LED matrices that display information to the driver.

## Components
- [WS2812B](https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf) - LED matrix (6x)
- [HB100](https://www.limpkin.fr/public/HB100/HB100_Microwave_Sensor_Application_Note.pdf) - Microwawe Sensor, works on a Doppler effect.
- STM32F103C8 (Blue pill) - microcontroller

### IDE's
[STM32 Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html) (version 1.6.1)

## Download and use
Ensure to have all required components for the project

Download STM32 Cube IDE (version 1.6.1).<br>
Clone this repository<br>
In the IDE got to *File/New/STM32 Project from an Existing STM32CubeMX Configuration File (.ioc)*
- Then replace generated ```Core/src/main.c``` with cloned version of the file
- Add ```Core/Src/WS_matrix.c``` file to ```Core/src/```
- Add ```Core/Inc/WS_matrix.h``` file to ```Core/inc/```
- Add ```Core/Inc/additional_signs.h``` file to ```Core/inc/```
- Add ```Core/Inc/digits.h``` file to ```Core/inc/```
- Add ```Core/Inc/sign_part.h``` file to ```Core/inc/```

And now the project is set up.

## Usage
After succesful compilation of the project. Flash the microcontroller. Everything should work properly. On the upper four matrices the sign fill be displayed.
The bottom two matrices will show current speed of the object that is moving to the sign.
The speed limit can be set in a special function - ```setSpeedLimit(int limit)```, which is called before the main while loop.
If target object is moving towards sign with velocity greater than set speed limit, then current speed will be dispalyed diferently: 
- color of digits changes to red
- two exclamation points appear on the right and left side of speed display

If target object is moving with velocity <= speed limit, then:
- color of digits is green

Additionlly the on matrices different signs can be displayed. 
In the project root is located folder with python code that receives image, crops and pixelates it and writes to file RGB representation of the sign. 
In order to dispaly signs that are generated this way:
- comment out the call of ```WsSetSign(avgVel)``` function
- uncomment ```WsImgSet(sign)```
After recompile the project and flash the microcontroller.

## Project scheme
![Project scheme](https://github.com/Ostap2003/SmartRoadSigns/blob/main/img/scheme.jpg)

## How everything works
### How matrices are set
All matrices are connected between each other. Data is sent using DMA. Firstly we write to LED_Data array all data for each LED, so it has its id (from 0 to 384) and RGB values (each value from 0 to 255). Then from this data we pass value to matrices using pwm and pwm_data array where all values for RGB are represented as duty cycles.<br>
The size of ```pwmData = 24 * 2``` so we can send data to two leds, this was done to decrease memory usage. Firstly we are passing data to first LED, after first LED is set, we get interrupt from DMA that transfer is half done, in the iterrupt we rewrite first 24 values in ```pwmData``` for the next transfer, while data from \[24-48) array elemets is being passed to the next LED. After all data from ```pwmData``` is transfered we get another interrupt from DMA, so in this interrrupt we set values in ```pwmData``` from \[24-48) for the future transfer.<br>
*(Idea was taken [here](https://www.thevfdcollective.com/blog/stm32-and-sk6812-rgbw-led))*

### How speed is measured
FOUT of the HB100 sensor is connected to the pin A8, which is configured as a Timer 1 with enabled input capture interruts. They are triggered on every rising edge. Using these interrupts we measure the period length of the signal from which calculate the velocity. From the [datasheet](https://www.limpkin.fr/public/HB100/HB100_Microwave_Sensor_Application_Note.pdf) the movement detection frequency of HB100 is 10.525 Ghz, Fd with Velocity in km/h is 19.49, 10.525Ghz / 19.49 = 51308. To get the velocity we divide 51308 by period length in microseconds and get velocity in km/h . Velocities are stored in a buffer, which is an array that holds ```VEL_BUFFER_SIZE``` (currently 100) values. The buffer resets every ```analysisEvery``` miliseconds.

Every ```analysisEvery``` (currently 200) miliseconds, we calculate the *"truncated mean"* of the velocity buffer using ```findAvg()``` function. Truncated mean is a statistical measure, which is calculated by discarding high and low ends(under 25%, and over 75% in our case) of the sorted sample and then calculating the mean.

Although the buffer has a constant size ```VEL_BUFFER_SIZE``` (currently 100), truncated mean is calculated only on non-zero values(i.e. if we got only 25 velocities, then we will calculate truncated mean only of these 25 values discarding the rest 75 null slots of the array).

During the analysis time in the while loop of the main function, interrupts are temporarily disabled to not interfere with the rest of the program. After successfuly performing all of the needed calculations and sending the data to the matrix, the interrupts are resumed.

### How the image is converted to an array of rgb values
The usage of custom images to display on our matrices(it will be displayed on the top 4 8x8 WS2812B matrices) is implemented using Python and [Pillow](https://github.com/python-pillow/Pillow), [NumPy](https://numpy.org/) libraries.

Pillow provides the ability to chose amongst several compressiom algorithms. After testing them on several road signs and considering the fact that the text, images, and borders on the signs must be concrete and clear, with high visibility from afar, we chose to use Nearest-Neighbor algorithm. It generally provides enough contrast, minimal color fuzziness, and sufficiently readable text.
![AlgoComparison](https://github.com/Ostap2003/SmartRoadSigns/blob/main/img/compressionAlgoComparison.jpg)

After resizing, the image is split in half vertically because of the way our matrices are physically connected. The right half must be flipped both horizontally and vertically to be properly displayed on the matrices.

Using numpy we create a list of pixels from the PIL.Image object. Then, flatten it using itertools.chain to sort and work with it more comfortably in the future.

Also, the brightness of the image could decreased by the user-configurable factor(could be used to save the power, or to fit in better with the time of the day).

## Team
- [Ustym Hanyk](https://github.com/UstymHanyk)
- [Ostap Dutka](https://github.com/Ostap2003)
- Dmytro Ryzhenkov (mentor)
