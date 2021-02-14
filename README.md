# Embedded Systems Project
## Fractals and Langtons Ant

### Assignment Specifications

It was our final assigment during our embedded systems course to program an application which utilizes several input/output modules of the Texas Instruments TM4C1294NCPDT board in combination with the TI Educational Booster Pack MKII.

The basic conditions given by our lecturer were simple:
* make use of/implement several modules(buttons, LEDs, buzzer, ...)
* DONT use Energia (since there are already a bunch of demo applications available in Energia, this would be too easy; most of the Energia libraries are in C++, we would be skipping all the low level C)
* have fun with electronics!

### Hardware & IDE
* TI TM4C1294NCPDT 
* TI Educational Booster Pack MKII
* TI Code Composer Studio

### Progress
I was not able to find a library that worked for the combination of these 2 boards.
After further research I managed to find this repository on github(which I couldnt get working on these boards)

https://github.com/LePoloni/Texas-LaunchPad-and-BoosterPack-MK-II

During the process I also tried some Energia Demo programs and rewrote some of the C++ Energia code into C.
(All the Energia Demo programs worked flawless but most of the code was in C++)

https://energia.nu/guide/tutorials/boosterpacks/tutorial_edumkii/

Additionally I used some example code from the TI documentation.

By learning from these sources and adapting some of the code into my project I was able to utilize the screen.
Nonetheless it took me several hours of troubleshooting(wrong clock speed, code not matching up with the pins being used, ... ) and many failed attempts. It was quite tricky to translate some of the Energia C++ functions into C.

I would like to thank the authors of the beforementioned sources, since their work helped me learn and finish my project.
Since I wanted to get a working projects as soon as possible the code is not very optimized and ended up in a single main.c. 

### User Manual
* after starting the program you should be seeing a mandelbulb fractal
* with the joystick you can navigate into each direction, zoom in and out with the 2 buttons
* by pressing both buttons you can switch into "langtons ant mode"
