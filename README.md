# f0discovery-drivers

This repo contains bare metal drivers for stm32f0discovery.

001led_toggle.c file uses these drivers and toggles green led atteched to PC9 using PUSH PULL configuretion.

In the next commit, it uses OPEN DRAIN configuration. In theory according to my source, it should toggle with very low intensity.
My source fixes this issue by connecting the pin(PC9 in my case) to +5V via 470 Ohm resistor, which is much lower value.
It toggles with normal intensity in my case. So, I think it is related to values of resistors in f0discovery LD3. My source uses 
stm32f4discovery.



002led_button.c file toggles led when user button is pressed.