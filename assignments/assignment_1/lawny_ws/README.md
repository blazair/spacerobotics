#Boustrephhedon pattern
>Assignment 1


The tutorial code in this repository was used as a reference before it was modified to have the dyynamic rqt parameter changing capabilites [Tutorial code](https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI/blob/main/assignments/first_order_boustrophedon_navigator/first_order_boustrophedon_navigator/boustrophedon_controller.py)


## Notes on the parameter

Kp_angular was always to be the first parameter that was to be chosen to get any semblance of the desired pattern, but the tutorial code had a Kp_linear value of 15.0 which made the turtlel to sway more often than not making it unreliable to tune the parameters properly. Hence a small detour to get a consistent value for Kp_linear which turned out to be <ins> 9.65 </ins>

One parameter was tweaked while the others were kept constant with the following order
The background color is `#fffffg` for light mode and `#000000` for dark mode.
