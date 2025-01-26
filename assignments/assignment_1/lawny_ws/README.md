#Boustrephhedon pattern
>Assignment 1


The tutorial code in this repository was used as a reference before it was modified to have the dyynamic rqt parameter changing capabilites [Tutorial code](https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI/blob/main/assignments/first_order_boustrophedon_navigator/first_order_boustrophedon_navigator/boustrophedon_controller.py)


## Notes on the parameter

The tweaks and their respective cross track erros can be found in this [log file](https://1drv.ms/x/s!As21D3LIKaTsgdpu_zVlr5nVmnvYqQ?e=waO8v4) \
$${\color{red}DISCLAIMER! \space This \space is\space not\space a\space professionally\space written\space log\space file.\space It\space was\space just\space used\space as\space a\space reference\space to\space learn\space how\space the\space pattern\space turns\space out\space for\space every\space change\space to\space guess\space with\space a\space certain\space degree\space of\space confidence\space the\space values\space and\space their\space impact\space on\space the\space patten\space }$$

One parameter was tweaked while the others were kept constant with the following order undertook
>Kp_linear\
>Kd_linear\
>Kp_angular\
>Kd_angular\

K<sub>p,angular</sub> was always to be the first parameter that was to be chosen to get any semblance of the desired pattern, but the tutorial code had a Kp_linear value of 15.0 which made the turtle to sway more often than not making it unreliable to tune the parameters properly. Hence a small detour to get a consistent value for Kp_linear which turned out to be <ins> 9.65 </ins>

With the new value, other values were selcted based on the following criteria with no particular order of imporatance heirarchy

1. Average cross track error
2. The way the pattern looks (more subjective than objective)
3. The turning speed
4. The consistent performace of the above points for a set value of parameters

The final value for <ins>linear proportional gain which is 9.4</ins> was selected despite having slightly worse cross track error for producing a more consistent pattern with effectively no failures in 10 runs.\
This is the reason why higher angular proportial gains

