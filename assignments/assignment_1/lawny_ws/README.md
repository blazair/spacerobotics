# Boustrephedon pattern
>Assignment 1


The tutorial code in this repository was used as a reference before it was modified to have the dynamic rqt parameter changing capabilites [Tutorial code](https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI/blob/main/assignments/first_order_boustrophedon_navigator/first_order_boustrophedon_navigator/boustrophedon_controller.py)


## General Notes

The tweaks and their respective cross track erros can be found in this [log file](https://1drv.ms/x/s!As21D3LIKaTsgdpu_zVlr5nVmnvYqQ?e=waO8v4) \
$${\color{red}DISCLAIMER! \space This \space is\space not\space a\space professionally\space written\space log\space file.\space It\space was\space just\space used\space as\space a\space reference\space to\space learn\space how\space the\space pattern\space turns\space out\space for\space every\space change\space to\space guess\space with\space a\space certain\space degree\space of\space confidence\space the\space values\space and\space their\space impact\space on\space the\space pattern\space }$$

One parameter was tweaked while the others were kept constant with the following order undertook
1. K<sub>p, linear</sub>
2. K<sub>d, linear</sub>
3. K<sub>p, angular</sub>
4. K<sub>d, angular</sub>

K<sub>p, angular</sub> was always the first parameter that was to be chosen to get any semblance of the desired pattern, but the tutorial code had a K<sub>p,linear</sub> value of <ins>15.0</ins> which made the turtle to sway more often than not making it unreliable to tune the parameters properly. Hence a small detour to get a consistent value for K<sub>p, linear</sub> which turned out to be <ins> 9.65 </ins>

With the new value, other values were selcted based on the following criteria with no specific heirarchy of importance

1. Average cross track error
2. The way the pattern looks (more subjective than objective)
3. The turning speed
4. The consistent performace of the above points for a set value of parameters

The final value for K<sub>p, linear</sub> = 9.4 was selected despite having slightly worse cross track error for producing a more consistent pattern with effectively no failures in 10 runs.

Lower K<sub>p, angular</sub> (around 6.0) can also be used if the shape of the path has more weight to the than the cross track error. So the below picture had an average cross track error of around 0.210 but produced much more rounded corners and consistent turning
![at 6 kpa](https://github.com/user-attachments/assets/64fb1fc6-06a1-478e-ac32-16890b9ad909)

Higher K<sub>p, angular</sub> despite having cross track errors of **<0.1** was not chosen becasue of the ridiculously high turning speeds and the jankier patterns.
This is a [run](https://github.com/user-attachments/assets/bffd77a7-c9b9-4a71-a372-27d7d9933e91) with angular values of 10.\ 

The video clearly shows the turtle turning at real high speeds (echoing the /turtle1/cmd_vel topic shows a speed of around 25). If these speeds are not bottlenecked by hardware, this value is the most optimal to get a near perfect traversal of the given waypoints. However damping needs to be increased and a low pass filter is recommended for gradual speed adjustment. Below is the final picture of the same run in the video

![at 10 kpa](https://github.com/user-attachments/assets/6f8820cd-bb8c-4280-9b3b-182dff99434b)


Despite this, to make higher K<sub>p, angular</sub> work two methods were tried

### A direct cap on the velocity
  Like the title suggests there was a hard cap made to the messages published to be less than a certain value. The outcome was lacluster just resembing a lower K<sub>p, angular</sub> value unsurprisingly

### Low pass filter
  This seemed to alleviate the turning speeds resembling a K<sub>p, angular</sub> = 8 while havin a value of 10 but the pattern looked jankier and worse than a K<sub>p, angular</sub> = 8 without a low pass filter, hence there was no reason to go ahead with it.

Custom messages were implemented and are showcased in a live rqt plot during the run. These are some of the messages that were included in the live plot
1. Cross track error
3. Current velocity
4. Distance to next waypoint
5. Completion percentage

The [controller.py]() has exhaustive comments on how these messages are calculated.

## Conclusion
These are the following conclusions made from undertaking this assignment 
1. Higher K<sub>p, angular</sub> gives the least cross track error **(<0.1)** and can be implemented blindly albeit with a low pass filter if there are no hardware constraints
2. The final set of parameters chosen to make the pattern look perfect while having a low cross track error is\
   a. K<sub>p, linear</sub> = 9.4\
   b. K<sub>d, linear</sub> = 0.2\
   c. K<sub>p, angular</sub> = 7.7\
   d. K<sub>d, angular</sub> = 0.05\
   
The [video](https://drive.google.com/file/d/1xKvzyddJLRFbOUQDtk_UdF7NOMceFI81/view?usp=sharing) demonstration is enclosed and a picture of the final path is given below.
![final final](https://github.com/user-attachments/assets/2f420562-22c0-46d4-9320-c54b194fd95d)




