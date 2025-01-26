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

K<sub>p, angular</sub> was always the first parameter that was to be chosen to get any semblance of the desired pattern, but the tutorial code had a K<sub>p,linear</sub> value of <ins>15.0</ins> which made the turtle to sway more often than not making it unreliable to tune the parameters properly. Hence a small detour to get a consistent value for Kp_linear which turned out to be <ins> 9.65 </ins>

With the new value, other values were selcted based on the following criteria with no particular order of imporatance heirarchy

1. Average cross track error
2. The way the pattern looks (more subjective than objective)
3. The turning speed
4. The consistent performace of the above points for a set value of parameters

The final value for <K<sub>p, linear</sub> = <ins>9.4</ins> was selected despite having slightly worse cross track error for producing a more consis
tent pattern with effectively no failures in 10 runs.\
This is the reason why higher <K<sub>p, angular</sub> despite having cross track errors of **<0.1** was not chosen becasue of the ridiculously high turning speeds and the jankier patterns. They were disregarded.

This is a [run](https://github.com/user-attachments/assets/bffd77a7-c9b9-4a71-a372-27d7d9933e91) with angular values of 10. The video clearly shows the turtle turning at real high speeds (echoing the /turtle1/cmd_vel topic shows a speed of around 25). If these speeds are not bottlenecked by hardware, this value is the most optimal to get a near perfect traversal of the given waypoints. However damping needs to be increased and a low pass filter is recommended for gradual speed adjustment.\

Despite this, to make higher K<sub>p, angular</sub> work two methods were tried

###A direct cap on the velocity
  Like the title suggests there was a hard cap made to the messages published to be less than a certain value. The outcome was lacluster just resembing a lower K<sub>p, angular</sub> value unsurprisingly

### Low pass filter
  This seemed to alleviate the turning speeds resembling a K<sub>p, angular</sub> = 8 while havin a value of 10 but the pattern looked jankier and worse than a K<sub>p, angular</sub> = 8 without a low pass filter, hence there was no reason to go ahead with it.


Custom messages were implemented and are showcased in a live rqt plot during the run. These are some of the messages that were included in the live plot
1. Cross track error
   '''python
# Calculate the error vector and signed cross-track error
error_vector = pos - projected_point
error_sign = np.sign(np.cross(path_unit, error_vector / np.linalg.norm(error_vector)))
error = np.linalg.norm(error_vector) * error_sign

# Publish the error as a Float64 message
error_msg = Float64()
error_msg.data = error
self.error_pub.publish(error_msg)
'''
3. Current velocity
4. Distance to next waypoint
5. Completion percentage




