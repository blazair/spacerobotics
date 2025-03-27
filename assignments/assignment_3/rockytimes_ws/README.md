Done with the height estimation

Had to code for a long time for this, long time because of some inherent problems I couldn't figure out like <br>

  1)Why am i getting pre flight errors? Was it because EKF2 needed time to stabilise ? <br>
  2)Why dd i have to constantly kill processes to stop it from launching with previous configuration<br>
  3)still do not understand the rtf factor, i am always off by three seconds regardless of tring to fix it <br>
  4)the detection if it in a circle had coinciding problems when the drone passed over the short cylinder and looked over the tall cylinder <br>
  5) I will keep updating this with questions I get for tomorrow's class.<br>

aboandoned idea
before i made it go to a hover where it can see both cylinders, i had it do a spiral slwly elevating and then going to 20m ghover to detect aruco and then land, but for the cylinder estimation, i just calculated every 2 seconds and had the median and that did not tourn out so well, so abandoned it


note to self<br>
the documentation will be quite different this time including the errors i faced and how i resolved it, so i can use this as a reference for future px4 projects<br>
![Screenshot from 2025-03-26 16-55-31](https://github.com/user-attachments/assets/cfb8a1a8-6cd8-4241-a2ee-fa75af061b03)
