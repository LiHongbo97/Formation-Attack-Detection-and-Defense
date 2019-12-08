# Formation-Attack-Detection-and-Defense
Multi-robot formation control. An attacker attack the formation and the formation can detect the attacker by SVM, then defend the attack by topology transform.
You can run attacker.m or defend.m. I have trained some data to the formation and they are stored in .mat.

The formation is modeled by five circles:
![image](https://github.com/redglassli/Formation-Attack-Detection-and-Defense/blob/master/pic/Consensus%20Formation.jpg)

And there is an attacker who is red:
![image](https://github.com/redglassli/Formation-Attack-Detection-and-Defense/blob/master/pic/attacker.jpg)

After a long time, the formation can detect the robot who is being attacked by SVM:
![image](https://github.com/redglassli/Formation-Attack-Detection-and-Defense/blob/master/pic/detection result.jpg)

After that, we output the path and the S to verify it:
![image](https://github.com/redglassli/Formation-Attack-Detection-and-Defense/blob/master/pic/path.jpg)

![image](https://github.com/redglassli/Formation-Attack-Detection-and-Defense/blob/master/pic/S%output.jpg)
