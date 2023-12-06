## Problem Description

After ssh-ing into Jetson, I wanted to start Arduino GUI but was getting this error:

```ssh
Picked up JAVA_TOOL_OPTIONS: 
Set log4j store directory /home/dashbot/.arduino15
java.awt.AWTError: Can't connect to X11 window server using '10.0.3.174:0' as the value of the DISPLAY variable.
	at sun.awt.X11GraphicsEnvironment.initDisplay(Native Method)
	at sun.awt.X11GraphicsEnvironment.access$200(X11GraphicsEnvironment.java:65)
	at sun.awt.X11GraphicsEnvironment$1.run(X11GraphicsEnvironment.java:115)
	at java.security.AccessController.doPrivileged(Native Method)
	at sun.awt.X11GraphicsEnvironment.<clinit>(X11GraphicsEnvironment.java:74)
	at java.lang.Class.forName0(Native Method)
	at java.lang.Class.forName(Class.java:264)
	at java.awt.GraphicsEnvironment.createGE(GraphicsEnvironment.java:103)
	at java.awt.GraphicsEnvironment.getLocalGraphicsEnvironment(GraphicsEnvironment.java:82)
	at sun.awt.X11.XToolkit.<clinit>(XToolkit.java:126)
	at java.lang.Class.forName0(Native Method)
	at java.lang.Class.forName(Class.java:264)
	at java.awt.Toolkit$2.run(Toolkit.java:860)
	at java.awt.Toolkit$2.run(Toolkit.java:855)
	at java.security.AccessController.doPrivileged(Native Method)
	at java.awt.Toolkit.getDefaultToolkit(Toolkit.java:854)
	at java.awt.SystemColor.updateSystemColors(SystemColor.java:473)
	at java.awt.SystemColor.<clinit>(SystemColor.java:465)
	at processing.app.Theme.init(Theme.java:343)
	at processing.app.Base.<init>(Base.java:250)
	at processing.app.Base.main(Base.java:150)
```
## Solution

The basis for the solution came from here: https://www.linuxquestions.org/questions/linux-general-1/can%27t-connect-to-x11-window-server-using-%27-0-0%27-as-the-value-of-the-display-variable-178234/
Steps for problem solving:

- download Xvfb:
```ssh
sudo apt-get install xvfb
```
- run Xvfb on local machine:
```ssh
Xvfb :1 -screen 0 1024x768x16
```
- verify Xvfb is running:
```ssh
ps -ef | grep Xvfb
```
- ssh into remote machine and start Arduino:
```ssh
arduino
```

