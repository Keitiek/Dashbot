# Dashbot 
Our second year project at Clevon academy is to make a bot that can move autonomously.
In our second years first semester we are going to make a bot that can be controlled from our computers using teleop.
In our second years second semester we are going to make the bot fully autonomous.

#Package architecture arm64

Links: https://docs.google.com/document/d/1k4zGNElfIEvhm66o9c2Qz7obtGh0yeYQ0cZfCKrCHxU/edit






põhilink
ROS installimine: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#additional-rmw-implementations-optional


installinime kuidas tegin:
https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html
sudo apt update
sudo apt install -y python-rosdep
ja siis läksin teisele juhendile üle
python3 -m pip install -U \
   flake8-blind-except \
   flake8-builtins \
   flake8-class-newline \
   flake8-comprehensions \
   flake8-deprecated \
   flake8-import-order \
   flake8-quotes \
   "pytest>=5.3" \
   pytest-repeat \
   pytest-rerunfailures
peale seda tehin pip update:
python3 -m pip install --upgrade pip

peale vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
tegin:
sudo apt-get install python3-vcstool

sudo rosdep init
peale seda
sudo rm -rf /etc/ros/rosdep/sources.list.d/*
jätkasin siis juhendiga
colcon tuli teade : tegin seda
sudo apt install python3-colcon-common-extensions
 jätkasin judengiga ja kui sai tehtud siis proovisin 
 Try some examples



https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html#additional-rmw-implementations-optional
