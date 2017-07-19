
------------------------------------------------------------------------------------------------------------------------------------

This repository is for Control Laboratory in PNU and is being developed for Dynamixel arms attached on Kobuki what used in the lab.

철수형과 상일이를 위하여!!!

Since 2017.06.28

------------------------------------------------------------------------------------------------------------------------------------

# 1. ROS를 설치합시다.
14.04 기준으로 딴거 필요 없고 다음과 같이 설치합시다.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall
```
이것으로 ROS 설치는 끝이 납니다.

# 2. Dynamixel SDK를 설치
-- 별도의 디렉토리를 생성해두면 편리합니다.
```
mkdir ~/DynamixelSDK && cd ~/DynamixelSDK
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

# 3. catkin_workspace
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
> - 위 페이지를 보고 잘 따라합시다.

# 4. Dynamixel SDK 설치
https://github.com/ROBOTIS-GIT/DynamixelSDK/wiki/4.-SDK-Example
> - 해당 페이지에서 "4.1 C" 및 다음 페이지에서 "LINUX" 선택하여 절차 수행
> - 그냥 따라치기보다는 어떤 작업을 하고 있는지 **반드시 숙지**하도록 합시다.*
	
# 5. git에서 dymamixel code 받아오기 
catkin_workspace/src/  하위에 적당한 폴더를 만듭시다.
```
sudo apt-get install ros-indigo-qt-build
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
cd ~/catkin_workspace/ && catkin_make
```
# 6. 본 소스 코드의 clone
```
cd ~/catkin_workspace/src/ && mkdir ctrl_lab_git 
cd ctrl_lab_git && git clone https://github.com/sh4j/ctrl_lab.git
```
	
# 7. 덮어쓰기
5에서 github에서 땡겨온 디렉토리 및 파일에 6번을 **덮어쓰기** 합시다.

# 8. catkin_make
먼저 launch파일이 들어있는 my* 디렉토리를 숨김처리 합시다. catkin_make시 에러가 나기 때문입니다. 이런 귀찮은 작업을 생략할 수 있는 방법을 조만간 찾아보겠습니다. 우선은 임시방편으로 디렉토리 명을 변경해가면서 씁시다.
간단하게 디렉토리 명을 my* -> .my* 로 변경하면 됩니다.
콘솔에서의 파일/디렉토리 명 변경은 mv 입니다.
익숙하지 않다면 GUI에서 바꾸어도 됩니다. 단축키는 F2.
```
mv my* .my*
cd ~/catkin_workspace && catkin_make
mv .my* my*
```	
# 9. launch 파일 실행
(터미널 창에서) roscore
(다른 터미널 창에서) roslaunch workspace_name launch_file_name.launch

# 10. error 발생시 알려주시기 바랍니다.
