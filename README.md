------------------------------------------------------------------------------------------------------------------------------------

This repository is for Control Laboratory in PNU and is being developed for Dynamixel arms attached on Kobuki what used in the lab.

철수형과 상일이를 위하여!!!

Since 2017.06.28

------------------------------------------------------------------------------------------------------------------------------------

순서

1. ROS를 먼저 설치합시다.

2. Dynamixel SDK를 설치하기 위하여 별도의 폴더 생성
	mkdir ~/DynamixelSDK && cd ~/DynamixelSDK
	git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
	
3. catkin_workspace 생성 및 catkin_make 를 통한 src 폴더 생성
	http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

4. Dynamixel SDK 설치 --> https://github.com/ROBOTIS-GIT/DynamixelSDK/wiki/4.-SDK-Example
	해당 페이지에서 "4.1 C" 및 다음 페이지에서 "LINUX" 선택하여 절차 수행
	그냥 따라치기보다는 어떤 작업을 하고 있는지 반드시 숙지하도록 합시다.
	
5. catkin_workspace/src/  하위에 적당한 폴더를 만듭시다.
	git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git
	git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git
	cd ~/catkin_workspace/ && catkin_make

6. 적당한 폴더를 만들어 본 소스 코드를 clone합시다.
	cd ~/catkin_workspace/src/ && mkdir ctrl_lab_git 
	cd ctrl_lab_git && git clone https://github.com/sh4j/ctrl_lab.git
	
7. 5에서 github에서 땡겨온 내용 '위에' 6번을 덮어쓰기 합시다.

8. catkin_make를 합시다.
	먼저 launch파일이 들어있는 my* 디렉토리를 숨김처리 합시다.
	간단하게 디렉토리 명을 my* -> .my* 로 변경하면 됨.
	cd ~/catkin_workspace && catkin_make
	
9. launch 파일 실행
	(터미널 창에서) roscore
	(다른 터미널 창에서) roslaunch workspace_name launch_file_name.launch

10. error 발생시 알려주시기 바랍니다.