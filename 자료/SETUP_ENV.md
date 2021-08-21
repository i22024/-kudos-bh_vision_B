
# 환경 세팅

1. op3와의 호환성을 위하여 jetson tx2에 jetpack 3.3을 설치한다
>jetpack3.3은 우분투 16.04를 지원하는 가장 최신 버전의 jetson os 이미지이다.  
>op3가 ROS melodic을 지원하지 않는 이상 jetpack 3.3 사용이 최선  
>cudnn 7.6.5를 따로 설치하면 gpu 성능을 증가시킬 수 있다.  
>설치시 cuda 버전에 맞추어서 설치해야한다.  


2. 우분투 패키지를 업데이트 한다.  
```
sudo apt-get upgrade
sudo apt-get update
```

3. ROS kinetict 설치
>참고링크:https://robertchoi.gitbook.io/ros/install
```
wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```

```
sudo apt-get install -y chrony ntpdate

sudo ntpdate -q ntp.ubuntu.com

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

sudo apt-get update && sudo apt-get upgrade -y

sudo apt-get install ros-kinetic-desktop-full

sudo apt-get install ros-kinetic-rqt*

sudo rosdep init

rosdep update

sudo apt-get install python-rosinstall

source /opt/ros/kinetic/setup.bash

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src

catkin_init_workspace

cd ~/catkin_ws/

catkin_make

source ~/catkin_ws/devel/setup.bash
```
> test시에 다음 명령어를 입력하여 작동을 확인
```
roscore
```
>다음으로 ROS환경설정을 한다
```
gedit ~/.bashrc
```
>>다음 내용이 삽입되어있는지 확인 후 없으면 넣는다.
```
alias eb =‘nano ~/.bashrc'
alias sb ='source ~/.bashrc'
alias cw ='cd ~/catkin_ws'
alias cs ='cd ~/catkin_ws/src'
alias cm ='cd ~/catkin_ws && catkin_make'
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
```

4. op3 설치
>참고 링크:https://emanual.robotis.com/docs/en/platform/op3/recovery/#recovery-of-robotis-op3  
>추가 로보티즈 ROS패키지를 설치한다.
```
sudo apt install libncurses5-dev v4l-utils

sudo apt install madplay mpg321

sudo apt install g++ git
```
>op3를 위한 ROS패키지를 설치한다.
```
cd ~/catkin_ws/src

git clone https://github.com/ROBOTIS-GIT/face_detection.git

cd ~/catkin_ws

catkin_make

sudo apt install ros-kinetic-robot-upstart

cd ~/catkin_ws/src

git clone https://github.com/bosch-ros-pkg/usb_cam.git

cd ~/catkin_ws

catkin_make

sudo apt install v4l-utils

sudo apt install ros-kinetic-qt-ros
```
>humanoid navigation을 설치한다.
```
sudo apt-get install ros-kinetic-map-server

sudo apt-get install ros-kinetic-humanoid-nav-msgs

sudo apt-get install ros-kinetic-nav-msgs

sudo apt-get install ros-kinetic-octomap

sudo apt-get install ros-kinetic-octomap-ros

sudo apt-get install ros-kinetic-octomap-server
```
>sbpl을 설치한다.
```
cd ~/catkin_ws/src

git clone https://github.com/sbpl/sbpl.git

cd sbpl

mkdir build

cd build

cmake ..

make

sudo make install
```
>humanoid navigation을 마저 설치한다.
>> ! catkin_make 중 humanoid_localization.cpp build에서 pcl/filters/uniform_sampling.h: No such file or directory 애러가 발생할수도 있는데 이는 op3가 pcl의 옛날 uniform_sampling의 패키지의 위치를 참조하기 때문에 발생하는 일임.  
>> ! catkin_ws/src/humanoid_navigation/humanoid_localization/src/HumanoidLocalization.cpp를 gedit으로 열고 #include <pcl/filters/uniform_sampling.h>을 #include <pcl/keypoints/uniform_sampling.h>로 고치자
```
cd ~/catkin_ws/src

git clone https://github.com/ROBOTIS-GIT/humanoid_navigation.git

cd ~/catkin_ws

catkin_make
```
>web_setting tools를 위한 패키지를 설치한다.
```
sudo apt install ros-kinetic-rosbridge-server ros-kinetic-web-video-server
```
>Robotis op3 Robotpackages를 설치하자
>> !catkin_make 중 Robotis-OP3-Tools에서 action_editor 빌드 오류가 날 수도 있는데 깔끔하게 이 레포지스토리에 있는 ROBOTIS-OP3-Tools로 교체하면 해결된다.
```
cd ~/catkin_ws/src

git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git

git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework.git

git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git

git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Math.git

git clone https://github.com/ROBOTIS-GIT/ROBOTIS-OP3.git

git clone https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Demo.git

git clone https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-msgs.git

git clone https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Tools.git

git clone https://github.com/ROBOTIS-GIT/ROBOTIS-OP3-Common.git

git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Utility.git

cd ~/catkin_ws

catkin_make
```

5. YOLO V4 설치
>참고주소: https://wendys.tistory.com/143  
>참고주소: https://eehoeskrap.tistory.com/355  
>참고주소: https://github.com/AlexeyAB/darknet#yolo-v4-in-other-frameworks  
>쿠다 path 불러오기
```
sudo apt-get update
export PATH=/usr/local/cuda-9.0/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-9.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```
>cmake update
```
cd 
wget https://cmake.org/files/v3.16/cmake-3.16.2.tar.gz
tar -xvzf 해당파일.tar.gz
./bootstrap --prefix=/usr/local
make
make install
```
>yolo project download
```
이 레포지스토리에 있는 darknet64efa....zip을 다운받은 이후 압축을 catkin/src에 해지하고 darknetb로 폴더 이름을 변경한다

cd darknetb

wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights

wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre/yolov4-tiny.weights
```
>make file 수정하기
```
sudo gedit Makefile
```
>다음과 같이 수정 후 저장
```
GPU=1
CUDNN=1
OPENCV=1
LIBSO=1
```
>make
```
mkdir build_release
cd build_release
cmake ..
cmake --build . --target install --parallel 8
```
>설치 완료 테스트
```
python darknet_images.py
#이미지 경로입력이 나오면 다음을 입력
#"./data/dog.jpg"
```

>설치완료 테스트  
>최종적으로 catkin_make를 하여 빌드 테스트를 해봅니다.  
>!catkin_make를 하다가 rospkg를 찾을 수 없다는 에러가 나오면 다음을 해봅니다.  
>!터미널을 새로 열고 가상환경(tensor27)이 활성화 되지 않은 상태로  
>!pip3 install -U rospkg를 입력하고 다시 cm을 입력하여 해결해봅시다    
  

  
6. ROS를 python3로 업데이트 해보자
```
1. sudo apt update  
2. sudo apt install software-properties-common  
3. sudo add-apt-repository ppa:deadsnakes/ppa  
4. sudo apt update
5. sudo apt install python3.7
```  
  
```
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.7 1
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.5 2
sudo update-alternatives --config python3
# python 3.7을 python3로 선택한다.
```

```
sudo apt install python3-pip python3-all-dev python3-rospkg
sudo apt install ros-kinetic-desktop-full --fix-missing
```

7.  qt5와 pyqt를 설치
```
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install qtcreator
sudo apt-get install -y qt5-default
```

```
pip install PyQt5
```


8. 쉬운 코드 편집 작업을 위한 VScode 설치
>참고 주소: https://opencourse.tistory.com/221  
>참고 주소: https://mylogcenter.tistory.com/7  
>참고 주소: https://makingrobot.tistory.com/83  
>설치
```
1. https://github.com/toolboc/vscode/releases에서 release 다운로드
2. 다운이 완료되면 다음 명령어로 설치진행 sudo dpkg -i code-oss_1.32.3-arm64.deb   
```
>파이썬 편집을 위한 세팅(필수 아님)(하지만 하면 좋음)
  1. VScode로 들어간다.
  2. Extension에서 python 설치
  3. Visual Studio IntelliCode 설치
  4. Python for VSCode 설치
  5. Python Extension Pack 설치
  6. code Runner 설치
  7. ctrl+shift+p를 눌러준 후 Python:Select Interpreter를 눌러주기
  8. 목록 중 이 튜토리얼에서 만든 tensor27을 선택하기
>이제 cpp파일을 열거나 python 파일등 문서 파일을 열 때 code 파일이름 을 치시면 됩니다.  
>cpp 위주로 편집하신다면 파이썬 편집을 위한 세팅 대신 검색하셔서 알맞은 환경 세팅을 부탁드립니다.  
>http://wanochoi.com/?p=4643에 따라 pycharm을 설치한다면 pycharm 실행 명령어는 다음과 같습니다.  
>sudo pycharm.sh  
>pycharm을 설치하시고 인터프리터 연결을 python 가상환경과 연결하면 자동 완성 기능 사용가능

## 빌드 중 em관련 에러가 나면 pip install empy를 해주시면 됩니다.

---

# 노트북 환경 세팅

- 위 환경세팅을 따라했다면 jetson에는 최종적으로 cuda 9.0, cudnn, opencv 3.3.1, yoloV4, op3가 설치되고  
   jetson의 anaconda 가상환경에는 python2.7, tensorflow-gpu 1.14.0, ros관련패키지가 설치되게 된다.  
- 노트북 우분투에서도 같은 환경으로 빌드하면 yolo나 tensorflow 훈련 파일을 공유해서 사용하거나 파이썬 코드 파일을 돌릴 수 있어 편하다  
!!!!!!!!!!nvidia그래픽 카드가 없는 노트북이라면 yolo나 텐서플로우를 cpu버전으로 사용해야 하는데 코드나 가중치값이 제대로 호환될 것이라는 것을 보장할 수 없다  
!!!!!!!!!!이 가이드는 nvidia 그래픽카드가 있는 컴퓨터를 기준으로 가이드가 진행된다.


1. 노트북의 그래픽카드를 업데이트 하고 cuda9.0과 그에 맞는 cudnn을 설치한다.
>참고: https://jdselectron.tistory.com/85
2. ros를 설치한다.
3. op3를 설치한다.(이때 시스템 파이썬이 2.7버전이여야 한다.)
4. anaconda를 설치한다.
>(ros와 anaconda 정식 릴리즈 사이에는 충돌이 발생할 수도 있다.)  
>참고:https://www.youtube.com/watch?v=EMF20z-gT5s 를 보면서 해결하면 된다.
5. 아나콘다에 가상환경(python=2.7)을 만들어주고 opencv 3.3.1과 tensorflow-gpu 1.14.0 을 설치해준다.
```
#가상환경 활성화
pip install --ignore-installed --upgrade tensorflow==1.14.0
#opencv는 3.3.1버전으로 다운로드
```
6. yoloV4를 설치한다.


