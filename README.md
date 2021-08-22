# -kudos-bh_vision_B
yolo기능과 선을 따는 기능이 합쳐져 있는 레포지스토리입니다.
---

# 공지
 - 버그, 문제 발생, 개선점 제안은 issues에 제안 부탁드립니다.
 - 최근 다크넷이 업데이트 하면서 cuda 9.X버전을 더 이상 지원하지 않게 되었습니다...   
 - alexyab 다크넷 다운시 자료 폴더의 darknet-64efa721.....zip파일을 이용해주세요

---

# 설명
- 자료/환경세팅에 관하여(SETUP ENV)
  >환경 세팅은 한줄씩 천천히 하세요  
  >환경 세팅은 오래 걸립니다. 여유를 가지고 진행하세요  

- 자료/op3_ini_env 폴더
  >빌드가 되는 것을 확인한 op3에 관련된 것만 있는 폴더입니다.
  >op3 빌드 중 문제가 발생시 문제가 되는 파일을 op3_ini_env 폴더 내부의 파일로 교체하세요

- 자료/README.MD
  >옛날 대문 설정 페이지

- kudos_vision.py
  >메인 비전 실행 코드입니다.  
  >내부 파라미터를 조절하여 가상환경과 연결하여 사용할 수 있습니다.  

- kudos_darknet.py
  >kudos_vision에서 사용되는 yolo 관련 함수들이 모여 있는 파일입니다.  
  >yolo내부 파라미터 등을 조절하고자 할 때 수정해서 사용할 수 있습니다.  

---

# 최신 비전으로 패치방법
- kudos_vision 베타 버전은 alexyAB의 yolo위에 올려서 만듭니다.
- 우선 SETUP_ENV에 나오는 가이드에 따라 Alexyab의 darknet을 설치해줍니다.
- 가이드를 잘 따라왔다면 폴더 이름이 darknetb로 되어 있을텐데 이곳에 해당 레포지스토리를 복붙해줍시다  
- 폴더는 합치도록 하고 덮어씌기는 하지 않도록 합니다.
- cm을 한번 수행해줍니다.
- python3에서 실행되는 코드들이기 때문에 ros자체를 파이썬으로 깔아줄 필요가 있습니다.
- SETUP_EVN가이드에 따라 우분투 python3를 3.5에서 3.7로 업데이트 하도록하고 ros 또한 python3로 업데이트 하도록 합시다.
- python에 사용되는 opencv버전은 opencv-python-headless버전으로 다시 깔아주도록 하고
- qt5를 SETUP_ENV 가이드에 따라 우분투 자체에서 설치해주고  
- python에 사용되는 pyqt5는 SETUP_ENV가이드에 따라 다시 잘 설치해주도록 합니다.  
- https://github.com/soyaCat/-kudos-localPackage 다음 페이지에서 mcl2패키지를 다운받아 주도록 합니다.  
- cm을 해주면 되는데 여기서 qt임포트에서 에러가 날 수도 있습니다. 이때는 CMakeList.txt를 잘 수정해줍니다.  
- catkin_ws/build/mcl2에 catkin_ws/src/mcl2에 있는 내용을 복붙해줍니다.   
- cm을 할 때 humonoid_navigation에서 에러가 발생할수도 있습니다.저 같은 경우는 폴더를 삭제했으니 알아서 잘 해결..
    
    
   ---
   
   # 비전 실행 방법
    - 터미널상에서 cd catkin_ws/src/darkneta로 간다.
    - 아나콘다 가상환경을 활성화해준다. source activate tensor27
    - python kudos_vision.py로 비전프로그램을 실행시켜준다.
    - 비전 결과는 message로 전송되는데 darkneta/position.msg파일을 임포트해주면 메세지를 읽을 수 있다.
    - 비전 결과를 읽어오는 cpp를 topic_subcriber.cpp에 만들어놓았다. 이것을 참고해서 cpp파일을 수정할 것
    - 만약 topic_subscriber.cpp를 실행하고 싶다면 다음 명령을 사용한다. rosrun darkneta topic_subscriber
    >팁  
    >우분투 메인폴더에서 ctrl+h를 누르면 숨겨진 파일들이 보이는데 .bashrc를 gedit으로 열어서 맨 밑에  
    >cd catkin_ws/src/darkneta  
    >source activate tensor27  
    >를 추가해주면 터미널을 열 때마다 darknetA폴더에서 tensor27가상환경이 활성화 되어있는 상태로 터미널을 열 수 있다.
