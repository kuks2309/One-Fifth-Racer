# 환경 설정 방법
 
 1. bashrc 편집 
  gedit ~/.bashrc 
  
 2. 아래 내용 추가 할 것 : bashrc 환경 설정 
  
  ```
  alias eb='gedit ~/.bashrc'

  alias sb='source ~/.bashrc'

  alias pu='rosclean purge -y'

  alias bao='source ~/one_fifth_catkin_ws/devel/setup.bash'
  
  alias cwo='cd ~/one_fifth_catkin_ws && source ~/one_fifth_catkin_ws/devel/setup.bash'
  
  alias cso='cd ~/one_fifth_catkin_ws/src && source ~/one_fifth_catkin_ws/devel/setup.bash'
  
  alias cmo='cd ~/one_fifth_catkin_ws && catkin_make && source ~/one_fifth_catkin_ws/devel/setup.bash'
  
  alias gzg='roslaunch uni_car uni_car_world.launch world_name:=empty.world x_pos:=0.0 y_pos:=0.0 yaw:=0'
  ```
  3. gazebo 실형명령
  
   $ roslaunch uni_car uni_car_world.launch
   
   $ gzg
   

