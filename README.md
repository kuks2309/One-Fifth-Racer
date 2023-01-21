# 환경 설정 방법
 
 1. bashrc 편집 
  gedit ~/.bashrc 
  
 2. 아래 내용 추가 할 것
  
  alias bg='source ~/one_fifth_catkin_ws/devel/setup.bash'
  
  alias cwo='cd ~/one_fifth_catkin_ws && source ~/one_fifth_catkin_ws/devel/setup.bash'
  
  alias cso='cd ~/one_fifth_catkin_ws/src && source ~/one_fifth_catkin_ws/devel/setup.bash'
  
  alias cmo='cd ~/one_fifth_catkin_ws && catkin_make && source ~/one_fifth_catkin_ws/devel/setup.bash'
  
