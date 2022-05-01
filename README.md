# icp_lidar_ceres_ros
icp_lidar_ceres 패키지의 코드에 ROS 통신 코드를 추가하여 ROS 환경에서도 사용할 수 있도록 작성한 패키지입니다.

## Dependencies
- [Eigen Library](https://eigen.tuxfamily.org/index.php?title=Main_Page)  
- [knncpp kd tree Library](https://github.com/Rookfighter/knn-cpp)  
- [Ceres Library](https://github.com/ceres-solver/ceres-solver)  
위의 4가지 라이브러리를 `catkin_ws/src` 폴더에 clone 한 후 `catkin_make`로 컴파일 후 실행이 가능합니다.

## 예제 실행 방법
```
cd ~/catkin_ws/src/
git clone https://github.com/Refstop/icp_lidar_ceres_ros.git
cd ..
catkin_make
source devel/setup.bash
```
catkin_make 이후의 예제 실행 방법은 다음과 같습니다.
```
roslaunch icp_lidar_ceres_ros icp_lidar_ceres.launch
```
해당 launch 파일 실행 시 패키지에 기본적으로 저장된 `2d_lidars_scan.bag` 파일에 대한 icp 알고리즘 실행 결과가 rviz에 표시됩니다.

## 예제 실행 결과
- `2d_lidars_scan.bag` 파일의 정렬 결과
<p align="center"><img src="/figs/icp_lidar_ceres.gif"></p>