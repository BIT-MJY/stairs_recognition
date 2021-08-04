# stairs_recogniton
Recognize stairs with lidar.

## Dependencies
* PCL 1.8
* Eigen 3.7
* Ceres 1.14

## How to use
```
mkdir stairs_recogniton
cd stairs_recogniton
mkdir src
cd src
git clone https://github.com/BIT-MJY/stairs_recogniton.git
cd ..
catkin_make
source devel/setup.bash
roslaunch stairs_recognition stairs_recognition.launch
```

<img src="https://github.com/BIT-MJY/stairs_recogniton/blob/main/stairs_recognition.png">




