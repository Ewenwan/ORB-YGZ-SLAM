# ORB-YGZ-SLAM
This is YGZ SLAM, a faster version folked from ORB-SLAM2 (see https://github.com/raulmur/ORB_SLAM2 and the README-ORB-SLAM2.md in this repo). We put the direct tracking in SVO to accelerate the feature matching in ORB-SLAM2. We can get an average 3x speed up and keep almost same accuracy. In addition we also support monocular Visual-Inertial SLAM (VI-SLAM), following idea proposed in Raul's paper.

     结合 SVO的直接法跟踪 来加速 orb-slam2中的特征匹配  3倍加速
     同时支持 单目-惯性SLAM MVI-SLAM

# Dependency  软件依赖=======
If you are using ubuntu, just type "./install_dependency.sh" to install all the dependencies except pangolin.

-可视化   Pangolin (for visualization): https://github.com/stevenlovegrove/Pangolin 

-矩阵     Eigen3: sudo apt-get install libeigen3-dev

-图优化   g2o: sudo apt-get install libcxsparse-dev libqt4-dev libcholmod3.0.6 libsuitesparse-dev qt4-qmake 

-2d图像   OpenCV: sudo apt-get install libopencv-dev

-日志     glog (for logging): sudo apt-get install libgoogle-glog-dev


# 第三方库 多加了
     一个 fast 角点库  支持 neno  sse 加速
     一个 sophus 李群李代数库  也就是 se3 so3    使用  Eigen 实现

# Compile  编译
run "./generate.sh" to compile all the things, or follow the steps in generate.sh

# Examples
We support all the examples in the original ORB-SLAM2, 
and also the monocular-inertial examples. 
You can try the EUROC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) and
run the monocular/mono-inertial examples by 
typing:

    单目-SLAM 例子运行

```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.bin Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

    单目-惯性SLAM 例子运行
```
 ./Examples/Monocular/mono_euroc_vins ./Vocabulary/ORBvoc.bin ./Examples/Monocular/EuRoC.yaml /var/dataset/euroc/V1_01_easy/cam0/data ./Examples/Monocular/EuRoC_TimeStamps/V101.txt /var/dataset/euroc/V1_01_easy/imu0/data.csv
```

to run the VIO case.

# Other things  改进
     直接法加速 特征匹配 鲁棒性不够
     双目-惯性-slam 待开发
     
We follow SVO when writing direct tracking, whose speed is very fast but robustness is not very good. 
In EUROC it can pass the test of MH01, MH02, MH03 and V101, V201. 
For difficult cases it may still fail. 
We are still improving it and writing a better solution for stereo-inertial case.

     YGZ  一锅粥 哈哈哈哈   特征点法 直接法  imu
      
YGZ stands for Yi-Guo-Zhou (a port of porridge, a group of mess) because it contains feature method, direct method and imu things.

The Note.md is a file of develop records.  错误日志

Contact me (gaoxiang12@mails.tsinghua.edu.cn) or Wang Jing (https://github.com/jingpang) for commercial use.

in "scirpts" you can find some pythons to evalute the keyframe trajectory (6DoF or 7DoF in monocular case).

Thanks the following companies/people for finantial support:

- Usens凌感科技
- Hyperception 远形时空
- Qfeeltech 速感科技
- 天之博特
- 视辰信息科技

