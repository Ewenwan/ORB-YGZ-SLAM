# ORB-YGZ-SLAM
This is YGZ SLAM, a faster version folked from ORB-SLAM2 (see https://github.com/raulmur/ORB_SLAM2 and the README-ORB-SLAM2.md in this repo). We put the direct tracking in SVO to accelerate the feature matching in ORB-SLAM2. We can get an average 3x speed up and keep almost same accuracy. In addition we also support monocular Visual-Inertial SLAM (VI-SLAM), following idea proposed in Raul's paper.

     结合 SVO的直接法跟踪 来加速 orb-slam2中的特征匹配  3倍加速
     同时支持 单目-惯性SLAM MVI-SLAM

# Dependency  软件依赖=======
If you are using ubuntu, just type "./install_dependency.sh" to install all the dependencies except pangolin.

-可视化   Pangolin (for visualization): https://github.com/stevenlovegrove/Pangolin 

-矩阵     Eigen3: sudo apt-get install libeigen3-dev

-图优化   g2o: sudo apt-get install libcxsparse-dev libqt4-dev libcholmod3.0.6 libsuitesparse-dev qt4-qmake 
          sudo apt-get install libsuitesparse-dev

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

#  锁

     (1)、std::mutex：该类表示普通的互斥锁, 不能递归使用。

	(2)、std::timed_mutex：该类表示定时互斥锁，不能递归使用。
	        std::time_mutex比std::mutex多了两个成员函数：

	A、try_lock_for()：函数参数表示一个时间范围，在这一段时间范围之内线程如果没有获得锁则保持阻塞；
	      如果在此期间其他线程释放了锁，则该线程可获得该互斥锁；
	      如果超时(指定时间范围内没有获得锁)，则函数调用返回false。

	B、try_lock_until()：函数参数表示一个时刻，在这一时刻之前线程如果没有获得锁则保持阻塞；
	      如果在此时刻前其他线程释放了锁，则该线程可获得该互斥锁；如果超过指定时刻没有获得锁，
	      则函数调用返回false。

	(3)、std::recursive_mutex：该类表示递归互斥锁。递归互斥锁可以被同一个线程多次加锁，
		以获得对互斥锁对象的多层所有权。例如，同一个线程多个函数访问临界区时都可以各自加锁，
		执行后各自解锁。std::recursive_mutex释放互斥量时需要调用与该锁层次深度相同次数的unlock()，
		即lock()次数和unlock()次数相同。可见，线程申请递归互斥锁时，
		如果该递归互斥锁已经被当前调用线程锁住，则不会产生死锁。
		此外，std::recursive_mutex的功能与 std::mutex大致相同。

	(4)、std::recursive_timed_mutex：带定时的递归互斥锁。

	互斥类的最重要成员函数是lock()和unlock()。在进入临界区时，执行lock()加锁操作，
	如果这时已经被其它线程锁住，则当前线程在此排队等待。退出临界区时，执行unlock()解锁操作。
# 添加的工具  /scripts
	 1. 数据集数据处理工具 
	    彩色图 深度图 位姿 数据 匹配 时间戳相识
	 2. 实验结果评估
	    相机轨迹 和 GT值 的 分析


# 代码 修改
     多的文件：
     /include/Align.h    有关align部分的算法 =================
          对齐算法  修改自 rpg_SVO     将像素与参考图像块对齐 配准
     /include/Common.h   常用的一些头文件 独立放在一起=========
     /include/NLSSolver.h            IMU 非线性最小二乘 优化求解器
     /include/NLSSolver_impl.hpp      实现 impltation 在 NLSSolver_impl.hpp
     /include/RobustCost.h            鲁棒核函数  重新下降M估计???
     /include/SparseImageAlign.h      稀疏直接法求解器
                                       请注意SVO的直接法用了一种逆向的奇怪解法，
                            它的雅可比是在Ref中估计而不是在Current里估计的，所以迭代过程中雅可比是不动的
     


