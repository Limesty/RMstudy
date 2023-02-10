## 双目解析算法

### 1. 相机标定

相机标定过程即为根据双目相机拍摄的照片来求出相机的内参数的一个过程。目前相机的标定过程已经有成熟的解决方案，常见的两种解决方案为MatLab所自带的计算机视觉工具包来进行标定；另一种是使用opencv自带的工具库来进行相机的标定。在这里，我们使用MatLab来进行相机的标定。

MatLab进行相机标定的过程比较繁琐，在这里不展开说明，请参考这个博客：https://blog.csdn.net/qq_42118719/article/details/112471107?utm_medium=distribute.pc_feed_404.none-task-blog-2~default~BlogCommendFromBaidu~Rate-3-112471107-blog-null.pc_404_mixedpudn&depth_1-utm_source=distribute.pc_feed_404.none-task-blog-2~default~BlogCommendFromBaidu~Rate-3-112471107-blog-null.pc_404_mixedpud

进行标定之后即可得到相机的内参数。需要注意的一点就是，在得出相机的畸变系数的时候，D矩阵是五维的一个向量，对应了径向畸变和切向畸变。在opencv（也即为我们的项目代码）之中，五个参数的排列方式为[K1, K2, D1, D2, K3]（没错，就是这么诡异的一个排列过程）。MatLab则会分别得到两组畸变系数，之间的转换需要注意。

标定还需要注意一点，参数R对应双目摄像机的旋转矩阵，在我们的项目代码之中，这个矩阵是3*3的，但是在MatLab标定所得出来的是一个3 * 1的一个旋转向量，需要通过下列的一段代码来进行转换：

~~~python
    import cv2
    import numpy as np
 
    # 定义旋转矩阵R，旋转向量om
    R = [[9.9999370551606337e-01, 7.8563882630048958e-04, 3.4600144345510440e-03],
         [-7.9503149273969136e-04, 9.9999600080163187e-01, 2.7140938945082542e-03],
         [-3.4578682997252063e-03, -2.7168276311286426e-03, 9.9999033095047696e-01]]
    R = np.asarray(R)
    print(f"旋转矩阵R:\n {R}")
    # 把旋转矩阵R转化为旋转向量om
    om, _ = cv2.Rodrigues(R)
    print(f"旋转向量om:\n {om}")
    # 把旋转向量om转换为旋转矩阵R
    R1, _ = cv2.Rodrigues(om)
    print(f"旋转矩阵R1:\n {R1}")
~~~

自此，标定过程结束。

### 2. 双目解析算法

有关于双目解析的相关内容都在StereoReconstruct类之中。

其中task方法接收参数为两个图像，分别为左目和右目相机传递进来的图像。经过task进行双目测距任务之后，得到了像素坐标的三维重建信息，存储在cv::Mat xyz_coord之中。如果需要打印信息，可以类似于下列代码来得到更详细的信息。

会得到[x, y, depth]格式的三维坐标，即为世界坐标系下的三维坐标。**世界坐标系的中心为左目相机的光心**。

~~~cpp
    cv::Point start;
    start = Point(203, 200);
    cout << " world coords=(x,y,depth)=" << detector->xyz_coord.at<Vec3f>(start) << endl;
   
~~~

### 3. 缺陷

本双目解析算法依据SGBM算法来进行三维重建，一般来说，误差较小。但是遇到光线变化不强烈的地方，重建效果并不好。还有一点需要注意的是，选择像素的时候尽量不要选择在边缘地点。