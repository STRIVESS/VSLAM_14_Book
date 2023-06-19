#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
  Quaterniond q1(0.35, 0.2, 0.3, 0.1), q2(-0.5, 0.4, -0.1, 0.2); // 实部w为0.35，虚部为x,y,z为0.2, 0.3, 0.1
  q1.normalize();                                                // 将四元数 q1 归一化，使其模长为 1
  q2.normalize();
  Vector3d t1(0.3, 0.1, 0.1), t2(-0.1, 0.5, 0.3); // 两个平移向量
  Vector3d p1(0.5, 0, 0.2);                       // 定义一个三维向量 p1，表示一个空间点的位置

  // 定义两个等距变换矩阵 T1w 和 T2w，分别表示从相机坐标系到世界坐标系的变换,构造函数的参数为四元数，表示旋转部分的变换
  Isometry3d T1w(q1), T2w(q2);
  T1w.pretranslate(t1); // 在等距变换矩阵 T1w 中添加一个平移向量 t1，表示从相机坐标系到世界坐标系的平移变换
  T2w.pretranslate(t2);

  // 将空间点 p1 从相机坐标系转换到世界坐标系下的位置，即先将它从相机坐标系变换到世界坐标系，再乘以变换矩阵的逆矩阵，即可得到在世界坐标系下的位置 p2
  // 其中，变换矩阵 T1w.inverse() 表示从世界坐标系到相机坐标系的变换
  Vector3d p2 = T2w * T1w.inverse() * p1;
  cout << endl
       << p2.transpose() << endl;
  return 0;
}