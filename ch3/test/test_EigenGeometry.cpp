#include <iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

// 具体可以参考【https://blog.csdn.net/u011092188/article/details/77430988】
// 【https://blog.csdn.net/delovsam/article/details/104427530?ydreferer=aHR0cHM6Ly93d3cuZ29vZ2xlLmNvbS5oay8%3D】
int main(int argc, char **argv)
{
     // Matrix3d::Identity()用单位矩阵对rotation_matrix变量进行了初始化
     Matrix3d rotation_matrix = Matrix3d::Identity();

     // 使用旋转角和旋转轴向量（此向量为单位向量）来初始化角轴;以（0,1,0）为旋转轴，旋转60度
     AngleAxisd rotation_vector(M_PI / 3, Vector3d(0, 1, 0));
     cout.precision(3);
     cout << "rotation matrix =\n"
          << rotation_vector.matrix() << endl;             // 用matrix()转换成矩阵
     rotation_matrix = rotation_vector.toRotationMatrix(); // 或者可以直接赋值

     Vector3d v(1, 0, 0);
     Vector3d v_rotated = rotation_vector * v;
     Vector3d V_rotated = rotation_matrix * v;
     cout << "(1,0,0) after rotation by angle axis = " << v_rotated.transpose() << endl;
     cout << "(1,0,0) after rotation by matrix = " << V_rotated.transpose() << endl;

     // 欧拉角
     Vector3d euler_angles = rotation_matrix.eulerAngles(1, 3, 0);
     cout << "yaw pitch roll: " << euler_angles.transpose() << endl;

     // 欧氏变换
     Isometry3d T = Isometry3d::Identity();
     T.rotate(rotation_vector);
     T.pretranslate(Vector3d(2, 6, 1));
     cout << "Transform matrix = \n"
          << T.matrix() << endl;

     Vector3d v_transformed = T * v;
     cout << "v transformed = " << v_transformed.transpose() << endl;

     // 四元数
     Quaterniond q1(1, 2, 3, 4);           // 实部w为1，虚部为x,y,z为2，3，4
     Quaterniond q2(Vector4d(1, 2, 3, 4)); // 实部w为4，虚部为x,y,z为1，2，3
     cout << "quaternion from (1, 2, 3, 4) " << q1.coeffs().transpose() << endl;
     cout << "quaternion from Vector4d(1, 2, 3, 4) " << q2.coeffs().transpose() << endl;

     Quaterniond q = Quaterniond(rotation_vector);
     Quaterniond Q = Quaterniond(rotation_matrix);
     cout << "quaternion from rotation vector" << q.coeffs().transpose() << endl;
     cout << "quaternion from rotation matrix" << Q.coeffs().transpose() << endl;

     // 以下两种写法是等价的，参考《视觉SLAM十四讲》P59，p'=q*p*q^(-1)
     Vector3d v1_rotated = q * v;
     Quaterniond v2_rotated = q * Quaterniond(0, 1, 0, 0) * q.inverse();

     cout << "(1,0,0) after rotation = " << v1_rotated.transpose() << endl;
     cout << "(1,0,0) after rotation = " << v2_rotated.coeffs().transpose() << endl;

     return 0;
}