#include <iostream>

using namespace std;

#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#define MATRIX_SIZE 50

clock_t start_t, end_t;
double total_t;
int time_cost();

// 整数类型主函数(整数类型统计参数个数,字符类型指针数组指向字符串参数)
int main(int argc, char **argv)
{
     // 相关知识参考【https://blog.csdn.net/MaybeTnT/article/details/110868488】
     // 动态大小的矩阵
     Matrix<double, Dynamic, Dynamic> matrix_dynamic;
     Matrix<int, Dynamic, 1> Vector_dynamic;

     // MatrixXf、VectorXd 默认的构造函数不执行任何空间分配，也不初始化矩阵的元素
     // K、J分别是动态大小的矩阵、向量，未分配空间，不能直接初始化，否则编译通过但运行报段空间错
     MatrixXf K;
     VectorXd J;

     // 对于固定或动态大小的矩阵，都提供统一的API分配空间大小，但未初始化内部元素
     // 用MatrixKd, 必须K<=4 才能初始化
     Matrix4d M(4, 4); // M分配了double[16]的空间，传递size是合法的（传递被忽略）
     MatrixXf N(2, 5); // N分配了float[10]的空间
     Matrix<float, 2, 4> matrix_24;
     Matrix<double, 3, 4> matrix_34;

     // 向量是一种特殊的矩阵（一行或一列）
     // 用VectorKd, K必须<=4 才能初始化
     VectorXd V_6d(6); // 分配空间大小为3的向量，未初始化
     Matrix<double, 4, 1> V_4d;

     // 初始化
     MatrixXd matrix_4d = Matrix4d::Zero();
     MatrixXf matrix_4f = Matrix4f::Zero();

     matrix_34 << 10, 2, 3, 4, 5, 6, 11, 22, 33, 44, 55, 66;
     cout << "matrix 3*4 from 1 to 12:\n"
          << matrix_34 << endl;

     matrix_24 << 44, 06, 81, 19, 98, 04, 11, 36;
     cout << "matrix 4*2 from 1 to 8:\n"
          << matrix_24 << endl;

     V_4d << 0, 1, 3, 6;
     cout << "elements of V_4d  from 1 to 4:\n"
          << V_4d << endl;

     Vector2d V_2d(5.1, 3.14); // 直接初始化
     V_6d << 9, 8, 4, 6, 0, 1;
     cout << "elements of V_6d from 1 to 6:\n"
          << V_6d << endl;

     cout << "elements of V_2d from 1 to 2:\n"
          << V_2d << endl;

     cout << "print matrix 3*4" << endl;
     for (int i = 0; i < 3; i++)
     {
          for (int j = 0; j < 4; j++)
               cout << matrix_34(i, j) << "\t";
          cout << endl;
     }

     cout << "print matrix 2*4" << endl;
     for (int i = 0; i < 2; i++)
     {
          for (int j = 0; j < 4; j++)
               cout << matrix_24(i, j) << "\t";
          cout << endl;
     }

     // 矩阵运算（PS：类型/维度要一致）
     Matrix<double, 3, 1> result = matrix_34 * V_4d;
     Matrix<float, 2, 1> result2 = matrix_24 * V_4d.cast<float>();

     cout << "[1, 2, 3, 4, 5, 6;11, 22, 33, 44, 55, 66]*[0, 1, 3, 6]:\n"
          << result << endl
          << "\n";
     cout << "[44, 06, 81, 19;98, 04, 11, 36]*[0, 1, 3, 6]:\n"
          << result2 << endl;

     // 对于N阶矩阵
     M = Matrix4d::Random();
     cout << "random matrix: \n"
          << M << endl;
     cout << "transpose: \n"
          << M.transpose() << endl;          // 转置
     cout << "sum: " << M.sum() << endl;     // 各元素和
     cout << "trace: " << M.trace() << endl; // 迹（主对角线上各元数求和）
     cout << "times 10: \n"
          << 10 * M << endl; // 数乘
     cout << "inverse: \n"
          << M.inverse() << endl;                // 逆
     cout << "det: " << M.determinant() << endl; // 行列式

     // Eigen::SelfAdjointEigenSolver类计算自伴随矩阵的特征值和特征向量
     // 特征值和特征向量分解的方法：EigenSolver、SelfAdjointEigenSolver、ComplexEigenSolver
     // 具体参考以下链接
     // 【https://zxl19.github.io/eigen-note/】
     // 【https://zhuanlan.zhihu.com/p/550779050】
     // 【https://www.cnblogs.com/dzw2017/p/8427677.html】

     // SelfAdjointEigenSolver<Matrix4f> eigen_solver(M.transpose() * M);
     // EigenSolver<Matrix4f> eigen_solver(M.transpose() * M);
     ComplexEigenSolver<Matrix4d> eigen_solver(M.transpose() * M);
     cout << "Eigen values: \n"
          << eigen_solver.eigenvalues() << endl; // 特征值
     cout << "Eigen vectors:\n"
          << eigen_solver.eigenvectors() << "\n"
          << endl; // 特征向量

     MatrixXd matrix_NN(MATRIX_SIZE, MATRIX_SIZE);
     matrix_NN = MatrixXd::Random(MATRIX_SIZE, 1);
     VectorXd v_Nd(MATRIX_SIZE, 1);
     v_Nd = VectorXd::Random(MATRIX_SIZE, 1);
     matrix_NN = matrix_NN * matrix_NN.transpose();

     MatrixXd a(MATRIX_SIZE, 1);

     for (int k = 0; k < 3; k++)
     {
          start_t = clock();
          if (k == 0)
               a = matrix_NN.inverse() * v_Nd;
          if (k == 1)
               a = matrix_NN.colPivHouseholderQr().solve(v_Nd);
          else
               a = matrix_NN.ldlt().solve(v_Nd);
          time_cost();
          cout << "a=" << a.transpose() << "\n"
               << endl;
     }

     return 0;
}

int time_cost()
{
     end_t = clock();
     total_t = (double)(end_t - start_t) / CLOCKS_PER_SEC;
     cout << "time of inverse is " << total_t << "ms" << endl;
     return 0;
}