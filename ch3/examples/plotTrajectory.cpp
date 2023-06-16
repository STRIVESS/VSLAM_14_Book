#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>

// 本例演示了如何画出一个预先存储的轨迹

using namespace std;
using namespace Eigen;

// path to trajectory file
string trajectory_file = "/home/kevin/Project_code/slambook2/ch3/examples/trajectory.txt";

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);

int main(int argc, char **argv)
{

  //*vector有两个参数，后面的参数一般是默认的，这里用适合Eigen库的对齐方式来初始化容器
  // 欧氏变换矩阵使用 Eigen::Isometry，虽然称为3d，实质上是4＊4的矩阵
  // 矩阵内存对齐，参考【https://blog.csdn.net/renweiyi1487/article/details/104097576】
  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
  ifstream fin(trajectory_file);
  if (!fin)
  {
    cout << "cannot find trajectory file at " << trajectory_file << endl;
    return 1;
  }

  while (!fin.eof())
  {
    double time, tx, ty, tz, qx, qy, qz, qw;
    fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
    Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
    // 把平移向量设成(tx, ty, tz)
    Twr.pretranslate(Vector3d(tx, ty, tz));
    poses.push_back(Twr);
  }
  cout << "read total " << poses.size() << " pose entries" << endl;

  // draw trajectory in pangolin
  DrawTrajectory(poses);
  return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses)
{
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  // 用来开启更新深度缓冲区的功能，也就是，如果通过比较后深度值发生变化了，会进行更新深度缓冲区的操作。
  // 启动它，OpenGL就可以跟踪再Z轴上的像素，这样，它只会再那个像素前方没有东西时，才会绘画这个像素。
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);                                // 启用混合，用来绘制透明或半透明的物体
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // 混合函数

  // 创建一个观察相机视图(定义目标并初始化)
  pangolin::OpenGlRenderState s_cam(
      // 该函数是生成一个4x4投影矩阵，用于将三维世界坐标系中的点投影到屏幕平面上，
      // 配置相机内参，内参依次为相机的图像宽度,高度，4个相机内参（焦距fx,fy,主点坐标cx,cy）单位为像素|近平面距离/远平面距离，单位为m
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      // 该函数设置模型视图矩阵,内参依次为相机位置在世界坐标系坐标(x,y,z),相机观察的目标点在世界坐标系坐标,相机上向量在世界坐标系中的x,y,z分量
      pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));
  // View &d_cam创建可视化窗口的函数
  pangolin::View &d_cam = pangolin::CreateDisplay() // 创建显示器
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                              // 设置视图的边界,参数l、r、b、t 分别为左右下上边界（取值范围为 [0,1]），aspect为视图的宽高比，即宽度除以高度
                              // 由于 OpenGL 中的投影矩阵是按照近、远平面，左、右、下、上边界的顺序进行设定的，因此 -1024.0f / 768.0f 参数实际上是设定了视图的近平面和远平面的位置
                              .SetHandler(new pangolin::Handler3D(s_cam)); // 设置视图的事件处理程序,Handler3D() 类用于处理三维交互事件

  // 一个全局函数，用于判断当前窗口是否应该退出，返回一个布尔值，当返回值为 true 时，表示当前窗口应该退出，否则应该继续运行
  while (pangolin::ShouldQuit() == false)
  {
    // 用于清空窗口的函数之一，它的参数是一个按位或运算符 "|" 分隔的位掩码，表示需要清空的缓冲区，参数分别是颜色缓冲区，深度缓冲区
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // 函数的作用是将当前可视化视图 d_cam 设置为指定的相机姿态 s_cam，然后进行渲染，可以在多个可视化视图之间切换不同的相机姿态，从而实现多视角的可视化效果
    d_cam.Activate(s_cam);
    // 设置窗口背景颜色的函数之一，四个参数依次表示红色、绿色、蓝色和透明度的值，取值范围为 [0,1]，参数都为 1.0f，表示将窗口的背景颜色设置为白色
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(2);
    for (size_t i = 0; i < poses.size(); i++)
    {
      // 画每个位姿的三个坐标轴
      Vector3d Ow = poses[i].translation();               // 获取第 i 个位姿的平移向量，即位姿矩阵 poses[i]的左下角 3x1 的向量
      Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0)); // 构造一个长度为 0.1 的向量，方向为 x 轴正方向，将向量 (0.1, 0, 0) 从相机坐标系转换到世界坐标系
      Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0)); // 构造一个长度为 0.1 的向量，方向为 y 轴正方向，将向量 (0, 0.1, 0) 从相机坐标系转换到世界坐标系
      Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1)); // 构造一个长度为 0.1 的向量，方向为 z 轴正方向，将向量 (0, 0, 0.1) 从相机坐标系转换到世界坐标系

      glBegin(GL_LINES);               // 开始绘制直线，每两个顶点之间会绘制一条线段
      glColor3f(1.0, 0.0, 0.0);        // 设置当前绘制的直线的颜色为红色
      glVertex3d(Ow[0], Ow[1], Ow[2]); // 添加一个顶点，表示直线的起点，坐标为 Ow，即相机的位置
      glVertex3d(Xw[0], Xw[1], Xw[2]); // 添加一个顶点，表示直线的终点，坐标为 Xw，即相机的右向量

      glColor3f(0.0, 1.0, 0.0);        // 设置当前绘制的直线的颜色为绿色
      glVertex3d(Ow[0], Ow[1], Ow[2]); // 添加一个顶点，表示直线的起点，坐标为 Ow，即相机的位置
      glVertex3d(Yw[0], Yw[1], Yw[2]); // 添加一个顶点，表示直线的终点，坐标为 Yw，即相机的上向量

      glColor3f(0.0, 0.0, 1.0);        // 设置当前绘制的直线的颜色为蓝色
      glVertex3d(Ow[0], Ow[1], Ow[2]); // 添加一个顶点，表示直线的起点，坐标为 Ow，即相机的位置
      glVertex3d(Zw[0], Zw[1], Zw[2]); // 添加一个顶点，表示直线的终点，坐标为 Zw，即相机的前向向量
      glEnd();                         // 结束绘制直线
    }
    // 画出连线
    for (size_t i = 0; i < poses.size(); i++)
    {
      glColor3f(0.0, 0.0, 0.0); // 设置当前绘制的直线的颜色为黑色
      glBegin(GL_LINES);
      auto p1 = poses[i], p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000); // sleep 5 ms
  }
}
