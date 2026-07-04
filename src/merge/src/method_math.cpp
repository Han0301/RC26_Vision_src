#ifndef __METHOD_MATH_CPP_
#define __METHOD_MATH_CPP_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// 引入tf库，用于四元数转欧拉角
#include <tf/transform_datatypes.h>
// 引入数学库，用于弧度转角度（可选）
#include <cmath>
#include <eigen3/Eigen/Core>       // 核心矩阵/向量
#include <eigen3/Eigen/Geometry>   // 几何变换（旋转、平移）
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
// PCL转换头文件
#include <pcl_conversions/pcl_conversions.h>
// PCL点类型和点云头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "method_math.h"

namespace Ten
{

    // struct XYZ
    // {
    //     double _x = 0.0;
    //     double _y = 0.0;
    //     double _z = 0.0;
    // };

    // struct RPY
    // {
    //     double _roll = 0.0;
    //     double _pitch = 0.0;
    //     double _yaw = 0.0;
    // };

    // struct XYZRPY
    // {
    //     XYZ _xyz;
    //     RPY _rpy;
    // };

    /**
        @brief 里程计消息转xyzrpy
        @param msg: nav_msgs::Odometry
        @return xyzrpy
    */
    XYZRPY Nav_Odometrytoxyzrpy(nav_msgs::Odometry msg)
    {
        XYZRPY change;

        // 1. 获取位姿基础数据（位置 + 四元数）
        change._xyz._x = msg.pose.pose.position.x;
        change._xyz._y = msg.pose.pose.position.y;
        change._xyz._z = msg.pose.pose.position.z;

        double ori_x = msg.pose.pose.orientation.x;
        double ori_y = msg.pose.pose.orientation.y;
        double ori_z = msg.pose.pose.orientation.z;
        double ori_w = msg.pose.pose.orientation.w;

        // 2. 四元数转欧拉角（Roll/Pitch/Yaw，单位：弧度）
        // 步骤1：构造tf四元数对象
        tf::Quaternion quat(ori_x, ori_y, ori_z, ori_w);
        // 步骤2：定义存储欧拉角的变量（roll:绕X轴, pitch:绕Y轴, yaw:绕Z轴）
        double roll, pitch, yaw;
        // 步骤3：转换为欧拉角 弧度
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        change._rpy._roll = roll;
        change._rpy._pitch = pitch;
        change._rpy._yaw = yaw;
        return change;
    }

    /**
        @brief 创建旋转矩阵
        @param rx: roll (弧度)
        @param ry: pitch (弧度)
        @param rz: yaw (弧度)
        @return Eigen::Matrix3d: 3x3的旋转矩阵
    */
    Eigen::Matrix3d createRotationMatrix(double rx, double ry, double rz) {
        // 弧度
        // 创建绕各轴的旋转矩阵
        Eigen::Matrix3d R_x;
        R_x << 1, 0, 0,
            0, cos(rx), -sin(rx),
            0, sin(rx), cos(rx);
        Eigen::Matrix3d R_y;
        R_y << cos(ry), 0, sin(ry),
            0, 1, 0,
            -sin(ry), 0, cos(ry);
        Eigen::Matrix3d R_z;
        R_z << cos(rz), -sin(rz), 0,
            sin(rz), cos(rz), 0,
            0, 0, 1;
        // 组合旋转矩阵 (Z-Y-X顺序: R = R_z * R_y * R_x)
        return R_z * R_y * R_x;
    }
    /**
        @brief 创建平移矩阵
        @param tx: x (米)
        @param ty: y (米)
        @param tz: z (米)
        @return Eigen::Vector3d: 1x3平移矩阵
    */
    Eigen::Vector3d createTranslationVector(double tx, double ty, double tz) {
        Eigen::Vector3d translation(tx, ty, tz);
        return translation;
    }
    /**
        @brief 创建旋转矩阵
        @param rotation: 旋转矩阵
        @param translation: 平移矩阵
        @return Eigen::Matrix4d: 4x4的RT矩阵
    */
    //组合现有旋转矩阵与平移向量
    Eigen::Matrix4d combineRotationAndTranslation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = rotation;
        transform.block<3, 1>(0, 3) = translation;
        return transform;
    }

    /**
        @brief 世界到当前 点变化
        @param xyz：当前点在世界坐标系的xyz坐标
        @param rpy: 当前点相对于世界坐标系的旋转和平移
    */
    Eigen::Matrix4d worldtocurrent(XYZ xyz, RPY rpy)
    {
        Eigen::Matrix3d rot = createRotationMatrix(-rpy._roll, -rpy._pitch, -rpy._yaw);
        Eigen::Vector3d tra = -(rot*createTranslationVector(xyz._x, xyz._y, xyz._z));
        Eigen::Matrix4d T = combineRotationAndTranslation(rot, tra);
        return T;
    }

    /**
        @brief 当前到世界 点变化
        @param xyz：当前点在世界坐标系的xyz坐标
        @param rpy: 当前点相对于世界坐标系的旋转和平移
    */
    Eigen::Matrix4d currenttoworld(XYZ xyz, RPY rpy)
    {
        Eigen::Matrix4d T = worldtocurrent(xyz, rpy);
        return T.inverse();
    }




    /**
        @brief 把Eigen::Matrix3d转cv::Mat rvec
        @param R: 旋转矩阵
        @return cv::Mat：rvec
    */
    cv::Mat RotationMatrixtorvec(Eigen::Matrix3d R)
    {
        cv::Mat rot_matrix = (cv::Mat_<double>(3, 3) <<
        R(0,0), R(0,1), R(0,2),
        R(1,0), R(1,1), R(1,2),
        R(2,0), R(2,1), R(2,2));
        cv::Mat rvec;
        cv::Rodrigues(rot_matrix, rvec);
        return rvec;
    }

    /**
        @brief 把cv::Mat rvec转 Eigen::Matrix3d
        @param rvec: 旋转向量 cv::Mat
        @return Eigen::Matrix3d：R
    */
    Eigen::Matrix3d rvectoRotationMatrix(cv::Mat rvec)
    {
        if(rvec.rows != 3 && rvec.cols != 1)
        {
            std::cout<< "error! Eigen::Matrix3d RotationMatrixtorvec(cv::Mat rvec)" << std::endl;
            return Eigen::Matrix3d();
        }
        cv::Mat rot_matrix;
        cv::Rodrigues(rvec, rot_matrix);
        //std::cout<<rot_matrix<<std::endl;
        Eigen::Matrix3d R;
        for(int i = 0; i < 3; i ++)
        {
            for(int j = 0; j < 3; j++)
            {
                R(i,j) = rot_matrix.at<double>(i, j);
            }
        }
        return R;
    }



    // /**
    //  * @brief 点云回调函数：将sensor_msgs::PointCloud2转为PointT
    //  * @param msg 订阅到的sensor_msgs/PointCloud2消息
    //  * @return pcl::PointCloud<PointT>::Ptr 对象的智能指针
    //  */
    // template<typename PointT>
    // typename pcl::PointCloud<PointT>::Ptr sensor_msgs_PointCloud2topcltpye(const sensor_msgs::PointCloud2ConstPtr& msg)
    // {
    //     // 声明PCL点云对象（智能指针避免内存泄漏）  修复：添加 typename 声明嵌套类型
    //     typename pcl::PointCloud<PointT>::Ptr cloud_xyz_inormal(new pcl::PointCloud<PointT>);
    //     try
    //     {
    //         // 核心转换函数：ROS PointCloud2 -> PCL PointXYZINormal
    //         pcl::fromROSMsg(*msg, *cloud_xyz_inormal);
    //         return cloud_xyz_inormal;
    //     }
    //     catch (const std::exception& e)
    //     {
    //         // 捕获转换异常（如字段缺失、类型不匹配）
    //         std::cout << e.what() << std::endl;
    //         return nullptr;
    //     } 
    // }


    //#define EPS 1e-6
    /**
     * @brief Eigen旋转矩阵转欧拉角（Z-Y-X 顺序：Yaw-Pitch-Roll）
     * @param R 3x3 Eigen旋转矩阵
     * @return RPY: roll pitch yaw (弧度)
     */
    RPY rotationMatrixToEulerAngles(const Eigen::Matrix3d R)
    {
        const double EPS = 1e-6;
        double roll = 0.0;
        double pitch  = 0.0;
        double yaw = 0.0;

        double sin_pitch = -R(2, 0);
        sin_pitch = std::max(std::min(sin_pitch, 1.0), -1.0);
        pitch = std::asin(sin_pitch);

        double cos_pitch = std::cos(pitch);

        if (std::abs(cos_pitch) > EPS) {
            roll = std::atan2(R(2, 1), R(2, 2));
            yaw = std::atan2(R(1, 0), R(0, 0));
        } else {
            yaw = 0.0;
            if (pitch > 0) {
                roll = std::atan2(-R(0, 1), R(0, 2));
            } else {
                roll = std::atan2(R(0, 1), -R(0, 2));
            }
        }
        RPY rpy;
        rpy._roll = -roll;
        rpy._pitch = -pitch;
        rpy._yaw = -yaw;
        return rpy;
    }


    /**
     * @brief 将变化矩阵转为XYZRPY
     * @param transform_matrix： 变换矩阵
     */
    XYZRPY transform_matrixtoXYZRPY(Eigen::Matrix4d transform_matrix)
    {
        Eigen::Matrix3d R = transform_matrix.block<3, 3>(0, 0).cast<double>();
        Eigen::Vector3d t = transform_matrix.block<3, 1>(0, 3).cast<double>();
        RPY rpy = rotationMatrixToEulerAngles(R);
        Eigen::Vector3d ttt = - R.inverse() * t;
        XYZRPY xyzrpy;
        xyzrpy._xyz._x = ttt[0];
        xyzrpy._xyz._y = ttt[1];
        xyzrpy._xyz._z = ttt[2];

        xyzrpy._rpy = rpy;
        return xyzrpy;
    }

    /**
     * @brief 将XYZRPY转为变化矩阵
     * @param xyzrpy： 旋转和平移
     */
    Eigen::Matrix4d XYZRPYtotransform_matrix(XYZRPY xyzrpy)
    {
        Eigen::Matrix3d rot = createRotationMatrix(-xyzrpy._rpy._roll, -xyzrpy._rpy._pitch, -xyzrpy._rpy._yaw);
        Eigen::Vector3d tra = -(rot*createTranslationVector(xyzrpy._xyz._x, xyzrpy._xyz._y, xyzrpy._xyz._z));
        Eigen::Matrix4d T = combineRotationAndTranslation(rot, tra);
        return T;
    }

    /**
     * @brief 平移向量 vector3dtotevc
     * @param T: 平移向量
     * @return cv::Mat:平移向量
     */
    cv::Mat vector3dtotvec(Eigen::Vector3d T)
    {
        cv::Mat tvec;
        //tevc = cv::Mat(3, 1, CV_32FC1, T.data()).clone();
        tvec = (cv::Mat_<double>(3, 1) << T(0), T(1), T(2));
        return tvec;
    }

    /**
     * @brief 平移向量 tvec转Eigen::Vector3d
     * @param tvec: 输入3x1的cv::Mat格式平移向量（CV_32FC1类型）
     * @return Eigen::Vector3d: 输出Eigen::格式平移向量
     */
    Eigen::Vector3d tvectovector3d(cv::Mat tvec)
    {
        // 第一步：校验tvec维度必须是3行1列，和你之前的rvec/tvec判断逻辑一致
        if (tvec.rows != 3 || tvec.cols != 1)
        {
            std::cerr << "error! tvectovector3d: tvec must be 3x1 cv::Mat" << std::endl;
            return Eigen::Vector3d::Zero(); // 维度错误返回零向量，避免程序崩溃
        }
        // 第三步：tvec转Eigen::Vector3d（两种写法，选一种即可，效果一致）
        // 写法1：逐元素赋值，和你原函数vector3dtotvec的写法对称，直观易读
        Eigen::Vector3d T;
        T(0) = tvec.at<double>(0, 0);
        T(1) = tvec.at<double>(1, 0);
        T(2) = tvec.at<double>(2, 0);
        return T;
    }

    /**
     * @brief 画调试图像
     * @param image: 图像
     * @param imagePoints：2d点对
     */
    void debug_draw_img(cv::Mat& image, std::vector<cv::Point2f>& imagePoints)
    {
        for (int i = 0; i < imagePoints.size(); i += 4) 
        {
            cv::line(image, cv::Point(cvRound(imagePoints[i].x), cvRound(imagePoints[i].y)),cv::Point(cvRound(imagePoints[i+1].x), cvRound(imagePoints[i+1].y)),cv::Scalar(0, 0, 255), 2);
            cv::line(image, cv::Point(cvRound(imagePoints[i+1].x), cvRound(imagePoints[i+1].y)),cv::Point(cvRound(imagePoints[i+2].x), cvRound(imagePoints[i+2].y)),cv::Scalar(0, 0, 255), 2);
            cv::line(image, cv::Point(cvRound(imagePoints[i+2].x), cvRound(imagePoints[i+2].y)),cv::Point(cvRound(imagePoints[i+3].x), cvRound(imagePoints[i+3].y)),cv::Scalar(0, 0, 255), 2);
            cv::line(image, cv::Point(cvRound(imagePoints[i+3].x), cvRound(imagePoints[i+3].y)),cv::Point(cvRound(imagePoints[i].x), cvRound(imagePoints[i].y)),cv::Scalar(0, 0, 255), 2);
        } 
    }

    /**
     * @brief 获取路径
     * @param txt_path: 文件路径
     * @param map: 输入地图
     * @param path： 输出路径
     * @return bool: 读取是否成功
     */
    bool getpath(std::string txt_path, std::vector<int>& map, std::vector<int>& path)
    {
        //先清理
        path.clear();
        // 创建一个map容器，键为string类型(编号)，值为vector<int>(数字集合)
        // 用于存储文件中读取的编号及其对应的数字
        std::map<std::string, std::vector<int>> data;

        // 创建输入文件流对象，并尝试打开名为"data.txt"的文件
        std::ifstream file(txt_path);

        // 检查文件是否成功打开
        if (!file.is_open()) {
            // 如果文件无法打开，输出错误信息到标准错误流
            std::cerr << "无法打开文件" << std::endl;
            // 返回1表示程序异常退出
            return false;
        }
        if(map.size()!= 12)
        {
            return false;
        }

        // 定义一个字符串变量line，用于存储从文件中读取的每一行
        std::string line;

        // 循环读取文件中的每一行，直到文件末尾
        while (std::getline(file, line)) {
            // 创建字符串流对象iss，并将当前行内容传入
            std::istringstream iss(line);
            // 定义字符串变量id，用于存储编号
            std::string id;
            // 从字符串流中读取第一个字符串作为编号
            iss >> id;

            // 创建vector<int>容器nums，用于存储当前编号对应的数字
            std::vector<int> nums;
            // 定义整数变量num，用于临时存储读取的数字
            int num;

            // 从字符串流中继续读取整数，直到行尾
            while (iss >> num) {
                // 将读取到的数字添加到vector容器中
                nums.push_back(num);
            }

            // 将编号及其对应的数字集合存入map容器
            data[id] = nums;
        }

        // 关闭文件流
        file.close();
        // 定义字符串变量inputId，用于存储用户输入的编号
        //std::string inputId("242144342121");
        std::string inputId;
        for(int i = 0; i < map.size(); i++)
        {
            inputId += map[i] + '0';
        }
        // 创建一个包含12个整数的数组arr，并初始化为0
        //int arr[12] = { 0 };

        // 在map中查找用户输入的编号
        auto it = data.find(inputId);

        // 如果找到了对应的编号
        if (it != data.end()) {

            for (size_t i = 0; i < it->second.size(); ++i) {
                //path[i] = it->second[i];
                path.push_back(it->second[i]);
            }

            // 输出找到的编号及其对应的数组
            std::cout << "找到编号 " << inputId << "，对应的数组为: ";
            for (int i = 0; i < path.size(); ++i) {
                std::cout << path[i] << " ";
            }
            std::cout << std::endl;
            return true;
        }
        else {
            // 如果未找到编号，输出提示信息
            std::cout << "未找到编号 " << inputId << std::endl;
            //std::exit(EXIT_SUCCESS);
            return false;
        }
    }

    /**
     * @brief 读取txt文件并返回XYZRPY数组（修复版）
     * @param filePath: txt的文件路径
     * @return std::vector<Ten::XYZRPY>: Ten::XYZRPY的容器
     */
    std::vector<Ten::XYZRPY> readPoseFromTxt(const std::string& filePath)
    {
        std::vector<Ten::XYZRPY> poseList;
        std::ifstream file(filePath);

        // 检查文件是否成功打开
        if (!file.is_open())
        {
            std::cerr << "Error: 无法打开文件 " << filePath << "，原因：" << strerror(errno) << std::endl;
            return poseList;
        }

        std::string line;
        Ten::XYZRPY currentPose;
        bool isParsingPose = false; // 标记是否正在解析Pose段
        // 新增：标记是否已解析到有效Pose数据（避免空数据块）
        bool hasParsedPoseData = false;

        // 逐行读取文件
        while (std::getline(file, line))
        {
            // 去除行首尾的空白字符（空格、换行、制表符等）
            line.erase(0, line.find_first_not_of(" \t\n\r"));
            line.erase(line.find_last_not_of(" \t\n\r") + 1);

            // 跳过空行
            if (line.empty())
                continue;

            // 关键修复1：检测时间戳行（新Pose块开始），保存上一个Pose数据
            // 时间戳特征：包含 "-" 和 ":"，且长度符合 1970-01-01 08:00:00.000000000
            if (line.find("-") != std::string::npos && line.find(":") != std::string::npos && line.length() >= 26)
            {
                // 如果上一个Pose块有有效数据，先保存
                if (hasParsedPoseData)
                {
                    poseList.push_back(currentPose);
                    // 重置当前Pose和标记
                    currentPose = Ten::XYZRPY();
                    hasParsedPoseData = false;
                }
                continue;
            }

            // 检测Pose段开始
            if (line == "Pose:")
            {
                isParsingPose = true;
                continue;
            }

            // 检测Speed段开始（Pose段结束）
            if (line == "Speed:")
            {
                isParsingPose = false;
                // 关键修复2：Pose段结束时标记数据有效
                hasParsedPoseData = true;
                continue;
            }

            // 只处理Pose段内的行
            if (isParsingPose)
            {
                // 分割键值对（格式：key: value）
                size_t colonPos = line.find(':');
                if (colonPos == std::string::npos)
                    continue;

                std::string key = line.substr(0, colonPos);
                std::string valueStr = line.substr(colonPos + 1);
                // 去除valueStr首尾空白
                valueStr.erase(0, valueStr.find_first_not_of(" \t"));
                valueStr.erase(valueStr.find_last_not_of(" \t") + 1);

                double value = 0.0;
                // 处理nan/-nan的情况
                if (valueStr == "nan" || valueStr == "-nan")
                {
                    value = std::numeric_limits<double>::quiet_NaN();
                }
                else
                {
                    // 转换普通数值（增加失败检测）
                    std::istringstream valueStream(valueStr);
                    if (!(valueStream >> value))
                    {
                        std::cerr << "警告：解析数值失败，行内容：" << line << std::endl;
                        continue;
                    }
                }

                // 赋值到对应的字段
                if (key == "x")
                    currentPose._xyz._x = value;
                else if (key == "y")
                    currentPose._xyz._y = value;
                else if (key == "z")
                    currentPose._xyz._z = value;
                else if (key == "roll")
                    currentPose._rpy._roll = value;
                else if (key == "pitch")
                    currentPose._rpy._pitch = value;
                else if (key == "yaw")
                    currentPose._rpy._yaw = value;
                
                // 标记已解析到有效数据
                hasParsedPoseData = true;
            }
        }

        // 处理文件末尾最后一个未保存的Pose
        if (hasParsedPoseData)
        {
            poseList.push_back(currentPose);
        }

        file.close();
        // 调试：打印解析结果数量（可选）
        std::cout << "解析完成，共读取到 " << poseList.size() << " 个Pose数据" << std::endl;
        return poseList;
    }
    

    /**
     * @brief 核心函数：读取单行数字txt文件，将每个字符转为整数并存储到vector<int>中
     * @param filePath 传入的txt文件路径（字符串常量引用，避免拷贝）
     * @return std::vector<int> 存储拆分后整数的容器
     */
    std::vector<int> readNumberFile(const std::string& filePath) 
    {
        // 定义整型向量，用于存储最终拆分后的数字
        std::vector<int> numVec;

        // 以只读模式打开指定路径的txt文件
        std::ifstream file(filePath);

        // 判断文件是否成功打开（文件不存在/权限不足会打开失败）
        if (!file.is_open()) {
            // 控制台输出错误信息
            std::cerr << "错误：无法打开文件 " << filePath << std::endl;
            // 返回空向量，终止函数
            return numVec;
        }

        // 定义字符串变量，存储读取到的文件一行内容
        std::string content;
        // 读取文件中唯一的一行文本，存入content字符串
        std::getline(file, content);

        // 遍历字符串中的每一个字符（即每个数字字符）
        for (char ch : content) {
            // 字符数字转整型数字：'0'的ASCII码是48，字符减'0'得到对应整数
            int num = ch - '0';
            std::cout << num << " ";
            // 将转换后的整数添加到向量末尾
            numVec.push_back(num);
        }
        std::cout << std::endl;

        // 读取完成，关闭文件（释放资源）
        file.close();

        // 返回存储好数字的向量
        return numVec;
    }




}




#endif

