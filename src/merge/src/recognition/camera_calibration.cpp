#ifndef __CAMERA_CALIBRATION_CPP_
#define __CAMERA_CALIBRATION_CPP_

#include "./camera_calibration.h"


namespace Ten
{


    //// 工具函数：判断字符串是否为纯数字（用于筛选数字命名的文件夹）
    // bool isNumberString(const std::string& str)
    // {
    //     for (char c : str)
    //     {
    //         if (!isdigit(c))
    //             return false;
    //     }
    //     return !str.empty();
    // }

    // // 工具函数：解析一行逗号分隔的3个数值为cv::Mat(3x1)
    // bool parseLineTo3x1Mat(const std::string& line, cv::Mat& outMat)
    // {
    //     outMat = cv::Mat::zeros(3, 1, CV_64F);
    //     std::stringstream ss(line);
    //     std::string token;
    //     int idx = 0;
        
    //     while (std::getline(ss, token, ',') && idx < 3)
    //     {
    //         // 去除token首尾空格
    //         token.erase(0, token.find_first_not_of(" \t"));
    //         token.erase(token.find_last_not_of(" \t") + 1);
            
    //         try {
    //             double val = std::stod(token);
    //             outMat.at<double>(idx, 0) = val;
    //             idx++;
    //         } catch (...) {
    //             std::cerr << "数值解析失败：" << token << std::endl;
    //             return false;
    //         }
    //     }
        
    //     return idx == 3; // 必须解析出3个数值
    // }
    
    

    // // 核心函数：读取rt.txt文件并返回对应的Ten_camerainfo
    // bool readRTFile(const std::string& rtFilePath, Ten::Ten_camerainfo& camInfo)
    // {
    //     std::ifstream file(rtFilePath);
    //     if (!file.is_open())
    //     {
    //         std::cerr << "无法打开文件：" << rtFilePath << "，原因：" << strerror(errno) << std::endl;
    //         return false;
    //     }

    //     std::string line;
    //     cv::Mat rvec, tvec;
        
    //     // 读取第一行（R）
    //     if (!std::getline(file, line))
    //     {
    //         std::cerr << "rt.txt文件无足够行：" << rtFilePath << std::endl;
    //         file.close();
    //         return false;
    //     }
    //     if (!parseLineTo3x1Mat(line, rvec))
    //     {
    //         std::cerr << "R行解析失败：" << rtFilePath << std::endl;
    //         file.close();
    //         return false;
    //     }

    //     // 读取第二行（T）
    //     if (!std::getline(file, line))
    //     {
    //         std::cerr << "rt.txt文件无T行：" << rtFilePath << std::endl;
    //         file.close();
    //         return false;
    //     }
    //     if (!parseLineTo3x1Mat(line, tvec))
    //     {
    //         std::cerr << "T行解析失败：" << rtFilePath << std::endl;
    //         file.close();
    //         return false;
    //     }

    //     file.close();
    //     // 设置RT参数
    //     camInfo.set_RT(rvec, tvec);
    //     return true;
    // }

    // /**
    //  * @brief 核心函数：遍历文件夹并返回Ten_camerainfo容器
    //  * @param rootDir: 总目录的文件夹路径
    //  * @return std::vector<Ten::Ten_camerainfo>
    //  */
    // std::vector<Ten::Ten_camerainfo> readCameraInfosFromDir(const std::string& rootDir)
    // {
    //     std::vector<Ten::Ten_camerainfo> camInfoList;
    //     std::vector<std::pair<int, std::string>> numDirs; // 存储<数字, 文件夹路径>

    //     // 打开根目录
    //     DIR* dir = opendir(rootDir.c_str());
    //     if (!dir)
    //     {
    //         std::cerr << "无法打开目录：" << rootDir << "，原因：" << strerror(errno) << std::endl;
    //         return camInfoList;
    //     }

    //     // 遍历目录项
    //     struct dirent* entry;
    //     while ((entry = readdir(dir)) != nullptr)
    //     {
    //         // 跳过.和..
    //         if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
    //             continue;

    //         // 检查是否为目录且名称为纯数字
    //         if (entry->d_type == DT_DIR && isNumberString(entry->d_name))
    //         {
    //             int dirNum = atoi(entry->d_name);
    //             std::string dirPath = rootDir + "/" + entry->d_name;
    //             numDirs.emplace_back(dirNum, dirPath);
    //         }
    //     }
    //     closedir(dir);

    //     // 按数字升序排序文件夹
    //     std::sort(numDirs.begin(), numDirs.end(), 
    //         [](const std::pair<int, std::string>& a, const std::pair<int, std::string>& b) {
    //             return a.first < b.first;
    //         });

    //     // 逐个处理数字文件夹
    //     for (const auto& [dirNum, dirPath] : numDirs)
    //     {
    //         std::string rtFilePath = dirPath + "/rt.txt";
    //         Ten::Ten_camerainfo camInfo;
            
    //         if (readRTFile(rtFilePath, camInfo))
    //         {
    //             camInfoList.push_back(camInfo);
    //             std::cout << "成功解析文件夹 " << dirNum << " 的rt.txt" << std::endl;
    //         }
    //         else
    //         {
    //             std::cerr << "解析文件夹 " << dirNum << " 的rt.txt失败" << std::endl;
    //         }
    //     }

    //     return camInfoList;
    // }


}



#endif



