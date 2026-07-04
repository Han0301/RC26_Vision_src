#ifndef __DETECTOR_H_
#define __DETECTOR_H_

#include "segmentation.h"


namespace Ten
{

    class detector
    {
    public:

        /**
         * @brief 按序号批量保存图片，序号从txt读写，支持断点续存
         * @param images 待保存的图像集合
         */
        void saveimg(const std::vector<cv::Mat>& images)
        {
            saveImagesWithIndex(images, std::string(ROOT_DIR) + std::string("src/nine/image"), std::string(ROOT_DIR) + std::string("src/nine/total.txt"));

        }

    private:
        /**
         * @brief 检查目录是否存在，不存在则创建（Linux环境单级目录，父目录需已存在）
         * @param folder_path 目录路径
         * @return 存在或创建成功返回true
         */
        static bool ensureFolderExists(const std::string& folder_path)
        {
            struct stat st;
            // 检查路径是否存在且为目录
            if (stat(folder_path.c_str(), &st) == 0)
            {
                return S_ISDIR(st.st_mode);
            }
            // 不存在则创建，权限0755
            return mkdir(folder_path.c_str(), 0755) == 0;
        }

        /**
         * @brief 按序号批量保存图片，序号从txt读写，支持断点续存
         * @param images 待保存的图像集合
         * @param save_folder 图片保存的文件夹路径
         * @param index_txt_path 序号存储txt文件路径，文件内仅存一个起始整数
         * @return 成功保存的图片数量，目录创建失败返回-1
         */
        int saveImagesWithIndex(const std::vector<cv::Mat>& images, 
                                const std::string& save_folder, 
                                const std::string& index_txt_path)
        {
            // ===================== 1. 读取起始序号 =====================
            int current_index = 0;
            std::ifstream ifs_index(index_txt_path);
            if (ifs_index.is_open())
            {
                // 读取第一个整数，读取失败（空文件/非数字）则保持默认0
                if (!(ifs_index >> current_index))
                {
                    current_index = 0;
                }
                ifs_index.close();
            }
            // 文件打不开（不存在）时默认从0开始

            // 序号合法性保护，避免负数
            if (current_index < 0)
            {
                current_index = 0;
            }

            // ===================== 2. 确保保存目录存在 =====================
            if (!ensureFolderExists(save_folder))
            {
                std::cerr << "错误：无法创建保存目录 " << save_folder << std::endl;
                return -1;
            }

            // ===================== 3. 批量保存图片 =====================
            int success_count = 0;
            for (size_t i = 0; i < images.size(); ++i)
            {
                const cv::Mat& img = images[i];
                if (img.empty())
                {
                    std::cerr << "警告：第" << i << "张图像为空，跳过保存" << std::endl;
                    continue;
                }

                // 格式化文件名：4位零填充 + .png，例如 0000.png
                std::ostringstream oss;
                oss << std::setw(4) << std::setfill('0') << current_index << ".png";
                std::string full_path = save_folder + "/" + oss.str();

                // 保存图片
                if (cv::imwrite(full_path, img))
                {
                    success_count++;
                    current_index++; // 成功保存才自增序号
                }
                else
                {
                    std::cerr << "错误：保存图片失败 " << full_path << std::endl;
                }
            }

            // ===================== 4. 更新序号写回txt =====================
            std::ofstream ofs_index(index_txt_path, std::ios::trunc); // 覆盖原内容写入
            if (ofs_index.is_open())
            {
                ofs_index << current_index;
                ofs_index.close();
            }
            else
            {
                std::cerr << "警告：无法写入序号文件 " << index_txt_path << "，序号未更新" << std::endl;
            }

            std::cout << "保存完成：成功保存 " << success_count << " 张图片，当前最新序号：" << current_index << std::endl;
            return success_count;
        }
    };


}

#endif
