#ifndef __ORB_DEBUG_CPP_
#define __ORB_DEBUG_CPP_


#include"orb_exhaust.h"
#include"json.hpp"
#include <filesystem>
#include <fstream>
#include <regex>
#include <stdexcept>

namespace Ten
{

    namespace ORB
    {

        /**
         * @brief 工具函数：从 image_x.png 中提取数字序号 x
         * @param filename 文件名（如 image_123.png）
         * @return 序号（123），失败返回-1
         */
        int extract_image_index(const std::string& filename) {
            std::regex pattern(R"(image_(\d+)\.png)");
            std::smatch match;
            if (std::regex_search(filename, match, pattern) && match.size() > 1) {
                return std::stoi(match[1].str());
            }
            return -1;
        }

        /**
         * @brief 加载数据集主函数
         * @param root_dir 总文件夹路径（如 "test"）
         * @return 外层vector：子文件夹1/2/3...；内层vector：每个文件对应的元素
         */
        std::vector<std::vector<debug_orb_exhaust_element>> load_exhaust_dataset(const std::string& root_dir)
        {
            std::vector<std::vector<debug_orb_exhaust_element>> result;

            // 1. 检查总文件夹是否存在
            if (!std::filesystem::exists(root_dir) || !std::filesystem::is_directory(root_dir)) {
                throw std::runtime_error("总文件夹不存在：" + root_dir);
            }

            // 2. 收集所有【数字命名】的子文件夹 (1,2,3...)，并按数字排序
            std::vector<int> sub_dir_indices;
            std::unordered_map<int, std::filesystem::path> sub_dir_map;

            for (const auto& entry : std::filesystem::directory_iterator(root_dir)) {
                if (!entry.is_directory()) continue;

                std::string dir_name = entry.path().filename().string();
                try {
                    int idx = std::stoi(dir_name);
                    sub_dir_indices.push_back(idx);
                    sub_dir_map[idx] = entry.path();
                } catch (...) {
                    continue;
                }
            }

            std::sort(sub_dir_indices.begin(), sub_dir_indices.end());

            // 3. 遍历每个子文件夹，加载数据
            for (int dir_idx : sub_dir_indices) {
                const std::filesystem::path& sub_dir = sub_dir_map[dir_idx];
                std::vector<debug_orb_exhaust_element> sub_folder_data;

                for (const auto& entry : std::filesystem::directory_iterator(sub_dir)) {
                    if (entry.path().extension() != ".png") continue;

                    std::string img_name = entry.path().filename().string();
                    int img_idx = extract_image_index(img_name);
                    if (img_idx == -1) continue;

                    std::filesystem::path json_path = sub_dir / ("label_" + std::to_string(img_idx) + ".json");
                    if (!std::filesystem::exists(json_path)) {
                        std::cerr << "警告：未找到匹配文件 → " << json_path << std::endl;
                        continue;
                    }

                    // 填充结构体
                    debug_orb_exhaust_element element;

                    // 读取图像
                    cv::Mat image = cv::imread(entry.path().string(), cv::IMREAD_COLOR);
                    if (image.empty()) {
                        std::cerr << "警告：图像读取失败 → " << entry.path() << std::endl;
                        continue;
                    }
                    element.oee.image_ = image.clone();

                    // 解析JSON
                    std::ifstream json_file(json_path);
                    nlohmann::json j;
                    try {
                        json_file >> j;
                    } catch (...) {
                        std::cerr << "警告：JSON解析失败 → " << json_path << std::endl;
                        continue;
                    }

                    // 赋值旋转向量 rvec
                    auto rvec_data = j["rvec"];
                    element.oee.rvec_ = (cv::Mat_<double>(3, 1) <<
                        rvec_data[0].get<double>(),
                        rvec_data[1].get<double>(),
                        rvec_data[2].get<double>()
                    );

                    // 赋值平移向量 tvec
                    auto tvec_data = j["tvec"];
                    element.oee.tvec_ = (cv::Mat_<double>(3, 1) <<
                        tvec_data[0].get<double>(),
                        tvec_data[1].get<double>(),
                        tvec_data[2].get<double>()
                    );

                    // 赋值标签
                    element.label = j["labels"].get<std::vector<int>>();

                    sub_folder_data.push_back(element);
                }

                result.push_back(sub_folder_data);
                std::cout << "已加载子文件夹 " << dir_idx << "，共 " << sub_folder_data.size() << " 个元素" << std::endl;
            }

            return result;
        }


        // ====================== 核心函数 ======================
        /**
         * @brief 生成JSON文件，label数组单行显示，loss单独一行
         * @param root_dir 主文件夹路径（如 test）
         * @param i 文件名序号，生成 i.json
         * @param label 标签向量（单行显示）
         * @param loss 损失值
         */
        void save_loss_label_to_json(const std::string& root_dir, int i, const std::vector<int>& label, double loss)
        {
            // 1. 确保文件夹存在
            if (!std::filesystem::exists(root_dir))
            {
                if (!std::filesystem::create_directories(root_dir))
                {
                    throw std::runtime_error("无法创建主文件夹：" + root_dir);
                }
            }

            // 2. 拼接文件路径 test/i.json
            std::filesystem::path json_file_path = std::filesystem::path(root_dir) / (std::to_string(i) + ".json");
            std::ofstream ofs(json_file_path);
            if (!ofs.is_open())
            {
                throw std::runtime_error("无法打开文件：" + json_file_path.string());
            }

            // 3. 手动格式化输出（label 单行，loss 单独行）
            nlohmann::json j;
            j["label"] = label;
            j["loss"] = loss;

            // 核心：自定义输出格式，数组紧凑一行，对象缩进格式化
            ofs << "{" << std::endl;
            ofs << "    \"label\": " << j["label"].dump() << "," << std::endl;
            ofs << "    \"loss\": " << j["loss"].dump() << std::endl;
            ofs << "}" << std::endl;

            ofs.close();
            std::cout << "文件已保存：" << json_file_path.string() << std::endl;
        }



    }

}





#endif