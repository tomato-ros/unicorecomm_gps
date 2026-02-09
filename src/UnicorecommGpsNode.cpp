#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include "unicorecomm_gps/UnicorecommGpsNode.hpp"

namespace unicorecomm_gps_node {
    UnicorecommGpsNode::UnicorecommGpsNode(const rclcpp::NodeOptions &options)
        : Node("unicorecomm_gps_node", options),
          serial_fd_(-1),
          running_(false) {
        // 声明并获取参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);
        this->declare_parameter<std::string>("frame_id", "gnss_link");
        this->declare_parameter<bool>("publish.gps.fix", true);
        this->declare_parameter<bool>("publish.gps.time_reference", true);
        this->declare_parameter<bool>("publish.gps.velocity", true);
        this->declare_parameter<bool>("publish.nmea.sentence", true);
        this->declare_parameter<bool>("publish.nmea.gpgsa", true);
        this->declare_parameter<bool>("publish.nmea.gpgsv", true);
        this->declare_parameter<bool>("publish.nmea.gprmc", true);

        this->get_parameter("serial_port", serial_port_);
        this->get_parameter("baud_rate", baud_rate_);
        this->get_parameter("frame_id", frame_id_);

        this->get_parameter("publish.gps.fix", publish_gps_fix_);
        this->get_parameter("publish.gps.time_reference", publish_gps_time_reference_);
        this->get_parameter("publish.gps.velocity", publish_gps_velocity_);
        this->get_parameter("publish.nmea.sentence", publish_gps_nmea_sentence_);
        this->get_parameter("publish.nmea.gpgsa", publish_gps_nmea_gpgsa_);
        this->get_parameter("publish.nmea.gpgsv", publish_gps_nmea_gpgsv_);
        this->get_parameter("publish.nmea.gprmc", publish_gps_nmea_gprmc_);

        // 打印参数日志（使用RCLCPP_INFO输出到控制台）
        RCLCPP_INFO(this->get_logger(), "串口端口: %s", serial_port_.c_str());
        RCLCPP_INFO(this->get_logger(), "波特率: %d", baud_rate_);
        RCLCPP_INFO(this->get_logger(), "坐标系ID: %s", frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "发布GPS定位: %s", publish_gps_fix_ ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "发布时间参考: %s", publish_gps_time_reference_ ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "发布速度信息: %s", publish_gps_velocity_ ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "发布NMEA通用句子: %s", publish_gps_nmea_sentence_ ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "发布GPGSA句子: %s", publish_gps_nmea_gpgsa_ ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "发布GPGSV句子: %s", publish_gps_nmea_gpgsv_ ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "发布GPRMC句子: %s", publish_gps_nmea_gprmc_ ? "是" : "否");

        // 创建发布器
        gps_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
        time_ref_pub_ = this->create_publisher<sensor_msgs::msg::TimeReference>("gps/time_reference", 10);
        gps_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("gps/velocity", 10);

        // NMEA原始语句
        nmea_sentence_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>("nmea/sentence", 10);

        // NMEA特定消息发布器
        gpgsa_pub_ = this->create_publisher<nmea_msgs::msg::Gpgsa>("nmea/gpgsa", 10);
        gpgsv_pub_ = this->create_publisher<nmea_msgs::msg::Gpgsv>("nmea/gpgsv", 10);
        gprmc_pub_ = this->create_publisher<nmea_msgs::msg::Gprmc>("nmea/gprmc", 10);

        // 初始化串口
        if (init_serial()) {
            RCLCPP_INFO(this->get_logger(), "串口初始化成功: %s, 波特率: %d", serial_port_.c_str(), baud_rate_);

            // 启动读取线程
            running_ = true;

            read_thread_ = std::thread(&UnicorecommGpsNode::read_serial_data, this);
        } else {
            RCLCPP_ERROR(this->get_logger(), "串口初始化失败: %s", serial_port_.c_str());
        }
    }

    UnicorecommGpsNode::~UnicorecommGpsNode() {
        // 停止线程
        running_ = false;

        if (read_thread_.joinable()) {
            read_thread_.join();
        }

        // 关闭串口
        if (serial_fd_ != -1) {
            close(serial_fd_);

            RCLCPP_INFO(this->get_logger(), "串口已关闭");
        }
    }

    bool UnicorecommGpsNode::init_serial() {
        // 打开串口
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "无法打开串口: %s, 错误: %s", serial_port_.c_str(), strerror(errno));

            return false;
        }

        // 配置串口
        struct termios tty;

        memset(&tty, 0, sizeof(tty));

        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "获取串口属性失败, 错误: %s", strerror(errno));

            close(serial_fd_);

            serial_fd_ = -1;

            return false;
        }

        // 设置波特率
        speed_t baud;
        switch (baud_rate_) {
            case 9600: baud = B9600;
                break;
            case 19200: baud = B19200;
                break;
            case 38400: baud = B38400;
                break;
            case 57600: baud = B57600;
                break;
            case 115200: baud = B115200;
                break;
            case 230400: baud = B230400;
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "不支持的波特率: %d", baud_rate_);
                close(serial_fd_);
                serial_fd_ = -1;
                return false;
        }

        cfsetospeed(&tty, baud);
        cfsetispeed(&tty, baud);

        // 设置数据格式: 8位数据, 无奇偶校验, 1位停止位
        tty.c_cflag &= ~PARENB; // 无奇偶校验
        tty.c_cflag &= ~CSTOPB; // 1位停止位
        tty.c_cflag &= ~CSIZE; // 清除数据位设置
        tty.c_cflag |= CS8; // 8位数据位

        // 禁用硬件流控制
        tty.c_cflag &= ~CRTSCTS;

        // 启用接收, 并将本地模式设置为常规模式
        tty.c_cflag |= (CLOCAL | CREAD);

        // 禁用软件流控制
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);

        // 原始输入
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

        // 原始输出
        tty.c_oflag &= ~OPOST;

        // 设置等待时间和最小接收字符
        tty.c_cc[VTIME] = 10; // 读取超时时间 (1秒)
        tty.c_cc[VMIN] = 0; // 最小读取字符数

        // 清除缓冲区
        tcflush(serial_fd_, TCIFLUSH);

        // 应用配置
        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "设置串口属性失败, 错误: %s", strerror(errno));

            close(serial_fd_);

            serial_fd_ = -1;

            return false;
        }

        return true;
    }

    void UnicorecommGpsNode::read_serial_data() {
        const int BUFFER_SIZE = 1024;

        char buffer[BUFFER_SIZE];

        while (running_ && rclcpp::ok()) {
            //RCLCPP_INFO(this->get_logger(), "...");

            ssize_t bytes_read = read(serial_fd_, buffer, BUFFER_SIZE - 1);

            if (bytes_read > 0) {
                std::lock_guard<std::mutex> lock(data_mutex_);

                nmea_buffer_.append(buffer, bytes_read);

                // 查找完整的NMEA句子 (以换行符结束)
                size_t pos;

                while ((pos = nmea_buffer_.find('\n')) != std::string::npos) {
                    std::string sentence = nmea_buffer_.substr(0, pos + 1);

                    nmea_buffer_.erase(0, pos + 1);

                    // 处理NMEA句子
                    process_nmea_sentence(sentence);
                }
            } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_ERROR(this->get_logger(), "串口读取错误: %s", strerror(errno));

                // 短暂休眠避免CPU占用过高
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } else {
                // 没有数据可读，短暂休眠
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    bool UnicorecommGpsNode::check_checksum(const std::string &sentence) {
        // 查找校验和分隔符*
        size_t star_pos = sentence.find('*');

        if (star_pos == std::string::npos || star_pos == sentence.length() - 1)
            return false; // 没有校验和或校验和不完整

        // 计算校验和
        uint8_t checksum = 0;
        for (size_t i = 1; i < star_pos; ++i) // 从$后的第一个字符开始
        {
            checksum ^= sentence[i];
        }

        // 解析句子中的校验和
        std::string checksum_str = sentence.substr(star_pos + 1, 2);

        uint8_t sentence_checksum;

        try {
            sentence_checksum = std::stoul(checksum_str, nullptr, 16);
        } catch (...) {
            return false; // 校验和格式错误
        }

        return (checksum == sentence_checksum);
    }

    double UnicorecommGpsNode::nmea_to_decimal(double value, char direction) {
        // NMEA格式: ddmm.mmmm 转换为十进制: dd + mm.mmmm/60
        int degrees = static_cast<int>(value / 100);

        double minutes = value - degrees * 100;

        double decimal = degrees + minutes / 60.0;

        // 根据方向调整符号
        if (direction == 'S' || direction == 'W') {
            decimal = -decimal;
        }

        return decimal;
    }

    std::vector<std::string> UnicorecommGpsNode::split_nmea_fields(const std::string &sentence) {
        std::vector<std::string> fields;
        std::stringstream ss(sentence);
        std::string field;

        // 分割句子为字段
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }

        return fields;
    }

    bool UnicorecommGpsNode::string_to_double(const std::string &s, double &val) {
        if (s.empty()) return false;
        try {
            val = std::stod(s);
            return true;
        } catch (...) {
            return false;
        }
    }

    bool UnicorecommGpsNode::string_to_int(const std::string &s, int &val) {
        if (s.empty()) return false;
        try {
            val = std::stoi(s);
            return true;
        } catch (...) {
            return false;
        }
    }

    // 解析 NMEA语句
    void UnicorecommGpsNode::process_nmea_sentence(const std::string &sentence) {
        // 去除首尾空白字符
        std::string trimmed;

        size_t start = sentence.find_first_not_of(" \t\n\r");

        if (start != std::string::npos) {
            size_t end = sentence.find_last_not_of(" \t\n\r");

            trimmed = sentence.substr(start, end - start + 1);
        }

        if (trimmed.empty())
            return;

        // 检查是否是有效的NMEA句子 (以$开头)
        if (trimmed[0] != '$')
            return;

        // 检查校验和
        if (!check_checksum(trimmed)) {
            RCLCPP_WARN(this->get_logger(), "NMEA校验和错误: %s", trimmed.c_str());

            return;
        }

        // 解析不同类型的NMEA句子
        if (trimmed.size() < 6) return;

        // 提取系统类型（前2个字符）和句子类型（接下来3个字符）
        std::string system_type = trimmed.substr(1, 2); // 第2-3个字符表示系统
        std::string sentence_type = trimmed.substr(3, 3); // 第4-6个字符表示句子类型

        //RCLCPP_INFO(this->get_logger(), "接收到 %s 句子: %s", system_type.c_str(), trimmed.c_str());

        // 发布原始语句
        if (publish_gps_nmea_sentence_) {
            auto sentence_msg = nmea_msgs::msg::Sentence();
            sentence_msg.header.stamp = this->get_clock()->now();
            sentence_msg.header.frame_id = frame_id_;
            sentence_msg.sentence = trimmed;

            nmea_sentence_pub_->publish(sentence_msg);
        }

        // GP：GPS 系统
        // BD：北斗系统
        // GL：GLONASS 系统（俄罗斯）
        // GA：Galileo 系统（欧盟）
        // GN：多系统联合定位

        // 根据系统类型和句子类型分发解析
        if (system_type == "GP") {
            // GPS 美国

            if (sentence_type == "GGA" && publish_gps_fix_ && publish_gps_time_reference_) {
                parse_gpgga(trimmed);
            } else if (sentence_type == "GSA" && publish_gps_nmea_gpgsa_) {
                parse_gpgsa(trimmed);
            } else if (sentence_type == "GSV" && publish_gps_nmea_gpgsv_) {
                parse_gpgsv(trimmed);
            } else if (sentence_type == "RMC" && publish_gps_nmea_gprmc_ && publish_gps_velocity_) {
                parse_gprmc(trimmed);
            }
        } else if (system_type == "BD") {
            // 北斗 中国

            if (sentence_type == "GGA" && publish_gps_fix_ && publish_gps_time_reference_) {
                parse_bdgga(trimmed);
            } else if (sentence_type == "GSA" && publish_gps_nmea_gpgsa_) {
                parse_bdgsa(trimmed);
            } else if (sentence_type == "GSV" && publish_gps_nmea_gpgsv_) {
                parse_bdgsv(trimmed);
            } else if (sentence_type == "RMC" && publish_gps_nmea_gprmc_ && publish_gps_velocity_) {
                parse_bdrmc(trimmed);
            }
        } else if (system_type == "GL") {
            // GLONASS 俄罗斯

            if (sentence_type == "GGA" && publish_gps_fix_ && publish_gps_time_reference_) {
                parse_glgga(trimmed);
            } else if (sentence_type == "GSA" && publish_gps_nmea_gpgsa_) {
                parse_glgsa(trimmed);
            } else if (sentence_type == "GSV" && publish_gps_nmea_gpgsv_) {
                parse_glgsv(trimmed);
            } else if (sentence_type == "RMC" && publish_gps_nmea_gprmc_ && publish_gps_velocity_) {
                parse_glrmc(trimmed);
            }
        } else if (system_type == "GA") {
            // Galileo 欧盟

            if (sentence_type == "GGA" && publish_gps_fix_ && publish_gps_time_reference_) {
                parse_gagga(trimmed);
            } else if (sentence_type == "GSA" && publish_gps_nmea_gpgsa_) {
                parse_gagsa(trimmed);
            } else if (sentence_type == "GSV" && publish_gps_nmea_gpgsv_) {
                parse_gagsv(trimmed);
            } else if (sentence_type == "RMC" && publish_gps_nmea_gprmc_ && publish_gps_velocity_) {
                parse_garmc(trimmed);
            }
        } else if (system_type == "GN") {
            // 多系统联合定位

            if (sentence_type == "GGA" && publish_gps_fix_ && publish_gps_time_reference_) {
                parse_gngga(trimmed);
            } else if (sentence_type == "GSA" && publish_gps_nmea_gpgsa_) {
                parse_gngsa(trimmed);
            } else if (sentence_type == "GSV" && publish_gps_nmea_gpgsv_) {
                parse_gngsv(trimmed);
            } else if (sentence_type == "RMC" && publish_gps_nmea_gprmc_ && publish_gps_velocity_) {
                parse_gnrmc(trimmed);
            }
        }
    }

    // GGA GPS
    void UnicorecommGpsNode::parse_gpgga(const std::string &sentence) {
        // GGA句子格式: $GPGGA,hhmmss.ss,ddmm.mmmm,a,ddmm.mmmm,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
        // 字段说明:
        // 1. 时间 (hhmmss.ss)
        // 2. 纬度 (ddmm.mmmm)
        // 3. 纬度方向 (N/S)
        // 4. 经度 (ddmm.mmmm)
        // 5. 经度方向 (E/W)
        // 6. GPS质量指示 (0=无效, 1=单点定位, 2=差分, 4=固定解, 5=浮点解)
        // 7. 卫星数量
        // 8. 水平精度因子 (HDOP)
        // 9. 海拔高度
        // 10. 海拔高度单位 (M)
        // 11. 大地水准面高度
        // 12. 大地水准面高度单位 (M)
        // 13. 差分时间 (秒)
        // 14. 差分站ID
        // 15. 校验和

        std::vector<std::string> fields;
        std::stringstream ss(sentence);
        std::string field;

        // 分割句子为字段
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }

        // 检查GGA句子是否有足够的字段
        if (fields.size() < 15) {
            RCLCPP_WARN(this->get_logger(), "GGA句子字段不足: %s", sentence.c_str());
            return;
        }

        // 创建GPS消息
        auto gps_msg = sensor_msgs::msg::NavSatFix();
        gps_msg.header.stamp = this->get_clock()->now();
        gps_msg.header.frame_id = frame_id_;

        // 创建时间参考消息
        auto time_ref_msg = sensor_msgs::msg::TimeReference();
        time_ref_msg.header.stamp = gps_msg.header.stamp;
        time_ref_msg.source = "gps";

        // 解析时间
        if (!fields[1].empty()) {
            std::string time_str = fields[1];

            if (time_str.length() >= 6) {
                int hours = std::stoi(time_str.substr(0, 2));

                int minutes = std::stoi(time_str.substr(2, 2));

                double seconds = std::stod(time_str.substr(4));

                // 这里简化处理，实际应用中可能需要结合日期信息
                time_ref_msg.time_ref.sec = hours * 3600 + minutes * 60 + static_cast<int>(seconds);

                time_ref_msg.time_ref.nanosec = static_cast<uint32_t>((seconds - static_cast<int>(seconds)) * 1e9);
            }
        }

        // 解析纬度
        if (!fields[2].empty() && !fields[3].empty()) {
            double lat_value = std::stod(fields[2]);

            char lat_dir = fields[3][0];

            gps_msg.latitude = nmea_to_decimal(lat_value, lat_dir);
        } else {
            RCLCPP_WARN(this->get_logger(), "GGA句子缺少纬度信息");

            return;
        }

        // 解析经度
        if (!fields[4].empty() && !fields[5].empty()) {
            double lon_value = std::stod(fields[4]);

            char lon_dir = fields[5][0];

            gps_msg.longitude = nmea_to_decimal(lon_value, lon_dir);
        } else {
            RCLCPP_WARN(this->get_logger(), "GGA句子缺少经度信息");

            return;
        }

        // 解析GPS质量指示
        if (!fields[6].empty()) {
            int fix_quality = std::stoi(fields[6]);

            // 设置定位类型
            switch (fix_quality) {
                case 0: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                    break;
                case 1: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    break;
                case 2: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                    break;
                case 4: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                    break;
                default: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            }

            // 设置服务类型为GPS
            gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        }

        // 解析卫星数量
        if (!fields[7].empty()) {
            int satellites = std::stoi(fields[7]);

            RCLCPP_DEBUG(this->get_logger(), "卫星数量: %d", satellites);
        }

        // 解析HDOP (水平精度因子)
        if (!fields[8].empty()) {
            double hdop = std::stod(fields[8]);

            // 简单将HDOP转换为位置协方差 (示例)
            gps_msg.position_covariance[0] = hdop * hdop; // 纬度方差

            gps_msg.position_covariance[4] = hdop * hdop; // 经度方差

            gps_msg.position_covariance[8] = (2 * hdop) * (2 * hdop); // 高度方差

            gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        }

        // 解析海拔高度
        if (!fields[9].empty() && !fields[10].empty() && fields[10] == "M") {
            gps_msg.altitude = std::stod(fields[9]);
        }

        // 发布消息
        gps_fix_pub_->publish(gps_msg);

        time_ref_pub_->publish(time_ref_msg);

        RCLCPP_DEBUG(this->get_logger(), "发布GPS数据: 纬度=%.6f, 经度=%.6f, 海拔=%.2f", gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);
    }

    // GSA GPS
    void UnicorecommGpsNode::parse_gpgsa(const std::string &sentence) {
        // GPGSA格式: $GPGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,2.5,1.5,2.0*hh
        std::vector<std::string> fields = split_nmea_fields(sentence);
        auto msg = nmea_msgs::msg::Gpgsa();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;

        if (fields.size() >= 2) msg.auto_manual_mode = fields[1];

        // 修正：将字符串转换为整数后再赋值给fix_mode
        if (fields.size() >= 3 && !fields[2].empty()) {
            try {
                msg.fix_mode = std::stoi(fields[2]); // 字符串转整数
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "GPGSA fix mode转换失败: %s", fields[2].c_str());
            }
        }

        // 解析卫星PRN码
        // $GNGSA,M,3,66,67,76,86,87,,,,,,,,1.2,0.6,1.1,2*3A
        for (size_t i = 0; i < 12 && fields.size() > i + 3; ++i) {
            try {
                std::string c = fields[i + 3];
                if (!c.empty()) {
                    msg.sv_ids[i] = std::stoi(c);
                } else {
                    msg.sv_ids[i] = 0;
                }
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "GPGSA sv_ids 转换失败: %s", fields[i + 3].c_str());
            }
        }

        if (fields.size() >= 16 && !fields[15].empty()) msg.pdop = std::stod(fields[15]);
        if (fields.size() >= 17 && !fields[16].empty()) msg.hdop = std::stod(fields[16]);
        if (fields.size() >= 18 && !fields[17].empty()) msg.vdop = std::stod(fields[17]);

        gpgsa_pub_->publish(msg);
    }

    // GSV GPS
    void UnicorecommGpsNode::parse_gpgsv(const std::string &sentence) {
        // GPGSV格式: $GPGSV,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x*hh
        std::vector<std::string> fields = split_nmea_fields(sentence);
        auto msg = nmea_msgs::msg::Gpgsv();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;

        msg.message_id = "";

        if (fields.size() >= 2 && !fields[1].empty()) msg.n_msgs = std::stoi(fields[1]);
        if (fields.size() >= 3 && !fields[2].empty()) msg.msg_number = std::stoi(fields[2]);
        if (fields.size() >= 4 && !fields[3].empty()) msg.n_satellites = std::stoi(fields[3]);

        // 解析卫星信息
        for (size_t i = 0; i < 4; ++i) {
            size_t base_idx = 4 + i * 4;
            if (base_idx + 3 >= fields.size()) break;

            nmea_msgs::msg::GpgsvSatellite sat;
            if (!fields[base_idx].empty()) sat.prn = std::stoi(fields[base_idx]);
            if (!fields[base_idx + 1].empty()) sat.elevation = std::stoi(fields[base_idx + 1]);
            if (!fields[base_idx + 2].empty()) sat.azimuth = std::stoi(fields[base_idx + 2]);
            if (!fields[base_idx + 3].empty()) sat.snr = std::stoi(fields[base_idx + 3]);

            msg.satellites[i] = sat;
        }

        gpgsv_pub_->publish(msg);
    }

    // RMC GPS
    void UnicorecommGpsNode::parse_gprmc(const std::string &sentence) {
        // GPRMC格式: $GPRMC,hhmmss.ss,A,ddmm.mmmm,a,ddmm.mmmm,a,x.x,x.x,ddmmyy,x.x,a*hh
        std::vector<std::string> fields = split_nmea_fields(sentence);
        auto msg = nmea_msgs::msg::Gprmc();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;

        //msg.utc_seconds = 0;

        if (fields.size() >= 2) msg.date = fields[1];
        if (fields.size() >= 3) msg.position_status = fields[2];

        // 解析纬度
        if (fields.size() >= 4 && !fields[3].empty() && fields.size() >= 5 && !fields[4].empty()) {
            double lat_value = std::stod(fields[3]);
            char lat_dir = fields[4][0];
            msg.lat = nmea_to_decimal(lat_value, lat_dir);
            msg.lat_dir = lat_dir;
        }

        // 解析经度
        if (fields.size() >= 6 && !fields[5].empty() && fields.size() >= 7 && !fields[6].empty()) {
            double lon_value = std::stod(fields[5]);
            char lon_dir = fields[6][0];
            msg.lon = nmea_to_decimal(lon_value, lon_dir);
            msg.lon_dir = lon_dir;
        }

        // 解析速度
        if (fields.size() >= 8 && !fields[7].empty()) {
            msg.speed = std::stod(fields[7]); // 节
            // 发布速度消息 (转换为m/s，1节 = 0.514444 m/s)
            auto vel_msg = geometry_msgs::msg::TwistStamped();
            vel_msg.header.stamp = msg.header.stamp;
            vel_msg.header.frame_id = frame_id_;
            vel_msg.twist.linear.x = msg.speed * 0.514444;
            gps_vel_pub_->publish(vel_msg);
        }

        if (fields.size() >= 9 && !fields[8].empty()) msg.track = std::stod(fields[8]);
        if (fields.size() >= 10) msg.date = fields[9];
        if (fields.size() >= 11 && !fields[10].empty()) msg.mag_var = std::stod(fields[10]);
        if (fields.size() >= 12) msg.mag_var_direction = fields[11];

        if (fields.size() >= 13) msg.mode_indicator = fields[12];

        gprmc_pub_->publish(msg);
    }

    // GGA 北斗
    void UnicorecommGpsNode::parse_bdgga(const std::string &sentence) {
        // BDGGA句子格式与GPGGA相同，仅系统标识不同
        // 字段说明参考parse_gpgga函数

        std::vector<std::string> fields;
        std::stringstream ss(sentence);
        std::string field;

        // 分割句子为字段
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }

        // 检查GGA句子是否有足够的字段
        if (fields.size() < 15) {
            RCLCPP_WARN(this->get_logger(), "BDGGA句子字段不足: %s", sentence.c_str());
            return;
        }

        // 创建GPS消息
        auto gps_msg = sensor_msgs::msg::NavSatFix();
        gps_msg.header.stamp = this->get_clock()->now();
        gps_msg.header.frame_id = frame_id_;

        // 创建时间参考消息
        auto time_ref_msg = sensor_msgs::msg::TimeReference();
        time_ref_msg.header.stamp = gps_msg.header.stamp;
        time_ref_msg.source = "beidou"; // 北斗系统

        // 解析时间 (与GPGGA处理相同)
        if (!fields[1].empty() && fields[1].length() >= 6) {
            std::string time_str = fields[1];
            int hours = std::stoi(time_str.substr(0, 2));
            int minutes = std::stoi(time_str.substr(2, 2));
            double seconds = std::stod(time_str.substr(4));

            time_ref_msg.time_ref.sec = hours * 3600 + minutes * 60 + static_cast<int>(seconds);
            time_ref_msg.time_ref.nanosec = static_cast<uint32_t>((seconds - static_cast<int>(seconds)) * 1e9);
        }

        // 解析纬度 (与GPGGA处理相同)
        if (!fields[2].empty() && !fields[3].empty()) {
            double lat_value = std::stod(fields[2]);
            char lat_dir = fields[3][0];
            gps_msg.latitude = nmea_to_decimal(lat_value, lat_dir);
        } else {
            RCLCPP_WARN(this->get_logger(), "BDGGA句子缺少纬度信息");
            return;
        }

        // 解析经度 (与GPGGA处理相同)
        if (!fields[4].empty() && !fields[5].empty()) {
            double lon_value = std::stod(fields[4]);
            char lon_dir = fields[5][0];
            gps_msg.longitude = nmea_to_decimal(lon_value, lon_dir);
        } else {
            RCLCPP_WARN(this->get_logger(), "BDGGA句子缺少经度信息");
            return;
        }

        // 解析定位质量指示
        if (!fields[6].empty()) {
            int fix_quality = std::stoi(fields[6]);

            // 设置定位类型
            switch (fix_quality) {
                case 0: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                    break;
                case 1: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    break;
                case 2: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                    break;
                case 4: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                    break;
                default: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            }

            // 设置服务类型为北斗
            gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;
        }

        // 解析卫星数量
        if (!fields[7].empty()) {
            int satellites = std::stoi(fields[7]);
            RCLCPP_DEBUG(this->get_logger(), "北斗卫星数量: %d", satellites);
        }

        // 解析HDOP (水平精度因子)
        if (!fields[8].empty()) {
            double hdop = std::stod(fields[8]);
            gps_msg.position_covariance[0] = hdop * hdop; // 纬度方差
            gps_msg.position_covariance[4] = hdop * hdop; // 经度方差
            gps_msg.position_covariance[8] = (2 * hdop) * (2 * hdop); // 高度方差
            gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        }

        // 解析海拔高度
        if (!fields[9].empty() && !fields[10].empty() && fields[10] == "M") {
            gps_msg.altitude = std::stod(fields[9]);
        }

        // 发布消息
        gps_fix_pub_->publish(gps_msg);
        time_ref_pub_->publish(time_ref_msg);

        RCLCPP_DEBUG(this->get_logger(), "发布北斗GPS数据: 纬度=%.6f, 经度=%.6f, 海拔=%.2f",
                     gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);
    }

    // GSA 北斗
    void UnicorecommGpsNode::parse_bdgsa(const std::string &sentence) {
        (void) sentence;
    }

    // GSV 北斗
    void UnicorecommGpsNode::parse_bdgsv(const std::string &sentence) {
        (void) sentence;
    }

    // RMC 北斗
    void UnicorecommGpsNode::parse_bdrmc(const std::string &sentence) {
        // BDRMC格式与GPRMC相同，仅系统标识不同
        // 格式: $BDRMC,hhmmss.ss,A,ddmm.mmmm,a,ddmm.mmmm,a,x.x,x.x,ddmmyy,x.x,a*hh
        std::vector<std::string> fields = split_nmea_fields(sentence);
        auto msg = nmea_msgs::msg::Gprmc(); // 使用相同的消息结构

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;

        // 解析UTC时间
        if (fields.size() >= 2) msg.date = fields[1];
        // 解析定位状态 (A=有效, V=无效)
        if (fields.size() >= 3) msg.position_status = fields[2];

        // 解析纬度
        if (fields.size() >= 4 && !fields[3].empty() && fields.size() >= 5 && !fields[4].empty()) {
            double lat_value = std::stod(fields[3]);
            char lat_dir = fields[4][0];
            msg.lat = nmea_to_decimal(lat_value, lat_dir);
            msg.lat_dir = lat_dir;
        }

        // 解析经度
        if (fields.size() >= 6 && !fields[5].empty() && fields.size() >= 7 && !fields[6].empty()) {
            double lon_value = std::stod(fields[5]);
            char lon_dir = fields[6][0];
            msg.lon = nmea_to_decimal(lon_value, lon_dir);
            msg.lon_dir = lon_dir;
        }

        // 解析速度 (节 -> m/s)
        if (fields.size() >= 8 && !fields[7].empty()) {
            msg.speed = std::stod(fields[7]); // 节
            auto vel_msg = geometry_msgs::msg::TwistStamped();
            vel_msg.header.stamp = msg.header.stamp;
            vel_msg.header.frame_id = frame_id_;
            vel_msg.twist.linear.x = msg.speed * 0.514444; // 转换为米/秒
            gps_vel_pub_->publish(vel_msg);
        }

        // 解析航向角
        if (fields.size() >= 9 && !fields[8].empty()) msg.track = std::stod(fields[8]);
        // 解析日期
        if (fields.size() >= 10) msg.date = fields[9];
        // 解析磁偏角
        if (fields.size() >= 11 && !fields[10].empty()) msg.mag_var = std::stod(fields[10]);
        if (fields.size() >= 12) msg.mag_var_direction = fields[11];
        // 解析模式指示
        if (fields.size() >= 13) msg.mode_indicator = fields[12];

        // 发布北斗RMC消息
        gprmc_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "解析北斗RMC数据: 速度=%.2f节, 航向=%.1f度",
                     msg.speed, msg.track);
    }

    // GGA GLONASS 俄罗斯
    void UnicorecommGpsNode::parse_glgga(const std::string &sentence) {
        // GLGGA句子格式与GPGGA相同，仅系统标识不同

        std::vector<std::string> fields;
        std::stringstream ss(sentence);
        std::string field;

        // 分割句子为字段
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }

        // 检查GGA句子是否有足够的字段
        if (fields.size() < 15) {
            RCLCPP_WARN(this->get_logger(), "GLGGA句子字段不足: %s", sentence.c_str());
            return;
        }

        // 创建GPS消息
        auto gps_msg = sensor_msgs::msg::NavSatFix();
        gps_msg.header.stamp = this->get_clock()->now();
        gps_msg.header.frame_id = frame_id_;

        // 创建时间参考消息
        auto time_ref_msg = sensor_msgs::msg::TimeReference();
        time_ref_msg.header.stamp = gps_msg.header.stamp;
        time_ref_msg.source = "glonass"; // GLONASS系统

        // 解析时间
        if (!fields[1].empty() && fields[1].length() >= 6) {
            std::string time_str = fields[1];
            int hours = std::stoi(time_str.substr(0, 2));
            int minutes = std::stoi(time_str.substr(2, 2));
            double seconds = std::stod(time_str.substr(4));

            time_ref_msg.time_ref.sec = hours * 3600 + minutes * 60 + static_cast<int>(seconds);
            time_ref_msg.time_ref.nanosec = static_cast<uint32_t>((seconds - static_cast<int>(seconds)) * 1e9);
        }

        // 解析纬度
        if (!fields[2].empty() && !fields[3].empty()) {
            double lat_value = std::stod(fields[2]);
            char lat_dir = fields[3][0];
            gps_msg.latitude = nmea_to_decimal(lat_value, lat_dir);
        } else {
            RCLCPP_WARN(this->get_logger(), "GLGGA句子缺少纬度信息");
            return;
        }

        // 解析经度
        if (!fields[4].empty() && !fields[5].empty()) {
            double lon_value = std::stod(fields[4]);
            char lon_dir = fields[5][0];
            gps_msg.longitude = nmea_to_decimal(lon_value, lon_dir);
        } else {
            RCLCPP_WARN(this->get_logger(), "GLGGA句子缺少经度信息");
            return;
        }

        // 解析定位质量指示
        if (!fields[6].empty()) {
            int fix_quality = std::stoi(fields[6]);

            // 设置定位类型
            switch (fix_quality) {
                case 0: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                    break;
                case 1: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    break;
                case 2: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                    break;
                case 4: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                    break;
                default: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            }

            // 设置服务类型为GLONASS
            gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;
        }

        // 解析卫星数量
        if (!fields[7].empty()) {
            int satellites = std::stoi(fields[7]);
            RCLCPP_DEBUG(this->get_logger(), "GLONASS卫星数量: %d", satellites);
        }

        // 解析HDOP和海拔高度（与GPGGA处理相同）
        if (!fields[8].empty()) {
            double hdop = std::stod(fields[8]);
            gps_msg.position_covariance[0] = hdop * hdop;
            gps_msg.position_covariance[4] = hdop * hdop;
            gps_msg.position_covariance[8] = (2 * hdop) * (2 * hdop);
            gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        }

        if (!fields[9].empty() && !fields[10].empty() && fields[10] == "M") {
            gps_msg.altitude = std::stod(fields[9]);
        }

        // 发布消息
        gps_fix_pub_->publish(gps_msg);
        time_ref_pub_->publish(time_ref_msg);

        RCLCPP_DEBUG(this->get_logger(), "发布GLONASS数据: 纬度=%.6f, 经度=%.6f, 海拔=%.2f",
                     gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);
    }

    // GSA GLONASS 俄罗斯
    void UnicorecommGpsNode::parse_glgsa(const std::string &sentence) {
        (void) sentence;
    }

    // GSV GLONASS 俄罗斯
    void UnicorecommGpsNode::parse_glgsv(const std::string &sentence) {
        (void) sentence;
    }

    // RMC GLONASS 俄罗斯
    void UnicorecommGpsNode::parse_glrmc(const std::string &sentence) {
        // GLRMC格式与GPRMC相同，仅系统标识不同
        // 格式: $GLRMC,hhmmss.ss,A,ddmm.mmmm,a,ddmm.mmmm,a,x.x,x.x,ddmmyy,x.x,a*hh
        std::vector<std::string> fields = split_nmea_fields(sentence);
        auto msg = nmea_msgs::msg::Gprmc();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;

        // 解析UTC时间
        if (fields.size() >= 2) msg.date = fields[1];
        // 解析定位状态
        if (fields.size() >= 3) msg.position_status = fields[2];

        // 解析纬度
        if (fields.size() >= 4 && !fields[3].empty() && fields.size() >= 5 && !fields[4].empty()) {
            double lat_value = std::stod(fields[3]);
            char lat_dir = fields[4][0];
            msg.lat = nmea_to_decimal(lat_value, lat_dir);
            msg.lat_dir = lat_dir;
        }

        // 解析经度
        if (fields.size() >= 6 && !fields[5].empty() && fields.size() >= 7 && !fields[6].empty()) {
            double lon_value = std::stod(fields[5]);
            char lon_dir = fields[6][0];
            msg.lon = nmea_to_decimal(lon_value, lon_dir);
            msg.lon_dir = lon_dir;
        }

        // 解析速度并发布
        if (fields.size() >= 8 && !fields[7].empty()) {
            msg.speed = std::stod(fields[7]);
            auto vel_msg = geometry_msgs::msg::TwistStamped();
            vel_msg.header.stamp = msg.header.stamp;
            vel_msg.header.frame_id = frame_id_;
            vel_msg.twist.linear.x = msg.speed * 0.514444;
            gps_vel_pub_->publish(vel_msg);
        }

        // 解析其他字段
        if (fields.size() >= 9 && !fields[8].empty()) msg.track = std::stod(fields[8]);
        if (fields.size() >= 10) msg.date = fields[9];
        if (fields.size() >= 11 && !fields[10].empty()) msg.mag_var = std::stod(fields[10]);
        if (fields.size() >= 12) msg.mag_var_direction = fields[11];
        if (fields.size() >= 13) msg.mode_indicator = fields[12];

        // 发布GLONASS RMC消息
        gprmc_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "解析GLONASS RMC数据: 速度=%.2f节, 航向=%.1f度",
                     msg.speed, msg.track);
    }

    // GGA Galileo 欧盟
    void UnicorecommGpsNode::parse_gagga(const std::string &sentence) {
        // GAGGA句子格式与GPGGA相同，仅系统标识不同

        std::vector<std::string> fields;
        std::stringstream ss(sentence);
        std::string field;

        // 分割句子为字段
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }

        // 检查GGA句子是否有足够的字段
        if (fields.size() < 15) {
            RCLCPP_WARN(this->get_logger(), "GAGGA句子字段不足: %s", sentence.c_str());
            return;
        }

        // 创建GPS消息
        auto gps_msg = sensor_msgs::msg::NavSatFix();
        gps_msg.header.stamp = this->get_clock()->now();
        gps_msg.header.frame_id = frame_id_;

        // 创建时间参考消息
        auto time_ref_msg = sensor_msgs::msg::TimeReference();
        time_ref_msg.header.stamp = gps_msg.header.stamp;
        time_ref_msg.source = "galileo"; // 伽利略系统

        // 解析时间、纬度、经度（与GPGGA处理相同）
        if (!fields[1].empty() && fields[1].length() >= 6) {
            std::string time_str = fields[1];
            int hours = std::stoi(time_str.substr(0, 2));
            int minutes = std::stoi(time_str.substr(2, 2));
            double seconds = std::stod(time_str.substr(4));

            time_ref_msg.time_ref.sec = hours * 3600 + minutes * 60 + static_cast<int>(seconds);
            time_ref_msg.time_ref.nanosec = static_cast<uint32_t>((seconds - static_cast<int>(seconds)) * 1e9);
        }

        if (!fields[2].empty() && !fields[3].empty()) {
            double lat_value = std::stod(fields[2]);
            char lat_dir = fields[3][0];
            gps_msg.latitude = nmea_to_decimal(lat_value, lat_dir);
        } else {
            RCLCPP_WARN(this->get_logger(), "GAGGA句子缺少纬度信息");
            return;
        }

        if (!fields[4].empty() && !fields[5].empty()) {
            double lon_value = std::stod(fields[4]);
            char lon_dir = fields[5][0];
            gps_msg.longitude = nmea_to_decimal(lon_value, lon_dir);
        } else {
            RCLCPP_WARN(this->get_logger(), "GAGGA句子缺少经度信息");
            return;
        }

        // 解析定位质量指示
        if (!fields[6].empty()) {
            int fix_quality = std::stoi(fields[6]);

            switch (fix_quality) {
                case 0: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                    break;
                case 1: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    break;
                case 2: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                    break;
                case 4: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                    break;
                default: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            }

            // 设置服务类型为伽利略
            gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
        }

        // 解析卫星数量、HDOP和海拔高度
        if (!fields[7].empty()) {
            int satellites = std::stoi(fields[7]);
            RCLCPP_DEBUG(this->get_logger(), "伽利略卫星数量: %d", satellites);
        }

        if (!fields[8].empty()) {
            double hdop = std::stod(fields[8]);
            gps_msg.position_covariance[0] = hdop * hdop;
            gps_msg.position_covariance[4] = hdop * hdop;
            gps_msg.position_covariance[8] = (2 * hdop) * (2 * hdop);
            gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        }

        if (!fields[9].empty() && !fields[10].empty() && fields[10] == "M") {
            gps_msg.altitude = std::stod(fields[9]);
        }

        // 发布消息
        gps_fix_pub_->publish(gps_msg);
        time_ref_pub_->publish(time_ref_msg);

        RCLCPP_DEBUG(this->get_logger(), "发布伽利略数据: 纬度=%.6f, 经度=%.6f, 海拔=%.2f",
                     gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);
    }

    // GSA Galileo 欧盟
    void UnicorecommGpsNode::parse_gagsa(const std::string &sentence) {
        (void) sentence;
    }

    // GSV Galileo 欧盟
    void UnicorecommGpsNode::parse_gagsv(const std::string &sentence) {
        (void) sentence;
    }

    // RMC Galileo 欧盟
    void UnicorecommGpsNode::parse_garmc(const std::string &sentence) {
        // GARMC格式与GPRMC相同，仅系统标识不同
        // 格式: $GARMC,hhmmss.ss,A,ddmm.mmmm,a,ddmm.mmmm,a,x.x,x.x,ddmmyy,x.x,a*hh
        std::vector<std::string> fields = split_nmea_fields(sentence);
        auto msg = nmea_msgs::msg::Gprmc();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;

        // 解析UTC时间
        if (fields.size() >= 2) msg.date = fields[1];
        // 解析定位状态
        if (fields.size() >= 3) msg.position_status = fields[2];

        // 解析纬度
        if (fields.size() >= 4 && !fields[3].empty() && fields.size() >= 5 && !fields[4].empty()) {
            double lat_value = std::stod(fields[3]);
            char lat_dir = fields[4][0];
            msg.lat = nmea_to_decimal(lat_value, lat_dir);
            msg.lat_dir = lat_dir;
        }

        // 解析经度
        if (fields.size() >= 6 && !fields[5].empty() && fields.size() >= 7 && !fields[6].empty()) {
            double lon_value = std::stod(fields[5]);
            char lon_dir = fields[6][0];
            msg.lon = nmea_to_decimal(lon_value, lon_dir);
            msg.lon_dir = lon_dir;
        }

        // 解析速度并发布
        if (fields.size() >= 8 && !fields[7].empty()) {
            msg.speed = std::stod(fields[7]);
            auto vel_msg = geometry_msgs::msg::TwistStamped();
            vel_msg.header.stamp = msg.header.stamp;
            vel_msg.header.frame_id = frame_id_;
            vel_msg.twist.linear.x = msg.speed * 0.514444;
            gps_vel_pub_->publish(vel_msg);
        }

        // 解析其他字段
        if (fields.size() >= 9 && !fields[8].empty()) msg.track = std::stod(fields[8]);
        if (fields.size() >= 10) msg.date = fields[9];
        if (fields.size() >= 11 && !fields[10].empty()) msg.mag_var = std::stod(fields[10]);
        if (fields.size() >= 12) msg.mag_var_direction = fields[11];
        if (fields.size() >= 13) msg.mode_indicator = fields[12];

        // 发布伽利略RMC消息
        gprmc_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "解析伽利略RMC数据: 速度=%.2f节, 航向=%.1f度",
                     msg.speed, msg.track);
    }

    // GGA 多系统联合定位
    void UnicorecommGpsNode::parse_gngga(const std::string &sentence) {
        // GNGGA是混合GNSS系统（多系统联合定位）的GGA句子

        std::vector<std::string> fields;
        std::stringstream ss(sentence);
        std::string field;

        // 分割句子为字段
        while (std::getline(ss, field, ',')) {
            fields.push_back(field);
        }

        // 检查GGA句子是否有足够的字段
        if (fields.size() < 15) {
            RCLCPP_WARN(this->get_logger(), "GNGGA句子字段不足: %s", sentence.c_str());
            return;
        }

        // 创建GPS消息
        auto gps_msg = sensor_msgs::msg::NavSatFix();
        gps_msg.header.stamp = this->get_clock()->now();
        gps_msg.header.frame_id = frame_id_;

        // 创建时间参考消息
        auto time_ref_msg = sensor_msgs::msg::TimeReference();
        time_ref_msg.header.stamp = gps_msg.header.stamp;
        time_ref_msg.source = "gnss"; // 混合GNSS系统

        // 解析时间、纬度、经度（与其他GGA处理相同）
        if (!fields[1].empty() && fields[1].length() >= 6) {
            std::string time_str = fields[1];
            int hours = std::stoi(time_str.substr(0, 2));
            int minutes = std::stoi(time_str.substr(2, 2));
            double seconds = std::stod(time_str.substr(4));

            time_ref_msg.time_ref.sec = hours * 3600 + minutes * 60 + static_cast<int>(seconds);
            time_ref_msg.time_ref.nanosec = static_cast<uint32_t>((seconds - static_cast<int>(seconds)) * 1e9);
        }

        if (!fields[2].empty() && !fields[3].empty()) {
            double lat_value = std::stod(fields[2]);
            char lat_dir = fields[3][0];
            gps_msg.latitude = nmea_to_decimal(lat_value, lat_dir);
        } else {
            RCLCPP_WARN(this->get_logger(), "GNGGA句子缺少纬度信息");
            return;
        }

        if (!fields[4].empty() && !fields[5].empty()) {
            double lon_value = std::stod(fields[4]);
            char lon_dir = fields[5][0];
            gps_msg.longitude = nmea_to_decimal(lon_value, lon_dir);
        } else {
            RCLCPP_WARN(this->get_logger(), "GNGGA句子缺少经度信息");
            return;
        }

        // 解析定位质量指示
        if (!fields[6].empty()) {
            int fix_quality = std::stoi(fields[6]);

            switch (fix_quality) {
                case 0: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                    break;
                case 1: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    break;
                case 2: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                    break;
                case 4: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                    break;
                default: gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            }

            // 设置服务类型为混合模式（可能包含多种系统）
            gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                                     sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
                                     sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS |
                                     sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;
        }

        // 解析卫星数量、HDOP和海拔高度
        if (!fields[7].empty()) {
            int satellites = std::stoi(fields[7]);
            RCLCPP_DEBUG(this->get_logger(), "混合GNSS卫星数量: %d", satellites);
        }

        if (!fields[8].empty()) {
            double hdop = std::stod(fields[8]);
            gps_msg.position_covariance[0] = hdop * hdop;
            gps_msg.position_covariance[4] = hdop * hdop;
            gps_msg.position_covariance[8] = (2 * hdop) * (2 * hdop);
            gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
        }

        if (!fields[9].empty() && !fields[10].empty() && fields[10] == "M") {
            gps_msg.altitude = std::stod(fields[9]);
        }

        // 发布消息
        gps_fix_pub_->publish(gps_msg);
        time_ref_pub_->publish(time_ref_msg);

        RCLCPP_DEBUG(this->get_logger(), "发布混合GNSS数据: 纬度=%.6f, 经度=%.6f, 海拔=%.2f",
                     gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);
    }

    // GSA 多系统联合定位
    void UnicorecommGpsNode::parse_gngsa(const std::string &sentence) {
        (void) sentence;
    }

    // GSV 多系统联合定位
    void UnicorecommGpsNode::parse_gngsv(const std::string &sentence) {
        (void) sentence;
    }

    // RMC 多系统联合定位
    void UnicorecommGpsNode::parse_gnrmc(const std::string &sentence) {
        // GNRMC是混合系统的RMC句子，格式与GPRMC相同
        // 格式: $GNRMC,hhmmss.ss,A,ddmm.mmmm,a,ddmm.mmmm,a,x.x,x.x,ddmmyy,x.x,a*hh
        std::vector<std::string> fields = split_nmea_fields(sentence);
        auto msg = nmea_msgs::msg::Gprmc();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = frame_id_;

        // 解析UTC时间
        if (fields.size() >= 2) msg.date = fields[1];
        // 解析定位状态
        if (fields.size() >= 3) msg.position_status = fields[2];

        // 解析纬度
        if (fields.size() >= 4 && !fields[3].empty() && fields.size() >= 5 && !fields[4].empty()) {
            double lat_value = std::stod(fields[3]);
            char lat_dir = fields[4][0];
            msg.lat = nmea_to_decimal(lat_value, lat_dir);
            msg.lat_dir = lat_dir;
        }

        // 解析经度
        if (fields.size() >= 6 && !fields[5].empty() && fields.size() >= 7 && !fields[6].empty()) {
            double lon_value = std::stod(fields[5]);
            char lon_dir = fields[6][0];
            msg.lon = nmea_to_decimal(lon_value, lon_dir);
            msg.lon_dir = lon_dir;
        }

        // 解析速度并发布
        if (fields.size() >= 8 && !fields[7].empty()) {
            msg.speed = std::stod(fields[7]);
            auto vel_msg = geometry_msgs::msg::TwistStamped();
            vel_msg.header.stamp = msg.header.stamp;
            vel_msg.header.frame_id = frame_id_;
            vel_msg.twist.linear.x = msg.speed * 0.514444;
            gps_vel_pub_->publish(vel_msg);
        }

        // 解析其他字段
        if (fields.size() >= 9 && !fields[8].empty()) msg.track = std::stod(fields[8]);
        if (fields.size() >= 10) msg.date = fields[9];
        if (fields.size() >= 11 && !fields[10].empty()) msg.mag_var = std::stod(fields[10]);
        if (fields.size() >= 12) msg.mag_var_direction = fields[11];
        if (fields.size() >= 13) msg.mode_indicator = fields[12];

        // 发布混合系统RMC消息
        gprmc_pub_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "解析混合GNSS RMC数据: 速度=%.2f节, 航向=%.1f度",
                     msg.speed, msg.track);
    }
} // unicorecomm_gps_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(unicorecomm_gps_node::UnicorecommGpsNode)
