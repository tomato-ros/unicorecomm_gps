#ifndef UNICORECOMM_GPS_UNICORECOMMGPSNODE_HPP
#define UNICORECOMM_GPS_UNICORECOMMGPSNODE_HPP

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nmea_msgs/msg/sentence.hpp"
#include "nmea_msgs/msg/gpgsa.hpp"
#include "nmea_msgs/msg/gpgsv.hpp"
#include "nmea_msgs/msg/gprmc.hpp"

namespace unicorecomm_gps_node {
    class UnicorecommGpsNode : public rclcpp::Node {
    public:
        /**
         * 构造函数
         * @param options 初始化参数
         */
        explicit UnicorecommGpsNode(const rclcpp::NodeOptions &options);

        /**
         * 析构函数
         */
        ~UnicorecommGpsNode() override;

    private:
        // 串口配置参数
        std::string serial_port_;
        int baud_rate_;
        int serial_fd_;

        // ROS发布器
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_pub_;
        rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_ref_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr gps_vel_pub_;
        rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr nmea_sentence_pub_;
        rclcpp::Publisher<nmea_msgs::msg::Gpgsa>::SharedPtr gpgsa_pub_;
        rclcpp::Publisher<nmea_msgs::msg::Gpgsv>::SharedPtr gpgsv_pub_;
        rclcpp::Publisher<nmea_msgs::msg::Gprmc>::SharedPtr gprmc_pub_;

        // 线程和同步变量
        std::thread read_thread_;
        std::mutex data_mutex_;
        std::atomic<bool> running_;

        // NMEA数据缓冲区
        std::string nmea_buffer_;

        // frame_id
        std::string frame_id_;

        // 控制话题消息是否发布
        bool publish_gps_fix_;
        bool publish_gps_time_reference_;
        bool publish_gps_velocity_;
        bool publish_gps_nmea_sentence_;
        bool publish_gps_nmea_gpgsa_;
        bool publish_gps_nmea_gpgsv_;
        bool publish_gps_nmea_gprmc_;

        // 初始化串口
        bool init_serial();

        // 读取串口数据的线程函数
        void read_serial_data();

        // 处理NMEA句子
        void process_nmea_sentence(const std::string &sentence);

        // 检查NMEA校验和
        bool check_checksum(const std::string &sentence);

        // 辅助函数：将NMEA坐标格式转换为十进制
        double nmea_to_decimal(double value, char direction);

        // GPS 美国
        void parse_gpgga(const std::string &sentence);
        void parse_gpgsa(const std::string &sentence);
        void parse_gpgsv(const std::string &sentence);
        void parse_gprmc(const std::string &sentence);

        // 北斗 中国
        void parse_bdgga(const std::string &sentence);
        void parse_bdgsa(const std::string &sentence);
        void parse_bdgsv(const std::string &sentence);
        void parse_bdrmc(const std::string &sentence);

        // GLONASS 俄罗斯
        void parse_glgga(const std::string &sentence);
        void parse_glgsa(const std::string &sentence);
        void parse_glgsv(const std::string &sentence);
        void parse_glrmc(const std::string &sentence);

        // Galileo 欧盟
        void parse_gagga(const std::string &sentence);
        void parse_gagsa(const std::string &sentence);
        void parse_gagsv(const std::string &sentence);
        void parse_garmc(const std::string &sentence);

        // 多系统联合定位
        void parse_gngga(const std::string &sentence);
        void parse_gngsa(const std::string &sentence);
        void parse_gngsv(const std::string &sentence);
        void parse_gnrmc(const std::string &sentence);

        // 辅助方法
        std::vector<std::string> split_nmea_fields(const std::string &sentence);

        bool string_to_double(const std::string &s, double &val);

        bool string_to_int(const std::string &s, int &val);
    };
} // unicorecomm_gps_node

#endif //UNICORECOMM_GPS_UNICORECOMMGPSNODE_HPP
