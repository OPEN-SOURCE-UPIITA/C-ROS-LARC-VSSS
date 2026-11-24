#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <opencv2/opencv.hpp>

class HomographyNode : public rclcpp::Node {
public:
    HomographyNode() : Node("homography_mapper") {
        // Intentar cargar calibración previa si existe
        cv::FileStorage fs("/tmp/homography.yaml", cv::FileStorage::READ);
        if (fs.isOpened()) {
            fs["H"] >> H_;
            fs.release();
            if (H_.rows == 3 && H_.cols == 3) {
                has_homography_ = true;
                RCLCPP_INFO(this->get_logger(), "Homografia cargada desde /tmp/homography.yaml");
            }
        }

        rclcpp::QoS qos_transient(1);
        qos_transient.transient_local();
        sub_homography_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/vision/homography_matrix", qos_transient,
            std::bind(&HomographyNode::homography_callback, this, std::placeholders::_1));

        sub_ball_px_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/vision/ball_position", 10,
            std::bind(&HomographyNode::ball_callback, this, std::placeholders::_1));

        pub_ball_world_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/vision/ball_world", 10);

        RCLCPP_INFO(this->get_logger(), "HomographyNode listo. Publicando en /vision/ball_world.");
    }

private:
    void homography_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 9) return;
        H_ = cv::Mat(3, 3, CV_64F);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                H_.at<double>(r, c) = msg->data[r * 3 + c];
            }
        }
        has_homography_ = true;
        RCLCPP_INFO(this->get_logger(), "Matriz de homografia actualizada desde topico.");
    }

    void ball_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        if (!has_homography_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                                 "Esperando matriz de homografia...");
            return;
        }

        std::vector<cv::Point2f> src = {
            cv::Point2f(static_cast<float>(msg->x), static_cast<float>(msg->y))
        };
        std::vector<cv::Point2f> dst(1);
        cv::perspectiveTransform(src, dst, H_);

        geometry_msgs::msg::Point world_msg;
        world_msg.x = dst[0].x;
        world_msg.y = dst[0].y;
        world_msg.z = 0.0;
        pub_ball_world_->publish(world_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_homography_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_ball_px_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_ball_world_;

    cv::Mat H_;
    bool has_homography_ = false;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HomographyNode>());
    rclcpp::shutdown();
    return 0;
}