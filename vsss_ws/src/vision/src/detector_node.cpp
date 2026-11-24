#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class DetectorNode : public rclcpp::Node {
public:
    DetectorNode() : Node("detector_vision") {
        // QoS para la cámara (SensorData)
        rclcpp::SensorDataQoS qos_profile_sensor;

        sub_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/vsss/image", 
            qos_profile_sensor,
            std::bind(&DetectorNode::image_callback, this, std::placeholders::_1));

        // QoS para Calibración
        rclcpp::QoS qos_profile_transient(1);
        qos_profile_transient.transient_local();
        
        sub_calibration_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/vision/calibration_data", qos_profile_transient,
            std::bind(&DetectorNode::calibration_callback, this, std::placeholders::_1));

        // Publicadores
        pub_point_ = this->create_publisher<geometry_msgs::msg::Point>("/vision/ball_position", 10);
        pub_debug_ = this->create_publisher<sensor_msgs::msg::Image>("/vision/debug_output", 10);

        // Valores iniciales por defecto (verde/amarillo de prueba)
        lower_hsv_ = cv::Scalar(0, 100, 100);
        upper_hsv_ = cv::Scalar(179, 255, 255);
        z_filtrada_ = 0.0f;

        RCLCPP_INFO(this->get_logger(), "Detector C-ROS reactivado con CLAHE y Filtro Alpha.");
    }

private:
    void calibration_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() == 6) {
            lower_hsv_ = cv::Scalar(msg->data[0], msg->data[1], msg->data[2]);
            upper_hsv_ = cv::Scalar(msg->data[3], msg->data[4], msg->data[5]);
            RCLCPP_INFO(this->get_logger(), "HSV actualizado.");
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 1. Convertir imagen ROS a OpenCV
            cv::Mat raw_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // 2. Rectificación de lente (actualizar matrices si cambia la cámara)
            cv::Mat frame_rect;
            cv::undistort(raw_frame, frame_rect, camera_matrix, dist_coeffs);

            // 3. Espacio de Color HSV y limpieza suave (sin CLAHE por defecto)
            cv::Mat hsv, mask;
            cv::cvtColor(frame_rect, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, lower_hsv_, upper_hsv_, mask);

            // Apertura morfológica ligera (1 iteración) para no borrar una pelota de ~10 px
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);

            // 4. Contornos con rango de área para 2m de altura
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            double max_area = 0;
            int max_idx = -1;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > 20.0 && area < 400.0 && area > max_area) {
                    max_area = area;
                    max_idx = i;
                }
            }

            // 5. Publicación de posición 2D (X, Y)
            if (max_idx >= 0) {
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(contours[max_idx], center, radius);

                auto point_msg = geometry_msgs::msg::Point();
                point_msg.x = center.x;
                point_msg.y = center.y;
                point_msg.z = 0.0; // Altura fija en visión superior
                pub_point_->publish(point_msg);

                // Dibujar UI en video
                cv::circle(frame_rect, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
                std::string info = "(" + std::to_string(static_cast<int>(center.x)) + ", " +
                                std::to_string(static_cast<int>(center.y)) + ")";
                cv::putText(frame_rect, info, cv::Point(center.x + 10, center.y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            }

            // 6. Publicar imagen Debug
            sensor_msgs::msg::Image::SharedPtr debug_msg =
                cv_bridge::CvImage(msg->header, "bgr8", frame_rect).toImageMsg();
            pub_debug_->publish(*debug_msg);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_calibration_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_point_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;
    
    cv::Scalar lower_hsv_, upper_hsv_;
    float z_filtrada_;
    const float ALPHA = 0.2f;

    // Matriz intrínseca exacta para 900x700 con HFOV = 1.22 rad
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 643.85, 0.0,    450.0, 0.0,    643.85, 350.0, 0.0,    0.0,    1.0);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1,5) << 0.0, 0.0, 0.0, 0.0, 0.0);
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectorNode>());
    rclcpp::shutdown();
    return 0;
}