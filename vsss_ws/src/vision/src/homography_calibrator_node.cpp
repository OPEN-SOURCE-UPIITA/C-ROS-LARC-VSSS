#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>

class HomographyCalibratorNode : public rclcpp::Node {
public:
    HomographyCalibratorNode() : Node("calibrador_homografia") {
        rclcpp::SensorDataQoS qos_sensor;
        sub_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/vsss/image", qos_sensor,
            std::bind(&HomographyCalibratorNode::image_callback, this, std::placeholders::_1));

        rclcpp::QoS qos_transient(1);
        qos_transient.transient_local();
        pub_homography_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/vision/homography_matrix", qos_transient);

        // Coordenadas reales (en metros) de las 4 cruces '+' exteriores (LARC VSSS)
        pts_world_ = {
            {-0.375f,  0.40f}, // 1. Superior Izquierda
            { 0.375f,  0.40f}, // 2. Superior Derecha
            { 0.375f, -0.40f}, // 3. Inferior Derecha
            {-0.375f, -0.40f}  // 4. Inferior Izquierda
        };

        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(window_name_, &HomographyCalibratorNode::on_mouse, this);

        RCLCPP_INFO(this->get_logger(),
            "Clic izq: marcar cruces (1:Sup-Izq, 2:Sup-Der, 3:Inf-Der, 4:Inf-Izq).");
        RCLCPP_INFO(this->get_logger(),
            "Rueda: zoom | Clic der + arrastrar: mover | [z] reset zoom | [r] reiniciar | [s] guardar | [ESC] salir.");
    }

private:
    void clamp_offsets() {
        if (img_w_ <= 0 || img_h_ <= 0) return;
        double roi_w = img_w_ / zoom_;
        double roi_h = img_h_ / zoom_;
        offset_x_ = std::clamp(offset_x_, 0.0, std::max(0.0, img_w_ - roi_w));
        offset_y_ = std::clamp(offset_y_, 0.0, std::max(0.0, img_h_ - roi_h));
    }

    cv::Point2f screen_to_image(int mx, int my) const {
        return cv::Point2f(
            static_cast<float>(offset_x_ + mx / zoom_),
            static_cast<float>(offset_y_ + my / zoom_)
        );
    }

    static void on_mouse(int event, int x, int y, int flags, void* userdata) {
        auto* self = static_cast<HomographyCalibratorNode*>(userdata);
        self->mouse_pos_ = cv::Point(x, y);

        if (event == cv::EVENT_LBUTTONDOWN) {
            if (self->pts_px_.size() < 4 && self->img_w_ > 0) {
                cv::Point2f img_pt = self->screen_to_image(x, y);
                self->pts_px_.push_back(img_pt);
                RCLCPP_INFO(self->get_logger(), "Punto %zu guardado: (%.2f, %.2f)",
                            self->pts_px_.size(), img_pt.x, img_pt.y);
            }
        } else if (event == cv::EVENT_MOUSEWHEEL) {
            int delta = cv::getMouseWheelDelta(flags);
            cv::Point2f before = self->screen_to_image(x, y);

            if (delta > 0) self->zoom_ = std::min(10.0, self->zoom_ * 1.25);
            else           self->zoom_ = std::max(1.0,  self->zoom_ / 1.25);

            // Mantener el punto bajo el cursor en la misma posición
            self->offset_x_ = before.x - x / self->zoom_;
            self->offset_y_ = before.y - y / self->zoom_;
            self->clamp_offsets();
        } else if (event == cv::EVENT_RBUTTONDOWN) {
            self->is_panning_ = true;
            self->pan_start_ = cv::Point(x, y);
        } else if (event == cv::EVENT_RBUTTONUP) {
            self->is_panning_ = false;
        } else if (event == cv::EVENT_MOUSEMOVE && self->is_panning_) {
            int dx = x - self->pan_start_.x;
            int dy = y - self->pan_start_.y;
            self->offset_x_ -= dx / self->zoom_;
            self->offset_y_ -= dy / self->zoom_;
            self->clamp_offsets();
            self->pan_start_ = cv::Point(x, y);
        }
    }

    void publish_and_save() {
        if (pts_px_.size() != 4) {
            RCLCPP_WARN(this->get_logger(), "Necesitas seleccionar exactamente 4 puntos.");
            return;
        }

        cv::Mat H = cv::getPerspectiveTransform(pts_px_, pts_world_);

        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(9);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                msg.data[r * 3 + c] = H.at<double>(r, c);
            }
        }
        pub_homography_->publish(msg);

        cv::FileStorage fs("/tmp/homography.yaml", cv::FileStorage::WRITE);
        fs << "H" << H;
        fs.release();

        RCLCPP_INFO(this->get_logger(), "Homografia calculada, publicada y guardada en /tmp/homography.yaml");
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            img_w_ = frame.cols;
            img_h_ = frame.rows;
            clamp_offsets();

            // Dibujar puntos y líneas en la imagen original antes del recorte
            for (size_t i = 0; i < pts_px_.size(); ++i) {
                cv::circle(frame, pts_px_[i], 2, cv::Scalar(0, 255, 0), -1);
                cv::putText(frame, std::to_string(i + 1),
                            cv::Point(static_cast<int>(pts_px_[i].x) + 5, static_cast<int>(pts_px_[i].y) - 5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
            }
            if (pts_px_.size() == 4) {
                for (int i = 0; i < 4; ++i) {
                    cv::line(frame, pts_px_[i], pts_px_[(i + 1) % 4], cv::Scalar(255, 0, 0), 1);
                }
            }

            // Aplicar recorte (ROI) y escalado para el zoom
            int roi_x = static_cast<int>(offset_x_);
            int roi_y = static_cast<int>(offset_y_);
            int roi_w = std::max(1, std::min(img_w_ - roi_x, static_cast<int>(img_w_ / zoom_)));
            int roi_h = std::max(1, std::min(img_h_ - roi_y, static_cast<int>(img_h_ / zoom_)));

            cv::Mat display;
            cv::resize(frame(cv::Rect(roi_x, roi_y, roi_w, roi_h)), display,
                       cv::Size(img_w_, img_h_), 0, 0, cv::INTER_NEAREST);

            // Mira de precisión en el cursor
            cv::line(display, cv::Point(mouse_pos_.x - 12, mouse_pos_.y),
                     cv::Point(mouse_pos_.x + 12, mouse_pos_.y), cv::Scalar(0, 0, 255), 1);
            cv::line(display, cv::Point(mouse_pos_.x, mouse_pos_.y - 12),
                     cv::Point(mouse_pos_.x, mouse_pos_.y + 12), cv::Scalar(0, 0, 255), 1);

            // Indicador de zoom en la esquina superior izquierda
            std::string zoom_txt = "Zoom: " + std::to_string(zoom_).substr(0, 4) + "x";
            cv::putText(display, zoom_txt, cv::Point(10, 25),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

            cv::imshow(window_name_, display);
            char key = static_cast<char>(cv::waitKey(1));

            if (key == 'r' || key == 'R') {
                pts_px_.clear();
                RCLCPP_INFO(this->get_logger(), "Puntos reiniciados.");
            } else if (key == 'z' || key == 'Z') {
                zoom_ = 1.0;
                offset_x_ = 0.0;
                offset_y_ = 0.0;
            } else if (key == 's' || key == 'S') {
                publish_and_save();
            } else if (key == 27 || cv::getWindowProperty(window_name_, cv::WND_PROP_VISIBLE) < 1) {
                rclcpp::shutdown();
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error cv_bridge: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_homography_;
    std::vector<cv::Point2f> pts_px_;
    std::vector<cv::Point2f> pts_world_;
    const std::string window_name_ = "Calibracion Homografia (4 Cruces)";

    // Variables de zoom y pan
    int img_w_ = 0, img_h_ = 0;
    double zoom_ = 1.0;
    double offset_x_ = 0.0, offset_y_ = 0.0;
    bool is_panning_ = false;
    cv::Point pan_start_{0, 0};
    cv::Point mouse_pos_{0, 0};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HomographyCalibratorNode>());
    rclcpp::shutdown();
    return 0;
}