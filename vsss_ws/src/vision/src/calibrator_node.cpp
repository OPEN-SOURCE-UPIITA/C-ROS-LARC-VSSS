#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

// Variables globales para los sliders
int low_H = 0, low_S = 100, low_V = 100;
int high_H = 179, high_S = 255, high_V = 255;

class CalibratorNode : public rclcpp::Node {
public:
    CalibratorNode() : Node("calibrador_hsv") {
        // Declarar parámetro de ROS 2 (reemplaza a std::cin)
        this->declare_parameter<bool>("calibrar", false);
        bool modo_calibracion = this->get_parameter("calibrar").as_bool();

        // Obtener ruta dinámica del paquete
        try {
            std::string package_share_dir = ament_index_cpp::get_package_share_directory("vision");
            config_dir_ = package_share_dir + "/config";
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "No se encontro el paquete 'vision': %s", e.what());
            config_dir_ = "config";
        }

        yaml_path_ = config_dir_ + "/hsv_calibration.yaml";

        if (!fs::exists(config_dir_)) {
            fs::create_directories(config_dir_);
        }

        // Intentar cargar el YAML si existe
        bool archivo_cargado = false;
        if (fs::exists(yaml_path_)) {
            if (cargar_yaml()) {
                archivo_cargado = true;
                RCLCPP_INFO(this->get_logger(), "Archivo cargado: %s", yaml_path_.c_str());
                RCLCPP_INFO(this->get_logger(), "Valores: Low(%d,%d,%d) High(%d,%d,%d)", 
                            low_H, low_S, low_V, high_H, high_S, high_V);
            } else {
                RCLCPP_WARN(this->get_logger(), "Error al leer YAML. Usando valores por defecto.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No existe calibracion previa. Forzando modo calibracion...");
            modo_calibracion = true; // Si no hay archivo, abre la ventana automáticamente
        }

        // Configurar Publicador (Transient Local)
        rclcpp::QoS qos_profile_transient(1); 
        qos_profile_transient.transient_local();

        pub_calibration_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/vision/calibration_data", qos_profile_transient);

        // Publicar datos de inmediato
        publicar_datos();

        // Si NO se pidió calibrar y el archivo existía, terminamos la configuración aquí
        if (!modo_calibracion && archivo_cargado) {
            RCLCPP_INFO(this->get_logger(), "Modo ejecucion: Publicando calibracion guardada (sin ventana).");
            RCLCPP_INFO(this->get_logger(), "Para recalibrar, lanza con: calibrar:=true");
            return;
        }

        // --- MODO CALIBRACIÓN (CON VENTANA) ---
        is_calibrating_ = true;
        rclcpp::SensorDataQoS qos_profile_sensor;
        sub_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/vsss/image", 
            qos_profile_sensor,
            std::bind(&CalibratorNode::image_callback, this, std::placeholders::_1));

        cv::namedWindow("Calibracion", cv::WINDOW_NORMAL);
        cv::resizeWindow("Calibracion", 400, 300);
        cv::createTrackbar("Low H", "Calibracion", &low_H, 179);
        cv::createTrackbar("High H", "Calibracion", &high_H, 179);
        cv::createTrackbar("Low S", "Calibracion", &low_S, 255);
        cv::createTrackbar("High S", "Calibracion", &high_S, 255);
        cv::createTrackbar("Low V", "Calibracion", &low_V, 255);
        cv::createTrackbar("High V", "Calibracion", &high_V, 255);

        // Dibujar ventana inicial de inmediato
        cv::Mat blank = cv::Mat::zeros(150, 200, CV_8UC1);
        cv::imshow("Calibracion", blank);
        cv::waitKey(10);

        RCLCPP_INFO(this->get_logger(), "Modo calibracion activo. Presiona ESC o cierra la ventana para guardar.");
    }

private:
    void publicar_datos() {
        std_msgs::msg::Int32MultiArray calib_msg;
        calib_msg.data = {low_H, low_S, low_V, high_H, high_S, high_V};
        pub_calibration_->publish(calib_msg);
    }

    void guardar_yaml() {
        cv::FileStorage fs(yaml_path_, cv::FileStorage::WRITE);
        if (fs.isOpened()) {
            fs << "low_H" << low_H;
            fs << "low_S" << low_S;
            fs << "low_V" << low_V;
            fs << "high_H" << high_H;
            fs << "high_S" << high_S;
            fs << "high_V" << high_V;
            fs.release();
            RCLCPP_INFO(this->get_logger(), "Calibracion guardada en: %s", yaml_path_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Error al guardar en: %s", yaml_path_.c_str());
        }
    }

    bool cargar_yaml() {
        cv::FileStorage fs(yaml_path_, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;

        fs["low_H"] >> low_H;
        fs["low_S"] >> low_S;
        fs["low_V"] >> low_V;
        fs["high_H"] >> high_H;
        fs["high_S"] >> high_S;
        fs["high_V"] >> high_V;
        fs.release();
        return true;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!is_calibrating_) return;

        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::Mat hsv, mask, resized_mask;

            cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, cv::Scalar(low_H, low_S, low_V), cv::Scalar(high_H, high_S, high_V), mask);

            cv::resize(mask, resized_mask, cv::Size(), 0.5, 0.5); 
            cv::imshow("Calibracion", resized_mask);
            
            publicar_datos();

            char key = (char)cv::waitKey(1);
            if (key == 27 || cv::getWindowProperty("Calibracion", cv::WND_PROP_VISIBLE) < 1) {
                RCLCPP_INFO(this->get_logger(), "Guardando calibracion...");
                guardar_yaml();
                cv::destroyAllWindows();
                is_calibrating_ = false; // Deja de calibrar pero mantiene el nodo vivo publicando el último dato
                RCLCPP_INFO(this->get_logger(), "Ventana cerrada. El nodo seguira publicando los nuevos valores.");
            }

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        }
    }

    std::string config_dir_;
    std::string yaml_path_;
    bool is_calibrating_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_camera_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_calibration_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalibratorNode>());
    rclcpp::shutdown();
    return 0;
}
