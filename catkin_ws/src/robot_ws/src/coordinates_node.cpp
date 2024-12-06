#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <cmath>

float current_x = 0.0;
float current_y = 0.0;

void moveToTarget(float target_x, float target_y, ros::Publisher& distance_pub, ros::Publisher& angle_pub) {
    // Calcula la distancia y el ángulo hacia el objetivo
    float delta_x = target_x - current_x;
    float delta_y = target_y - current_y;

    // Calcula la distancia euclidiana
    float distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);

    // Calcula el ángulo a girar en radianes
    float angle = std::atan2(delta_y, delta_x);
    int angle_deg = static_cast<int>(angle * 180.0 / M_PI);  // Convertir a grados

    // Publica el ángulo para rotar
    std_msgs::Int32 angle_msg;
    angle_msg.data = angle_deg;
    angle_pub.publish(angle_msg);
    ROS_INFO("Rotating to angle: %d degrees", angle_deg);

    // Simula tiempo para completar el giro
    ros::Duration(1.0).sleep();

    // Publica la distancia para avanzar
    std_msgs::Int32 distance_msg;
    distance_msg.data = static_cast<int>(distance);
    distance_pub.publish(distance_msg);
    ROS_INFO("Moving forward: %.2f meters", distance);

    // Actualiza la posición actual
    current_x = target_x;
    current_y = target_y;
}

void targetCallback(const std_msgs::Float32MultiArray::ConstPtr& msg, ros::Publisher& distance_pub, ros::Publisher& angle_pub) {
    if (msg->data.size() < 2) {
        ROS_WARN("Received insufficient data for target coordinates");
        return;
    }

    float target_x = msg->data[0];
    float target_y = msg->data[1];
    ROS_INFO("Received target coordinates: (%.2f, %.2f)", target_x, target_y);

    // Mueve el robot hacia las coordenadas objetivo
    moveToTarget(target_x, target_y, distance_pub, angle_pub);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinate_controller");
    ros::NodeHandle nh;

    // Publisher para los tópicos de control de distancia y ángulo
    ros::Publisher distance_pub = nh.advertise<std_msgs::Int32>("motor_distanceMeter", 10);
    ros::Publisher angle_pub = nh.advertise<std_msgs::Int32>("motor_AngleAxis", 10);

    // Subscriber para las coordenadas objetivo
    ros::Subscriber target_sub = nh.subscribe<std_msgs::Float32MultiArray>(
        "target_coordinates", 10,
        boost::bind(targetCallback, _1, boost::ref(distance_pub), boost::ref(angle_pub))
    );

    ros::spin();

    return 0;
}
