#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "roboclaw.hpp"

Roboclaw roboclaw("/dev/ttyACM0", 115200);  // Configura el puerto y la velocidad baud

// Callback para recibir la velocidad desde el publicador
void commandCallback(const std_msgs::Int32::ConstPtr& msg) {
    int32_t motor_speed = msg->data;
    uint8_t address = 0x80;  // Dirección del Roboclaw
    
    // Controla el motor 1 con la velocidad recibida
    bool success = roboclaw.SpeedM1(address, motor_speed);
    
    if(success) {
        ROS_INFO("Motor 1 está recibiendo velocidad: %d", motor_speed);
    } else {
        ROS_ERROR("Error al enviar la velocidad al motor 1.");
    }
}

// Función para leer y publicar la velocidad del motor 1
void publishMotorSpeed(ros::Publisher& pub) {
    uint8_t address = 0x80;
    uint8_t status;
    bool valid;
    
    int32_t speed = roboclaw.ReadSpeedM1(address, &status, &valid);
    if(valid) {
        std_msgs::Int32 speed_msg;
        speed_msg.data = speed;
        pub.publish(speed_msg);
        ROS_INFO("Velocidad actual del motor 1: %d", speed);
    } else {
        ROS_ERROR("Error al leer la velocidad del motor 1.");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_control_node");
    ros::NodeHandle nh;
    
    // Subscriber para recibir comandos de velocidad
    ros::Subscriber sub = nh.subscribe("motor_speed_command", 10, commandCallback);
    
    // Publisher para publicar la velocidad actual del motor 1
    ros::Publisher speed_pub = nh.advertise<std_msgs::Int32>("motor_speed_feedback", 10);
    
    ros::Rate loop_rate(10);  // Frecuencia de 10 Hz
    
    while (ros::ok()) {
        publishMotorSpeed(speed_pub);  // Publica la velocidad actual del motor 1
        
        ros::spinOnce();  // Llama los callbacks
        loop_rate.sleep();
    }
    
    return 0;
}
