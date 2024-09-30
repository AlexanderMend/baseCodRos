#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "roboclaw.hpp"  // Incluye tu librería Roboclaw

// Inicializa Roboclaw con el puerto adecuado y baudrate
Roboclaw roboclaw("/dev/ttyACM0", 115200);  

void commandCallback(const std_msgs::Int32::ConstPtr& msg) {
    int32_t speed = msg->data;  // Recibe el valor de velocidad desde el publicador
    uint8_t address = 0x80;     // Dirección del Roboclaw

    float Kp1 = 2.12612, Ki1 = 0.33859, Kd1 = 0.0;
    //float Kp2 = 4.8033, Ki2 = 0.97178, Kd2 = 0.0;
    uint32_t qpps1 = 12187;  // Pulsos por segundo
    //uint32_t qpps2 = 4500;

    roboclaw.SetM1VelocityPID(0x80, Kp1, Ki1, Kd1, qpps1);
    //roboclaw.SetM2VelocityPID(0x80, Kp2, Ki2, Kd2, qpps2);


    // Establece la velocidad del motor 1 usando la función SpeedM1
    if (roboclaw.SpeedM1(address, speed)) {
        ROS_INFO("Motor M1 set to speed: %d", speed);
    } else {
        ROS_ERROR("Failed to set speed for Motor M1");
    }

    // Establece la velocidad del motor 2 usando la función SpeedM2
    if (roboclaw.SpeedM2(address, speed-speed)) {
        ROS_INFO("Motor M2 set to speed: %d", speed);
    } else {
        ROS_ERROR("Failed to set speed for Motor M2");
    }
    
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n;

    // No se requiere "connect()" ya que al crear la instancia de Roboclaw, el puerto se abre.

    ros::Subscriber sub = n.subscribe("motor_speed", 1000, commandCallback);

    // Publicador para monitorear la velocidad del motor 1
    ros::Publisher speed_pub = n.advertise<std_msgs::Int32>("motor1_speed_feedback", 1000);

    ros::Rate loop_rate(10);  // Frecuencia de 10Hz

    while (ros::ok()) {
        uint8_t status;
        bool valid;
        int32_t motor_speed1 = roboclaw.ReadSpeedM1(0x80, &status, &valid);  // Lee la velocidad de M1

        if (valid) {
            std_msgs::Int32 speed_msg;
            speed_msg.data = motor_speed1;
            speed_pub.publish(speed_msg);  // Publica la velocidad actual del motor 1
        } else {
            ROS_ERROR("Failed to read motor speed M1");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
