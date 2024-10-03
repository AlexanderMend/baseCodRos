#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "roboclaw.hpp"  // Incluir la cabecera de Roboclaw

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roboclaw_reader");
    ros::NodeHandle nh;

    // Crear una instancia de Roboclaw con puerto y baudrate adecuados
    Roboclaw roboclaw("/dev/ttyACM0", 115200);  // Ajusta el puerto y baudrate según corresponda

    // Crear publicadores para los datos
    ros::Publisher motor1_pub = nh.advertise<std_msgs::Int32>("motor1_encoder", 10);
    ros::Publisher motor2_pub = nh.advertise<std_msgs::Int32>("motor2_encoder", 10);
    ros::Publisher current_pub = nh.advertise<std_msgs::Float32>("motor_current", 10);

    ros::Rate loop_rate(10);  // Frecuencia de 10 Hz

    while (ros::ok())
    {
        // Leer valores de los encoders
        int32_t enc1 = roboclaw.ReadEncM1(0x80);  // Dirección predeterminada del Roboclaw
        int32_t enc2 = roboclaw.ReadEncM2(0x80);

        // Leer la corriente de los motores (corregido)
        int16_t current1, current2;
        if (roboclaw.ReadCurrents(0x80, current1, current2))
        {
            // Convertir la corriente a float si es necesario
            float current1_float = static_cast<float>(current1);
            float current2_float = static_cast<float>(current2);

            // Publicar corriente promedio
            std_msgs::Float32 current_msg;
            current_msg.data = (current1_float + current2_float) / 2.0;  // Promedio de las corrientes
            current_pub.publish(current_msg);
        }
        else
        {
            ROS_ERROR("Failed to read currents from Roboclaw");
        }

        // Publicar valores de encoders
        std_msgs::Int32 enc1_msg;
        enc1_msg.data = enc1;
        motor1_pub.publish(enc1_msg);

        std_msgs::Int32 enc2_msg;
        enc2_msg.data = enc2;
        motor2_pub.publish(enc2_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

