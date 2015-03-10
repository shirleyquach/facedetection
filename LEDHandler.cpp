#include <fstream>
#include <iostream>
#include "ros/ros.h"
#include "selfiebot/Value.h"

#define LED_FILE "/sys/class/gpio/gpio206/value"
#define FREQUENCY 2.0

selfiebot::ValueConstPtr g_value;

void getValue(const selfiebot::ValueConstPtr value)
{
    g_value = value;
}

void write(ofstream file, char character)
{
    file.seekp(0);
    file << character;
}

void setLED(ros::Rate rate, std::ofstream file)
{
    switch ( g_value->value )
    {
        /* NO FACE DETECTED OR SLEEP MODE */
        case 0:
            write(file, '0');
            rate.sleep();
            rate.sleep();
            break;
        /* TAKING PICTURE: FACE DETECTED AND NO MOTION */
        case 1:
            write(file, '1');
            rate.sleep();
            write(file, '0');
            rate.sleep();
            break;
        /* PICTURE TAKEN; ON COOLDOWN */
        case 2:
            write(file, '1');
            rate.sleep();
            rate.sleep();
            break;
        default:
            break;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "led_handler");
    ros::NodeHandle n;

    ros:: Subscriber sub = n.subscribe("led_value", 1000, getMode)
    ros::Rate rate(FREQUENCY);
    ofstream file(LED_FILE);
    while ( ros::ok() )
    {
        setLED(rate, file);
        ros::spinOnce();
    }
    return 0;
}
