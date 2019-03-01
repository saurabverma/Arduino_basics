/* 
 * Aim : This ROS code runs on an Arduino platform to collect ADC data from four
 * IR sensors and publishes the data on the respective topics. Further, the
 * approximate sampling rate is allowable for tuning using a subscriber. Please
 * see corresponding IR sensor manuals for pin connection information.
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>

// NOTE: User input varialbes
const int sensor_0_pin = A0;
const int sensor_1_pin = A1;
const int sensor_2_pin = A2;
const int sensor_3_pin = A3;

const char frameid_0[] = "/IR_sensor_0";
const char frameid_1[] = "/IR_sensor_1";
const char frameid_2[] = "/IR_sensor_2";
const char frameid_3[] = "/IR_sensor_3";

long delay_time = 50; // ms, time between two consecutive sample publish

// NOTE: For GP2Y0A710K0F only. Adjust for other IR rangers using respective manual
const float min_range = 0.45;     // m
const float max_range = 5.50;     // m
const float field_of_view = 0.01; // rad
const float min_volt = 1.4;       // V, for max_range because of inverse proportionality
const float max_volt = 3.1;       // V, for min_range because of inverse proportionality

// NOTE: Internal global variables, please DO NOT update
ros::NodeHandle nh;

sensor_msgs::Range sensor_0_msg;
sensor_msgs::Range sensor_1_msg;
sensor_msgs::Range sensor_2_msg;
sensor_msgs::Range sensor_3_msg;

ros::Publisher pub_range_0("IR_sensor_0_range", &sensor_0_msg);
ros::Publisher pub_range_1("IR_sensor_1_range", &sensor_1_msg);
ros::Publisher pub_range_2("IR_sensor_2_range", &sensor_2_msg);
ros::Publisher pub_range_3("IR_sensor_3_range", &sensor_3_msg);

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// convert voltage to ADC range, then constrain within ADC range
const float min_val = constrain(map_float(min_volt, 0, 5, 0, 1023), 0, 1023);
const float max_val = constrain(map_float(max_volt, 0, 5, 0, 1023), 0, 1023);

float sensor_0_range = 0.0;
float sensor_1_range = 0.0;
float sensor_2_range = 0.0;
float sensor_3_range = 0.0;

unsigned long range_timer;

void setPeriod(const std_msgs::Float32 &rate)
{
    delay_time = 1000.0 / rate.data;
}

ros::Subscriber<std_msgs::Float32> sub_rate("approx_sample_rate", &setPeriod);

void setup()
{
    // Setup node and publishers
    nh.initNode();
    nh.subscribe(sub_rate);
    nh.advertise(pub_range_0);
    nh.advertise(pub_range_1);
    nh.advertise(pub_range_2);
    nh.advertise(pub_range_3);

    // Prepapre pins
    pinMode(sensor_0_pin, INPUT);
    pinMode(sensor_1_pin, INPUT);
    pinMode(sensor_2_pin, INPUT);
    pinMode(sensor_3_pin, INPUT);

    // Setup publishing topics
    sensor_0_msg.header.frame_id = frameid_0;
    sensor_0_msg.radiation_type = sensor_msgs::Range::INFRARED;
    sensor_0_msg.min_range = min_range;
    sensor_0_msg.max_range = max_range;
    sensor_0_msg.field_of_view = field_of_view;

    sensor_1_msg.header.frame_id = frameid_1;
    sensor_1_msg.radiation_type = sensor_msgs::Range::INFRARED;
    sensor_1_msg.min_range = min_range;
    sensor_1_msg.max_range = max_range;
    sensor_1_msg.field_of_view = field_of_view;

    sensor_2_msg.header.frame_id = frameid_2;
    sensor_2_msg.radiation_type = sensor_msgs::Range::INFRARED;
    sensor_2_msg.min_range = min_range;
    sensor_2_msg.max_range = max_range;
    sensor_2_msg.field_of_view = field_of_view;

    sensor_3_msg.header.frame_id = frameid_3;
    sensor_3_msg.radiation_type = sensor_msgs::Range::INFRARED;
    sensor_3_msg.min_range = min_range;
    sensor_3_msg.max_range = max_range;
    sensor_3_msg.field_of_view = field_of_view;

    // Initial time
    range_timer = millis();
}

float convert_val_2_range(float val)
{
    return constrain(map_float(val, min_val, max_val, max_range, min_range), min_range, max_range); // NOTE: min_val to max_range and vice-versa
}

void prepare_data(void)
{
    // Initialize
    double sensor_0_val = 0;
    double sensor_1_val = 0;
    double sensor_2_val = 0;
    double sensor_3_val = 0;
    float n = 0;

    // Collect all data for sampling period
    while ((millis() - range_timer) < delay_time)
    {
        sensor_0_val += analogRead(sensor_0_pin);
        sensor_1_val += analogRead(sensor_1_pin);
        sensor_2_val += analogRead(sensor_2_pin);
        sensor_3_val += analogRead(sensor_3_pin);
        n += 1.0;
    }
    range_timer = millis();

    sensor_0_range = convert_val_2_range(sensor_0_val / n);
    sensor_1_range = convert_val_2_range(sensor_1_val / n);
    sensor_2_range = convert_val_2_range(sensor_2_val / n);
    sensor_3_range = convert_val_2_range(sensor_3_val / n);
}

void loop()
{
    // Average out data until the sampling period is over
    prepare_data();

    // Publish the new data
    sensor_0_msg.range = sensor_0_range;
    sensor_0_msg.header.stamp = nh.now();
    pub_range_0.publish(&sensor_0_msg);

    sensor_1_msg.range = sensor_1_range;
    sensor_1_msg.header.stamp = sensor_0_msg.header.stamp;
    pub_range_1.publish(&sensor_1_msg);

    sensor_2_msg.range = sensor_2_range;
    sensor_2_msg.header.stamp = sensor_0_msg.header.stamp;
    pub_range_2.publish(&sensor_2_msg);

    sensor_3_msg.range = sensor_3_range;
    sensor_3_msg.header.stamp = sensor_0_msg.header.stamp;
    pub_range_3.publish(&sensor_3_msg);

    // Spin once
    nh.spinOnce();
}
