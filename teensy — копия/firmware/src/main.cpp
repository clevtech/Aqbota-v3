// #include <Arduino.h>
#include <WiFi.h>
#include "analogWrite.h"
#include "ros.h"
#include "ros/time.h"
//header file for publishing velocities for odom
#include "lino_msgs/Velocities.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
//header file for imu
#include "lino_msgs/Imu.h"

#include "lino_base_config.h"


#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 5

const char* ssid     = "Friends";
const char* password = "The2ndl@w";
// Set the rosserial socket server IP address
IPAddress server(192,168,1,149);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;


// CONSTANTS

int MOTOR_A_FWD = 13;
int MOTOR_A_BCW = 12;
int MOTOR_B_FWD = 14;
int MOTOR_B_BCW = 27;

int L_ENC = 34;
int R_ENC = 35;


float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long prev_control_time = 0;
unsigned long prev_debug_time = 0;


float last_dir1 = 1;
float last_dir2 = 1;

unsigned long g_prev_command_time = 0;

void commandCallback(const geometry_msgs::Twist& cmd_msg);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);


lino_msgs::Velocities raw_vel_msg;
ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void moveBase();
void stopBase();
void printDebug();
void getRPM();

// Encoder part
volatile int interruptCounterL = 0;
int numberOfInterruptsL = 0;
int lastL = millis();
int deltaL = 0;
int nowL = 0;
double RPML = 0;
int echoL = 0;
volatile int interruptCounterR = 0;
int numberOfInterruptsR = 0;
int lastR = millis();
int deltaR = 0;
int nowR = 0;
double RPMR = 0;
int echoR = 0;


portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR leftEncoder() {
  portENTER_CRITICAL_ISR(&mux);
  interruptCounterL = interruptCounterL + 1;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR rightEncoder() {
  portENTER_CRITICAL_ISR(&mux);
  interruptCounterR = interruptCounterR + 1;
  portEXIT_CRITICAL_ISR(&mux);
}


void setup()
{   
    pinMode(L_ENC, INPUT_PULLUP);
    pinMode(R_ENC, INPUT_PULLUP);
    pinMode(MOTOR_A_FWD, OUTPUT);
    pinMode(MOTOR_B_FWD, OUTPUT);
    pinMode(MOTOR_A_BCW, OUTPUT);
    pinMode(MOTOR_B_BCW, OUTPUT);
      // Use ESP8266 serial to monitor the process
    Serial.begin(115200);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    // Connect the ESP8266 the the wifi AP
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    nh.initNode();
    Serial.println("Inited");
    // nh.getHardware()->setBaud(57600);
    nh.getHardware()->setConnection(server, serverPort);
    nh.subscribe(cmd_sub);
    // nh.advertise(raw_vel_pub);

    Serial.println("Connecting...");
    while (!nh.connected())
    {
        nh.spinOnce();
    }
    
    Serial.println("CONNECTED");
    // attachInterrupt(digitalPinToInterrupt(L_ENC), leftEncoder, RISING);
    // attachInterrupt(digitalPinToInterrupt(R_ENC), rightEncoder, RISING);
    Serial.println("CONNECTED...");
    nh.loginfo("LINOBASE CONNECTED");
    
}

void loop()
{
    Serial.println("CONNECTED!");

    //this block drives the robot based on defined rate
    if ((millis() - prev_control_time) >= (100))
    {
        moveBase();
        prev_control_time = millis();
    }

    //this block stops the motor when no command is received
    if ((millis() - g_prev_command_time) >= 400)
    {
        stopBase();
        moveBase();
    }


    //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(DEBUG)
    {
        if ((millis() - prev_debug_time) >= (2000))
        {
            printDebug();
            prev_debug_time = millis();
        }
    }
    //call all the callbacks waiting to be called
    nh.spinOnce();
}


void commandCallback(const geometry_msgs::Twist& cmd_msg)
{   
    g_req_linear_vel_x = cmd_msg.linear.x;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = cmd_msg.angular.z;
    g_prev_command_time = millis();
}

void moveBase()
{
    int pwmR = 0;
    int pwmL = 0;
    int dirR = 0;
    int dirL = 0;
    
    pwmR = (g_req_linear_vel_x - g_req_angular_vel_z/2)*255;
    pwmL = (g_req_linear_vel_x + g_req_angular_vel_z/2)*255;

    pwmR = constrain(pwmR, -255, 255);
    pwmL = constrain(pwmL, -255, 255);

    if(pwmL>0){
        analogWrite(MOTOR_A_BCW, 0);
        analogWrite(MOTOR_A_FWD, abs(pwmL));
        dirL = 1;
    }
    else{
        analogWrite(MOTOR_A_FWD, 0);
        analogWrite(MOTOR_A_BCW, abs(pwmL));
        dirL = -1;
    }
    if(pwmR>0){
        analogWrite(MOTOR_B_BCW, 0);
        analogWrite(MOTOR_B_FWD, abs(pwmR));
        dirR = 1;
    }
    else{
        analogWrite(MOTOR_B_FWD, 0);
        analogWrite(MOTOR_B_BCW, abs(pwmR));
        dirR = -1;
    }

    getRPM();

    //convert average revolutions per minute to revolutions per second
    float average_rps_x = ((float)(dirL*RPML + dirR*RPMR) / 2) / 60; // RPM
    raw_vel_msg.linear_x = average_rps_x * 0.54; // m/s
    raw_vel_msg.linear_y = 0;

    //convert average revolutions per minute to revolutions per second
    float average_rps_a = ((float)(-dirL*RPML + dirR*RPMR) / 2) / 60;
    raw_vel_msg.angular_z =  (average_rps_a * 0.54) / (0.25 / 2); //  rad/s

    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
    g_req_linear_vel_x = 0;
    g_req_linear_vel_y = 0;
    g_req_angular_vel_z = 0;
}


void printDebug()
{
    getRPM();
    char buffer[50];
    sprintf (buffer, "RPM left  : %f", RPML);
    nh.loginfo(buffer);
    sprintf (buffer, "RPM right : %f", RPMR);
    nh.loginfo(buffer);
}

void getRPM(){
    echoL = millis();
    echoR = echoL;
    portENTER_CRITICAL(&mux);
    numberOfInterruptsL = interruptCounterL;
    interruptCounterL = 0;
    nowL = millis();
    deltaL = nowL - lastL;
    lastL = nowL;
    numberOfInterruptsR = interruptCounterR;
    interruptCounterR = 0;
    deltaR = nowL - lastR;
    lastR = nowR;
    portEXIT_CRITICAL(&mux);
    RPML = (numberOfInterruptsL*60000)/(2*8*deltaL);
    RPMR = (numberOfInterruptsR*60000)/(2*8*deltaR);
}