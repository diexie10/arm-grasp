/********************************************************************************************************
  6自由度机械手逆运动学控制示例
  手爪和旋转手爪部位可通过自行设定角度，机械臂可通过识别给定角度位置

        S3  0°        S4  0°     S5        
      90° |---------------|-------|---  90°
          | 180°          180°       S6        
          |
          |
          |
          | 90°
   180°   | S2     0°
   -----------------
          S1
  
  舵机每个都调整好角度后再安装，设置角度为90度（中间角度），舵机分布安装上图；
  舵机S3,S4如果角度上为180°，下为0°则底部代码需将代码(180-theta3)改为theta3，theta4同理。
  S1，S2如安装角度方向相反也同样需要使用补角，改为(180-theta1)，(180-theta2)。

  @Author: YFROBOT-WST
  @Version: V1.0
  @Date: 05/30/2023
  @URL: www.yfrobot.com.cn
********************************************************************************************************/
#include <Servo.h>

Servo servo1;   // 旋转伺服
Servo servo2;   // 第二伺服
Servo servo3;   // 第三伺服
Servo servo4;   // 第四伺服
Servo servo5;   // 第五伺服
Servo gripper;  // 手爪伺服

int pin_servo1 = 8;    // 底部旋转舵机（黄色电缆）连接到arduino板上的引脚8
int pin_servo2 = 9;    // 抬臂1舵机（黄色电缆）连接到arduino板上的引脚9
int pin_servo3 = 10;   // 抬臂2舵机（黄色电缆）连接到arduino板上的引脚10
int pin_servo4 = 11;   // 抬臂3舵机（黄色电缆）连接到arduino板上的引脚11
int pin_servo5 = 12;   // 手爪旋转舵机（黄色电缆）连接到arduino板上的引脚12
int pin_gripper = 13;  // 手爪舵机（黄色电缆）连接到arduino板上的引脚13

float Pi = 3.1415926;  // π取值
float L1 = 72;         // 从表面到第二关节位置的第一个链接的高度  unit:mm
float L2 = 105;        // 从第2个关节到第3个关节的第二个链接的长度
float L3 = 128;        // 第3关节到第4关节的长度  145
float L4 = 170;        // 第4个关节到手臂尖端的长度（夹具）,包含手爪旋转舵机

float X_EE;  // 手爪的x轴坐标
float Y_EE;  // 手爪的y轴坐标
float Z_EE;  // 手爪的z轴坐标
float Zoffset, D, d, R;
float alpha1, alpha2, alpha3;
float Theta_1, Theta_2, Theta_3, Theta_4;

float min_PWM;  //default arduino 544
float max_PWM;  //default arduino 2400

void Inverse_kinematics(double X_EE, double Y_EE, double Z_EE) {
  if (X_EE > 0 && Z_EE >= L1) {
    D = sqrt(pow(X_EE, 2) + pow(Z_EE, 2));
    Theta_1 = (atan(Z_EE / X_EE)) * (180.00 / Pi);  //theta 1
    d = D - L4;
    Zoffset = Y_EE - L1;
    R = sqrt(pow(d, 2) + pow(Zoffset, 2));
    alpha1 = (acos(d / R)) * (180.00 / Pi);
    alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
    Theta_2 = (alpha1 + alpha2);                                                                //theta 2
    Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
    alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1);                                 //alpha3
    Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
  } else if (X_EE > 0 && Z_EE <= L1) {
    D = sqrt(pow(X_EE, 2) + pow(Z_EE, 2));
    Theta_1 = (atan(Z_EE / X_EE)) * (180.00 / Pi);  //theta 1
    d = D - L4;
    Zoffset = Y_EE - L1;
    R = sqrt(pow(d, 2) + pow(Zoffset, 2));
    alpha1 = (acos(d / R)) * (180.00 / Pi);
    alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
    Theta_2 = (alpha2 - alpha1);                                                                //theta 2
    Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
    alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1);                                 //alpha3
    Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
  } else if (X_EE == 0 && Z_EE >= L1) {
    D = sqrt(pow(X_EE, 2) + pow(Y_EE, 2));
    Theta_1 = 90.00;  //theta 1
    d = D - L4;
    Zoffset = Z_EE - L1;
    R = sqrt(pow(d, 2) + pow(Zoffset, 2));
    alpha1 = (acos(d / R)) * (180.00 / Pi);
    alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
    Theta_2 = alpha1 + alpha2;                                                                  //theta 2
    Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
    alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1);                                 //alpha3
    Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
  } else if (X_EE == 0 && Z_EE <= L1) {
    D = sqrt(pow(X_EE, 2) + pow(Z_EE, 2));
    Theta_1 = 90.00;  //theta 1
    d = D - L4;
    Zoffset = Y_EE - L1;
    R = sqrt(pow(d, 2) + pow(Zoffset, 2));
    alpha1 = (acos(d / R)) * (180.00 / Pi);
    alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
    Theta_2 = (alpha2 - alpha1);                                                                //theta 2
    Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
    alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1);                                 //alpha3
    Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
  } else if (X_EE < 0 && Z_EE >= L1) {
    D = sqrt(pow(X_EE, 2) + pow(Z_EE, 2));
    Theta_1 = 90.00 + (90.00 - abs((atan(Z_EE / X_EE)) * (180.00 / Pi)));  //theta 1
    d = D - L4;
    Zoffset = Y_EE - L1;
    R = sqrt(pow(d, 2) + pow(Zoffset, 2));
    alpha1 = (acos(d / R)) * (180.00 / Pi);
    alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
    Theta_2 = (alpha1 + alpha2);                                                                //theta 2
    Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
    alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) - alpha1);                                 //alpha3
    Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
  } else if (X_EE < 0 && Z_EE <= L1) {
    D = sqrt(pow(X_EE, 2) + pow(Z_EE, 2));
    Theta_1 = 90.00 + (90.00 - abs((atan(Z_EE / X_EE)) * (180.00 / Pi)));  //theta 1
    d = D - L4;
    Zoffset = Y_EE - L1;
    R = sqrt(pow(d, 2) + pow(Zoffset, 2));
    alpha1 = (acos(d / R)) * (180.00 / Pi);
    alpha2 = (acos((pow(L2, 2) + pow(R, 2) - pow(L3, 2)) / (2 * L2 * R))) * (180.00 / Pi);
    Theta_2 = (alpha2 - alpha1);                                                                //theta 2
    Theta_3 = ((acos((pow(L2, 2) + pow(L3, 2) - pow(R, 2)) / (2 * L2 * L3))) * (180.00 / Pi));  //theta 3
    alpha3 = 180.00 - ((180.00 - (alpha2 + Theta_3)) + alpha1);                                 //alpha3
    Theta_4 = 180 + 90 - alpha3;                                                                //theta 4
  }
}

void setup() {
  Serial.begin(9600);
  // servo1.attach(pin_servo1, min_PWM = 550.0, max_PWM = 550.00 + (180.00 / (209.00 / (2250.00 - 550.00))));
  servo1.attach(pin_servo1, min_PWM = 550.0, max_PWM = 2500.00);
  servo2.attach(pin_servo2, min_PWM = 500.0, max_PWM = 2500.00);
  servo3.attach(pin_servo3, min_PWM = 500.0, max_PWM = 2500.00);
  servo4.attach(pin_servo4, min_PWM = 500.0, max_PWM = 2500.00);
  servo5.attach(pin_servo5, min_PWM = 500.0, max_PWM = 2500.00);
  // servo4.attach(pin_servo4, min_PWM = 550.0, max_PWM = 550.00 + (180.00 / (164.00 / (2000.00 - 550.00))));
  gripper.attach(pin_gripper);  //gripper
}

void loop() {
  // for (int i = 100; i <= 250; i += 1) {
  //   Inverse_kinematics(0, 240, i);  // Inverse_kinematics(左右，前后，高度)  单位mm
  //   servo2.write(Theta_2);
  //   servo3.write(180 - Theta_3);
  //   servo4.write(180 - Theta_4);
  //   delay(10);
  // }
  // for (int i = 250; i >= 100; i -= 1) {
  //   Inverse_kinematics(0, 240, i);  // Inverse_kinematics(左右，前后，高度)  单位mm
  //   servo2.write(Theta_2);
  //   servo3.write(180 - Theta_3);
  //   servo4.write(180 - Theta_4);
  //   delay(10);
  // }

  Inverse_kinematics(0, 150, 200);  // Inverse_kinematics(左右，前后，高度)  单位mm
  servo1.write(Theta_1);
  delay(200);
  servo2.write(Theta_2);
  delay(200);
  servo3.write(180 - Theta_3);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
  delay(200);
  servo4.write(180 - Theta_4);  // 舵机安装方向导致角度不正确，选择补角，用180-计算得出的角度
  delay(200);
  servo5.write(90);
  delay(50);
  gripper.write(90);
  delay(50);

  Serial.print(" Servo: ");
  Serial.print(Theta_1);
  Serial.print("     ");
  Serial.print(Theta_2);
  Serial.print("     ");
  Serial.print(180 - Theta_3);
  Serial.print("     ");
  Serial.println(180 - Theta_4);
}
