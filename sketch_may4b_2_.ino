#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#define IN1 50
#define IN2 51
#define IN3 48
#define IN4 49
#define ENA 9//PWMA
#define ENB 8//PWMB
#define STANDBY_PIN 52    // 驱动芯片使能引脚
#define k5     35
#define k4     37
#define k3     39  //五个循迹灯，灰度传感器
#define k2     41
#define k1     43
#define R 13
#define G 12
#define B 11
#define weigh 7
#define TX_1   18  
#define RX_1   19
int num=0;


Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;


String inputBuffer = "";
unsigned long lastCmdTime = 0;
int crossing=0; //十字路口的数量，到达指定值时转弯
int num1,num2;
float gx, gy, gz;
float gx1, gy1, gz1, gx2, gy2, gz2;
float gyro_yaw = 0; //最终解算得到的角度（航向角）
float gz0;
float pi=3.1415926536;
  float Kp=15;
  float Ki=0;
  float Kd=8;
  float PID;//算出的PID量，总的，
  float r_speed, l_speed;
  float summ =0;//针对Ki的误差累积量
  float err=0;
  float old_err=0;
  float speed=50; //预期速度，慢一点稳定一些
int a=1;
unsigned long now,lastTime;
float dt;
bool go_ok = false; //确认满足过去路径
bool back_ok = false;//确认满足回来路径
int a1,a2,a3,a4,a5,b1,b2,b3;



void setpwm(int motor, int pwm); //控制电机的函数
void angle(); //测量角度的函数
void PID_xunxian_go();//过去的巡线代码
void PID_xunxian_back();//回来的巡线代码
void turn_off(); //关灯
void stop(); //停车
void turn(int direction,float Angle);  //转弯代码
void Serial_input(); //利用串口得到数字


void Serial_input()
{
    if (Serial1.available()) {
    String data = Serial1.readStringUntil('\n');
    //delay(20);
    Serial.print("Received: ");
    Serial.println(data); // 在串口监视器查看接收内容
    num=data.toInt();//利用串口得到num的值
    if (num>2)  //测试用代码，用于优化速度
   {
    speed=80;
   }
  }
}



void stop()
{
  setpwm(1,0);
  setpwm(2,0);
}

void turn_off()
{
  digitalWrite(R,0);
  digitalWrite(G,0);
  digitalWrite(B,0);
}




void angle()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gz1=g.gyro.z-gz0;
  delay(3);
  gz2=g.gyro.z-gz0;
  if((fabs(gz1) <= 0.01|| fabs(gz2) <= 0.01) && fabs(gz1 - gz2) <= 0.01) //暴力去除增量
  {
    gz=0;
  }
  else
  {
    gz=(gz1 + gz2) / 2;
  }
  now=millis();
  dt=(now-lastTime)/1000.0; //得到时间间隔（每一次的）
  lastTime=now;
  gyro_yaw+=gz*dt / pi * 180;//积分累加，同时把弧度转化为角度
}

void turn(int direction,float Angle)  //转弯代码
{
  angle();//角度更新
  float currentAngle=gyro_yaw; //当前的角度，然后用之后的实时角度与目前作比较
  if (direction==2)//左转
  {
    while(gyro_yaw<currentAngle+Angle)
    {
      angle();
      setpwm(1,-53);
      setpwm(2,53);
      delay(10);
    }
  }
  else if(direction==1)//右转
  {
    if(Angle>=175)
    {
      while(gyro_yaw>currentAngle-Angle)
    {
      angle();
      setpwm(1,60);
      setpwm(2,-60);
      delay(10);
    }
    }
    else
    {
    while(gyro_yaw>currentAngle-Angle)
    {
      angle();
      setpwm(1,53);
      setpwm(2,-53);
      delay(10);
    }
    }
  }
  setpwm(1,0);
  setpwm(2,0);
}



void setpwm(int motor, int pwm) //控制电机，motor为选择哪个电机，pwm为速度 motor1为左边两个轮，motor2为右边两个轮(已测试完毕，调速可用)
{
  if(motor==1&&pwm>=0)//motor=1左边轮
  {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
    analogWrite(ENB, pwm);
  }
  else if(motor==1&&pwm<0)//motor=1代表控制电机A，pwm<0则(AIN1, AIN2)=(0, 1)为反转
  {
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
    analogWrite(ENB, -pwm);
  }
  else if(motor==2&&pwm>=0)//motor=2右边两个轮子
  {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
    analogWrite(ENA, pwm);
  }
  else if(motor==2&&pwm<0)//motor=2代表控制电机B，pwm<0则(BIN1, BIN2)=(1, 0)为反转
  {
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
    analogWrite(ENA, -pwm);
  }
}

void PID_xunxian_go_1()
{
  int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  
  int s=0;//最终的权重变量，其实也就是预定误差量
  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  else if(a1==1 && a5==1 && crossing==0) //说明检测到十字路口
  {
      delay(100);
      turn(2,90);
      delay(300);
      crossing++;
  }
    // else if(crossing=1)
    // {
    //  delay(100);//给出一部分时间，防止重复计数
    //  turn(2,90);
    //  delay(300);
    //  crossing++;
    // }
  
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing>=1) //满足要求，达到停止条件
  {
    crossing=1;
    setpwm(1,0);
    setpwm(2,0);
    delay(400);
    turn(1,175);  //达到条件后，掉头
    go_ok=1;
    digitalWrite(R,255);
    l_speed=0;
    r_speed=0;
    delay(300);
  }
}
void PID_xunxian_back_1()
{
    int a1,a2,a3,a4,a5,b1,b2,b3;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  b1=digitalRead(R);
  b2=digitalRead(G);
  b3=digitalRead(B);
  int s=0;//最终的权重变量，其实也就是预定误差量
  turn_off();
  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  else if(a1==1 && a5==1 && crossing==1) //说明检测到十字路口
  {
      delay(100);
      turn(1,90);
      crossing--;
    
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing==0) //满足要求，达到停止条件
  {
    stop();
    delay(400);
    turn(1,175);  //达到条件后，掉头
    digitalWrite(G,255);
    delay(1000000);
    
  }
}

void PID_xunxian_go_2() //过去的巡线代码
{
  int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  
  int s=0;//最终的权重变量，其实也就是预定误差量
  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  else if(a1==1 && a5==1 && crossing==0) //说明检测到十字路口
  {
      delay(10);
      turn(1,90);
      delay(300);
      crossing++;
  }
    // else if(crossing=1)
    // {
    //  delay(100);//给出一部分时间，防止重复计数
    //  turn(2,90);
    //  delay(300);
    //  crossing++;
    // }
  
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing>=1) //满足要求，达到停止条件
  {
    crossing=1;
    setpwm(1,0);
    setpwm(2,0);
    delay(400);
    turn(1,175);  //达到条件后，掉头
    go_ok=1;
    digitalWrite(R,255);
    l_speed=0;
    r_speed=0;
    delay(300);
  }
}

void PID_xunxian_back_2() //回来的巡线代码
{
  int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  int s=0;//最终的权重变量，其实也就是预定误差量
  turn_off();
  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  else if(a1==1 && a5==1 && crossing==1) //说明检测到十字路口
  {
      delay(100);
      turn(2,90);
      crossing--;
    
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing==0) //满足要求，达到停止条件
  {
    stop();
    delay(400);
    turn(1,175);  //达到条件后，掉头
    digitalWrite(G,255);
    delay(1000000);
  }
}

void PID_xunxian_go_3()
{
    int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  
  int s=0;//最终的权重变量，其实也就是预定误差量
  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  else if(a1==1 && a5==1) //说明检测到十字路口
  {
    if(crossing>=1) //路过n-1个十字路口后的第n个十字路口转弯
    {
      delay(100);
      turn(2,90);
    }
    else
    {
     crossing++;
     speed=50;
     delay(300);//给出一部分时间，防止重复计数
    }
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing>=1) //满足要求，达到停止条件
  {
    crossing=1;
    setpwm(1,0);
    setpwm(2,0);
    delay(400);
    turn(1,175);  //达到条件后，掉头
    go_ok=1;
    digitalWrite(R,255);
    l_speed=0;
    r_speed=0;
    delay(300);
  }
}
void PID_xunxian_back_3()
{
    int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  int s=0;//最终的权重变量，其实也就是预定误差量
  turn_off();
  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  else if(a1==1 && a5==1) //说明检测到十字路口
  {
    if(crossing>=1) //路过n-1个十字路口后的第n个十字路口转弯
    {
      delay(100);
      turn(1,90);
      crossing--;
      speed=80;
    }
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing<1) //满足要求，达到停止条件
  {
    stop();
    delay(400);
    turn(1,175);  //达到条件后，掉头
    digitalWrite(G,255);
    delay(1000000);
  }
}
void PID_xunxian_go_4()
{
    int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  
  int s=0;//最终的权重变量，其实也就是预定误差量
  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  else if(a1==1 && a5==1) //说明检测到十字路口
  {
    if(crossing>=1) //路过n-1个十字路口后的第n个十字路口转弯
    {
      delay(50);
      turn(1,90);
    }
    else
    {
     crossing++;
     speed=50;
     delay(300);//给出一部分时间，防止重复计数
    }
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing>=1) //满足要求，达到停止条件
  {
    crossing=1;
    setpwm(1,0);
    setpwm(2,0);
    delay(400);
    turn(1,175);  //达到条件后，掉头
    go_ok=1;
    digitalWrite(R,255);
    l_speed=0;
    r_speed=0;
    delay(300);
  }
}
void PID_xunxian_back_4()
{
    int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  int s=0;//最终的权重变量，其实也就是预定误差量
  turn_off();
  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  else if(a1==1 && a5==1) //说明检测到十字路口
  {
    if(crossing>=1) //路过n-1个十字路口后的第n个十字路口转弯
    {
      delay(100);
      turn(2,90);
      crossing--;
      speed=80;
    }
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing<1) //满足要求，达到停止条件
  {
    stop();
    delay(400);
    turn(1,175);  //达到条件后，掉头
    digitalWrite(G,255);
    delay(1000000);
  }
}

void PID_xunxian_go_5()
{
    int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  
  int s=0;//最终的权重变量，其实也就是预定误差量
  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  else if(a1==1 && a5==1) //说明检测到十字路口
  {
    if(crossing>=2) //路过n-1个十字路口后的第n个十字路口转弯
    { 
      if(crossing==2)
      {
        delay(50);
        turn(2,90);
        crossing++;
      }
      else if(crossing==3 && a1==1 && a5==1)
      {
        delay(50);
        turn(1,90);
      }
    }
    else if (crossing==1)
    {
     crossing++;
     delay(300);//给出一部分时间，防止重复计数
     speed=50;
    }
    else if(crossing==0)
    {
      crossing++;
      delay(300);
    }
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing>=3) //满足要求，达到停止条件
  {
    crossing=3;
    setpwm(1,0);
    setpwm(2,0);
    delay(400);
    turn(1,175);  //达到条件后，掉头
    go_ok=1;
    digitalWrite(R,255);
    l_speed=0;
    r_speed=0;
    delay(400);
  }
}
void PID_xunxian_back_5()
{
    int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  delay(10);
  int s=0;//最终的权重变量，其实也就是预定误差量
  if(a5==1 && a1==0 && crossing==3) //说明检测到十字路口
  {
        delay(50);
        turn(2,90);
        delay(50);
        crossing--;
    
  }
  else if(a5==0 && a1==1 && crossing==2)
  {
      delay(40);
      turn(1,90);
      crossing--;
      speed=80;
  }
  

  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing<=1) //满足要求，达到停止条件
  {
    stop();
    delay(400);
    turn(1,175);  //达到条件后，掉头
    digitalWrite(G,255);
    delay(1000000);
  }
}
void PID_xunxian_go_6()
{
  int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  
  int s=0;//最终的权重变量，其实也就是预定误差量
  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  else if(a1==1 && a5==1) //说明检测到十字路口
  {
    if(crossing>=2) //路过n-1个十字路口后的第n个十字路口转弯
    { 
      if(crossing==2)
      {
        delay(50);
        turn(1,90);
        crossing++;
      }
      else if(crossing==3 && a1==1 && a5==1)
      {
        delay(50);
        turn(1,90);
      }
    }
    else if(crossing=1)
    {
     crossing++;
     delay(300);//给出一部分时间，防止重复计数
     speed=50;
    }
    else if(crossing=0)
    {
     crossing++;
     delay(300);//给出一部分时间，防止重复计数
    }
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing>=3) //满足要求，达到停止条件
  {
    crossing=3;
    setpwm(1,0);
    setpwm(2,0);
    delay(400);
    turn(1,175);  //达到条件后，掉头
    go_ok=1;
    digitalWrite(R,255);
    l_speed=0;
    r_speed=0;
    delay(400);
  }
}
void PID_xunxian_back_6()
{
      int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  delay(10);
  int s=0;//最终的权重变量，其实也就是预定误差量
  turn_off();
  if(a5==1 && a1==0 && crossing==3) //说明检测到十字路口
  {
        delay(50);
        turn(2,90);
        delay(50);
        crossing--;
    
  }
  else if(a5==1  && a4==1 && a1==0  && crossing==2)
  {
      delay(40);
      turn(2,90);
      crossing--;
      speed=80;
  }
  

  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing<=1) //满足要求，达到停止条件
  {
    stop();
    delay(400);
    turn(1,175);  //达到条件后，掉头
    digitalWrite(G,255);
    delay(1000000);
  }
}
void PID_xunxian_go_7()
{
  int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  
  int s=0;//最终的权重变量，其实也就是预定误差量
  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  else if(a1==1 && a5==1) //说明检测到十字路口
  {
    if(crossing>=2) //路过n-1个十字路口后的第n个十字路口转弯
    { 
      if(crossing==2)
      {
        delay(50);
        turn(2,90);
        crossing++;
      }
      else if(crossing==3 && a1==1 && a5==1)
      {
        delay(50);
        turn(2,90);
      }
    }
    else if(crossing==1)
    {
     crossing++;
     delay(300);//给出一部分时间，防止重复计数
     speed=50;
    }
    else if(crossing==0)
    {
     crossing++;
     delay(300);//给出一部分时间，防止重复计数
    }
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing>=3) //满足要求，达到停止条件
  {
    crossing=3;
    setpwm(1,0);
    setpwm(2,0);
    delay(400);
    turn(1,175);  //达到条件后，掉头
    go_ok=1;
    digitalWrite(R,255);
    l_speed=0;
    r_speed=0;
    delay(400);
  }
}
void PID_xunxian_back_7()
{
  int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  delay(10);
  int s=0;//最终的权重变量，其实也就是预定误差量
  if(a5==0 && a1==1 && crossing>=2) //说明检测到十字路口
  {
        delay(50);
        turn(1,90);
        delay(50);
        if (crossing==2)
        {
          speed=80;
        }
        crossing--;
  }
  

  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing<=1) //满足要求，达到停止条件
  {
    stop();
    delay(400);
    turn(1,175);  //达到条件后，掉头
    digitalWrite(G,255);
    delay(1000000);
 
  } 
}
void PID_xunxian_go_8()
{
  int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  
  int s=0;//最终的权重变量，其实也就是预定误差量
  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  else if(a1==1 && a5==1) //说明检测到十字路口
  {
    if(crossing>=2) //路过n-1个十字路口后的第n个十字路口转弯
    { 
      if(crossing==2)
      {
        delay(50);
        turn(1,90);
        crossing++;
      }
      else if(crossing==3 && a1==1 && a5==1)
      {
        delay(50);
        turn(2,90);
      }
    }
    else if(crossing==1)
    {
     crossing++;
     delay(300);//给出一部分时间，防止重复计数
     speed=50;
    }
    else if(crossing==0)
    {
      crossing++;
      delay(300);
    }
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing>=3) //满足要求，达到停止条件
  {
    crossing=3;
    setpwm(1,0);
    setpwm(2,0);
    delay(400);
    turn(1,175);  //达到条件后，掉头
    go_ok=1;
    r_speed=0;
    l_speed=0;
    digitalWrite(R,255);
    delay(400);
  }
}
void PID_xunxian_back_8()
{
      int a1,a2,a3,a4,a5;
  a1=digitalRead(k1);  //读取每个传感器的读数，接触到红线/黑线为1,以234为基础去巡线
  a2=digitalRead(k2);
  a3=digitalRead(k3);
  a4=digitalRead(k4);
  a5=digitalRead(k5);
  delay(10);
  turn_off();
  int s=0;//最终的权重变量，其实也就是预定误差量
  if(a5==0 && a1==1 && crossing==3) //说明检测到十字路口
  {
        delay(50);
        turn(1,90);
        delay(50);
        crossing--;
    
  }
  else if(a5==1  && a4==1 && a1==0  && crossing==2)
  {
      delay(40);
      turn(2,90);
      crossing--;
      speed=80;
  }
  

  if(a2==0 && a3==1 && a4==0)
  {
    s=0;
  }
  else if(a2==0 && a3==0 && a4==1)
  {
    s=-2;
  }
  else if(a2==1 && a3==0 && a4==0)
  {
    s=2;
  }
  else if(a2==1 && a3==1 && a4==0)
  {
    s=1;
  }
  else if(a2==0 && a3==1 && a4==1)   //权重完成
  {
    s=-1;
  }
  err=s;
  summ+=err;
  PID=Kp*err+Kd*(err-old_err)/50+Ki*summ;
  r_speed =speed-PID;
  l_speed =speed+PID;
  old_err=err;
  if(a1==0 && a2==0 && a3==0 && a4==0 && a5==0 && crossing<=1) //满足要求，达到停止条件
  {
    stop();
    delay(400);
    turn(1,175);  //达到条件后，掉头
    digitalWrite(G,255);
    delay(1000000);
  }  
}




void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  mpu.begin();
  bmp.begin();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  for (int i = 1; i <= 100; i++) {//对所有读数统计初始偏移量
    gz0 += g.gyro.z;
  }
  gz0 /= 10;
   pinMode(STANDBY_PIN, OUTPUT);
   pinMode(IN1,OUTPUT);
   pinMode(IN2,OUTPUT);
   pinMode(IN3,OUTPUT);
   pinMode(IN4,OUTPUT);
   pinMode(ENA,OUTPUT);
   pinMode(ENB,OUTPUT);
   pinMode(k1,INPUT);
   pinMode(k2,INPUT);
   pinMode(k3,INPUT);
   pinMode(k4,INPUT);
   pinMode(k5,INPUT);
   pinMode(R,OUTPUT);   //这三个是负责灯光的
   pinMode(G,OUTPUT);
   pinMode(B,OUTPUT);
   pinMode(weigh,INPUT);
   digitalWrite(STANDBY_PIN, HIGH);
}


void loop() 
{
  Serial_input();
  if (num!=0)
  {
   Serial.println(num);
   digitalWrite(B,255); //亮个灯看看效果
   delay(1000);
   turn_off();
   switch(num)
   {
    case 1:
        while(1)
        {
          a = digitalRead(weigh);
          angle();
          if (!go_ok && a==0)
          {
            PID_xunxian_go_1();
          }
          else if(go_ok && a==1) 
          {
            PID_xunxian_back_1();
          }
          else if(go_ok && a==1 && b1==1 && b2==1 && b3==1){
            break;
          }
          setpwm(1,l_speed);
          setpwm(2,r_speed);
          
        }

    case 2:
        while(1)
        {
          angle();
          a = digitalRead(weigh);
          if (!go_ok && a==0)
          {
            PID_xunxian_go_2();
          }
          else if (go_ok && a==1)
          {
            PID_xunxian_back_2();
          }
          setpwm(1,l_speed);
          setpwm(2,r_speed);
        }
    case 3:
        while(1)
        {
          angle();
          a = digitalRead(weigh);
          if (!go_ok && a==0)
          {
            PID_xunxian_go_3();
          }
          else if(go_ok && a==1)
          {
            PID_xunxian_back_3();
          }
          setpwm(1,l_speed);
          setpwm(2,r_speed);
        }
    case 4:
        while(1)
        {
          angle();
          a = digitalRead(weigh);
          if (!go_ok && a==0)
          {
            PID_xunxian_go_4();
          }
          else if(go_ok && a==1)
          {
            PID_xunxian_back_4();
          }
          setpwm(1,l_speed);
          setpwm(2,r_speed);
        }
    case 5:
        while(1)
        {
          angle();
          a = digitalRead(weigh);
          if (!go_ok && a==0)
          {
            PID_xunxian_go_5();
          }
          else if(go_ok && a==1)
          {
            PID_xunxian_back_5();
          }
          setpwm(1,l_speed);
          setpwm(2,r_speed);
        }
    case 6:
        while(1)
        {
          angle();
          //Serial.println("OK");
          a = digitalRead(weigh);
          if (!go_ok && a==0)
          {
            PID_xunxian_go_6();
            Serial.println("ok");
          }
          else if(go_ok && a==1)
          {
            PID_xunxian_back_6();
          }
          setpwm(1,l_speed);
          setpwm(2,r_speed);
        }
    case 7:
        while(1)
        {
          angle();
          a = digitalRead(weigh);
          if (!go_ok && a==0)
          {
            PID_xunxian_go_7();
          }
          else if(go_ok && a==1)
          {
            PID_xunxian_back_7();
          }
          setpwm(1,l_speed);
          setpwm(2,r_speed);
        }
    case 8:
        while(1)
        {
          angle();
          a = digitalRead(weigh);
          if (!go_ok && a==0)
          {
            PID_xunxian_go_8();
          }
          else if(go_ok && a==1)
          {
            PID_xunxian_back_8();
          }
          setpwm(1,l_speed);
          setpwm(2,r_speed);
        }

    }
  }
}
