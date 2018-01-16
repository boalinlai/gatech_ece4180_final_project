/*
    ECE 4180 Final project Real Time tracking system.
    Boa-Lin Lai, Samuel Choi IoT 
*/
#define M_PI 3.14159265358979323846
#include "Motor.h"
#include "rtos.h"
#include <string>
#include "mbed.h"
#include "math.h"
#include "GPS.h"
Serial pc(USBTX, USBRX);
Serial esp(p28, p27); // tx, rx
DigitalOut reset(p29);
DigitalOut led1(LED1); DigitalOut led2(LED2); DigitalOut led3(LED3); DigitalOut led4(LED4);
Timer t;
InterruptIn lenc(p26); //left encoder
InterruptIn renc(p25); //right encoder
Motor mr(p22, p12, p11); // pwm, fwd, rev
Motor ml(p21, p16, p15);
volatile int ticksR = 0; //Number of times right registered
const float robot_speed_r = 0.513;
const float robot_speed_l = 0.5;
const double robot_body = 6.25;
volatile double angle = -M_PI/2.0;
volatile int ticksL = 0; //Number of times left registered
volatile int moving = 0;
const int cmd_length = 10;
volatile bool robotReady = true;
volatile bool robotIdle = true;
struct Point{
  double x;
  double y;   
};
volatile Point p;
volatile Point real_p;
enum Dir{n,e,s,w};
volatile Dir curDir = e;
int maskx[4] = {0,1,0,-1}; 
int masky[4] = {1,0,-1,0};
int rot = 0;
int state = 0;
Ticker Sampler;
float sL = 0.8;
float sR = 0.8;
int input[] = {1,0};
int size = sizeof(input);
volatile int botcmd = 0;
bool condmet = 0;
void cl () { ticksL++;}
void cr () { ticksR++; }
char data[2000];
char myCMD[cmd_length];
int  count,ended,timeout;
char buf[2024];
char snd[1024];
void SendCMD(),getreply(),ESPconfig(),ESPsetbaudrate(), getCMD();
void getPt(){   
    float dis = ((ticksR + ticksL)/2)/25;  
    pc.printf("====\nCur direction %d ==== \n", curDir);
    p.x += maskx[curDir]*dis;
    p.y += masky[curDir]*dis; 
    char tmp[50];
    sprintf(tmp,"%0.2f,%0.2f<br>",p.x,p.y); 
    pc.printf("\nfinal  data:%s\n", tmp);
    strcat(data,tmp);
    memset(tmp,0,sizeof(tmp));
}
/*
* Send data to http server
*/
void update_httpserver(){       
        pc.printf("\n---------- updating up http server ----------\r\n");
        strcpy(snd, "srv=net.createServer(net.TCP)\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "srv:listen(80,function(conn)\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "conn:on(\"receive\",function(conn,payload)\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "print(payload)\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "conn:send(\"<!DOCTYPE html>\")\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "conn:send(\"<html>\")\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "conn:send(\"<h1>Robot data Logs</h1>\")\r\n");
        SendCMD();
        char tmp[500];
        sprintf(tmp, "conn:send(\"<p>%s</p>\")\r\n", data);
        wait(0.2);
        pc.printf("\n === data put in server %s === \n", data);      
        esp.printf("%s", tmp); // same as 
        wait(2);
        //memset(data,0,sizeof(data)); // clean the data buffer
        strcpy(snd, "conn:send(\"</html>\")\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "end)\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "conn:on(\"sent\",function(conn) conn:close() end)\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "end)\r\n");
        SendCMD();
        wait(0.2);
        timeout=5;
        getreply();
        getCMD();
        pc.printf(buf);
        wait(0.2);
        pc.printf("\r\nfinished setting up http server\n");
        wait(0.2);
        memset(tmp,0,sizeof(tmp));      // clear data 
}
Ticker robotCMD;   
Ticker update;
Ticker led_ticker;
volatile int r = 100;
void getCMD(){
    char * substr1 = "CMDSTART";
    char * substr2 = "CMDEND";
    char* cmdStart = strstr(buf, substr1);
    if(cmdStart)
    {
        char* cmdEnd = strstr(buf, substr2);
        char tmpCMD[cmd_length];
        int size = cmdEnd - cmdStart - 8;
        memcpy(tmpCMD, cmdStart + 8 , size);
        pc.printf("\n===find the command %s === \n", tmpCMD);
        robotIdle = false;
        pc.printf("\n== size %d === \n", size);
        for(int i = 0; i < cmd_length; i++){
           if(i > size-1){
               myCMD[i] = '0';
           }
           else{
               myCMD[i] = tmpCMD[i];
           }
        }     
    }
}
void changeDir(bool isLeft){
    if(isLeft){
        if(curDir == n){
            curDir = w;
        }
        else if(curDir == w){
            curDir = s;
        }
        else if(curDir == s){
            curDir = e;
        }
        else{
            curDir = n;
        }
    }else
    {
        if(curDir == n){
            curDir = e;
        }
        else if(curDir == w){
            curDir = n;
        }
        else if(curDir == s){
            curDir = w;
        }
        else{
            curDir = s;
        }
    }
}
Point rotate(double ang, double c){
    Point tmp;
    tmp.x = -c*sin(ang);
    tmp.y = c*cos(ang);
    //formula = [cos(angle), -sin(angle);sin(angle),cos(angle)];
    return tmp;
}
void forward(){
    ticksL = 0;
    ticksR = 0;
    while((ticksL < 400*moving) && (ticksR < 400*moving)){
        pc.printf("==\nL:%d, R: %d", ticksL,ticksR);
        ml.speed(robot_speed_l);
        mr.speed(robot_speed_r); 
    }
    ml.speed(0);
    mr.speed(0);
    int diff = abs(ticksL-ticksR);
    if(diff == 0) diff = 1;
    double offset  = (diff/25.0)/robot_body;
    double r = min(ticksL,ticksR)/25.0/offset;
    double a = r+3.25-cos(offset)*(r+3.25);
    double b = sin(offset) *(r+3.25);
    double c = hypot(a,b);
    printf("\n\n=== a:%.2f, b:%.2f, c:%0.2f, r:%0.2f === \n\n ",a,b,c,r);
    double phi;
    if(ticksL < ticksR){
        phi = atan(a/b) + angle;
        angle += offset;
    }else{
        phi = atan(a/b) + angle;
        angle -= offset;
    }
    Point new_p = rotate(phi, c);
    real_p.x = real_p.x + new_p.x;
    real_p.y = real_p.y + new_p.y;    
    char tmp[50];
    sprintf(tmp,"%0.2f,%0.2f<br>",real_p.x,real_p.y); 
    pc.printf("\n foward data :%s\n", tmp);
    strcat(data,tmp);
    memset(tmp,0,sizeof(tmp));
    ticksL = 0;
    ticksR = 0;
}
void ledflash() {
    led1 = 0; led2 = 0; led3 = 0; 
    static int i  = 0;
    if(robotReady){
        // mathcing the function with hash 
        char c = myCMD[i%cmd_length];
        if(c > 'a' && c <= 'z') {
            //myCMD[i%cmd_length] = c-1;
            moving = c - 'a';
            myCMD[i%cmd_length] = '0';
            led3= 1;
            botcmd = 3; 
            robotReady = false;
            i++;
        }
        else if(c == 'a' || c == 'A'){
            // reach the base go to next cmd
            myCMD[i%cmd_length] = '0';
            i++;
            robotReady = false;
        }
        if(c > 'A' && c <= 'Z') {
            led3= 1;
            myCMD[i%cmd_length] = c-1;
            ml.speed(-robot_speed_l);
            mr.speed(-robot_speed_r);
        }
        else if(c=='1'){
            //left turn
            myCMD[i%cmd_length] = '0';
            i++;
            //angle += M_PI/2.0;
            botcmd = 1;
            robotReady = false;
            led1 = 1;
        }else if (c=='2'){
            // right turn
            myCMD[i%cmd_length] = '0';
            i++;
            //angle -= M_PI/2.0;
            botcmd = 2;
            robotReady = false;
            led2 = 1;
        }
        else if(c == '0' && robotReady){
          led4 = !led4;
          ml.speed(0.0);
          mr.speed(0.0);
          robotIdle = true;
        }
    }
}
void createServer(); 
void turn(bool isLeft)
{
  ml.speed(0); mr.speed(0);
  int limit;
  float turn_speed;
  if(isLeft){
      limit = 116; 
      turn_speed = 0.22;
  }else{
       turn_speed = 0.22;
      limit = 116; 
  }
  ticksR = 0; ticksL = 0; 
  while(1){
    if (ticksL < limit) {
        if(isLeft){
            ml.speed(-turn_speed);
        }else{
            ml.speed(turn_speed);
        }
    }
    if (ticksR < limit) {
        if(isLeft){
            mr.speed(turn_speed);
        }else{
            mr.speed(-turn_speed);
        }
    }      
    if(ticksR >= limit || ticksL >= limit ) {
        double t = ((ticksL+ticksR)/50.0)/(robot_body/2.0);
        printf("\n\n=== Turning angle %0.2f ===\n\n ", t);
        if(isLeft){
           angle+=M_PI/2.0;
        }else{
           angle-=M_PI/2.0;
        }
        mr.speed(0);
        ml.speed(0);   
        wait(0.2);  
        break;
    }
  }
}    
int main(){
    renc.rise(&cr);
    lenc.rise(&cl);
    real_p.x += 8.0;
    real_p.y += 8.0;
    reset=0; //hardware reset for 8266
    pc.baud(9600);  // set what you want here depending on your terminal program speed
    pc.printf("\f\n\r-------------ESP8266 Hardware Reset-------------\n\r");
    wait(0.5);
    reset=1;
    timeout=2;
    getreply();
    esp.baud(9600);   // change this to the new ESP8266 baudrate if it is changed at any time.
    createServer();
    for(int i =0; i < cmd_length; i++) myCMD[i] = '0';
      led_ticker.attach(&ledflash, 1);
      p.x = 0;
      p.y = 0;
     while(1){
       wait(0.2);
       if(!robotReady){
            ml.speed(0); mr.speed(0);
            // print ticker 
            pc.printf("\n---- ticks L: %d-----\n", ticksL);
            pc.printf("\n---- ticks R: %d-----\n", ticksR);
            update_httpserver();
            if(botcmd == 1){
                 pc.printf("====\nLeft turn\n====");
                 turn(true);
                 changeDir(true);
                 botcmd = 0;
                 robotReady = true;
            }
            else if(botcmd == 2){
                 pc.printf("====\nRight turn\n====");
                 turn(false);
                 changeDir(false);
                 botcmd = 0;
                 robotReady = true;
            }else if(botcmd == 3){
                pc.printf("====\n Foward \n====");
                forward();
                botcmd = 0;
                robotReady = true;
            }

       }else if(robotIdle){
            // waiting for commad;
         timeout=5;
         getreply();
         getCMD();
         pc.printf(buf);
         wait(0.2);
       }
     }
}
void SendCMD(){
    esp.printf("%s", snd);
}
void getreply(){
    memset(buf, '\0', sizeof(buf));
    t.start();
    ended=0;
    count=0;
    while(!ended) {
        if(esp.readable()) {
            buf[count] = esp.getc();
            count++;
        }
        if(t.read() > timeout) {
            ended = 1;
            t.stop();
            t.reset();
        }
    }
}
/*
* send init the server 
*/
void createServer()
{
        pc.printf("\n---------- Setting up http server for first time ----------\r\n");
        strcpy(snd, "srv=net.createServer(net.TCP)\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "srv:listen(80,function(conn)\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "conn:on(\"receive\",function(conn,payload)\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "print(payload)\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "conn:send(\"<!DOCTYPE html>\")\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "conn:send(\"<html>\")\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "conn:send(\"<p>0.0,0.0</p>\")\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "conn:send(\"</html>\")\r\n");
        SendCMD();
        wait(0.2);    
        strcpy(snd, "end)\r\n");
        SendCMD();
        wait(0.2);    
        strcpy(snd, "conn:on(\"sent\",function(conn) conn:close() end)\r\n");
        SendCMD();
        wait(0.2);
        strcpy(snd, "end)\r\n");
        SendCMD();
        wait(0.2);
        timeout=1;
        getreply();
        pc.printf(buf);
        pc.printf("\n---------- Get IP's ----------\r\n");
        strcpy(snd, "print(wifi.sta.getip())\r\n");
        SendCMD();
        timeout=3;
        getreply();
        pc.printf(buf);
        wait(1);
        pc.printf("\r\nDONE\n");
}
/*
void getGPS()
{
    int i = 2;
    static float j = 1.0;
    char final_data[200];
    while(i>0){
      if(gps.sample()) {
            i--;
            j++;
             //pc.printf("%f;%c;%f;%c", gps.longitude, gps.ns, gps.latitude, gps.ew);
             char tmp[50];
             sprintf(tmp,"%0.6f,%0.6f<br>",j,j); 
             //sprintf(tmp,"%0.6f,%0.6f<br>",gps.longitude,gps.latitude); 
             strcat(final_data,tmp);
      }
    }
    pc.printf("\nfinal  data:%s\n", final_data);
    strcpy(data,final_data);
    memset(final_data,0,sizeof(final_data));
}
*/

