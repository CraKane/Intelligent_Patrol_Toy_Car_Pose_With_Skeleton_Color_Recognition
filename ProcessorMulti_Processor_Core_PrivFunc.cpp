#include "../NoEdit/ProcessorMulti_Processor_Core_PrivFunc.h"
#include <cmath>
#include <iostream>
using namespace std;
#include <algorithm>
#include <vector>
#include <string.h>
#define ACTUAL_STEER 0.987778
#define TARGET_DIS 1.0  //m
#define MAX_POSE 400  //人所在位置最大边界
#define MIN_POSE 260  //人所在位置最小边界
#define SAME 15  //几乎相同的阈值
#define DIFF 50  //伸展手臂的阈值

int now_time = 0;
int last_time = 0;
int mode = 0;  // 0为直行模式，1为转弯模式
int steer = 0;
int speed = 0;
//整个原地转弯过程的转弯阶段标记
bool flag_turn_1 = false; bool flag_turn_2 = false; bool flag_turn_3 = false;
bool pose_recognition_init = true; bool left_turn = false; bool right_turn =false;
bool isPerson = false; 

//PID各项参数的变量结构体
struct _pid {
    float SetDis;//输入变量，即期望输出的变量值
    float ActualDis;//实际输出变量，即采样回来的输出变量
    float Err; //误差值
    float Err_last;//上一次误差值
    float Kp, Ki, Kd;//比例 积分 微分系数
    float Voltage;//定义电压值，最后的输出都要转化为电压来控制
    float Integral; //积分值
} pid;
//PID控制的初始化函数
//初始化完成后主要对Kp、Ki、Kd不断地调节来优化控制效果
void PID_Init(double dis) {
    //cout << "PID_Ini9t begin!" << endl;
    pid.SetDis = 0;
    pid.ActualDis = dis;
    pid.Err = 0;
    pid.Err_last = 0;
    pid.Kp = 1.0;
    pid.Ki = 0.015;
    pid.Kd = 0.2;
    pid.Voltage = 0;
    pid.Integral = 0;
    //cout << "PID_Init end!" << endl;
}

float PID_Cal(float Dis)
{
    pid.SetDis = Dis;
    pid.Err = pid.SetDis - pid.ActualDis;//误差的计算，即比例控制
    pid.Integral += pid.Err;//误差相加，即积分控制
    pid.Voltage = pid.Kp * pid.Err;// + pid.Ki * pid.Integral + pid.Kd *
       // (pid.Err - pid.Err_last);//根据位置型PID控制的公式
    pid.Err_last = pid.Err;
    pid.ActualDis = pid.Voltage * -1.0; //转换
    //cout<<"err  "<<pid.Err<<endl;
    return pid.ActualDis;//PID控制后的实际输出值
}

//Function: the Control of Going Forward
void Go_Forward(double right_dis) {
    PID_Init(right_dis);
    double ActualDis = PID_Cal(TARGET_DIS);
    steer = ActualDis*300*ACTUAL_STEER;
    speed = 140;
    cout<<"go forward speed = "<<speed<<"  ;steer = "<<steer<<endl;
    return;
}

// calculate distance
double forward(QVector<SensorTimer_Sensor_URG_Data *> inputdata_1, string pos, double unit){
    //unit---inputparams_1.front()->unit
    if(pos.compare("front") == 0){ //front distance
        //angle 75-105
        vector<double> ypos;
        for(int i = 75*2;i <=105*2;++i){
            if(inputdata_1.front()->data[i] == 0) continue;
            if(inputdata_1.front()->data[i]/unit >= inputdata_1.front()->data[76*2]/unit +1) continue;
            double dis = inputdata_1.front()->data[i] / unit;
            double angle = (i*0.5)*3.1415926535/180; //rads
            double x = dis * cos(angle);
            double y = dis * sin(angle);
            ypos.push_back(y);
        }
        double sumy = 0;
        for(int i=0; i<ypos.size(); ++i){
            sumy += ypos[i];
        }
        return sumy / ypos.size();
    }
    if(pos.compare("foot") == 0){
        //angle 75-105
        vector<double> ypos;
        for(int i = 76*2;i <=104*2;++i){
            if(inputdata_1.front()->data[i] == 0) continue;
            if(inputdata_1.front()->data[i]/unit >= inputdata_1.front()->data[80*2]/unit +1) continue;
            double dis = inputdata_1.front()->data[i] / unit;
            double angle = (i*0.5)*3.1415926535/180; //rads
            double x = dis * cos(angle);
            double y = dis * sin(angle);
            if(y <= 4) ypos.push_back(y);
        }
        double sumy = 0;
        for(int i=0; i<ypos.size(); ++i){
            sumy += ypos[i];
        }
        double avry = sumy / ypos.size();
        return avry;
    }
    if(pos.compare("left") == 0){ //left distance
        //angle 0-30
        double avr_dis_left = 0;
        for(int i = 0*2;i <=40*2;++i){
            if(inputdata_1.front()->data[i] == 0) continue;
            double dis = inputdata_1.front()->data[i] / unit;
            //cout << dis<<endl;
            double angle = (i*0.5)*3.1415926535/180; //rads
            avr_dis_left += dis*cos(angle);
        }
        avr_dis_left /= (40*2 + 1);
        return fabs(avr_dis_left);
    }
    if(pos.compare("right") == 0){ //right distance
        //angle 140-180
        double avr_dis_right = 0;
        for(int i = 140*2;i <=180*2;++i){
            if(inputdata_1.front()->data[i] == 0) continue;
            double dis = inputdata_1.front()->data[i] / unit;
            //cout << dis<<endl;
            double angle = (i*0.5)*3.1415926535/180; //rads
            avr_dis_right += dis*cos(3.1415926535-angle);
        }
        avr_dis_right /= (40*2 + 1);
        return fabs(avr_dis_right);
    }
    cout<<"string input in func_forward has failed"<<endl;
    return 0.00;
}

int color_identification(QVector<SensorTimer_Sensor_xtion_Data *> inputdata_2){
    //return target -1:enemy, 0:nobody, 1:friend
    //
    // test the cv functions
    cv::Mat img;
    inputdata_2.front()->cvColorImg.copyTo(img);
    vector<cv::Mat> channels;
    cv::split(img,channels);
    cv::Mat HSVimg;
    cv::cvtColor(img, HSVimg, CV_BGR2HSV);
    cv::Mat H_channel;
    H_channel.create(HSVimg.size(), HSVimg.depth());
    int ch[] = {0,0};
    cv::mixChannels(&HSVimg, 1, &H_channel, 1, ch, 1);
    cv::Mat blue = channels.at(0);
    cv::Mat red  = channels.at(2);
    cv::Mat green= channels.at(1);
    cv::Point compass = cv::Point(300, 200);
    cv::Point bodysize = cv::Point(100, 106);
    cv::line(img,
             cv::Point(compass.x - bodysize.x/2,compass.y - bodysize.y/2),
             cv::Point(compass.x + bodysize.x/2,compass.y - bodysize.y/2),
             cv::Scalar(0,255,0), 2, CV_AA);
    cv::line(img,
             cv::Point(compass.x - bodysize.x/2,compass.y + bodysize.y/2),
             cv::Point(compass.x + bodysize.x/2,compass.y + bodysize.y/2),
             cv::Scalar(0,255,0), 2, CV_AA);
    cv::line(img,
             cv::Point(compass.x - bodysize.x/2,compass.y - bodysize.y/2),
             cv::Point(compass.x - bodysize.x/2,compass.y + bodysize.y/2),
             cv::Scalar(0,255,0), 2, CV_AA);
    cv::line(img,
             cv::Point(compass.x + bodysize.x/2,compass.y - bodysize.y/2),
             cv::Point(compass.x + bodysize.x/2,compass.y + bodysize.y/2),
             cv::Scalar(0,255,0), 2, CV_AA);
    cv::imshow("color", img);
    //HSV judge blue or red
    int blue_size[] = {100,130};
    int red_size[] = {0, 10};
    int bluesum = 0;
    int redsum = 0;
    int ybegin = compass.y - bodysize.y/2, yend = compass.y + bodysize.y/2;
    int xbegin = compass.x - bodysize.x/2, xend = compass.x + bodysize.x/2;
    for(int i=ybegin;i<=yend;i++)
        for(int j=xbegin;j<=xend;j++){
            if(int(H_channel.at<uchar>(i, j))<blue_size[1] && int(H_channel.at<uchar>(i, j))>blue_size[0]) bluesum+=1;
            if(int(H_channel.at<uchar>(i, j))<red_size[1] && int(H_channel.at<uchar>(i, j))>red_size[0]) redsum+=1;
        }
    cv::imshow("color", img);
    cout << bluesum<<" "<<redsum<<endl;
    if(bluesum > redsum) return -1;
    else return 1;
    //judge blue or red
    /*
    double bluesum = 0,redsum = 0, greensum = 0,blueallsum = 0, redallsum = 0;
    int ybegin = compass.y - bodysize.y/2, yend = compass.y + bodysize.y/2;
    int xbegin = compass.x - bodysize.x/2, xend = compass.x + bodysize.x/2;
    for(int j=ybegin;j<=yend;j++)
        for(int i=xbegin;i<=xend;i++){
            bluesum += blue.data[i*blue.cols+j];
            redsum += red.data[i*blue.cols+j];
            greensum += green.data[i*blue.cols+j];
            blueallsum += red.data[i*blue.cols+j] + green.data[i*blue.cols+j];
            redallsum += blue.data[i*blue.cols+j] + green.data[i*blue.cols+j];
        }
    bluesum /= (bodysize.x * bodysize.y);
    redsum /= (bodysize.x * bodysize.y);
    greensum /= (bodysize.x * bodysize.y);
    blueallsum /= (bodysize.x * bodysize.y*2);
    redallsum /= (bodysize.x * bodysize.y*2);
    double blue_rate = bluesum / blueallsum;
    double red_rate = redsum / redallsum;
    //cout << "blue_rate="<<blue_rate<<",  red_rate="<<red_rate<<endl;
    cout << "blue="<<bluesum<<"  gree="<<greensum<<"  red="<<redsum<<endl;
    if(blue_rate >= 1.5) return -1;
    else if(red_rate >= 1.5) return 1;
    else return 0;*/
}

//0: 没有识别出pose
//1：向左转
//2：向右转
int RecogPose(cv::Point (&poseInfo)[15], QVector<SensorTimer_Sensor_xtion_Data *> inputdata_2) {
    //---------------- xtion person bones recognition INFO ------------//
    //通过xtion是否可视化初步断定是否识别到人
    bool isPerson_init = inputdata_2.front()->isPersonVisable; 

    //cout << "personInfo :" << isPerson_init << endl;
    //第一种方法：将脖子、肩部三点横坐标取平均断定，人是否在画面中心，进而断定时候正确识别人
    //问题简化，能识别的人必须在中心位置：超参为 260~400
    //通过跑数据，效果可以
    double personX = (poseInfo[1].x+poseInfo[2].x+poseInfo[3].x)/3.0;

    //检测人的横坐标平均值是否在规定的中心范围内，同时结合初步断定值，最终确认人是否被xtion正确识别
    if ((personX <= MAX_POSE && personX >= MIN_POSE) && isPerson_init) {
        isPerson = true;
        //cout << poseInfo[1].x << " " << poseInfo[2].x << " " << poseInfo[3].x << endl;
        //cout << "center: " << personX << endl;
        cout << "person is in the center & recognized..." << endl;
    }
    else {
        isPerson = false;
        cout << "person lost... Where r u..." << endl;
    }

    //检测left
    //垂直距离近乎相同  阈值为9
    //cout << "Cheking Left pose..." << endl;
    double ver_dis_1 = abs(poseInfo[3].y - poseInfo[5].y); double ver_dis_2 = abs(poseInfo[5].y - poseInfo[7].y);
    double ver_dis_3 = abs(poseInfo[3].y - poseInfo[7].y); double hor_dis = abs(poseInfo[3].x - poseInfo[7].x);
    //cout << "3 ver_dis: " << ver_dis_1 << " " << ver_dis_2 << " " << ver_dis_3 << endl;
    //cout << "horizon dis: " << hor_dis << endl;
    if ((ver_dis_1 <= SAME && ver_dis_2 <= SAME && ver_dis_3 <= SAME) && hor_dis >= DIFF)
        return 1;

    //检测right
    //垂直距离近乎相同  阈值为9
    //cout << "Cheking Right pose..." << endl;
    ver_dis_1 = abs(poseInfo[2].y - poseInfo[4].y); ver_dis_2 = abs(poseInfo[4].y - poseInfo[6].y);
    ver_dis_3 = abs(poseInfo[2].y - poseInfo[6].y); hor_dis = abs(poseInfo[2].x - poseInfo[6].x);
    //cout << "3 ver_dis: " << ver_dis_1 << " " << ver_dis_2 << " " << ver_dis_3 << endl;
    //cout << "horizon dis: " << hor_dis << endl;
    if ((ver_dis_1 <= SAME && ver_dis_2 <= SAME && ver_dis_3 <= SAME) && hor_dis >= DIFF)
        return 2;
    return 0;
}

//按照时间进行转弯
void turn_around(int mode) {
    if (mode == 1) {  //left
        if (now_time - last_time < 120) {
            steer = -600;
            speed = 120;
            cout << "Turning Left..." << endl;
        }
        else {
            left_turn = false;
            steer = 10;
        }
    }
    else if (mode == 2) {  //right
        if (now_time - last_time < 120) {
            steer = 700;
            speed = 120;
            cout << "Turning Right..." << endl;
        }
        else {
            right_turn = false;
            steer = 10;
        }
    }
    return;
}

bool DECOFUNC(setParamsVarsOpenNode)(QString qstrConfigName, QString qstrNodeType, QString qstrNodeClass, QString qstrNodeName, void * paramsPtr, void * varsPtr)
{
	XMLDomInterface xmlloader(qstrConfigName,qstrNodeType,qstrNodeClass,qstrNodeName);
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	/*======Please Program below======*/
	/*
	Function: open node.
	Procedure:
	1: load parameters (params). [GetParamValue(xmlloader,params,tag); GetEnumParamValue(xmlloader,params,tag); GetUEnumParamValue(xmlloader,params,tag)]
	2: initialize variables (vars).
	3: If everything is OK, return 1 for successful opening and vice versa.
	*/
	
	return 1;
}

bool DECOFUNC(handleVarsCloseNode)(void * paramsPtr, void * varsPtr)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	/*======Please Program below======*/
	/*
	Function: close node.
	Procedure:
	1: handle/close variables (vars).
	2: If everything is OK, return 1 for successful closing and vice versa.
	*/
	
	return 1;
}

void DECOFUNC(getInternalTrigger)(void * paramsPtr, void * varsPtr, QObject * & internalTrigger, QString & internalTriggerSignal)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	internalTrigger=NULL;
	internalTriggerSignal=QString();
	/*======Occasionally Program above======*/
	/*
	Function: get internal trigger [defined in vars] for node.
	You need to program here when you need internal trigger (internalTrigger + internalTriggerSignal) for node.
	E.g.
	internalTrigger=&(vars->trigger);
	internalTriggerSignal=QString(SIGNAL(triggerSignal()));
	*/
}

void DECOFUNC(initializeOutputData)(void * paramsPtr, void * varsPtr, boost::shared_ptr<void> & outputDataPtr)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	outputDataPtr=boost::shared_ptr<void>(new SourceDrainMono_Sensor_EncoderIMU_Order_InputData());
	/*======Occasionally Program below/above======*/
	/*
	Function: initial output data.
	You need to program here when you need to manually initialize output data.
	*/
	
}

void DECOFUNC(getMultiInputDataSize)(void * paramsPtr, void * varsPtr, QList<int> & inputDataSize)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	inputDataSize=QList<int>();
	/*======Please Program above======*/
	/*
	Function: get input data size to be grabbed from buffer.
	Rules:
	inputDataSize=0: grab and remove all data from buffer.
	inputDataSize>0: grab inputDataSize latest data from buffer.
	inputDataSize<0: grab and remove inputDataSize ancient data from buffer.
	E.g.
	inputDataSize=QList<int>()<<0<<1<<-1...;
	*/
}

//Input Port #0: Buffer_Size = 10, Params_Type = SourceDrainMono_Sensor_EncoderIMU_Params, Data_Type = SourceDrainMono_Sensor_EncoderIMU_Data
//Input Port #1: Buffer_Size = 10, Params_Type = SensorTimer_Sensor_URG_Params, Data_Type = SensorTimer_Sensor_URG_Data
//Input Port #2: Buffer_Size = 10, Params_Type = SensorTimer_Sensor_xtion_Params, Data_Type = SensorTimer_Sensor_xtion_Data
bool DECOFUNC(processMultiInputData)(void * paramsPtr, void * varsPtr, QVector<QVector<void *> > inputParams, QVector<QVector<void *> > inputData, void * outputData, QList<int> & outputPortIndex)
{
	ProcessorMulti_Processor_Core_Params * params=(ProcessorMulti_Processor_Core_Params *)paramsPtr;
	ProcessorMulti_Processor_Core_Vars * vars=(ProcessorMulti_Processor_Core_Vars *)varsPtr;
	QVector<SourceDrainMono_Sensor_EncoderIMU_Params *> inputparams_0; copyQVector(inputparams_0,inputParams[0]);
	QVector<SensorTimer_Sensor_URG_Params *> inputparams_1; copyQVector(inputparams_1,inputParams[1]);
	QVector<SensorTimer_Sensor_xtion_Params *> inputparams_2; copyQVector(inputparams_2,inputParams[2]);
	QVector<SourceDrainMono_Sensor_EncoderIMU_Data *> inputdata_0; copyQVector(inputdata_0,inputData[0]);
	QVector<SensorTimer_Sensor_URG_Data *> inputdata_1; copyQVector(inputdata_1,inputData[1]);
	QVector<SensorTimer_Sensor_xtion_Data *> inputdata_2; copyQVector(inputdata_2,inputData[2]);
	SourceDrainMono_Sensor_EncoderIMU_Order_InputData * outputdata=(SourceDrainMono_Sensor_EncoderIMU_Order_InputData *)outputData;
	outputPortIndex=QList<int>();
	if(inputdata_0.size()==0){return 0;}
	if(inputdata_1.size()==0){return 0;}
	if(inputdata_2.size()==0){return 0;}
	/*======Please Program below======*/
	/*
	Step 1: process inputdata_index, then store it into outputdata.
	Step 2 [optional]: determine the outputPortIndex. (if not, outputdata will be sent by all ports)
	E.g. outputPortIndex=QList<int>()<<(outportindex1)<<(outportindex2)...
	*/

    //inputdata_0                                             // EncoderIMU
    //inputdata_1                                             // URG
    //inputdata_2                                             // Xtion (RGB && depth)
    //cv::imshow("color", inputdata_2.front()->cvColorImg);   // Show RGB image
    //cv::imshow("depth", inputdata_2.front()->cvDepthImg);   // Show depth image
	now_time += 1;           //时间戳
    double avr_dis_front = forward(inputdata_1, "front", inputparams_1.front()->unit);
    double avr_dis_right = forward(inputdata_1, "left", inputparams_1.front()->unit);
    double avr_dis_left = forward(inputdata_1, "right", inputparams_1.front()->unit);
    double avr_dis_foot = forward(inputdata_1, "foot", inputparams_1.front()->unit);
    cout << avr_dis_front << " " << avr_dis_left << " " << avr_dis_right << " foot: " << avr_dis_foot << endl;

	int mode_now = 0;  //0:为锁定目标   1:目标为敌人   2:目标为朋友

    //将人体骨架15点坐标信息保存
    cv::Point personInfo[15] = inputdata_2.front()->jointPos2D;
	// ---- mode_1 处理图片 识别人形 测量距离 ----
    int turn = RecogPose(personInfo, inputdata_2);

    //--------------- turn mode ------------------//
    if (left_turn) {
        turn_around(1);
        goto L;
    }
    else if(right_turn) {
        turn_around(2);
        goto L;
    }

    //function：处理图片
    if(isPerson){
        cout << "Found Person..." << endl;
        pose_recognition_init = true;
        if(color_identification(inputdata_2) == 1 && turn){ //friend
			if (turn == 1) { //left
                //保证不进入原地倒车
                flag_turn_1 = false; flag_turn_2 = false; flag_turn_3 = false;
                pose_recognition_init = true;
                last_time = now_time;
                left_turn = true;
                cout << "Now turning left..." << endl;
            }
            else if (turn == 2) { //right
                //保证不进入原地倒车
                flag_turn_1 = false; flag_turn_2 = false; flag_turn_3 = false;
                pose_recognition_init = true;
                right_turn = true;
                last_time = now_time;
                cout << "Now turning right..." << endl;
            }
		}
		else {
            if (avr_dis_foot < 3) {
                steer = 10;
                speed = -120;
                goto L;
            }
            else if(avr_dis_foot >= 3 && avr_dis_foot < 3.2) {
                steer = 0;
                speed = 0;
                goto L;
            }
            else {
                Go_Forward(avr_dis_right);
                goto L;
            }
		}
	}
	else{
        mode = 0; //init is go forward
        //每次骨架丢失，我都重新定义识别时间
        if (pose_recognition_init) {
            last_time = now_time;
            pose_recognition_init = false;
        }
        if (avr_dis_front > 3) {
            mode = 0;
        }
        else mode = 1;
        cout << "Nobody Found..." << endl;
	}
	//end

	if (mode_now == 0) { //无法锁定目标，直行或原地掉头
        if (mode == 0) { //距离前方2米以上，直行
			Go_Forward(avr_dis_right);
		}
		else { //距墙过近，掉头，假设车辆很敏感，那么只要PID控制较好，车辆居中，就可以使用固定参数完成掉头
			mode = 1;
            if (!flag_turn_1 && !flag_turn_2 && !flag_turn_3) {
                flag_turn_1 = true;flag_turn_2 = false;flag_turn_3 = false;
            }
            //steer = 500;
            //speed = 120;
		}
	}
	if(mode == 1){ //此时转弯模式
        if(flag_turn_1 && !flag_turn_2 && !flag_turn_3){  //这是你第一个需要改的超参数，40表示转弯转40桢
            steer = 600;
			speed = 120;
            cout << "turn mode: NO.1" << endl;
            // 当前方距离小于等于0.6m时，转弯第一阶段结束
            if(avr_dis_front <= 0.6) {
                flag_turn_1 = false;
                flag_turn_2 = true;  //第二阶段开始
                flag_turn_3 = false;
            }
		}
        else if(!flag_turn_1 && flag_turn_2 && !flag_turn_3){  //这是你第二个需要改的超参数，70-40为倒车30桢
            steer = -700;
			speed = -120;
            cout << "turn mode: NO.2" << endl;
            // 当前方距离大于等于1.5m时，转弯第二阶段结束
            if(avr_dis_front >= 1.7) {
                flag_turn_1 = false;
                flag_turn_2 = false;
                flag_turn_3 = true;  //第三阶段开始
            }
		}
        else if(!flag_turn_1 && !flag_turn_2 && flag_turn_3){  //这是你第二个需要改的超参数，70-40为倒车30桢
            steer = 600;
            speed = 130;
            cout << "turn mode: NO.3" << endl;
            if (avr_dis_front <= 0.6) {
                flag_turn_2 = true;
                flag_turn_1 = false;
                flag_turn_3 = false;
            }
            // 当前方距离大于等于3m 同时左右方距离都小于等于2m时，转弯第三阶段结束
            if(avr_dis_front >= 1.7 && avr_dis_left <= 2 && avr_dis_right <= 2) {
                goto T;
            }
        }
		else{
            T:
            cout << "turn over" << endl;
			mode = 0; //掉头时间已到，结束转弯模式
            flag_turn_1 = false;
            flag_turn_2 = false;
            flag_turn_3 = false;
            steer = 100;
            speed = 140;
            pose_recognition_init = true;
        }
	}

    //short steer = 100;           // [-400, 400]
    //short speed = 100;           // [-180, 180]

    // Show RGB image && compass
    L:
    double ori = - ((double)steer / 400.0) * (M_PI / 2.0);
    cv::Mat img;
    inputdata_2.front()->cvColorImg.copyTo(img);
	cv::flip(img, img, 1);
    cv::Point compass = cv::Point(100, 100);
    cv::circle(img, compass, 80, cv::Scalar(0,255,0), 1, CV_AA);
    cv::line(img, compass,
             cv::Point(compass.x - sin(ori) * 80,
                       compass.y - cos(ori) * 80),
             cv::Scalar(0,255,0), 3, CV_AA);
    //cv::imshow("color", img);

    //--------------------------------------------
    int maxSpeed = 180;
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < -maxSpeed) speed = -maxSpeed;
    char dataput[20];
    dataput[0] = 0xF8;
    dataput[1] = 4;
    *(short*)&dataput[2] = (short)steer;
    *(short*)&dataput[4] = (short)speed;
    dataput[6] = 0x8F;
    outputdata->datagram = QByteArray(dataput, 7);
	return 1;
}

