#include "bodyhub/bodyhub.h"
#include "controller.h"
#include "imuData.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"

#define TO_INT16(a, b)                             \
  ((int16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | \
             ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))

#define BASE_BOARD_ID 200
#define BASE_BOARD_ADDR 24
#define BASE_BOARD_LEN 0

#define GYRO_COEFFICIENT (1000.0 / 32768)
#define ACC_COEFFICIENT (8.0 * 9.8 / 32768)

#define BULK_READ_PRESENT_POSITION_ADDR 36
#define BULK_READ_PRESENT_POSITION_LEN 2
#define RIGHT_FSR_ID 112
#define LEFT_FSR_ID 111
#define FSR_ADDR 90
#define FSR_ADDR_LEN 4
#define MediMotoAlpha 12.80
#define SmalMotoAlpha 18.61
#define Servo_Value_MAX 4035
#define Servo_Value_MIN 60
#define Servo_Head_21_MAX  3300
#define Servo_Head_21_MIN 800
#define Servo_Head_22_MAX 3000
#define Servo_Head_22_MIN 1700
#define ArrLenght(lenght, bytes) lenght/bytes
#define Servo_Temperature_Register_Address 43
#define OneByte 1
#define NumberOfFsr 2

uint8_t dxlIds[DXL_ID_COUNT_MAX];
uint8_t numberOfId = 0;
uint8_t currentControlId = 0;
uint8_t bodyhubState = 0;
std::string stateNewStr;

std::string offsetFile;
std::string InitPoseFile;
std::string sensorNameIDFile;

ImuData torsoImu;
uint8_t imuType = 0;
static double_t dxlGyro[3] = {0,0,0};
static double_t dxlAcc[3] = {0,0,0};
static double measuredJointPos[30];
uint8_t armMode = 1;  // 0 为手保持模式
uint8_t failed_ID[SERVO_NUM] = {0};

int WalkJointDirection[SERVO_NUM] = {1,  1, -1, -1, 1, -1, 1, 1, 1, 1, -1,
                                     -1, 1, 1,  1,  1, 1,  1, 1, 1, 1, 1};
uint32_t SERVERS_LIMITS[22][2]={//limit value of each servo
  {Servo_Value_MIN,Servo_Value_MAX},{Servo_Value_MIN,Servo_Value_MAX},//1,2
  {Servo_Value_MIN,Servo_Value_MAX},{Servo_Value_MIN,Servo_Value_MAX},
  {Servo_Value_MIN,Servo_Value_MAX},{Servo_Value_MIN,Servo_Value_MAX},//5,6
  {Servo_Value_MIN,Servo_Value_MAX},{Servo_Value_MIN,Servo_Value_MAX},
  {Servo_Value_MIN,Servo_Value_MAX},{Servo_Value_MIN,Servo_Value_MAX},//9,10
  {Servo_Value_MIN,Servo_Value_MAX},{Servo_Value_MIN,Servo_Value_MAX},
  {Servo_Value_MIN,Servo_Value_MAX},{Servo_Value_MIN,Servo_Value_MAX},//13,14
  {Servo_Value_MIN,Servo_Value_MAX},{Servo_Value_MIN,Servo_Value_MAX},
  {Servo_Value_MIN,Servo_Value_MAX},{Servo_Value_MIN,Servo_Value_MAX},//17,18
  {Servo_Value_MIN,Servo_Value_MAX},{Servo_Value_MIN,Servo_Value_MAX},//19,20
  {Servo_Head_21_MIN,Servo_Head_21_MAX},{Servo_Head_22_MIN,Servo_Head_22_MAX}//21,22
};

float AngleAlpha[SERVO_NUM] = {
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha,
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, MediMotoAlpha,
    MediMotoAlpha, MediMotoAlpha, MediMotoAlpha, SmalMotoAlpha, SmalMotoAlpha,
    MediMotoAlpha, SmalMotoAlpha, SmalMotoAlpha, SmalMotoAlpha, SmalMotoAlpha,
    SmalMotoAlpha, SmalMotoAlpha};

std::vector<double> motoDataValuePre = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

double standPos[SERVO_NUM] = {0, 0, 0,   0,   0, 0,  0,  0, 0, 0, 0,
                              0, 0, -75, -10, 0, 75, 10, 0, 0, 0, 0};                                

double servoOffset[SERVO_NUM] = {0};

bool noRobotStand = false;
bool HeadQueueRec=false;//HeadQueue exit or not

// 舵机下发数据记录存储
int32_t ServoAllReadStore[30];

ServoStore_s ServoStore;

std::mutex mtxFSM;

std::mutex mtxJointTrajQueue;
std::queue<sensor_msgs::JointState> jointTrajQueue;

std::queue<bodyhub::JointControlPoint> motoQueue;
std::queue<bodyhub::JointControlPoint> headCtrlQueue;

DynamixelWorkbench dxl_wb;
GaitManager::CPWalking5 *GaitManager::CPWalking5::m_UniqueInstance =
    new CPWalking5;
GaitManager::LIPMWalk mWalk;      // walking
GaitManager::CPWalking5 *cpWalk;  // walking

//创建互斥锁
pthread_mutex_t mtxMo;
pthread_mutex_t mtxHe;
pthread_mutex_t mtxWl;
pthread_mutex_t mtxSL;
pthread_mutex_t mtxOpDxl;//dxl_wb.writeOnlyRegister,dxl_wb.syncWrite
pthread_mutex_t mtxBs;//update status,dxlBulkRead


std_msgs::UInt16 budyhubStateMsg;

//话题
ros::Publisher WalkingStatusPub;
ros::Publisher jointPosTargetPub;
ros::Publisher jointPosMeasurePub;
ros::Publisher jointVelTargetPub;
ros::Publisher jointVelMeasurePub;
ros::Publisher contactState_pub;
ros::Publisher stepPhase_pub;
ros::Publisher cpref_pub;
ros::Publisher cpC_pub;
ros::Publisher copm_pub;
ros::Publisher copD_pub;
ros::Publisher copref_pub;
// com
ros::Publisher comRefe_pub;
ros::Publisher comMea_pub;
ros::Publisher comEsti_pub;
// com velocity
ros::Publisher comVRefe_pub;
ros::Publisher comVMea_pub;
ros::Publisher comVEsti_pub;
// left foot
ros::Publisher leftFootRefe_pub;
ros::Publisher leftFootMea_pub;
// right foot
ros::Publisher rightFootRefe_pub;
ros::Publisher rightFootMea_pub;

ros::Publisher Torso_Ppub;
ros::Publisher Torso_Rpub;
ros::Publisher imuDxl_pub;
ros::Publisher imuTorso_pub;
ros::Publisher StatusPub;
ros::Publisher ServoPositionPub;


ros::Publisher ik_state_pub;


//服务
ros::ServiceServer imuStateService;
ros::ServiceServer StateService;
ros::ServiceServer MasterIDService;
ros::ServiceServer GetStatusService;
ros::ServiceServer GetJointAngleService;
ros::ServiceServer ScanServoService;
ros::ServiceServer PresentTemperatureService;
ros::ServiceServer FSRService;

ros::ServiceServer InstReadValService;
ros::ServiceServer InstWriteValService;
ros::ServiceServer SyncWriteValService;
ros::ServiceServer SetTarPositionValService;
ros::ServiceServer SetTarPositionValAllService;
ros::ServiceServer GetPositionValAllService;

ros::ServiceServer InstReadService;
ros::ServiceServer InstWriteService;
ros::ServiceServer SyncWriteService;
ros::ServiceServer SetLockStateService;
ros::ServiceServer SetLockStateAllService;
ros::ServiceServer GetLockStateAllService;
ros::ServiceServer SetTarPositionService;
ros::ServiceServer SetTarPositionAllService;
ros::ServiceServer GetPositionAllService;
ros::ServiceServer GetRawBulkReadDataService;

void get_ServoAllReadStore(int32_t * tar_storeGetData, int32_t * tar_readGetData)
{
  for (int32_t i = 0; i < 30; i++){
    tar_storeGetData[i] = tar_readGetData[i];
    //ROS_INFO("writeSetData: %d\n", tar_storeGetData[i]);
  }
}

void vectorOut(std::vector<double> &vector_in) {
  /* printf vector double */
  for (unsigned int i = 0; i < vector_in.size(); i++) {
    if (i == vector_in.size() - 1) {
      std::cout << vector_in[i] << std::endl;
    } else
      std::cout << vector_in[i] << ",";
  }
  std::cout << std::endl;
}

int *limit_servo_value(int *arr, uint8_t *idArr, uint8_t idCnt) {
  /*
    arr: data
    idArr: 0 ~ 21
    idCnt: id count
  */
  for (uint8_t index = 0; index < idCnt; index++) {
    if (arr[index] > SERVERS_LIMITS[idArr[index]-1][1]) arr[index] = SERVERS_LIMITS[idArr[index]-1][1];
    else if (arr[index] < SERVERS_LIMITS[idArr[index]-1][0]) arr[index] = SERVERS_LIMITS[idArr[index]-1][0];
  }
}

void ClearQueue(std::queue<bodyhub::JointControlPoint> &Qtempt) {
  if (!Qtempt.empty()) {
    std::queue<bodyhub::JointControlPoint> empty;
    swap(empty, Qtempt);
  }
}

int getch() {
// 获取按键
#if defined(__linux__) || defined(__APPLE__)
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

double Angle2Radian(double angle) { return (angle * PI) / 180; }
double Radian2Angle(double Radian) { return (Radian * 180) / PI; }
double convertValueTOAngle(uint8_t dxlID, int motoValue) {
  return ((motoValue - 2048) / AngleAlpha[dxlID - 1]);
}  // value_to_angle
int convertAngleTOValue(uint8_t dxlID, double motoAngle) {
  return (motoAngle * AngleAlpha[dxlID - 1] + 2048);
}  // angle_to_value

bool isJointError()
{
  if(!ros::ok())
    return true;
  return false;
}

bool WalkingReceiveData(void) {
  /* BulkRead servo Position  FSR*/
  // 硬件模式
  if (!SimControll::simEnable) {
    int32_t present_position[30];
    int32_t present_current[30];
    int32_t present_velocity[30];  // FIXME:
    const char *log = NULL;
    bool result = false;
    float L_average = 0,R_average = 0;

    uint8_t readCount = 12;
    uint8_t bulkReadID[readCount];
    uint16_t bulkReadAddress[readCount];
    uint16_t bulkReadLength[readCount];
    uint8_t readDataLength = 0;
    int32_t bulkReadData[255];
    int32_t bulkReadWord = 0;

    // BulkRead servo Position

    for (uint8_t idNum = 0; idNum < 12; idNum++)  // VALUE_TO_DEGREE
    {
      measuredJointPos[idNum] = mWalk.jointValue[idNum];
    }

    // BulkRead dxl
    readDataLength = 0;
    uint16_t idx = 0;
    // bulkReadID[idx] = BASE_BOARD_ID;
    // bulkReadAddress[idx] = BASE_BOARD_ADDR;
    // bulkReadLength[idx] = BASE_BOARD_LEN;
    // readDataLength += BASE_BOARD_LEN;

    // idx++;
    bulkReadID[idx] = LEFT_FSR_ID;
    bulkReadAddress[idx] = FSR_ADDR;
    bulkReadLength[idx] = FSR_ADDR_LEN;
    readDataLength += FSR_ADDR_LEN;

    idx++;
    bulkReadID[idx] = RIGHT_FSR_ID;
    bulkReadAddress[idx] = FSR_ADDR;
    bulkReadLength[idx] = FSR_ADDR_LEN;
    readDataLength += FSR_ADDR_LEN;

    // ROS_INFO("WalkingReceiveData dxlBulkRead");
    result = dxlBulkRead(bulkReadID, idx + 1, bulkReadAddress, bulkReadLength,
                         bulkReadData);
    if (result == false) {
      return false;
    }

    // uint8_t startAddr = BASE_BOARD_ADDR;
    // dxlGyro[0] = GYRO_COEFFICIENT * TO_INT16(bulkReadData[38 - startAddr],
    //                                          bulkReadData[39 - startAddr]);
    // dxlGyro[1] = GYRO_COEFFICIENT * TO_INT16(bulkReadData[40 - startAddr],
    //                                          bulkReadData[41 - startAddr]);
    // dxlGyro[2] = GYRO_COEFFICIENT * TO_INT16(bulkReadData[42 - startAddr],
    //                                          bulkReadData[43 - startAddr]);
    // dxlAcc[0] = ACC_COEFFICIENT * TO_INT16(bulkReadData[44 - startAddr],
    //                                        bulkReadData[45 - startAddr]);
    // dxlAcc[1] = ACC_COEFFICIENT * TO_INT16(bulkReadData[46 - startAddr],
    //                                        bulkReadData[47 - startAddr]);
    // dxlAcc[2] = ACC_COEFFICIENT * TO_INT16(bulkReadData[48 - startAddr],
    //                                        bulkReadData[49 - startAddr]);
    if (imuType == 0) {
      torsoImu.setGyro(dxlGyro[1], dxlGyro[0], -dxlGyro[2]);
      torsoImu.setAcc(dxlAcc[1], dxlAcc[0], -dxlAcc[2]);
    }
#if 0
  std::cout
      <<"gx:"<<gyro[0]
      <<std::setw(10)<<"gy:"<<gyro[1]
      <<std::setw(10)<<"gz:"<<gyro[2]
      <<std::setw(10)<<"ax:"<<acc[0]
      <<std::setw(10)<<"ay:"<<acc[1]
      <<std::setw(10)<<"az:"<<acc[2]
      <<std::setw(5)<<"\r";
#endif

    int L_values[4] = {bulkReadData[0 + 4 + BASE_BOARD_LEN], 
                      bulkReadData[1 + 4 + BASE_BOARD_LEN], 
                      bulkReadData[2 + 4 + BASE_BOARD_LEN], 
                      bulkReadData[3 + 4 + BASE_BOARD_LEN]};
    int R_values[4] = {bulkReadData[0 + BASE_BOARD_LEN], 
                      bulkReadData[1 + BASE_BOARD_LEN], 
                      bulkReadData[2 + BASE_BOARD_LEN], 
                      bulkReadData[3 + BASE_BOARD_LEN]};
    if ((L_values[0]>50)||(L_values[1]>50)||(L_values[2]>50)||(L_values[3]>50))
      L_average = 200;
    if ((R_values[0]>50)||(R_values[1]>50)||(R_values[2]>50)||(R_values[3]>50))
      R_average = 200;
    // L_average = (bulkReadData[0 + 4 + BASE_BOARD_LEN] + bulkReadData[1 + 4 + BASE_BOARD_LEN] + bulkReadData[2 + 4 + BASE_BOARD_LEN] + bulkReadData[3 + 4 + BASE_BOARD_LEN])/4.0;
    // R_average = (bulkReadData[0 + BASE_BOARD_LEN] + bulkReadData[1 + BASE_BOARD_LEN] + bulkReadData[2 + BASE_BOARD_LEN] + bulkReadData[3 + BASE_BOARD_LEN])/4.0;
    for (uint8_t i = 0; i < 4; i++) {
      mWalk.FSR_L[i] = L_average;   //bulkReadData[i + 4 + BASE_BOARD_LEN];
      mWalk.FSR_R[i] = R_average;   //bulkReadData[i + BASE_BOARD_LEN];
    }



#if 1
  std::cout << std::setw(5) << mWalk.FSR_L[0] << ":"
      << std::setw(5) << mWalk.FSR_L[1] << ":"
      << std::setw(5) << mWalk.FSR_L[2] << ":"
      << std::setw(5) << mWalk.FSR_L[3] << "  R----L"
      << std::setw(5) << mWalk.FSR_R[0] << ":"
      << std::setw(5) << mWalk.FSR_R[1] << ":"
      << std::setw(5) << mWalk.FSR_R[2] << ":"
      << std::setw(5) << mWalk.FSR_R[3] << "#\r"; // \r  //行打印
#endif
  } else {
    std_msgs::Float64MultiArray jointPosition =
        SimControll::SimRobotData.getJointPostion();

    for (uint8_t i = 0; i < jointPosition.data.size(); i++) {
      measuredJointPos[i] =
          WalkJointDirection[i] * Radian2Angle(jointPosition.data.at(i));
    }

    std_msgs::Float64MultiArray leftFT = SimControll::SimRobotData.getLeftFT();
    std_msgs::Float64MultiArray rightFT =
        SimControll::SimRobotData.getRightFT();
    // ROS_INFO("leftFT: %f\trightFT: %f", leftFT.data.at(5),
    // rightFT.data.at(5));

    if (leftFT.data.at(5) > 0.5)
      leftFT.data[5] = 220;
    else
      leftFT.data[5] = 0;

    if (rightFT.data.at(5) > 0.5)
      rightFT.data[5] = 220;
    else
      rightFT.data[5] = 0;

    for (uint8_t i = 0; i < 4; i++) {
      mWalk.FSR_L[i] = leftFT.data[5];
      mWalk.FSR_R[i] = rightFT.data[5];
    }
  }
}

bool WalkingSendData(void) {
  if (!SimControll::simEnable) {
    const char *log = NULL;
    bool result = false;

    double_t armSwingCoeff = 3;
    uint16_t idIndex = 0;
    uint8_t legIds[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    uint8_t bodyIds[22] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
    int32_t motorPos[22] = {0};
    bodyhub::JointControlPoint headCtrMsg;
    int32_t SetHeadPos[2];
    uint8_t head_servo_id[2] = {21, 22};
    uint8_t head_servo_idCnt = 2;


    if (headCtrlQueue.size() > 0) {//if have head queues,cover the value
      pthread_mutex_lock(&mtxHe);
      headCtrMsg = headCtrlQueue.front();

      //id21
      SetHeadPos[0]=convertAngleTOValue(HEAD_SERVO_ID + 1,headCtrMsg.positions.at(0));
      ServoStore.value[HEAD_SERVO_ID] = SetHeadPos[0];
      //id22
      SetHeadPos[1]=convertAngleTOValue(HEAD_SERVO_ID + 2, headCtrMsg.positions.at(1));
      ServoStore.value[HEAD_SERVO_ID+1] = SetHeadPos[1];

      headCtrlQueue.pop();
      pthread_mutex_unlock(&mtxHe);
      //calculate the limit value
      limit_servo_value(SetHeadPos, head_servo_id, head_servo_idCnt);

      // recovert value to angle for saving
      standPos[HEAD_SERVO_ID ]=convertValueTOAngle(HEAD_SERVO_ID + 1, SetHeadPos[0]);
      standPos[HEAD_SERVO_ID + 1]=convertValueTOAngle(HEAD_SERVO_ID + 2, SetHeadPos[1]);
      // ROS_INFO("HEAD CTRL QDATE:21:%d 22:%d", SetHeadPos[0], SetHeadPos[1]);
    }



    for (uint8_t i = 0; i < numberOfId; i++)  // ANGLE_TO_VALUE
    {
      idIndex = dxlIds[i] - 1;
      if (idIndex < 12)
        motorPos[idIndex] =
            convertAngleTOValue(idIndex + 1, WalkJointDirection[idIndex] *
                                                 mWalk.jointValue[idIndex]) +
            servoOffset[idIndex];
      else if (idIndex == 12)
        motorPos[idIndex] =
            convertAngleTOValue(idIndex + 1,
                                -(armSwingCoeff * mWalk.armSwingCount)) +
            servoOffset[idIndex];
      else if (idIndex == 15)
        motorPos[idIndex] =
            convertAngleTOValue(idIndex + 1,
                                -(armSwingCoeff * mWalk.armSwingCount)) +
            servoOffset[idIndex];
      else if (idIndex == 13)
        motorPos[idIndex] = ((standPos[idIndex] - ServoStore.angle[idIndex]) *
                                 mWalk.squatCount / mWalk.squatStep +
                             ServoStore.angle[idIndex]) *
                                AngleAlpha[idIndex] +
                            2048 + servoOffset[idIndex];
      else if (idIndex == 14)
        motorPos[idIndex] = ((standPos[idIndex] - ServoStore.angle[idIndex]) *
                                 mWalk.squatCount / mWalk.squatStep +
                             ServoStore.angle[idIndex]) *
                                AngleAlpha[idIndex] +
                            2048 + servoOffset[idIndex];
      else if (idIndex == 16) 
        motorPos[idIndex] = ((standPos[idIndex] - ServoStore.angle[idIndex]) *
                                 mWalk.squatCount / mWalk.squatStep +
                             ServoStore.angle[idIndex]) *
                                AngleAlpha[idIndex] +
                            2048 + servoOffset[idIndex];
      else if (idIndex == 17)
        motorPos[idIndex] = ((standPos[idIndex] - ServoStore.angle[idIndex]) *
                                 mWalk.squatCount / mWalk.squatStep +
                             ServoStore.angle[idIndex]) *
                                AngleAlpha[idIndex] +
                            2048 + servoOffset[idIndex];
      else
        motorPos[idIndex] =
            convertAngleTOValue(idIndex + 1, standPos[idIndex]) +
            servoOffset[idIndex];
    }
    if (mWalk.RobotState == mWalk.Action_YawAround) {
      motorPos[12] = WalkJointDirection[12] * mWalk.LArm_P * AngleAlpha[12] +
                     2048 + servoOffset[12];
      motorPos[13] = WalkJointDirection[13] * -mWalk.LArm_R * AngleAlpha[13] +
                     2048 + servoOffset[13];
      motorPos[14] =
          WalkJointDirection[14] * -mWalk.LArm_elbow * AngleAlpha[14] + 2048 +
          servoOffset[14];
      motorPos[15] = WalkJointDirection[15] * -mWalk.RArm_P * AngleAlpha[15] +
                     2048 + servoOffset[15];
      motorPos[16] = WalkJointDirection[16] * mWalk.RArm_R * AngleAlpha[16] +
                     2048 + servoOffset[16];
      motorPos[17] =
          WalkJointDirection[17] * mWalk.RArm_elbow * AngleAlpha[17] + 2048 +
          servoOffset[17];

      // ROS_WARN("13:%f, 14:%f, 15:%f, 16:%f, 17:%f,
      // 18:%f",mWalk.LArm_P,-mWalk.LArm_R,-mWalk.LArm_elbow,-mWalk.RArm_P,mWalk.RArm_R,mWalk.RArm_elbow);
    }
    if (armMode == 1) {    
      pthread_mutex_lock(&mtxOpDxl);
      result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, bodyIds,
                                22, motorPos, 1, &log);                                                                                                                
      pthread_mutex_unlock(&mtxOpDxl);
    } else {
      pthread_mutex_lock(&mtxOpDxl);
      result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, legIds,
                                12, motorPos, 1, &log);                                                       
      pthread_mutex_unlock(&mtxOpDxl);
    }
    if (result == false) {
      ROS_ERROR("fail to syncWrite: %s", log);
    }
  } else {//in sim
    uint8_t i = 0;
    std::vector<double> jointCmd;
    std_msgs::Float64MultiArray jointPosition =
        SimControll::SimRobotData.getJointPostion();
    for (i = 0; i < 12; i++) {
      jointCmd.push_back(
          Angle2Radian(WalkJointDirection[i] * mWalk.jointValue[i]));
    }
    if (armMode == 1) {
      for (; i < 18; i++) {
        jointCmd.push_back(0.0);
        if ((i == 12) || (i == 15)) {
          jointCmd[i] =
              Angle2Radian(-1.8 * mWalk.armSwingCount) * WalkJointDirection[i];
        } else {
          jointCmd[i] = Angle2Radian(standPos[i]) * WalkJointDirection[i];
        }
      }
    } else {  // 保持原状态
      for (; i < jointPosition.data.size(); i++) {
        jointCmd.push_back(jointPosition.data[i]);
      }
    }
    SimControll::updateJointCmdQueue(jointCmd);
  }
}


void WalkingStatePublish(void) {
  static bool lastOnWalking = false;
  double_t coeff = 1.0;
  std_msgs::Float64 f64Msg;
  std_msgs::Float64MultiArray f64ArrayMsg;
  std_msgs::Float64MultiArray jointPos;
  jointPos.data.resize(12);

  for (int i = 0; i < 12; ++i) {
    jointPos.data[i] =
        mWalk.measuredJointValue[i] *
        coeff;  // measuredJointPos[i];//measuredJointCurrent[i];//cpWalk->measuredJointValue[i];//
  }
  jointPosMeasurePub.publish(jointPos);
  for (int i = 0; i < 12; ++i) {
    jointPos.data[i] =
        mWalk.jointValue[i] *
        coeff;  // mWalk.jointValue[i];//cpWalk->jointVelocity[i];//
  }
  jointPosTargetPub.publish(jointPos);

  for (int i = 0; i < 12; ++i) {
    jointPos.data[i] =
        cpWalk->measuredJointVelocity[i] *
        coeff;  // measuredJointVel[i];//measuredJointCurrent[i];//cpWalk->measuredJointValue[i];//
  }
  jointVelMeasurePub.publish(jointPos);
  for (int i = 0; i < 12; ++i) {
    jointPos.data[i] = cpWalk->jointVelocity[i] *
                       coeff;  // cpWalk->jointValue[i];//mWalk.jointValue[i];//
  }
  jointVelTargetPub.publish(jointPos);

  if (lastOnWalking != cpWalk->onWalking) {
    lastOnWalking = cpWalk->onWalking;
    f64Msg.data = lastOnWalking;
    WalkingStatusPub.publish(f64Msg);
  }

  f64ArrayMsg.data.resize(6);
  f64ArrayMsg.data[0] = dxlGyro[0];
  f64ArrayMsg.data[1] = dxlGyro[1];
  f64ArrayMsg.data[2] = dxlGyro[2];
  f64ArrayMsg.data[3] = dxlAcc[0];
  f64ArrayMsg.data[4] = dxlAcc[1];
  f64ArrayMsg.data[5] = dxlAcc[2];
  imuDxl_pub.publish(f64ArrayMsg);

  ImuParam_t imuData = torsoImu.getData();
  f64ArrayMsg.data.resize(9);
  f64ArrayMsg.data[0] = imuData.angularVelocity.x;
  f64ArrayMsg.data[1] = imuData.angularVelocity.y;
  f64ArrayMsg.data[2] = imuData.angularVelocity.z;
  f64ArrayMsg.data[3] = imuData.linearAcceleration.x;
  f64ArrayMsg.data[4] = imuData.linearAcceleration.y;
  f64ArrayMsg.data[5] = imuData.linearAcceleration.z;
  f64ArrayMsg.data[6] = imuData.attitude.roll;
  f64ArrayMsg.data[7] = imuData.attitude.pitch;
  f64ArrayMsg.data[8] = imuData.attitude.yaw;
  imuTorso_pub.publish(f64ArrayMsg);

  if (mWalk.StepCountTarget != 0)  //
  {
    std_msgs::Float64 f64Msg;
    std_msgs::Float64MultiArray f64ArrayMsg;

    if (mWalk.StepPhase == GaitManager::LIPMWalk::DoubleSupport)
      f64Msg.data = 0;
    else if (mWalk.StepPhase == GaitManager::LIPMWalk::LeftStance)
      f64Msg.data = -1;
    else if (mWalk.StepPhase == GaitManager::LIPMWalk::RightStance)
      f64Msg.data = 1;
    stepPhase_pub.publish(f64Msg);

    if (mWalk.ContactState == GaitManager::LIPMWalk::DoubleContact)
      f64Msg.data = 0;
    else if (mWalk.ContactState == GaitManager::LIPMWalk::LeftContact)
      f64Msg.data = -0.5;
    else if (mWalk.ContactState == GaitManager::LIPMWalk::RightContact)
      f64Msg.data = 0.5;
    contactState_pub.publish(f64Msg);

    // com
    f64ArrayMsg.data.resize(3);
    f64ArrayMsg.data[0] = mWalk.comRefInWorld(0);
    f64ArrayMsg.data[1] = mWalk.comRefInWorld(1);
    f64ArrayMsg.data[2] = mWalk.comRefInWorld(2);
    comRefe_pub.publish(f64ArrayMsg);

    f64ArrayMsg.data.resize(3);
    f64ArrayMsg.data[0] = mWalk.measuredComInWorld(0);
    f64ArrayMsg.data[1] = mWalk.measuredComInWorld(1);
    f64ArrayMsg.data[2] = mWalk.measuredComInWorld(2);
    comMea_pub.publish(f64ArrayMsg);

    f64ArrayMsg.data.resize(3);
    f64ArrayMsg.data[0] = mWalk.estimatedComInWorld(0);
    f64ArrayMsg.data[1] = mWalk.estimatedComInWorld(1);
    f64ArrayMsg.data[2] = mWalk.estimatedComInWorld(2);
    comEsti_pub.publish(f64ArrayMsg);

    // com velocity
    f64ArrayMsg.data.resize(3);
    f64ArrayMsg.data[0] = mWalk.comvRefInWorld(0);
    f64ArrayMsg.data[1] = mWalk.comvRefInWorld(1);
    f64ArrayMsg.data[2] = mWalk.comvRefInWorld(2);
    comVRefe_pub.publish(f64ArrayMsg);

    f64ArrayMsg.data.resize(3);
    f64ArrayMsg.data[0] = mWalk.measuredComVelocityInWorld(0);
    f64ArrayMsg.data[1] = mWalk.measuredComVelocityInWorld(1);
    f64ArrayMsg.data[2] = mWalk.measuredComVelocityInWorld(2);
    comVMea_pub.publish(f64ArrayMsg);

    f64ArrayMsg.data.resize(3);
    f64ArrayMsg.data[0] = mWalk.estimatedComVelocityInWorld(0);
    f64ArrayMsg.data[1] = mWalk.estimatedComVelocityInWorld(1);
    f64ArrayMsg.data[2] = mWalk.estimatedComVelocityInWorld(2);
    comVEsti_pub.publish(f64ArrayMsg);

    // left foot
    f64ArrayMsg.data.resize(6);
    f64ArrayMsg.data[0] = mWalk.PosPara_wF.Lfoot_x;
    f64ArrayMsg.data[1] = mWalk.PosPara_wF.Lfoot_y;
    f64ArrayMsg.data[2] = mWalk.PosPara_wF.Lfoot_z;
    f64ArrayMsg.data[3] = mWalk.PosPara_wF.Lfoot_R * 180.0 / PI;
    f64ArrayMsg.data[4] = mWalk.PosPara_wF.Lfoot_P * 180.0 / PI;
    f64ArrayMsg.data[5] = mWalk.PosPara_wF.Lfoot_Y * 180.0 / PI;
    leftFootRefe_pub.publish(f64ArrayMsg);

    f64ArrayMsg.data.resize(6);
    f64ArrayMsg.data[0] = mWalk.CurrentPosInP.Lfoot_x;
    f64ArrayMsg.data[1] = mWalk.CurrentPosInP.Lfoot_y;
    f64ArrayMsg.data[2] = mWalk.CurrentPosInP.Lfoot_z;
    f64ArrayMsg.data[3] = mWalk.CurrentPosInP.Lfoot_R * 180.0 / PI;
    f64ArrayMsg.data[4] = mWalk.CurrentPosInP.Lfoot_P * 180.0 / PI;
    f64ArrayMsg.data[5] = mWalk.CurrentPosInP.Lfoot_Y * 180.0 / PI;
    leftFootMea_pub.publish(f64ArrayMsg);

    // right foot
    f64ArrayMsg.data.resize(6);
    f64ArrayMsg.data[0] = mWalk.PosPara_wF.Rfoot_x;
    f64ArrayMsg.data[1] = mWalk.PosPara_wF.Rfoot_y;
    f64ArrayMsg.data[2] = mWalk.PosPara_wF.Rfoot_z;
    f64ArrayMsg.data[3] = mWalk.PosPara_wF.Rfoot_R * 180.0 / PI;
    f64ArrayMsg.data[4] = mWalk.PosPara_wF.Rfoot_P * 180.0 / PI;
    f64ArrayMsg.data[5] = mWalk.PosPara_wF.Rfoot_Y * 180.0 / PI;
    rightFootRefe_pub.publish(f64ArrayMsg);

    f64ArrayMsg.data.resize(6);
    f64ArrayMsg.data[0] = mWalk.CurrentPosInP.Rfoot_x;
    f64ArrayMsg.data[1] = mWalk.CurrentPosInP.Rfoot_y;
    f64ArrayMsg.data[2] = mWalk.CurrentPosInP.Rfoot_z;
    f64ArrayMsg.data[3] = mWalk.CurrentPosInP.Rfoot_R * 180.0 / PI;
    f64ArrayMsg.data[4] = mWalk.CurrentPosInP.Rfoot_P * 180.0 / PI;
    f64ArrayMsg.data[5] = mWalk.CurrentPosInP.Rfoot_Y * 180.0 / PI;
    rightFootMea_pub.publish(f64ArrayMsg);

    // ..
    f64Msg.data = mWalk.CurrentPosInP.Torso_R * 180.0 / PI;
    Torso_Rpub.publish(f64Msg);
    f64Msg.data = mWalk.CurrentPosInP.Torso_P * 180.0 / PI;
    Torso_Ppub.publish(f64Msg);
  }



}

void control_thread_robot() {
  // printf("running control_thread_robot ing .....");
  if (pthread_mutex_trylock(&mtxWl) != 0) return;

  WalkingReceiveData();  // instead of  receiveDataFromDxl()
  if (imuType == 0) torsoImu.datafusion();
  for (int i = 0; i < 12; i++)
    mWalk.measuredJointValue[i] = measuredJointPos[i];
  mWalk.measuredJointVelocity =
      (mWalk.measuredJointValue - mWalk.lastMeasuredJointValue) /
      mWalk.timeStep;
  mWalk.lastMeasuredJointValue.segment(0, 12) =
      mWalk.measuredJointValue.segment(0, 12);
  mWalk.talosRobot.measuredmbc.q =
      sVectorToParam(mWalk.talosRobot.talos,
                     mWalk.measuredJointValue.segment(0, 12) * Util::TO_RADIAN);
  mWalk.talosRobot.measuredmbc.alpha = sVectorToDof(
      mWalk.talosRobot.talos,
      mWalk.measuredJointVelocity.segment(0, 12) * Util::TO_RADIAN);

  mWalk.run();
  WalkingSendData();
  WalkingStatePublish();

  pthread_mutex_unlock(&mtxWl);
}



void jointTrajQueuePoll() {
  if (jointTrajQueue.size() > 0) {
    sensor_msgs::JointState jointStateMsg;

    mtxJointTrajQueue.lock();
    jointStateMsg = jointTrajQueue.front();
    jointTrajQueue.pop();
    mtxJointTrajQueue.unlock();





    uint8_t number = jointStateMsg.name.size();
    uint8_t ids[number] = {0};
    int32_t motorPos_without_offset[number] = {0};
    int32_t motorPos_with_offset[number] = {0};

    for (uint8_t i = 0; i < number; i++) {
      ids[i] = std::stoi(jointStateMsg.name[i]);
      motorPos_without_offset[i] = convertAngleTOValue(ids[i], jointStateMsg.position[i]);
      ServoStore.angle[ids[i] - 1] = jointStateMsg.position[i];
    }

    limit_servo_value(motorPos_without_offset, ids, number);

    for (uint8_t i = 0; i < number; i++) {
      motorPos_with_offset[i] = motorPos_without_offset[i] + servoOffset[ids[i] - 1];
      ServoStore.value[ids[i] - 1] = motorPos_without_offset[i];
    }

    bool result = false;
    const char *log = NULL;
    pthread_mutex_lock(&mtxOpDxl);
    result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, ids, number,
                              motorPos_with_offset, 1, &log);  //同步写指令                       
    pthread_mutex_unlock(&mtxOpDxl);
    if (result == false) {
      ROS_ERROR("fail to syncWrite: %s", log);
    }

    bodyhub::ServoPositionAngle servoPositionsMsg;
    servoPositionsMsg.angle = ServoStore.angle;
    ServoPositionPub.publish(servoPositionsMsg);
  }
}

void *timerThread(void *ptr) {

  // printf("running timerThread ing ....\n");

  if (SimControll::simEnable) return NULL;
  ROS_INFO("Start 'timerThread' thrand...");

  static struct timespec nextTime;
  struct timespec realTime, lastrealTime;
  double loopTimeout;

  const char *log = NULL;
  bool result = false;
  std::vector<double> ServoRadianStore;

  clock_gettime(CLOCK_MONOTONIC, &nextTime);
  clock_gettime(CLOCK_MONOTONIC, &lastrealTime);

  while (ros::ok()) {
    // timeset
    nextTime.tv_sec = lastrealTime.tv_sec +
                      (lastrealTime.tv_nsec + 10 * 1000000) / 1000000000;
    nextTime.tv_nsec = (lastrealTime.tv_nsec + 10 * 1000000) % 1000000000;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &nextTime,
                    NULL);  // start run
    clock_gettime(CLOCK_MONOTONIC, &realTime);
    loopTimeout = ((double)realTime.tv_sec * 1000.0 +
                   (double)realTime.tv_nsec * 0.001 * 0.001) -
                  ((double)lastrealTime.tv_sec * 1000.0 +
                   (double)lastrealTime.tv_nsec * 0.001 * 0.001);
    if (loopTimeout > 10.5)
      ROS_WARN("timerThread() timeout: %f ms", loopTimeout);
    lastrealTime = realTime;

    //== StateEnum::walking
    //下发舵机数据
    if (bodyhubState != StateEnum::walking && bodyhubState != StateEnum::preReady) {
      if (jointTrajQueue.size() > 0) {
        jointTrajQueuePoll();
      } else if ((motoQueue.size() > 0) || (headCtrlQueue.size() > 0)) {
        bodyhub::JointControlPoint jntCtrMsg;
        bodyhub::JointControlPoint headCtrMsg;
        bodyhub::ServoPositionAngle servoPositionsMsg;
        int32_t goalPosition_without_offset[SERVO_NUM];
        int32_t goalPosition_with_offset[SERVO_NUM];
        uint8_t idArray[SERVO_NUM] = {0};
        uint8_t idCnt = 0;

        if (motoQueue.size() > 0) {
          pthread_mutex_lock(&mtxMo);
          jntCtrMsg = motoQueue.front();
          for (uint8_t idNum = 0;
               idNum < SIZE_LIMIT(jntCtrMsg.positions.size(), SERVO_NUM);
               idNum++) {
            ServoStore.angle[idNum] = jntCtrMsg.positions.at(idNum);
            idArray[idCnt] = idNum + 1;

            goalPosition_without_offset[idCnt] =
                convertAngleTOValue(idArray[idCnt],
                                    jntCtrMsg.positions.at(idNum));
            idCnt++;
          }
          for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
            goalPosition_with_offset[idNum] = goalPosition_without_offset[idNum] + servoOffset[idArray[idNum] - 1];
            ServoStore.value[idNum] = goalPosition_with_offset[idCnt];
          }
          motoQueue.pop();
          pthread_mutex_unlock(&mtxMo);
        }
        if (headCtrlQueue.size() > 0) {
          pthread_mutex_lock(&mtxHe);
          if (idCnt > JOINT_SERVO_NUM) idCnt = JOINT_SERVO_NUM;
          headCtrMsg = headCtrlQueue.front();
          for (uint8_t idNum = 0;
               idNum < SIZE_LIMIT(headCtrMsg.positions.size(), HEAD_SERVO_NUM);
               idNum++) {
            ServoStore.angle[idNum + JOINT_SERVO_NUM] =
                headCtrMsg.positions.at(idNum);
            idArray[idCnt] = idCnt + JOINT_SERVO_NUM + 1;  // ID21 ID22

            goalPosition_without_offset[idCnt] =
                convertAngleTOValue(idArray[idCnt],
                                    headCtrMsg.positions.at(idNum));
            idCnt++;
          }  // ID19 ID20
          for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
            goalPosition_with_offset[idNum] = goalPosition_without_offset[idNum] + servoOffset[idArray[idNum] - 1];
            ServoStore.value[idNum + JOINT_SERVO_NUM] = goalPosition_with_offset[idCnt];
          }
          headCtrlQueue.pop();
          pthread_mutex_unlock(&mtxHe);
        }

        // #ifdef DEBUG
        //   ROS_INFO("舵机下发：");
        //   vectorOut(ServoStore.angle);
        //   vectorOut(ServoStore.value);
        // #endif
        limit_servo_value(goalPosition_without_offset, idArray, idCnt);
        
        pthread_mutex_lock(&mtxOpDxl);
        result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray,
                                  idCnt, goalPosition_with_offset, 1, &log);  //同步写指令                             
        pthread_mutex_unlock(&mtxOpDxl);
        if (result == false) {
          ROS_ERROR("SigHandler()---%s", log);
        }
        ServoRadianStore.clear();
        servoPositionsMsg.angle = ServoStore.angle;
        ServoPositionPub.publish(servoPositionsMsg);
      }
    } else if (bodyhubState == StateEnum::walking){//in StateEnum::walking
      control_thread_robot();
    } else {// in StateEnum::preReady
      if (jointTrajQueue.size() > 0 || (motoQueue.size() > 0) || (headCtrlQueue.size() > 0)) {
        ROS_WARN("NOT IN ready state, unable to execute action");
        ClearTimerQueue();
      } 
    }
  }

  return NULL;
}



bool ServoBulkRead(uint8_t *bulkReadID, uint8_t readCount, std::string itemName,
                   int32_t *bulkReadData) {
  /* Dynamixels bulkread */
  const char *log = NULL;
  bool result = false;

  pthread_mutex_lock(&mtxOpDxl);
  dxl_wb.clearBulkReadParam();
  for (uint8_t idNum = 0; idNum < readCount; idNum++) {
    result = dxl_wb.addBulkReadParam(bulkReadID[idNum], itemName.c_str(), &log);
    if (result == false) ROS_ERROR("ServoBulkRead# %s", log);
  }
  result = dxl_wb.bulkRead(&log);
  if (result == false) {
    ROS_ERROR("ServoBulkRead$ %s\n", log);
    pthread_mutex_unlock(&mtxOpDxl);
    return false;
  }

  result = dxl_wb.getBulkReadData(&bulkReadData[0], &log);
  if (result == false) {
    ROS_ERROR("ServoBulkRead# %s\n", log);
    pthread_mutex_unlock(&mtxOpDxl);
    return false;
  }
  pthread_mutex_unlock(&mtxOpDxl);
  return true;
}

bool dxlBulkRead(uint8_t *bulkReadID, uint8_t readCount,
                 uint16_t *bulkReadAddress, uint16_t *bulkReadLength,
                 int32_t *bulkReadData) {
  const char *log = NULL;
  bool result = false;

  pthread_mutex_lock(&mtxOpDxl);
  dxl_wb.clearBulkReadParam();
  for (uint8_t idNum = 0; idNum < readCount; idNum++) {
    result = dxl_wb.addBulkReadParam(bulkReadID[idNum], bulkReadAddress[idNum],
                                     bulkReadLength[idNum], &log);
    if (result == false) {
      ROS_ERROR("dxlBulkRead error1: %s", log);
      pthread_mutex_unlock(&mtxOpDxl);
      return false;
    }
  }
  result = dxl_wb.bulkRead(&log);
  if (result == false) {
    ROS_WARN("dxlBulkRead error22: %s ", log);
    for (uint8_t idNum = 0; idNum < readCount; idNum++) {
      ROS_WARN("dxlBulkRead error23: %d",  bulkReadID[idNum]);
    }
    pthread_mutex_unlock(&mtxOpDxl);
    return false;
  }
  result =
      dxl_wb.getRawBulkReadData(&bulkReadID[0], readCount, &bulkReadAddress[0],
                                &bulkReadLength[0], &bulkReadData[0], &log);
  if (result == false) {
    ROS_ERROR("dxlBulkRead error3: %s", log);
    pthread_mutex_unlock(&mtxOpDxl);
    return false;
  }

  pthread_mutex_unlock(&mtxOpDxl);
  return true;
}

bool SensorBulkWrite(uint8_t WriteCount, uint8_t *bulkWriteID,
                     uint16_t *bulkWriteAddress, uint16_t *bulkWriteLenght,
                     int32_t *bulkWriteData) {
  const char *log = NULL;
  bool result = false;

  pthread_mutex_lock(&mtxOpDxl);
  for (uint8_t index = 0; index < WriteCount; index++) {
    result = dxl_wb.addBulkWriteParam(
        bulkWriteID[index], bulkWriteAddress[index], bulkWriteLenght[index],
        bulkWriteData[index], &log);
    if (result == false) {
      ROS_ERROR("addBulkWriteParam() %s", log);
      pthread_mutex_unlock(&mtxOpDxl);
      return false;
    }
  }
  result = dxl_wb.bulkWrite(&log);
  if (result == false) {
    ROS_ERROR("bulkWrite() %s\n", log);
    pthread_mutex_unlock(&mtxOpDxl);
    return false;
  }
  pthread_mutex_unlock(&mtxOpDxl);
  return true;
}

// bool ServoMovingGet() {
//   /* at least 1 servo is moving return true ,else return false */
//   int32_t movingGet[20] = {0};

//   ServoBulkRead(dxlIds, numberOfId, "Moving", &movingGet[0]);
//   for (uint8_t idNum = 0; idNum < numberOfId; idNum++) {
//     if (movingGet[idNum]) {
// #ifdef DEBUG
//       ROS_INFO("Moving !!");
// #endif
//       return true;
//     }
//   }
//   return false;
// }

std::vector<double> linspace(double start, double end, int num) {
  std::vector<double> linspaced;

  if (num == 0) {
    return linspaced;
  }
  if (num == 1) {
    linspaced.push_back(start);
    return linspaced;
  }

  double delta = (end - start) / (num);

  for (int i = 0; i <= num; ++i) {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);

  return linspaced;
}

void RobotStand(uint8_t *ids, uint8_t idCnt, int32_t velocity) {
  
  std::vector<double> diffAngle(standPos, standPos+SERVO_NUM);                             
  bodyhub::ServoPositionAngle servoPositionsMsg;

  if (!SimControll::simEnable) {
    const char *log = NULL;
    uint8_t idArray[30] = {0};
    int32_t standPositionGoal[30];
    double_t linspaceTime;
    uint8_t intervalCount;
    std::vector<std::vector<double>> motoMoveSequence(30);
    std::cout << "diffAngle:" << std::endl;
    for (uint8_t i = 0; i < SERVO_NUM; i++) {
      //printf("%.1f  ",ServoStore.angle[i]);
      //printf("---- %.1f  ",diffAngle[i]);
      diffAngle[i] = fabs(diffAngle[i] - ServoStore.angle[i]);
      printf("%.1f  ",diffAngle[i]);
    }
    std::cout << std::endl;
    auto maxAngle = max_element(diffAngle.begin(), diffAngle.end());   
    linspaceTime = *maxAngle / 100;    //  100°/s
    intervalCount = linspaceTime / mWalk.timeStep;  //插值次数
  
    printf("maxAngle：%.2f, linspaceTime: %.2f, intervalCount: %d \n", *maxAngle, linspaceTime, intervalCount);
    for (uint8_t idNum = 0; idNum < idCnt; idNum++) { 
      idArray[idNum] = ids[idNum];
      standPositionGoal[idNum] =
          convertAngleTOValue(idArray[idNum], standPos[idArray[idNum] - 1]);
      motoMoveSequence[idNum] =
          linspace(ServoStore.value[idArray[idNum] - 1],
                   standPositionGoal[idNum], intervalCount);
      ServoStore.angle[idArray[idNum] - 1] = standPos[idArray[idNum] - 1];
      ServoStore.value[idArray[idNum] - 1] = standPositionGoal[idNum];
    }
    ros::Rate loopRate(1.0 / mWalk.timeStep);
    get_ServoAllReadStore(ServoAllReadStore, standPositionGoal);
    //ROS_INFO("222222------>>>>>");
    pthread_mutex_lock(&mtxOpDxl);
    for (uint8_t standCycleT = 0; standCycleT < intervalCount; standCycleT++) {
      for (uint8_t idNum = 0; idNum < idCnt; idNum++)
        standPositionGoal[idNum] = motoMoveSequence[idNum][standCycleT] +
                                   servoOffset[idArray[idNum] - 1];
      dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray, idCnt,
                       standPositionGoal, 1, &log);  //同步写指令             
                                                     // TODO: fail return                                                                                                                                          
      loopRate.sleep();
    }
    pthread_mutex_unlock(&mtxOpDxl);
  } else  // simulation joint command update
  {
    int16_t frameNumber = 1.5 / mWalk.timeStep;  // 1.5s
    std::vector<std::vector<double>> positionVector;
    std::vector<double> jointPosition;
    std_msgs::Float64MultiArray getPosition =
        SimControll::SimRobotData.getJointPostion();  // radian

    for (uint8_t i = 0; i < SERVO_NUM; i++) {
      positionVector.push_back(linspace(
          getPosition.data[i], standPos[i] * Util::TO_RADIAN, frameNumber));
      ServoStore.angle[i] = standPos[i];
    }
    for (uint16_t n = 0; n < frameNumber; n++) {
      for (uint16_t m = 0; m < SERVO_NUM; m++) {
        if ((m >= 12) && (armMode == 0))
          jointPosition.push_back(getPosition.data[m]);
        else
          jointPosition.push_back(positionVector[m][n]);
      }
      SimControll::updateJointCmdQueue(jointPosition);  // radian
      jointPosition.clear();
    }
  }
  servoPositionsMsg.angle = ServoStore.angle;
  ServoPositionPub.publish(servoPositionsMsg);
}

void motionToPosture(double_t duration, uint8_t number, uint8_t *ids,
                     double_t *position) {
  bool result;
  const char *log = NULL;
  int32_t readData = 0;
  uint32_t numberOfFrame = duration / mWalk.timeStep;  // duration s
  std::vector<std::vector<double>> positionVector;
  int32_t motorPos[number];

  pthread_mutex_lock(&mtxOpDxl);
  for (uint8_t i = 0; i < number; i++) {
    for (uint8_t c = 0; c < 3; c++)  // read 3 times
    {
      result = dxl_wb.readRegister(ids[i], "Present_Position", &readData, &log);
      if (result == true) break;
    }
    if (result == false) {
      ROS_ERROR("motionToPosture error: ID is %d, %s", ids[i], log);
    } else {
      ServoStore.angle[ids[i] - 1] =
          convertValueTOAngle(ids[i], readData - servoOffset[ids[i] - 1]);
      ServoStore.value[ids[i] - 1] = readData;
    }
  }
  pthread_mutex_unlock(&mtxOpDxl);
  // for (uint8_t i = 0; i < SERVO_NUM; i++)
  //   ROS_INFO("ServoStore.angle[%d]: %f", i, ServoStore.angle[i]);

  double_t targetValue;
  for (uint8_t i = 0; i < number; i++) {
    targetValue =
        convertAngleTOValue(ids[i], position[i]) + servoOffset[ids[i] - 1];
    positionVector.push_back(
        linspace(ServoStore.value[ids[i] - 1], targetValue, numberOfFrame));
    ServoStore.angle[ids[i]] = position[i];
    ServoStore.value[ids[i] - 1] = targetValue;
  }
  ros::Rate loopRate(1.0 / mWalk.timeStep);
  pthread_mutex_lock(&mtxOpDxl);
  for (uint16_t n = 0; n < numberOfFrame; n++) {
    for (uint16_t m = 0; m < number; m++) {
      motorPos[m] = positionVector[m][n];
    }
    result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, ids, number,
                              motorPos, 1, &log);  //同步写指令                            
    if (result == false) {
      ROS_ERROR("motionToPosture error: %s", log);
    }
    loopRate.sleep();
  }
  pthread_mutex_unlock(&mtxOpDxl);
}

void LoadOffset(const std::string path) {
  std::map<std::string, double> offsetMap;
  std::string stringID = "ID";
  std::string IDNameStr;

  YAML::Node offsetDoc;
  try {
    offsetDoc = YAML::LoadFile(path.c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Fail to load offset yaml.");
    return;
  }

  YAML::Node itemData = offsetDoc["offset"];
  if (itemData.size() == 0) return;

  for (YAML::const_iterator itItemNum = itemData.begin();
       itItemNum != itemData.end(); itItemNum++) {
    std::string IDName = itItemNum->first.as<std::string>();
    double offsetValue = itItemNum->second.as<double>();

    offsetMap[IDName] = offsetValue;
  }
  std::cout << "servo offset:\n";
  for (uint8_t i = 0; i < SERVO_NUM; i++) {
    IDNameStr = stringID + std::to_string(i + 1);
    if (offsetMap.find(IDNameStr) == offsetMap.end())
      ROS_WARN("\nwithout find offset of %s ", IDNameStr.c_str());
    else {
      servoOffset[i] = offsetMap[IDNameStr];
    }
    std::cout << servoOffset[i] << "  ";
  }
  std::cout << "\n";
}

void LoadDxlInitPose(const std::string path) {
  std::map<std::string, double> initPosMap;
  std::string stringID = "ID";
  std::string IDNameStr;

  YAML::Node initPoseDoc;
  try {
    initPoseDoc = YAML::LoadFile(path.c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Fail to load dxlinitPose yaml.");
    return;
  }

  YAML::Node itemData = initPoseDoc["InitPose"];
  if (itemData.size() == 0) return;

  for (YAML::const_iterator itItemNum = itemData.begin();
       itItemNum != itemData.end(); itItemNum++) {
    std::string IDName = itItemNum->first.as<std::string>();
    double initPoseValue = itItemNum->second.as<double>();

    initPosMap[IDName] = initPoseValue;
  }
  std::cout << "stand position:\n";
  for (uint8_t i = 0; i < SERVO_NUM; i++) {
    IDNameStr = stringID + std::to_string(i + 1);
    if (initPosMap.find(IDNameStr) == initPosMap.end())
      ROS_WARN("\nwithout find standPos of %s ", IDNameStr.c_str());
    else {
      standPos[i] = initPosMap[IDNameStr];
    }
    std::cout << standPos[i] << "  ";
  }
  std::cout << "\n";
}




void jointControlCallback(const sensor_msgs::JointState::ConstPtr &msg) {
  // ROS_INFO("jointControlCallback received, names count=%lu", msg->name.size());
  // msg->name.size()=23
  if (!mtxFSM.try_lock()) return;
  uint16_t msgId;
  try {
    msgId = std::stoi(msg->name.at(msg->name.size() - 1));
  } catch (const std::exception &e) {
    std::cerr << "catch error: " << e.what() << '\n';
    return;
  }

  if (currentControlId == msgId) {
    sensor_msgs::JointState jointStateMsg;
    jointStateMsg = *msg;
    jointStateMsg.name.pop_back();
    for (uint8_t i = 0; i < jointStateMsg.name.size(); i++) {
      if (i >= jointStateMsg.position.size())
        jointStateMsg.position.push_back(0.0f);

      if (i >= jointStateMsg.velocity.size())
        jointStateMsg.velocity.push_back(0.0f);

      if (i >= jointStateMsg.effort.size())
        jointStateMsg.effort.push_back(0.0f);
    }

    mtxJointTrajQueue.lock();
    jointTrajQueue.push(jointStateMsg);
    // for(uint8_t i=0; i<12; i++)
    //   ROS_INFO("jointStateMsg push ID:%d Pos:%.2f", i+1, jointStateMsg.position[i]);
    mtxJointTrajQueue.unlock();

   

    //数据到达
    if ((bodyhubState == StateEnum::ready) ||
        (bodyhubState == StateEnum::pause))
      UpdateState(StateEnum::running);
  }
  mtxFSM.unlock();
}

void MotoPositionCallback(const bodyhub::JointControlPoint::ConstPtr &msg) {
  if (!mtxFSM.try_lock()) return;
  if (currentControlId == msg->mainControlID) {
    bodyhub::JointControlPoint jointControlMsg;
    jointControlMsg.mainControlID = msg->mainControlID;
    jointControlMsg.positions = msg->positions;
    for (uint8_t i = 0; i < msg->positions.size(); i++) {
      if (i < msg->velocities.size())
        jointControlMsg.velocities.push_back(msg->velocities.at(i));
      else
        jointControlMsg.velocities.push_back(0.0f);

      if (i < msg->accelerations.size())
        jointControlMsg.accelerations.push_back(msg->accelerations.at(i));
      else
        jointControlMsg.accelerations.push_back(0.0f);
    }

    pthread_mutex_lock(&mtxMo);
    motoQueue.push(jointControlMsg);
 
    pthread_mutex_unlock(&mtxMo);

    //数据到达
    if ((bodyhubState == StateEnum::ready) ||
        (bodyhubState == StateEnum::pause))
      UpdateState(StateEnum::running);
  }
  mtxFSM.unlock();
}

void HeadPositionCallback(const bodyhub::JointControlPoint::ConstPtr &msg) {
  /* 仅头部舵机数据接收 0_ID19 1_ID20 */
  if (!mtxFSM.try_lock()) return;
  if (currentControlId == msg->mainControlID) {
    bodyhub::JointControlPoint jointControlMsg;
    jointControlMsg.mainControlID = msg->mainControlID;
    jointControlMsg.positions = msg->positions;
    for (uint8_t i = 0; i < msg->positions.size(); i++) {
      if (i < msg->velocities.size())
        jointControlMsg.velocities.push_back(msg->velocities.at(i));
      else
        jointControlMsg.velocities.push_back(0.0f);

      if (i < msg->accelerations.size())
        jointControlMsg.accelerations.push_back(msg->accelerations.at(i));
      else
        jointControlMsg.accelerations.push_back(0.0f);
    }

    pthread_mutex_lock(&mtxMo);
    headCtrlQueue.push(jointControlMsg);
    pthread_mutex_unlock(&mtxMo);

    //数据到达
    if ((bodyhubState == StateEnum::ready) ||
        (bodyhubState == StateEnum::pause))
      UpdateState(StateEnum::running);
  }
  mtxFSM.unlock();
}

bool InstReadValSrvCallback(bodyhub::SrvInstRead::Request &req,
                            bodyhub::SrvInstRead::Response &res) {
  bool result = false;
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t readGetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    pthread_mutex_lock(&mtxOpDxl);
    result = dxl_wb.readRegister(dxlID, itemName.c_str(), &readGetData, &log);
    pthread_mutex_unlock(&mtxOpDxl);
    if (result == false) {
      ROS_WARN("%s\n", log);
      res.getData = SERVO_DEFAULT_VALUE;
    } else
      res.getData = readGetData;
  } else {
    res.getData = SERVO_DEFAULT_VALUE;
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}
bool InstWriteValSrvCallback(bodyhub::SrvInstWrite::Request &req,
                             bodyhub::SrvInstWrite::Response &res) {
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    writeSetData = req.setData;
    pthread_mutex_lock(&mtxOpDxl);
    dxl_wb.writeOnlyRegister(dxlID, itemName.c_str(), writeSetData, &log);
    pthread_mutex_unlock(&mtxOpDxl);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}
bool SyncWriteValSrvCallback(bodyhub::SrvSyncWrite::Request &req,
                             bodyhub::SrvSyncWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;
  uint8_t handleIndex = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;

    if (req.itemName == "Goal_Position") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_POSITION;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }
    } else if (req.itemName == "Moving_Speed") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }
    } else {
      res.complete = result;
      return true;
    }

    pthread_mutex_lock(&mtxOpDxl);
    result = dxl_wb.syncWrite(handleIndex, idArray, idCnt, writeSetData, 1,
                              &log);  //同步写指令                            
    pthread_mutex_unlock(&mtxOpDxl);
  } else
    {
      ROS_WARN("YOU ARE NOT IN directOperate");
      return false;
    }
  res.complete = result;
  return true;
}

bool SetServoTarPositionValCallback(bodyhub::SrvServoWrite::Request &req,
                                    bodyhub::SrvServoWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      jointPosition.data[req.dxlID] =
          Angle2Radian(convertValueTOAngle(req.dxlID, req.setData));
      SimControll::updateJointCmdQueue(jointPosition.data);
    } else {
      dxlID = req.dxlID;
      writeSetData = req.setData;
      pthread_mutex_lock(&mtxOpDxl);
      dxl_wb.writeOnlyRegister(dxlID, "Goal_Position", writeSetData, &log);
      pthread_mutex_unlock(&mtxOpDxl);
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}

bool SetServoTarPositionValAllCallback(
    bodyhub::SrvServoAllWrite::Request &req,
    bodyhub::SrvServoAllWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < req.idArray.size(); i++)
        jointPosition.data[req.idArray[i] - 1] =
            Angle2Radian(convertValueTOAngle(req.idArray[i], req.setData[i]));
      SimControll::updateJointCmdQueue(jointPosition.data);
    } else {
      idCnt = req.idCnt;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }   
      get_ServoAllReadStore(ServoAllReadStore, writeSetData);

      // for (int i = 0; i < 22; i++){
      //   ROS_WARN("ServoAllReadStore: %d\n", ServoAllReadStore[i]);
      //   ServoStore.angle[i] = convertValueTOAngle(i + 1, ServoAllReadStore[i]);
      // }

      //ROS_INFO("33333----->>>>>\n");
      pthread_mutex_lock(&mtxOpDxl);
      result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray,
                                idCnt, writeSetData, 1, &log);  //同步写指令                                                                                                          
      pthread_mutex_unlock(&mtxOpDxl);
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}

bool GetServoPositionValAllCallback(bodyhub::SrvServoAllRead::Request &req,
                                    bodyhub::SrvServoAllRead::Response &res) {
  uint8_t idArray[30] = {0};
  uint8_t idCnt = 0;
  int32_t readGetData[30];


  const char *log = NULL;
  bool result = false;


  //读值前使用上次零点调试舵机下发值，出现读值失败时使用此默认值 
  get_ServoAllReadStore(readGetData, ServoAllReadStore);
  //ROS_INFO("4444------->>>>s");
  //ROS_INFO("ServoAllReadStore: %d", ServoAllReadStore[0]);

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < req.idArray.size(); i++)
        res.getData.push_back(convertAngleTOValue(
            req.idArray[i],
            Radian2Angle(jointPosition.data[req.idArray[i] - 1])));
    } else {
      idCnt = req.idCnt;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
      }

      // ServoBulkRead(idArray, idCnt, "Present_Position", &readGetData[0]);

      for (uint8_t idNum = 0; idNum < SERVO_NUM; idNum++) {
        result = dxl_wb.readRegister(idNum + 1, "Present_Position", &readGetData[idNum], &log);
        usleep(1000);
        // ROS_WARN("BACK BBBBB %d   %d", idNum+1, readGetData[idNum]);
      }

      
      for (uint8_t idNum = 0; idNum < idCnt; idNum++)
        res.getData.push_back(readGetData[idNum]);  

      if (numberOfId != 22) {
        ROS_WARN("Failed to scan %d servo(s)",SERVO_NUM - numberOfId);
        for (uint8_t i = 0; i < SERVO_NUM - numberOfId; i++) {
        if (failed_ID[i] == 0)
          break;
        res.getData.insert(res.getData.begin()+failed_ID[i]-1, ServoStore.value[failed_ID[i]-1]);
        res.getData.pop_back();
        ROS_INFO("Insert ServoStore value in ID: %d. times: %d", failed_ID[i], i+1);
        }
      }                      
    }
  } else {
    res.getData.push_back(0);
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}

bool InstReadSrvCallback(bodyhub::SrvInstRead::Request &req,
                         bodyhub::SrvInstRead::Response &res) {
  bool result = false;
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t readGetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    pthread_mutex_lock(&mtxOpDxl);
    result = dxl_wb.readRegister(dxlID, itemName.c_str(), &readGetData, &log);
    pthread_mutex_unlock(&mtxOpDxl);
    if (result == false) {
      ROS_WARN("%s\n", log);
      res.getData = 998;
    } else if (itemName == "Present_Position")
      res.getData = convertValueTOAngle(dxlID, readGetData);  // value_to_angle
    else
      res.getData = readGetData;
  } else {
    res.getData = 999;
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}

bool InstWriteSrvCallback(bodyhub::SrvInstWrite::Request &req,
                          bodyhub::SrvInstWrite::Response &res) {
  const char *log = NULL;
  std::string itemName;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    itemName = req.itemName;
    if (itemName == "Goal_Position")
      writeSetData = convertAngleTOValue(dxlID, req.setData);
    else
      writeSetData = req.setData;
    pthread_mutex_lock(&mtxOpDxl);
    dxl_wb.writeOnlyRegister(dxlID, itemName.c_str(), writeSetData, &log);
    pthread_mutex_unlock(&mtxOpDxl);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}

bool SyncWriteSrvCallback(bodyhub::SrvSyncWrite::Request &req,
                          bodyhub::SrvSyncWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;
  uint8_t handleIndex = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;

    if (req.itemName == "Goal_Position") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_POSITION;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] =
            convertAngleTOValue(idArray[idNum], req.setData[idNum]);
      }
    } else if (req.itemName == "Moving_Speed") {
      handleIndex = SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] = req.setData[idNum];
      }
    } else {
      res.complete = result;
      return true;
    }

    pthread_mutex_lock(&mtxOpDxl);
    result = dxl_wb.syncWrite(handleIndex, idArray, idCnt, writeSetData, 1,
                              &log);  //同步写指令                             
    pthread_mutex_unlock(&mtxOpDxl);
  } else
    {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  res.complete = result;
  return true;
}

bool SetServoLockStateCallback(bodyhub::SrvServoWrite::Request &req,
                               bodyhub::SrvServoWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    dxlID = req.dxlID;
    writeSetData = (int32_t)req.setData;
    pthread_mutex_lock(&mtxOpDxl);
    if (writeSetData == 0)
      dxl_wb.torqueOff(dxlID, &log);
    else
      dxl_wb.torqueOn(dxlID, &log);
    pthread_mutex_unlock(&mtxOpDxl);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}
bool SetServoLockStateAllCallback(bodyhub::SrvServoAllWrite::Request &req,
                                  bodyhub::SrvServoAllWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;
    pthread_mutex_lock(&mtxOpDxl);
    for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
      idArray[idNum] = req.idArray[idNum];       // test
      writeSetData[idNum] = req.setData[idNum];  // test
      if (writeSetData[idNum] == 0)
        dxl_wb.writeRegister(idArray[idNum], "Torque_Enable", 0, &log);
      else if (writeSetData[idNum] == 1)
        dxl_wb.writeRegister(idArray[idNum], "Torque_Enable", 1, &log);
    }
    pthread_mutex_unlock(&mtxOpDxl);
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}
bool GetServoLockStateAllCallback(bodyhub::SrvServoAllRead::Request &req,
                                  bodyhub::SrvServoAllRead::Response &res) {
  uint8_t idArray[30] = {0};
  uint8_t idCnt = 0;
  int32_t readGetData[30] = {0};

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    idCnt = req.idCnt;
    for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
      idArray[idNum] = req.idArray[idNum];
    }
    ServoBulkRead(idArray, idCnt, "Torque_Enable", &readGetData[0]);
    for (uint8_t idNum = 0; idNum < idCnt; idNum++)
      res.getData.push_back(readGetData[idNum]);
  } else {
    res.getData.push_back(0);
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}
bool SetServoTarPositionCallback(bodyhub::SrvServoWrite::Request &req,
                                 bodyhub::SrvServoWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t dxlID = 0;
  int32_t writeSetData = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      jointPosition.data[req.dxlID] = Angle2Radian(req.setData);
      SimControll::updateJointCmdQueue(jointPosition.data);
    } else {
      dxlID = req.dxlID;
      writeSetData = convertAngleTOValue(dxlID, req.setData);
      pthread_mutex_lock(&mtxOpDxl);
      dxl_wb.writeRegister(dxlID, "Goal_Position", writeSetData, &log);
      pthread_mutex_unlock(&mtxOpDxl);
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}
bool SetServoTarPositionAllCallback(bodyhub::SrvServoAllWrite::Request &req,
                                    bodyhub::SrvServoAllWrite::Response &res) {
  bool result = false;
  const char *log = NULL;
  uint8_t idArray[30] = {0};
  int32_t writeSetData[30];
  uint8_t idCnt = 0;

  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < req.idArray.size(); i++)
        jointPosition.data[req.idArray[i] - 1] = Angle2Radian(req.setData[i]);
      SimControll::updateJointCmdQueue(jointPosition.data);
    } else {
      idCnt = req.idCnt;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
        writeSetData[idNum] =
            convertAngleTOValue(req.idArray[idNum], req.setData[idNum]);
      }
      pthread_mutex_lock(&mtxOpDxl);
      result = dxl_wb.syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idArray,
                                idCnt, writeSetData, 1, &log);  //同步写指令                            
      pthread_mutex_unlock(&mtxOpDxl);
    }
    res.complete = true;
  } else {
    res.complete = false;
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}
bool GetServoPositionAllCallback(bodyhub::SrvServoAllRead::Request &req,
                                 bodyhub::SrvServoAllRead::Response &res) {
  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate || bodyhubState == StateEnum::external_control) {
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < req.idArray.size(); i++)
        res.getData.push_back(
            Radian2Angle(jointPosition.data[req.idArray[i] - 1]));
    } else {
      uint8_t idArray[30] = {0};
      uint8_t idCnt = 0;
      int32_t readGetData[30] = {0};
      bool result;

      idCnt = req.idCnt;
      for (uint8_t idNum = 0; idNum < idCnt; idNum++) {
        idArray[idNum] = req.idArray[idNum];
      }
      result = ServoBulkRead(idArray, idCnt, "Present_Position", &readGetData[0]);
      if (result == false) {
        return false;
      }
      for (uint8_t idNum = 0; idNum < idCnt; idNum++)
        res.getData.push_back(convertValueTOAngle(
            req.idArray[idNum], readGetData[idNum]));  // value_to_angle
    }
  } else {
    res.getData.push_back(0);
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }
  return true;
}

bool GetRawBulkReadDataCallback(bodyhub::SrvRawBulkReadData::Request &req,
                                 bodyhub::SrvRawBulkReadData::Response &res) {
  //服务被请求
  if (bodyhubState == StateEnum::ready) UpdateState(StateEnum::directOperate);

  if (bodyhubState == StateEnum::directOperate || bodyhubState == StateEnum::external_control) {
    uint8_t number = req.number;
    
    uint8_t idArray[number] = {0};
    uint16_t addr[number] = {0};
    uint16_t length[number] = {0};
    int32_t rData[6 * number] = {0};
    bool result = false;

    for (uint8_t idNum = 0; idNum < number; idNum++){
      idArray[idNum] = req.idArray[idNum];
      addr[idNum] = req.addr[idNum];
      length[idNum] = req.length[idNum];
    }

    result = dxlBulkRead(idArray, number, addr, length, rData);
    if (result == false)
    {
      ROS_ERROR("GetRawBulkReadData failed");
      return false;
    }
    for (uint8_t idNum = 0; idNum < 6*number; idNum++)
      res.rawData.push_back(rData[idNum]);
    
  } else {
    ROS_WARN("YOU ARE NOT IN directOperate");
    return false;
  }

  return true;
}

bool StateSrvCallback(bodyhub::SrvState::Request &req,
                      bodyhub::SrvState::Response &res) {
  if (mtxFSM.try_lock()) {
    if ((req.masterID == currentControlId) || (currentControlId == 0)) {
      if (req.stateReq == "setStatus") {
        currentControlId = req.masterID;
        if ((bodyhubState == StateEnum::preReady) ||
            (bodyhubState == StateEnum::external_control)|| 
            (bodyhubState == StateEnum::directOperate)) {
          UpdateState(StateEnum::ready);
        }
      } else if (req.stateReq == "resetStatus") {
      } else if (req.stateReq == "break") {
      } else if (req.stateReq == "pause"){    // TTT:增加pause状态设置
        if((bodyhubState == StateEnum::walking)){
          pthread_mutex_lock(&mtxWl);
          ClearTimerQueue();
          UpdateState(StateEnum::pause);
          pthread_mutex_unlock(&mtxWl);
        }
      } 
      else if (req.stateReq == "stop") {
        if ((bodyhubState == StateEnum::running) ||
            (bodyhubState == StateEnum::pause) ||
            (bodyhubState == StateEnum::walking)) {
          pthread_mutex_lock(&mtxWl);
          ClearTimerQueue();
          UpdateState(StateEnum::stoping);
          UpdateState(StateEnum::ready);
          pthread_mutex_unlock(&mtxWl);
        }
      } else if (req.stateReq == "reset") {
        if (bodyhubState != StateEnum::stoping) {
          pthread_mutex_lock(&mtxWl);
          UpdateState(StateEnum::preReady);
          currentControlId = 0;
          pthread_mutex_unlock(&mtxWl);
        }
      } else if (req.stateReq == "walking") {
        if ((bodyhubState == StateEnum::ready) ||
            (bodyhubState == StateEnum::pause)) {
          UpdateState(StateEnum::walking);
        }
      } else if (req.stateReq == "external_control") {
        if (bodyhubState != StateEnum::preReady) {
          if (bodyhubState == StateEnum::ready) {
            UpdateState(StateEnum::preReady);
          } else if ((bodyhubState == StateEnum::running) ||
                      (bodyhubState == StateEnum::pause) ||
                      (bodyhubState == StateEnum::walking)) {
            pthread_mutex_lock(&mtxWl);
            ClearTimerQueue();
            UpdateState(StateEnum::stoping);
            UpdateState(StateEnum::ready);
            UpdateState(StateEnum::preReady);
            pthread_mutex_unlock(&mtxWl);
          }
        }
        UpdateState(StateEnum::external_control);
      }
      res.stateRes = bodyhubState;
    } else
      res.stateRes = currentControlId;
    mtxFSM.unlock();
    return true;
  }
  ROS_WARN("FSM service busy! Request_state:%s", req.stateReq.c_str());
  return false;
}

bool GetStatusCallback(bodyhub::SrvString::Request &req,
                       bodyhub::SrvString::Response &res) {
  if (req.str != "") {
    res.data = stateNewStr;
    res.poseQueueSize = motoQueue.size();
    res.jointQueueSize = jointTrajQueue.size();
  }
  return true;
}

bool GetJointAngleCallback(bodyhub::SrvServoAllRead::Request &req,
                           bodyhub::SrvServoAllRead::Response &res) {
  res.getData = ServoStore.angle;
  return true;
}

bool MasterIDSrvCallback(bodyhub::SrvTLSstring::Request &req,
                         bodyhub::SrvTLSstringResponse &res) {
#ifdef DEBUG
// ROS_INFO("MasterIDSrvCallback get masterID %s", req.str.c_str());
#endif
  res.data = currentControlId;
  return true;
}

void *queueThread(void *ptr) {
  ROS_INFO("Start 'queueThread' thrand...");
  ros::NodeHandle n;
  ros::CallbackQueue topicQueue;

  n.setCallbackQueue(&topicQueue);

  ros::Subscriber MotoPositionSub = n.subscribe(
      "MediumSize/BodyHub/MotoPosition", 1000, MotoPositionCallback);
  ros::Subscriber HeadPositionSub = n.subscribe(
      "MediumSize/BodyHub/HeadPosition", 1000, HeadPositionCallback);

  ros::Subscriber jointControlSub = n.subscribe(
      "MediumSize/BodyHub/jointControl", 1000, jointControlCallback);

  while (n.ok()) {
    topicQueue.callAvailable(ros::WallDuration(0.01));
  }
  return NULL;
}

bool initDxlHandlers(void) {
  bool result = false;
  const char *log = NULL;
  pthread_mutex_lock(&mtxOpDxl);
  result = dxl_wb.addSyncWriteHandler(dxlIds[0], "Goal_Position", &log);
  if (result == false) {
    ROS_ERROR("%s", log);
    pthread_mutex_unlock(&mtxOpDxl);
    return result;
  }

  result = dxl_wb.addSyncWriteHandler(dxlIds[0], "Moving_Speed",
                                      &log);  //第一个舵机号要为存在舵机
  if (result == false) {
    ROS_ERROR("%s", log);
    pthread_mutex_unlock(&mtxOpDxl);
    return result;
  }

  result = dxl_wb.initBulkRead(&log);
  if (result == false) {
    ROS_ERROR("%s", log);
    pthread_mutex_unlock(&mtxOpDxl);
    return result;
  }

  result = dxl_wb.initBulkWrite(&log);
  if (result == false) {
    ROS_ERROR("%s", log);
    pthread_mutex_unlock(&mtxOpDxl);
    return result;
  }

  pthread_mutex_unlock(&mtxOpDxl);
  return result;
}

void ReadJointValue() {
  uint8_t failed_cnt = 0;
  const char *log = NULL;
  bool result = false;
  int32_t readGetData = 0;
  uint16_t model_number = 0;

  // usleep(1000000);
  // usleep(1000000);
  pthread_mutex_lock(&mtxOpDxl);
  


  for (uint8_t idNum = 0; idNum < SERVO_NUM; idNum++) {
    // pthread_mutex_lock(&mtxOpDxl);
    for (uint8_t i = 0; i < 3; i++) {
      result = dxl_wb.ping(idNum + 1, &model_number, &log);
      if (result == false)
      {
        printf("%s\n", log);
        ROS_WARN("Failed to ping\n");
      }
      else
      {
        // ROS_WARN("Succeeded to ping\n");
        // printf("id : %d, model_number : %d\n", idNum + 1, model_number);
      }
      // usleep(500000);
      result = dxl_wb.readRegister(idNum + 1, "Present_Position", &readGetData, &log);
      // usleep(500000);
      usleep(10000);
      if (result == true) break;
    }
    // pthread_mutex_unlock(&mtxOpDxl);
    if (result == false) {
      ROS_ERROR("failed to read servo, ID is %d, %s", idNum + 1, log);
      failed_ID[failed_cnt] = idNum + 1;
      failed_cnt++;
    } else {
      ServoStore.value[idNum] = readGetData - servoOffset[idNum];
      ServoStore.angle[idNum] = convertValueTOAngle(idNum + 1, ServoStore.value[idNum]);
      // ROS_WARN("success to read servo, ID is %d, %s", idNum + 1, log);
    }
  }

  pthread_mutex_unlock(&mtxOpDxl);
  failed_cnt = 0;
}

void UpdateState(uint8_t stateNew) {
  bodyhub::ServoPositionAngle servoPositionsMsg;
  const char *log = NULL;
  bool result = false;
  pthread_mutex_lock(&mtxBs);
  switch (stateNew) {
  case StateEnum::init:
    stateNewStr = "init";
    break;
  case StateEnum::preReady:
    stateNewStr = "preReady";
    break;
  case StateEnum::ready:
    stateNewStr = "ready";
    break;
  case StateEnum::running:
    stateNewStr = "running";
    break;
  case StateEnum::pause:
    stateNewStr = "pause";
    break;
  case StateEnum::stoping:
    stateNewStr = "stoping";
    break;
  case StateEnum::error:
    stateNewStr = "error";
    break;
  case StateEnum::directOperate:
    stateNewStr = "directOperate";
    break;
  case StateEnum::walking:
    stateNewStr = "walking";
    break;
  case StateEnum::external_control:
    stateNewStr = "external_control";
    break;
  default:
    break;
  }

  // ROS_INFO("Try UpdateState %s-%d ", stateNewStr.c_str(), stateNew);
 
  if ((stateNew == StateEnum::preReady) || (stateNew == StateEnum::stoping) ) {
    if (!SimControll::simEnable) {
       //reset the standPos
      standPos[HEAD_SERVO_ID ]=0;
      standPos[HEAD_SERVO_ID + 1]=0;
      
      mWalk.quitWalk();
      ReadJointValue();
      servoPositionsMsg.angle = ServoStore.angle;
      ServoPositionPub.publish(servoPositionsMsg);
      if (bodyhubState == StateEnum::directOperate) {
        // reload offset
        if (offsetFile != "") LoadOffset(offsetFile);
#ifdef DEBUG
        for (uint8_t idNum = 0; idNum < SERVO_NUM; idNum++)
          ROS_INFO("directOperate exit Servo Present_Position ID%d: %f", idNum,
                   ServoStore.value[idNum]);
#endif
      }
      pthread_mutex_lock(&mtxOpDxl);
      result = dxl_wb.writeOnlyRegister(254, "P_gain", 6, &log);  // init
      usleep(100);//writeOnlyRegister() 100us
      result = dxl_wb.writeOnlyRegister(254, "Min_Voltage_Limit", 90, &log);
      usleep(100);//writeOnlyRegister() 100us
      for (int id =1;id <= 12; id++)
      {
        result = dxl_wb.writeOnlyRegister(id, "Temperature_Limit", 85, &log);
        usleep(100);//writeOnlyRegister() 100us
        result = dxl_wb.writeOnlyRegister(id, "Max_Torque", 300, &log);
        usleep(100);//writeOnlyRegister() 100us
      }
      pthread_mutex_unlock(&mtxOpDxl);
    } else {//simEnable
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();
      for (uint8_t i = 0; i < jointPosition.data.size(); i++)
        servoPositionsMsg.angle.push_back(Radian2Angle(jointPosition.data[i]));
      ServoPositionPub.publish(servoPositionsMsg);
    }
    if (!noRobotStand) RobotStand(dxlIds, numberOfId, 1000);
    ClearTimerQueue();
  } else if (stateNew == StateEnum::walking) {
    if (!SimControll::simEnable) {
      // P P P P P P_gain
      int16_t P_gainList[22] = {
          20, 35, 30, 30, 30, 35, 20, 35, 30, 30, 30, 35,
      };
      pthread_mutex_lock(&mtxOpDxl);
      for (uint16_t i = 0; i < 12; i++) {
        result = dxl_wb.writeOnlyRegister(i + 1, "P_gain", P_gainList[i], &log);
        usleep(100);//writeOnlyRegister() 100us
        if (result == false) {
          printf("Failed to P_gain: %d, %s\n", i + 1, log);
        }
      }
      // hand
      result = dxl_wb.writeOnlyRegister(13, "P_gain", 8, &log);
      usleep(100);//writeOnlyRegister() 100us
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }
      result = dxl_wb.writeOnlyRegister(16, "P_gain", 8, &log);
      pthread_mutex_unlock(&mtxOpDxl);
      usleep(100);//writeOnlyRegister() 100us
      if (result == false) {
        printf("%s\n", log);
        printf("Failed to P_gain \n");
      }
    }
    
    // RobotSquat
    Eigen::VectorXd currentJointV;
    currentJointV.resize(SERVO_NUM);
    std::vector<Eigen::VectorXd> squatSequence;
    if (SimControll::simEnable) {
      std_msgs::Float64MultiArray jointPosition =
          SimControll::SimRobotData.getJointPostion();

      for (uint8_t i = 0; i < jointPosition.data.size(); i++) {
        currentJointV[i] =
            WalkJointDirection[i] * Radian2Angle(jointPosition.data.at(i));
      }
    } else {// !simEnable
      ReadJointValue();
      for (uint16_t i = 0; i < SERVO_NUM; i++) {
        currentJointV[i] = ServoStore.angle[i] * WalkJointDirection[i];
      }
    }
    ros::Rate loopRate(1.0 / mWalk.timeStep);
    squatSequence = mWalk.initSquat(currentJointV);
    if(!isJointError())
    {
      mWalk.squatCount = 0;
      for (uint8_t cycleCount = 0; cycleCount < squatSequence.size();
          cycleCount++) {
        mWalk.squatCount++;
        mWalk.jointValue.segment(0, 12) =
            squatSequence[cycleCount];  // FIXME: (0,22)
        WalkingSendData();
        loopRate.sleep();
      }
    }
  } else if (stateNew == StateEnum::ready) {
    if (SimControll::simEnable && SimControll::simState != 1)
      SimControll::simStart();
    else
      servoParamInit(dxlIds, numberOfId);
  } else if (stateNew == StateEnum::error) {
    if (SimControll::simEnable) SimControll::simPause();
  } else if (stateNew == StateEnum::external_control) {
    ROS_INFO("state num is external_control!!!");
  }

  bodyhubState = stateNew;
  budyhubStateMsg.data = stateNew;
  StatusPub.publish(budyhubStateMsg);
  pthread_mutex_unlock(&mtxBs);

  ROS_INFO("The new bodyhubState: %s--%d ", stateNewStr.c_str(), stateNew);
}

void ClearTimerQueue() {
  pthread_mutex_lock(&mtxMo);
  ClearQueue(motoQueue);
  ROS_INFO("motoQueue is empty = %d", motoQueue.empty());
  pthread_mutex_unlock(&mtxMo);

  pthread_mutex_lock(&mtxHe);
  ClearQueue(headCtrlQueue);
  ROS_INFO("headCtrlQueue is empty = %d", headCtrlQueue.empty());
  pthread_mutex_unlock(&mtxHe);

  if (!jointTrajQueue.empty()) {
    std::queue<sensor_msgs::JointState> emptyQ;
    mtxJointTrajQueue.lock();
    jointTrajQueue.swap(emptyQ);
    ROS_INFO("jointTrajQueue is empty = %d", jointTrajQueue.empty());
    mtxJointTrajQueue.unlock();
  }
}

bool dxlReadImu(double_t *gyro, double_t *acc) {
  bool result = false;
  const char *log = NULL;
  uint32_t getData[BASE_BOARD_LEN];

  pthread_mutex_lock(&mtxOpDxl);
  result = dxl_wb.readRegister(BASE_BOARD_ID, BASE_BOARD_ADDR, BASE_BOARD_LEN,
                               getData, &log);
  pthread_mutex_unlock(&mtxOpDxl);
  if (result == false) {
    // ROS_ERROR("read imu error, %s", log);
    return false;
  }

  // uint8_t startAddr = BASE_BOARD_ADDR;
  // gyro[0] = GYRO_COEFFICIENT *
  //           TO_INT16(getData[38 - startAddr], getData[39 - startAddr]);
  // gyro[1] = GYRO_COEFFICIENT *
  //           TO_INT16(getData[40 - startAddr], getData[41 - startAddr]);
  // gyro[2] = GYRO_COEFFICIENT *
  //           TO_INT16(getData[42 - startAddr], getData[43 - startAddr]);
  // acc[0] = ACC_COEFFICIENT *
  //          TO_INT16(getData[44 - startAddr], getData[45 - startAddr]);
  // acc[1] = ACC_COEFFICIENT *
  //          TO_INT16(getData[46 - startAddr], getData[47 - startAddr]);
  // acc[2] = ACC_COEFFICIENT *
  //          TO_INT16(getData[48 - startAddr], getData[49 - startAddr]);

  return false;
}

void imuCheck() {
  uint16_t count = 0;
  double gyro[3], acc[3];
  ros::Rate loopRate(100);

  imuType = 1;
  count = 10;
  while (ros::ok() && count) {
    count--;
    if (dxlReadImu(gyro, acc) == true) {
      if (((gyro[0] != 0.0) && (gyro[1] != 0.0) && (gyro[2] != 0.0)) &&
          ((gyro[0] != 0.0) && (gyro[1] != 0.0) && (gyro[2] != 0.0))) {
        imuType = 0;
      }
    }
    loopRate.sleep();
  }
}

void imuInit() {
  uint16_t number = 20;
  uint16_t count = 0;
  double gyro[3], acc[3];
  double gyroSum[3], accSum[3];
  ros::Rate loopRate(100);

  imuCheck();
  if (imuType == 0) {
    ROS_INFO("calibrate imu...");
    while (ros::ok() && count) {
      if (dxlReadImu(gyro, acc) == true) {
        count--;
        for (uint16_t i = 0; i < 3; i++) {
          gyroSum[i] += gyro[i];
          accSum[i] += acc[i];
        }
      }
      loopRate.sleep();
    }
    for (uint16_t i = 0; i < 3; i++) {
      gyro[i] = gyroSum[i] / number;
      acc[i] = accSum[i] / number;
    }
    torsoImu.calibrateGyro(gyro[1], gyro[0], -gyro[2]);
#if 1
    std::cout << "imuOffset gx:" << gyro[1] << std::setw(10) << "gy:" << gyro[0]
              << std::setw(10) << "gz:" << -gyro[2] << std::setw(10) << "\n";
#endif
    ROS_INFO("calibrate imu over.");

    torsoImu.setGyroLpf(100, 20);
    torsoImu.setAccLpf(100, 20);

    count = 100;
    while (ros::ok() && count) {
      if (dxlReadImu(gyro, acc) == true) {
        count--;
        torsoImu.setGyro(gyro[1], gyro[0], -gyro[2]);
        torsoImu.setAcc(acc[1], acc[0], -acc[2]);
        torsoImu.datafusion();
      }
      loopRate.sleep();
    }
  } else {
    extern void jy901ModuleInit(void);
    jy901ModuleInit();
  }
}


void servoParamInit(uint8_t *ids, uint8_t number) {
  bool result = false;
  const char *log = NULL;
  const char **log_LOG = NULL;
  int32_t pGain[22] = {30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30,
                       30, 15, 50, 50, 15, 50, 50, 50, 50, 80, 80};
  int32_t maxTorque[22] = {0,    0,    0,    0,    0, 0,    0,    0,
                           0,    0,    0,    0,    0, 1023, 1023, 1023,
                           1023, 1023, 1023, 1023, 0, 0};

  std::cout << "set P_gain\n";
  pthread_mutex_lock(&mtxOpDxl);
  result = dxl_wb.setPacketHandler(1.0f, log_LOG);
  for (uint8_t i = 0; i < number; i++) {
    result =
        dxl_wb.writeOnlyRegister(ids[i], "P_gain", pGain[ids[i] - 1], &log);
    usleep(100);//writeOnlyRegister() 100us
    if (result == false) {
      ROS_WARN("set p_gain %d %d failed, %s", ids[i],i, log);
    }
  }
  std::cout << std::endl;

  std::cout << "set Max_Torque miniSERVO\n";
  for (uint8_t i = 0; i < number; i++) {
    if (maxTorque[ids[i] - 1] == 0) continue;
    result = dxl_wb.writeOnlyRegister(ids[i], "Max_Torque",
                                      maxTorque[ids[i] - 1], &log);
    usleep(80000);//writeOnlyRegister() 80ms
    if (result == false) {
      ROS_WARN("set Max_Torque %d failed, %s", ids[i], log);
    }
  }
  std::cout << std::endl;

  std::cout << "set torque\n";
  for (uint8_t i = 0; i < number; i++) {
    result = dxl_wb.writeOnlyRegister(ids[i], "Torque_Enable", 1, &log);
    usleep(100);//writeOnlyRegister() 100us
    if (result == false) {
      ROS_WARN("torqueOn %d failed, %s", ids[i], log);
    }
  }
  pthread_mutex_unlock(&mtxOpDxl);
  std::cout << std::endl;
}


static uint8_t portInit(const char *portName, uint32_t baudrate) {
  const char *log = NULL;
  bool result = false;
  uint16_t model_number = 0;
  uint8_t dxl_id[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48};

  pthread_mutex_lock(&mtxOpDxl);
  result = dxl_wb.init(portName, baudrate, &log);  // initWorkbench
  if (result == false) {
    pthread_mutex_unlock(&mtxOpDxl);
    return 1;
  }


  for (uint8_t s_num = 0; s_num <= 2; s_num++) {
    dxl_wb.scan(dxlIds, &numberOfId, DXL_ID_SCAN_RANGE, &log);
    if (numberOfId == SERVO_NUM) {
      break;
    }
    usleep(1000 * 20);
  }

  
  pthread_mutex_unlock(&mtxOpDxl);
  if (numberOfId == 0) {
    return 2;
  }

  return 0;
}

bool servoInit() {
  const char *log = NULL;
  bool result = false;
  uint8_t ret = 0;
  const char *portName = "/dev/usb_servo";
  uint32_t baudrate[] = {1000000, 2000000};
  for (uint16_t i = 0; i < 2; i++) {
    std::cout << "try baudrate: " << baudrate[i] << "\n";
    ret = portInit(portName, baudrate[i]);
    if (ret == 0) break;
  }
  if (ret != 0) {
    switch (ret) {
      case 1:
        ROS_ERROR("open port failed!");
        break;
      case 2:
        ROS_ERROR("no dynamixel device found!");
        break;
    }
    return false;
  }


  for (uint8_t i = 0; i < numberOfId; i++) {
    std::cout << (int)dxlIds[i] << "  ";
  }
  std::cout << std::endl;

  result = initDxlHandlers();
  if (result == false) {
    ROS_ERROR("initDxlHandlers failed! 2474 ");
    return false;
  }
  servoParamInit(dxlIds, numberOfId);
  return true;
}

void sigintHandler(int sig) {
#if 0  //
  if(!SimControll::simEnable)
  {
    uint8_t ids[12] = {1,2,3,4,5,6,7,8,9,10,11,12};
    double_t position[12] = {0, 0, 65, -105, -42, 0, 0, 0, -65, 105, 42, 0};
    motionToPosture(1.5, 12, ids, position);
  }
#endif

  if (SimControll::simEnable) SimControll::simStop();
  ros::shutdown();
}

void LoadAssembleParam(ros::NodeHandle nh) {
  nh.param<std::string>("poseOffsetPath", offsetFile, "");
  if (access(offsetFile.c_str(), F_OK) != 0) {
    nh.param<std::string>("packOffsetFile", offsetFile, "");
  }
}

bool imuStateCallback(bodyhub::SrvimuState::Request &req,
                      bodyhub::SrvimuState::Response &res) {
  ROS_INFO("Request from imuState : %s",req.state.c_str());
  if (req.state == "on") {
    setRestrainWobbleOnoff(true);
    res.freeback = "[server]Successful!Now imuState is on";
  } else if (req.state == "off") {
    setRestrainWobbleOnoff(false);
    res.freeback = "[server]Successful!Now imuState is off";
  } else {
    res.freeback = "[server]Failed!Please input on or off";
  }  

  return true;
}

bool ScanServoSrvCallback(bodyhub::SrvServoScan::Request &req,
                          bodyhub::SrvServoScan::Response &res) {
  const char *log = NULL;
  uint8_t temp_ids[22];
  uint8_t temp_numberOfId = 0;

  if (req.scanmode == "all"){
    pthread_mutex_lock(&mtxOpDxl);

    //ScanServo
    dxl_wb.scan(dxlIds, &numberOfId, DXL_ID_SCAN_RANGE, &log);
    for (uint8_t i = 0; i < numberOfId; i++) {
      res.getData.push_back(dxlIds[i]);
    }

    //Scan fsr
    dxl_wb.scan(temp_ids, &temp_numberOfId, RIGHTFSR, LEFTFSR, &log);
    for (uint8_t i = 0; i < temp_numberOfId; i++) {
      res.getData.push_back(temp_ids[i]);
    }

    //Scan baseboard
    dxl_wb.scan(temp_ids, &temp_numberOfId, BASEBOARD, BASEBOARD, &log);
    for (uint8_t i = 0; i < temp_numberOfId; i++) {
      res.getData.push_back(temp_ids[i]);
    }

    //ScanServo again
    dxl_wb.scan(dxlIds, &numberOfId, DXL_ID_SCAN_RANGE, &log);
    pthread_mutex_unlock(&mtxOpDxl);
    return true;
  } else if (req.scanmode == "torso") { //range of servos id to scan: 1~22 & 111~112 & 200
    pthread_mutex_lock(&mtxOpDxl);
    dxl_wb.scan(temp_ids, &temp_numberOfId, DXL_ID_SCAN_RANGE, &log);
  } else if (req.scanmode == "fsr") {
    pthread_mutex_lock(&mtxOpDxl);
    dxl_wb.scan(temp_ids, &temp_numberOfId, RIGHTFSR, LEFTFSR, &log);
  } else if (req.scanmode == "baseboard") {
    pthread_mutex_lock(&mtxOpDxl);
    dxl_wb.scan(temp_ids, &temp_numberOfId, BASEBOARD, BASEBOARD, &log);
  } else {
    printf("plase input legal scanmode!");
    return false;
  }

  //ScanServo again
  dxl_wb.scan(dxlIds, &numberOfId, DXL_ID_SCAN_RANGE, &log);
  pthread_mutex_unlock(&mtxOpDxl);
  
  for (uint8_t i = 0; i < temp_numberOfId; i++) {
    res.getData.push_back(temp_ids[i]);
  }
  return true;
}

bool PresentTemperatureSrvCallback(bodyhub::SrvPresentTemperature::Request &req,
                                  bodyhub::SrvPresentTemperature::Response &res) {
  bool result = false;
  int32_t servoTemperature[SERVO_NUM] = {0};
  result = GetServoPresentTemperature(servoTemperature);
  if (!result)
    return false;
  for (uint8_t i=0; i<SERVO_NUM; i++) {
    res.presentTemperature.push_back(servoTemperature[i]);
  }
  return true;
}

bool FSRSrvCallback(bodyhub::SrvFSR::Request &req,
                    bodyhub::SrvFSR::Response &res) {
  bool result = false;
  int32_t fsrData[FSR_ADDR_LEN * 2] = {0};
  result = GetFsrValue(fsrData);
  if (!result)
    return false;
  for (uint8_t i=0; i<FSR_ADDR_LEN; i++) {
    res.leftFSR.push_back(fsrData[i + FSR_ADDR_LEN]);
    res.rightFSR.push_back(fsrData[i]);
  }
  return true;
}

void controllerParamInit() {
  double_t wobble_k[] = {0.035, 0.03};
  setRestrainWobbleParam(wobble_k);
  setRestrainWobbleOnoff(true);
  double_t balanceRoll_k[] = {1.2, 0.04, 0.1, 0.04};
  double_t balancePitch_k[] = {1.1, 0.02, 0.1, 0.04};
  setSlopeBalanceRParam(balanceRoll_k);
  setSlopeBalancePParam(balancePitch_k);
  setSlopeBalanceOnoff(true);
}

void STATEinit(ros::NodeHandle nh) {
  

  StatusPub = nh.advertise<std_msgs::UInt16>("MediumSize/BodyHub/Status", 0);
  ServoPositionPub = nh.advertise<bodyhub::ServoPositionAngle>(
      "MediumSize/BodyHub/ServoPositions", 0, true);
  imuStateService =
      nh.advertiseService("imuState", imuStateCallback);
  StateService =
      nh.advertiseService("MediumSize/BodyHub/StateJump", StateSrvCallback);
  GetStatusService =
      nh.advertiseService("MediumSize/BodyHub/GetStatus", GetStatusCallback);
  MasterIDService = nh.advertiseService("MediumSize/BodyHub/GetMasterID",
                                        MasterIDSrvCallback);
  GetJointAngleService = nh.advertiseService("MediumSize/BodyHub/GetJointAngle",
                                             GetJointAngleCallback);
  ScanServoService =
      nh.advertiseService("MediumSize/BodyHub/ScanServo", ScanServoSrvCallback);
  PresentTemperatureService = nh.advertiseService("MediumSize/BodyHub/PresentTemperature",
                                                  PresentTemperatureSrvCallback);
  FSRService = nh.advertiseService("MediumSize/BodyHub/FSR", FSRSrvCallback);

  // VALUE
  InstReadValService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstReadVal", InstReadValSrvCallback);
  InstWriteValService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstWriteVal", InstWriteValSrvCallback);
  SyncWriteValService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SyncWriteVal", SyncWriteValSrvCallback);

  SetTarPositionValService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoTarPositionVal",
      SetServoTarPositionValCallback);
  SetTarPositionValAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoTarPositionValAll",
      SetServoTarPositionValAllCallback);
  GetPositionValAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/GetServoPositionValAll",
      GetServoPositionValAllCallback);

  // ANGLE
  InstReadService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstRead", InstReadSrvCallback);
  InstWriteService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/InstWrite", InstWriteSrvCallback);
  SyncWriteService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SyncWrite", SyncWriteSrvCallback);

  SetLockStateService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/SetServoLockState",
                          SetServoLockStateCallback);
  SetLockStateAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoLockStateAll",
      SetServoLockStateAllCallback);
  GetLockStateAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/GetServoLockStateAll",
      GetServoLockStateAllCallback);

  SetTarPositionService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/SetServoTarPosition",
                          SetServoTarPositionCallback);
  SetTarPositionAllService = nh.advertiseService(
      "MediumSize/BodyHub/DirectMethod/SetServoTarPositionAll",
      SetServoTarPositionAllCallback);
  GetPositionAllService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/GetServoPositionAll",
                          GetServoPositionAllCallback);
  GetRawBulkReadDataService =
      nh.advertiseService("MediumSize/BodyHub/DirectMethod/GetRawBulkReadData",
                          GetRawBulkReadDataCallback);

  /***walking***walking***walking***/
  jointPosTargetPub =
      nh.advertise<std_msgs::Float64MultiArray>("joint/angle/target", 1000);

  jointPosMeasurePub =
      nh.advertise<std_msgs::Float64MultiArray>("joint/angle/measure", 1000);

  jointVelTargetPub =
      nh.advertise<std_msgs::Float64MultiArray>("joint/velocity/target", 1000);
  jointVelMeasurePub =
      nh.advertise<std_msgs::Float64MultiArray>("joint/velocity/measure", 1000);

  contactState_pub = nh.advertise<std_msgs::Float64>("/contactState", 1000);
  stepPhase_pub = nh.advertise<std_msgs::Float64>("/stepPhase", 1000);

  cpref_pub = nh.advertise<std_msgs::Float64>("cpref", 1000);
  cpC_pub = nh.advertise<std_msgs::Float64>("cpC", 1000);

  copm_pub = nh.advertise<std_msgs::Float64>("copm", 1000);
  copD_pub = nh.advertise<std_msgs::Float64>("copD", 1000);
  copref_pub = nh.advertise<std_msgs::Float64>("copref", 1000);

  comRefe_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/com/reference", 1000);
  comMea_pub = nh.advertise<std_msgs::Float64MultiArray>("/com/measure", 1000);
  comEsti_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/com/estimate", 1000);

  comVRefe_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/comV/reference", 1000);
  comVMea_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/comV/measure", 1000);
  comVEsti_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/comV/estimate", 1000);

  leftFootRefe_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/leftFootPR/reference", 1000);
  leftFootMea_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/leftFootPR/measure", 1000);

  rightFootRefe_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/rightFootPR/reference", 1000);
  rightFootMea_pub =
      nh.advertise<std_msgs::Float64MultiArray>("/rightFootPR/measure", 1000);

  Torso_Rpub = nh.advertise<std_msgs::Float64>("Torso_R", 1000);
  Torso_Ppub = nh.advertise<std_msgs::Float64>("Torso_P", 1000);

  imuDxl_pub = nh.advertise<std_msgs::Float64MultiArray>("/imu/dxl", 1000);
  imuTorso_pub = nh.advertise<std_msgs::Float64MultiArray>("/imu/Torso", 1000);


  WalkingStatusPub = nh.advertise<std_msgs::Float64>(
      "/MediumSize/BodyHub/WalkingStatus", 1000);
  /***walking***walking***walking***/

  ik_state_pub = nh.advertise<std_msgs::Int8>("/IK/state", 1000);

  UpdateState(StateEnum::init);



  // load offset (value)
  if (offsetFile != "") LoadOffset(offsetFile);
  // load dxlinitpose (angle)
  if (InitPoseFile != "") LoadDxlInitPose(InitPoseFile);

  pthread_mutex_init(&mtxWl, NULL);
  pthread_mutex_init(&mtxMo, NULL);
  pthread_mutex_init(&mtxHe, NULL);
  pthread_mutex_init(&mtxSL, NULL);
  pthread_mutex_init(&mtxOpDxl, NULL);
  pthread_mutex_init(&mtxBs, NULL);

  ServoStore.angle.resize(SERVO_NUM);
  ServoStore.value.resize(SERVO_NUM);
  for (uint8_t i = 0; i < SERVO_NUM; i++) {
    ServoStore.angle[i] = SERVO_DEFAULT_ANGLE;
    ServoStore.value[i] = SERVO_DEFAULT_VALUE;
  }

  if (SimControll::simEnable)  // Vrep Simulation
  {
    SimControll::simInit(nh);
  } else {
    if (servoInit() == false) {
#if 1
      ROS_ERROR("servoInit failed, press enter to exit!");
      getchar();
      exit(0);
#else
      SimControll::simEnable = true;
      SimControll::simInit(nh);
#endif
    }
    imuInit();
    controllerParamInit();
  }
}

void STATEpreReady() { const char *log = NULL; }

void STATEready() { const char *log = NULL; }
void STATErunning() {
  // check empty & Moving
  if ((motoQueue.empty()) && (headCtrlQueue.empty()) && jointTrajQueue.empty())
    UpdateState(StateEnum::pause);  //更新下一个状态
}
void STATEpause() { const char *log = NULL; }
void STATEstoping() {}
void STATEerror() { const char *log = NULL; }

void STATEdirectOperate() { const char *log = NULL; }

void STATEwalking() { 
  const char *log = NULL;
  // if ((motoQueue.empty()) && (headCtrlQueue.empty()) && jointTrajQueue.empty())
  //   UpdateState(StateEnum::pause);  //更新下一个状态
  
}

void IKFailedCallback(const GaitManager::LIPMWalk::IKContext_t ik_c)
{
  // 发布ik状态
  // 0:正常
  // 1:左腿IK失败
  // 2:右腿IK失败
  std_msgs::Int8 msg;
  msg.data = ik_c.i;
  ik_state_pub.publish(msg);
}

bool GetServoPresentTemperature(int32_t *bulkReadData) {
    uint16_t idcount = SERVO_NUM;
    uint8_t idLists[idcount]={1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};
    uint16_t bulkReadAddress[idcount];
    std::fill_n(bulkReadAddress, idcount, Servo_Temperature_Register_Address);
    uint16_t bulkReadLength[idcount];
    std::fill_n(bulkReadLength, idcount, OneByte);
    bool result = false;
    result = dxlBulkRead(idLists, idcount, bulkReadAddress, bulkReadLength, bulkReadData);
    return result;
}

bool GetFsrValue(int32_t *bulkReadData) {
  uint8_t idCount = NumberOfFsr;
  uint8_t idLists[idCount];
  uint16_t bulkReadAddress[idCount];
  uint16_t bulkReadLength[idCount];
  bool result = false;
 
  uint16_t idx = 0;
  idLists[idx] = LEFT_FSR_ID;
  bulkReadAddress[idx] = FSR_ADDR;
  bulkReadLength[idx] = FSR_ADDR_LEN;

  idx++;
  idLists[idx] = RIGHT_FSR_ID;
  bulkReadAddress[idx] = FSR_ADDR;
  bulkReadLength[idx] = FSR_ADDR_LEN;

  result = dxlBulkRead(idLists, idx+1, bulkReadAddress, bulkReadLength,
                      bulkReadData);
  return result;
}

int main(int argc, char **argv) {
  //初始化节点
  ros::init(argc, argv, "bodyhub_node");
  ros::NodeHandle nodeHandle;
  signal(SIGINT, sigintHandler);

  /* Load ROS Parameter */
  LoadAssembleParam(nodeHandle);
  nodeHandle.getParam("setnostand", noRobotStand);
  nodeHandle.getParam("simenable", SimControll::simEnable);
  nodeHandle.param<std::string>("poseInitPath", InitPoseFile, "");
  nodeHandle.param<std::string>("sensorNameIDPath", sensorNameIDFile, "");

  STATEinit(nodeHandle);
  extern void actionManageInit();
  actionManageInit();

  if (SimControll::simEnable)
    mWalk.setRunMode(1);
  else
    mWalk.setRunMode(0);
  cpWalk = GaitManager::CPWalking5::GetInstance();
  cpWalk->commandRosInit();

  mWalk.setIKFailedCallback(IKFailedCallback);

  int32_t ret;
  pthread_attr_t attr;
  struct sched_param param;
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO); // 调度策略设置为 SCHED_FIFO（先进先出）
  pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM); // 将争用范围设置为 PTHREAD_SCOPE_SYSTEM

  param.sched_priority = 99;
  pthread_attr_setschedparam(&attr, &param);
  pthread_t threadTimer;
  ret = pthread_create(&threadTimer, &attr, timerThread, NULL);
  if (ret != 0) {
    ROS_ERROR("create timerThread failed, err code %d", ret);
    ROS_ERROR("press enter to exit!");
    getchar();
    exit(1);
  }

  param.sched_priority = 98;
  pthread_attr_setschedparam(&attr, &param);
  pthread_t threadQueue;
  ret = pthread_create(&threadQueue, &attr, queueThread, NULL);
  if (ret != 0) {
    ROS_ERROR("create queueThread failed, err code %d", ret);
    ROS_ERROR("press enter to exit!");
    getchar();
    exit(1);
  }

  pthread_attr_destroy(&attr);

  std::thread threadSensor(sensorThread);
  std::thread threadSim(SimControll::simThread);

  if (SimControll::simEnable)
    while (ros::ok() && !SimControll::jointState()) usleep(100 * 1000);
  UpdateState(StateEnum::preReady);

  ros::Rate loop_rate(100);
  while (ros::ok()) {
    if (bodyhubState == StateEnum::preReady)
      STATEpreReady();
    else if (bodyhubState == StateEnum::ready)
      STATEready();
    else if (bodyhubState == StateEnum::running)
      STATErunning();
    else if (bodyhubState == StateEnum::pause)
      STATEpause();
    else if (bodyhubState == StateEnum::stoping)
      STATEstoping();
    else if (bodyhubState == StateEnum::error)
      STATEerror();
    else if (bodyhubState == StateEnum::directOperate)
      STATEdirectOperate();
    else if (bodyhubState == StateEnum::walking)
      STATEwalking();

    ros::spinOnce();
    loop_rate.sleep();
  }

  pthread_join(threadTimer, NULL);
  pthread_join(threadQueue, NULL);
  threadSensor.join();
  threadSim.join();

  return 0;
}

