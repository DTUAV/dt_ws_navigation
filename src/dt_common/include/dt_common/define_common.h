#ifndef DEFINE_COMMON_H
#define DEFINE_COMMON_H
#include "x2struct/x2struct.hpp"
using namespace std;
namespace DTUAV {
/*
 * ==========================> Define Object ID <=======================================
*/
//-------------------------->The Object Id for Physical Uav (1-->100)<--------------------------------
#define Gloal_Server_0  0
#define R_UAV_1  1
#define R_UAV_2  2
#define R_UAV_3  3
#define R_UAV_4  4
#define R_UAV_5  5
#define R_UAV_6  6
#define R_UAV_7  7
#define R_UAV_8  8
#define R_UAV_9  9
#define R_UAV_10  10

#define R_CAR_1 11
#define R_CAR_2 12
#define R_CAR_3 13

//-------------------------->Transfer Packet Format<-----------------------------------------
struct IotMessage
{
  int TargetID;
  int SourceID;
  int MessageID;
  string MessageData;
  double TimeStamp;
  XTOSTRUCT(O(TargetID,SourceID,MessageID,MessageData,TimeStamp));
};

//=======================================================================================================================
//=============================> Define Message Format V2 <===============================
//=======================================================================================================================
#define CurrentLocalPositionMsgID 1
struct CurrentLocalPositionMsg
{
  double position_x;
  double position_y;
  double position_z;
  double rotation_x;
  double rotation_y;
  double rotation_z;
  double rotation_w;
  XTOSTRUCT(O(position_x,position_y,position_z,rotation_w,rotation_x,rotation_y,rotation_z));
};

#define CurrentGlobalPositionMsgID 2
struct CurrentGlobalPositionMsg
{
  double latitude;
  double longitude;
  double altitude;
  XTOSTRUCT(O(latitude,longitude,altitude));
};

#define CurrentVelocityMsgID 3
struct CurrentVelocityMsg
{
  double linear_velocity_x;
  double linear_velocity_y;
  double linear_velocity_z;
  double anger_velocity_x;
  double anger_velocity_y;
  double anger_velocity_z;
  XTOSTRUCT(O(linear_velocity_x,linear_velocity_y,linear_velocity_z,anger_velocity_x,anger_velocity_y,anger_velocity_z));
};

#define CurrentStateMsgID  4
struct CurrentStateMsg
{
  bool connected;
  bool armed;
  bool guided;
  bool manual_input;
  string mode;
  uint system_status;
  XTOSTRUCT(O(connected,armed,guided,manual_input,mode,system_status));
};

#define TargetLocalPositionMsgID 11
struct TargetLocalPositionMsg
{
  double position_x;
  double position_y;
  double position_z;
  double rotation_x;
  double rotation_y;
  double rotation_z;
  double rotation_w;
  XTOSTRUCT(O(position_x,position_y,position_z,rotation_w,rotation_x,rotation_y,rotation_z));
};

#define TargetGlobalPositionMsgID 12
struct TargetGlobalPositionMsg
{
  double latitude;
  double longitude;
  double altitude;
  XTOSTRUCT(O(latitude,longitude,altitude));
};

#define TargetVelocityMsgID 13
struct TargetVelocityMsg
{
  double linear_velocity_x;
  double linear_velocity_y;
  double linear_velocity_z;
  double anger_velocity_x;
  double anger_velocity_y;
  double anger_velocity_z;
  XTOSTRUCT(O(linear_velocity_x,linear_velocity_y,linear_velocity_z,anger_velocity_x,anger_velocity_y,anger_velocity_z));
};

#define TargetCmdTypeMsgID 14
struct TargetCmdTypeMsg
{
  int type;
  XTOSTRUCT(O(type));
};

#define ControlApplyMsgID 15
struct ControlApplyMsg
{
  bool apply_info;
  XTOSTRUCT(O(apply_info));
};

#define HeartMsgID 605
struct HeartMsg
{
  bool is_get;
  XTOSTRUCT(O(is_get));
};

#define UavStartID 604
struct UavStart
{
  bool is_start;
  XTOSTRUCT(O(is_start));
};


#define UavCommandID 603
struct UavCommand
{
  /*
   * 1:IsOffboard
   * 2:IsArm
   * 4:IsStart
   * 3:IsOffboard+IsArm
   * 5:IsOffboard+IsStart
   * 6:IsArm+IsStart
   * 7:IsOffboard+IsArm+IsStart
  */
  int ComMode;
  bool IsOffboard;//if enter in offboard mode
  bool IsArm;//if arm
  bool IsStart;//if start control by VR
  XTOSTRUCT(O(ComMode,IsOffboard,IsArm,IsStart));
};
#define ComputerControlID 602
struct ComputerControl
{
  bool IsClose;//close the computer
  XTOSTRUCT(O(IsClose));
};
#define UavControlID 601
struct UavControl
{
  int Mode;//0:position,1:velocity
  double ComLX;
  double ComLY;
  double ComLZ;
  double ComAX;
  double ComAY;
  double ComAZ;
  XTOSTRUCT(O(Mode,ComLX,ComLY,ComLZ,ComAX,ComAY,ComAZ));
};
#define UavInfoID 600
struct UavInfo
{
  //The Position, Rotation, Linear Velocity, Angular Velocity
  double PosX;
  double PosY;
  double PosZ;
  double RotX;
  double RotY;
  double RotZ;
  double RotW;
  double LVelX;
  double LVelY;
  double LVelZ;
  double AVelX;
  double AVelY;
  double AVelZ;
  //The Safe Checking
  bool NetPx4;//the connective of the computer and PX4
  //The Status of UAV
  bool IsArm;
  /*
         * 0: the manual mode
         * 1: the stabilizing mode
         * 2: the altitude mode
         * 3: the position mode
         * 4: the offboard mode
         * 5: the return mode
         */
  int FMode;//the flight mode of UAV:
  float Voltage;//the current voltage of UAV
  float Remaining;//the remaining of battery
  bool IsStart;//the state of UAV control
  XTOSTRUCT(O(PosX,PosY,PosZ,RotX,RotY,RotZ,RotW,LVelX,LVelY,LVelZ,AVelX,AVelY,AVelZ,NetPx4,IsArm,FMode,Voltage,Remaining,IsStart));
};

#define ApplyCameraID 604
struct ApplyCameraMsg
{
  bool isOpen;
  XTOSTRUCT(O(isOpen));
};

#define CompressedImageMessageID 109
struct CompressedImageMessage
{
  string format;//jpeg png
  vector<int> data;
  XTOSTRUCT(O(format,data));
};

#define TargetObjectPoseMessageID 606
struct TargetObjPoseMessage
{
  float posX;
  float posY;
  float posZ;
  float rotX;
  float rotY;
  float rotZ;
  float rotW;
  XTOSTRUCT(O(posX,posY,posZ,rotX,rotY,rotZ,rotW));
};

#define AllUavStateMessageID 607
struct AllUavStateMessage
{
  vector<float> xs;
  vector<float> ys;
  vector<float> zs;
  vector<float> vxs;
  vector<float> vys;
  vector<float> vzs;
  XTOSTRUCT(O(xs, ys, zs, vxs, vys, vzs));
};

#define FinishOneMissionMessageID 608
struct FinishOneMissionMessage {
  bool isFinish;
  XTOSTRUCT(O(isFinish));
};

#define FinishAllMissionMessageID 609
struct FinishAllMissionMessage {
  bool isFinish;
  XTOSTRUCT(O(isFinish));
};

#define MissionWaypointsMessageID 610
struct MissionWaypointsMessage {
  vector<float> xs;
  vector<float> ys;
  vector<float> zs;
  XTOSTRUCT(O(xs, ys, zs));
};

#define Lidar2dMessageID 611
struct Lidar2dMessage
{
  std::string data;
  XTOSTRUCT(O(data));
};


}
#endif // DEFINE_COMMON_H
