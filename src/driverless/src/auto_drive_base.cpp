#include "driverless/auto_drive_base.h"

Path AutoDriveBase::global_path_ ;            //全局路径
Path AutoDriveBase::local_path_;              //局部路径
VehicleState AutoDriveBase::vehicle_state_;   //汽车状态
VehicleParams AutoDriveBase::vehicle_params_; //汽车参数
