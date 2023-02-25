/**
  *top.h
  *brief:main of node
  *author:Chen Xiaofeng
  *date:20191028
  **/

#include "top.h"

// USB Key，只在生成发布版的时候才解注释
// #include "../../../common/Pwd_8/SoftkeyPWD.h"

using namespace std;

int main(int argc, char **argv)
{
    // 只在生成发布版的时候才解注释，否则在自己电脑上无法调试
    // if(!checkUSBKey()) return 0;

    ros::init(argc, argv, "localization_filter");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    Top top(nh,nh_priv);
    ros::spin();
    return 0;
}