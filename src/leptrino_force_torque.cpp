/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Imai Laboratory, nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 * Notice: Modified & copied from Leptrino CD example source code
 * Notice: Modified for ROS2 by Vineet
 * Notice: Ver 2.0.0 is modified for ROS2 by Muhammad Labiyb Afakh
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <array>

#include <leptrino/pCommon.h>
#include <leptrino/rs_comm.h>
#include <leptrino/pComResInternal.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#define PRG_VER "Ver 2.0.0"

#define DEBUG 0

using namespace std::chrono_literals;
using std::placeholders::_1;

class LeptrinoNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::TimerBase::SharedPtr timer_acquisition_;
  rclcpp::TimerBase::SharedPtr timer_publisher_;

  void App_Init();
  void App_Close(rclcpp::Logger logger);
  ULONG SendData(UCHAR* pucInput, USHORT usSize);
  void GetProductInfo(rclcpp::Logger logger);
  void GetLimit(rclcpp::Logger logger);
  void SerialStart(rclcpp::Logger logger);
  void SerialStop(rclcpp::Logger logger);

  struct ST_SystemInfo
  {
    int com_ok;
  };

  struct FS_data
  {
    std::array<double, 3> force;
    std::array<double, 3> moment;
  };

  UCHAR CommRcvBuff[256];
  UCHAR CommSendBuff[1024];
  UCHAR SendBuff[512];
  double conversion_factor[FN_Num];

  std::string g_com_port = "/dev/ttyACM1";
  int g_rate = 1000;

  void SensorAquistionCallback();
  void PublisherCallback();

public:
  ST_R_DATA_GET_F* stForce;
  ST_R_DATA_GET_F* grossForce;
  ST_R_GET_INF* stGetInfo;
  ST_R_LEP_GET_LIMIT* stGetLimit;
  ST_SystemInfo gSys;
  FS_data sensor_data;
  FS_data offset;

  LeptrinoNode();
  ~LeptrinoNode();
  void SensorCallback();
  void init(rclcpp::Logger logger);
  void SensorNormalize(rclcpp::Logger logger);
};

LeptrinoNode::LeptrinoNode() : Node("Leptrino")
{
  wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("leptrino_force_torque", 10);

  LeptrinoNode::App_Init();

  if (gSys.com_ok == NG)
  {
    RCLCPP_ERROR(this->get_logger(), "%s open failed\n", g_com_port.c_str());
    exit(0);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Leptrino is connected");
  }

  LeptrinoNode::SerialStart(this->get_logger());
  LeptrinoNode::SensorNormalize(this->get_logger());

  timer_acquisition_ = this->create_wall_timer(1ms, std::bind(&LeptrinoNode::SensorCallback, this));
  timer_publisher_ = this->create_wall_timer(1ms, std::bind(&LeptrinoNode::PublisherCallback, this));
}

LeptrinoNode::~LeptrinoNode()
{
  LeptrinoNode::SerialStop(this->get_logger());
  LeptrinoNode::App_Close(this->get_logger());
  RCLCPP_WARN(this->get_logger(), "Leptrino has been stopped");
}

void LeptrinoNode::init(rclcpp::Logger logger)
{
  LeptrinoNode::SensorNormalize(logger);
}

void LeptrinoNode::PublisherCallback()
{
  if (rclcpp::ok())
  {
    auto wrench_msg = std::make_shared<geometry_msgs::msg::WrenchStamped>();

    wrench_msg->header.stamp = this->now();
    wrench_msg->header.frame_id = "leptrino_sensor";
    wrench_msg->wrench.force.x = sensor_data.force[1];
    wrench_msg->wrench.force.y = sensor_data.force[2];
    wrench_msg->wrench.force.z = sensor_data.force[3];
    wrench_msg->wrench.torque.x = sensor_data.moment[1];
    wrench_msg->wrench.torque.y = sensor_data.moment[2];
    wrench_msg->wrench.torque.z = sensor_data.moment[3];

    wrench_pub_->publish(*wrench_msg);
  }
}

void LeptrinoNode::SensorCallback()
{
  if (rclcpp::ok())
  {
    Comm_Rcv();

    if (Comm_CheckRcv() != 0)
    {
      memset(CommRcvBuff, 0, sizeof(CommRcvBuff));

      auto rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0)
      {
        stForce = (ST_R_DATA_GET_F*)CommRcvBuff;
        if (DEBUG)
        {
          RCLCPP_INFO(this->get_logger(), "%d,%d,%d,%d,%d,%d", stForce->ssForce[0], stForce->ssForce[1],
                      stForce->ssForce[2], stForce->ssForce[3], stForce->ssForce[4], stForce->ssForce[5]);
        }

        sensor_data.force[1] = stForce->ssForce[0] - offset.force[1];
        sensor_data.force[2] = stForce->ssForce[1] - offset.force[2];
        sensor_data.force[3] = stForce->ssForce[2] - offset.force[3];
        sensor_data.moment[1] = stForce->ssForce[3] - offset.moment[1];
        sensor_data.moment[2] = stForce->ssForce[4] - offset.moment[2];
        sensor_data.moment[3] = stForce->ssForce[5] - offset.moment[3];

        // usleep(1000);
      }
    }
  }
}

void LeptrinoNode::App_Init()
{
  int rt;

  // Commポート初期化
  gSys.com_ok = NG;
  rt = Comm_Open(g_com_port.c_str());
  if (rt == OK)
  {
    Comm_Setup(460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
    gSys.com_ok = OK;
  }
}

void LeptrinoNode::App_Close(rclcpp::Logger logger)
{
  RCLCPP_DEBUG(logger, "Application close\n");

  if (gSys.com_ok == OK)
  {
    Comm_Close();
  }
}

ULONG LeptrinoNode::SendData(UCHAR* pucInput, USHORT usSize)
{
  USHORT usCnt;
  UCHAR ucWork;
  UCHAR ucBCC = 0;
  UCHAR* pucWrite = &CommSendBuff[0];
  USHORT usRealSize;

  // データ整形
  *pucWrite = CHR_DLE;  // DLE
  pucWrite++;
  *pucWrite = CHR_STX;  // STX
  pucWrite++;
  usRealSize = 2;

  for (usCnt = 0; usCnt < usSize; usCnt++)
  {
    ucWork = pucInput[usCnt];
    if (ucWork == CHR_DLE)
    {                       // データが0x10ならば0x10を付加
      *pucWrite = CHR_DLE;  // DLE付加
      pucWrite++;           // 書き込み先
      usRealSize++;         // 実サイズ
      // BCCは計算しない!
    }
    *pucWrite = ucWork;  // データ
    ucBCC ^= ucWork;     // BCC
    pucWrite++;          // 書き込み先
    usRealSize++;        // 実サイズ
  }

  *pucWrite = CHR_DLE;  // DLE
  pucWrite++;
  *pucWrite = CHR_ETX;  // ETX
  ucBCC ^= CHR_ETX;     // BCC計算
  pucWrite++;
  *pucWrite = ucBCC;  // BCC付加
  usRealSize += 3;

  Comm_SendData(&CommSendBuff[0], usRealSize);

  return OK;
}

void LeptrinoNode::GetProductInfo(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Get sensor information");
  len = 0x04;                 // データ長
  SendBuff[0] = len;          // レングス
  SendBuff[1] = 0xFF;         // センサNo.
  SendBuff[2] = CMD_GET_INF;  // コマンド種別
  SendBuff[3] = 0;            // 予備

  SendData(SendBuff, len);
}

void LeptrinoNode::GetLimit(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Get sensor limit");
  len = 0x04;
  SendBuff[0] = len;            // レングス length
  SendBuff[1] = 0xFF;           // センサNo. Sensor no.
  SendBuff[2] = CMD_GET_LIMIT;  // コマンド種別 Command type
  SendBuff[3] = 0;              // 予備 reserve

  SendData(SendBuff, len);
}

void LeptrinoNode::SerialStart(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Start sensor");
  len = 0x04;                    // データ長
  SendBuff[0] = len;             // レングス
  SendBuff[1] = 0xFF;            // センサNo.
  SendBuff[2] = CMD_DATA_START;  // コマンド種別
  SendBuff[3] = 0;               // 予備

  SendData(SendBuff, len);
}

void LeptrinoNode::SerialStop(rclcpp::Logger logger)
{
  USHORT len;

  RCLCPP_INFO(logger, "Stop sensor\n");
  len = 0x04;                   // データ長
  SendBuff[0] = len;            // レングス
  SendBuff[1] = 0xFF;           // センサNo.
  SendBuff[2] = CMD_DATA_STOP;  // コマンド種別
  SendBuff[3] = 0;              // 予備

  SendData(SendBuff, len);
}

void LeptrinoNode::SensorNormalize(rclcpp::Logger logger)
{
  RCLCPP_INFO(logger, "Normalizing sensor data\n");
  int counter = 0;

  while (counter < 1000)
  {
    Comm_Rcv();

    if (Comm_CheckRcv() != 0)
    {
      memset(CommRcvBuff, 0, sizeof(CommRcvBuff));

      auto rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0)
      {
        grossForce = (ST_R_DATA_GET_F*)CommRcvBuff;
        if (DEBUG)
        {
          RCLCPP_INFO(this->get_logger(), "%d,%d,%d,%d,%d,%d", grossForce->ssForce[0], grossForce->ssForce[1],
                      grossForce->ssForce[2], grossForce->ssForce[3], grossForce->ssForce[4], grossForce->ssForce[5]);
        }
        offset.force[1] += grossForce->ssForce[0] * 0.001;
        offset.force[2] += grossForce->ssForce[1] * 0.001;
        offset.force[3] += grossForce->ssForce[2] * 0.001;
        offset.moment[1] += grossForce->ssForce[3] * 0.001;
        offset.moment[2] += grossForce->ssForce[4] * 0.001;
        offset.moment[3] += grossForce->ssForce[5] * 0.001;
      }
    }
    counter++;
    usleep(1000);
  }

  RCLCPP_INFO(this->get_logger(), "Offset-> Fx:%f, Fy:%f, Fx:%f, Mx:%f, My:%f, Mz:%f", offset.force[1], offset.force[2], offset.force[3],
              offset.moment[1], offset.moment[2], offset.moment[3]);

  RCLCPP_INFO(logger, "Normalizing done\n");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeptrinoNode>());
  rclcpp::shutdown();
  return 0;
}
