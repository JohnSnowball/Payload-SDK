/**
 ********************************************************************
 * @file    test_flight_control.h
 * @brief   This is the header file for "test_flight_control.c", defining the structure and
 * (exported) function prototypes.
 *
 * @copyright (c) 2021 DJI. All rights reserved.
 *
 * All information contained herein is, and remains, the property of DJI.
 * The intellectual and technical concepts contained herein are proprietary
 * to DJI and may be covered by U.S. and foreign patents, patents in process,
 * and protected by trade secret or copyright law.  Dissemination of this
 * information, including but not limited to data and other proprietary
 * material(s) incorporated within the information, in any form, is strictly
 * prohibited without the express written consent of DJI.
 *
 * If you receive this source code without DJI’s authorization, you may not
 * further disseminate the information, and you must immediately remove the
 * source code and notify DJI of its removal. DJI reserves the right to pursue
 * legal actions against you for any loss(es) or damage(s) caused by your
 * failure to do so.
 *
 *********************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef TEST_FLIGHT_CONTROL_H
#define TEST_FLIGHT_CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "dji_typedef.h"

#include "pthread.h"

#include "dji_fc_subscription.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/
typedef enum {
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_LANDING,
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_POSITION_CTRL_LANDING,
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_GO_HOME_FORCE_LANDING,
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_VELOCITY_CTRL_LANDING,
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_ARREST_FLYING,
    E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_SET_GET_PARAM,
    SAV_SUB_AND_CTRL_SAMPLE
} E_DjiTestFlightCtrlSampleSelect;

T_DjiReturnCode All_Topic_Init(void);

/* declaire all subscribed data api here*/
typedef struct
{
    T_DjiFcSubscriptionQuaternion quaternion;
    T_DjiDataTimestamp timestamp;
}quatrenion_data_node;
void* get_Quaternion_data_address(void);
static T_DjiReturnCode Sav_Quaternion_data_Callback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);

typedef struct 
{
    T_DjiFcSubscriptionAccelerationRaw acc;
    T_DjiDataTimestamp timestamp;
}acceleration_data_node;
void* get_Acceleration_data_address(void);
static T_DjiReturnCode Sav_Acceleration_data_Callback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);

typedef struct 
{
    T_DjiFcSubscriptionPositionFused pos;
    T_DjiDataTimestamp timestamp;
}position_data_node;
void* get_Position_data_address(void);
static T_DjiReturnCode Sav_Position_data_Callback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);

typedef struct 
{
    T_DjiFcSubscriptionVelocity vel;
    T_DjiDataTimestamp timestamp;
}velocity_data_node;
void* get_Velocity_data_address(void);
static T_DjiReturnCode Sav_Velocity_data_Callback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);


typedef struct 
{
    T_DjiFcSubscriptionRCWithFlagData rc;
    T_DjiDataTimestamp timestamp;
}rc_stick_data_node;
void* get_RC_stick_data_address(void);
static T_DjiReturnCode Sav_RC_stick_data_Callback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp);

/*end of subsribed data api decalration*/

#pragma pack(1)
typedef struct {
    dji_f32_t x;
    dji_f32_t y;
    dji_f32_t z;
} T_DjiTestFlightControlVector3f; // pack(1)
#pragma pack()

/* Exported types ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
T_DjiReturnCode DjiTest_FlightControlRunSample(E_DjiTestFlightCtrlSampleSelect flightCtrlSampleSelect);

T_DjiReturnCode Sav_FlightControl_Logger_Sample(void);

void DjiTest_FlightControlVelocityAndYawRateCtrl(const T_DjiTestFlightControlVector3f offsetDesired, float yawRate,
                                                 uint32_t timeMs);

void SAV_ControlVelocity_Yawrate_BodyCoord(const T_DjiTestFlightControlVector3f offsetDesired, float yawRate,
                                                 uint32_t timeMs);

#ifdef __cplusplus
}
#endif

#endif // TEST_FLIGHT_CONTROL_H
/************************ (C) COPYRIGHT DJI Innovations *******END OF FILE******/
