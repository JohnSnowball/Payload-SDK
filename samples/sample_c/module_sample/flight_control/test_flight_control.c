/**
 ********************************************************************
 * @file    test_flight_control.c
 * @brief
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

/* Includes ------------------------------------------------------------------*/
#include "dji_flight_controller.h"
#include "test_flight_control.h"
#include "dji_fc_subscription.h"
#include "dji_platform.h"
#include "dji_logger.h"
#include <math.h>
#include <widget_interaction_test/test_widget_interaction.h>
#include <dji_aircraft_info.h>
/* Private constants ---------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
typedef struct {
    E_DjiFcSubscriptionDisplayMode displayMode;
    char *displayModeStr;
} T_DjiTestFlightControlDisplayModeStr;

/* Private values -------------------------------------------------------------*/
static T_DjiOsalHandler *s_osalHandler = NULL;
static const double s_earthCenter = 6378137.0;
static const double s_degToRad = 0.01745329252;

static const T_DjiTestFlightControlDisplayModeStr s_flightControlDisplayModeStr[] = {
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE, .displayModeStr = "attitude mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS, .displayModeStr = "p_gps mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF, .displayModeStr = "assisted takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF, .displayModeStr = "auto takeoff mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING, .displayModeStr = "auto landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME, .displayModeStr = "go home mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING, .displayModeStr = "force landing mode"},
    {.displayMode = DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START, .displayModeStr = "engine start mode"},
    {.displayMode = 0xFF, .displayModeStr = "unknown mode"}
};

/* Private functions declaration ---------------------------------------------*/
static uint8_t DjiTest_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode);
static T_DjiFcSubscriptionFlightStatus DjiTest_FlightControlGetValueOfFlightStatus(void);
static T_DjiFcSubscriptionDisplaymode DjiTest_FlightControlGetValueOfDisplayMode(void);
static T_DjiFcSubscriptionHeightFusion DjiTest_FlightControlGetValueOfHeightFusion(void);
static T_DjiFcSubscriptionQuaternion DjiTest_FlightControlGetValueOfQuaternion(void);
static T_DjiFcSubscriptionPositionFused DjiTest_FlightControlGetValueOfPositionFused(void);

static T_DjiFcSubscriptionAccelerationRaw DjiTest_FlightControlGetValueOfAcceleration(void);
static T_DjiFcSubscriptionVelocity DjiTest_FlightControlGetValueOfVelocity(void);
static T_DjiFcSubscriptionWholeBatteryInfo DjiTest_FlightControlGetValueOfBattery(void);
static T_DjiFcSubscriptionEscData DjiTest_FlightControlGetValueOfESCData(void);
static T_DjiFcSubscriptionRCWithFlagData DjiTest_FlightControlGetValueOfRC(void);

static dji_f32_t DjiTest_FlightControlGetValueOfRelativeHeight(void);
static bool DjiTest_FlightControlMotorStartedCheck(void);
static bool DjiTest_FlightControlTakeOffInAirCheck(void);
static bool DjiTest_FlightControlLandFinishedCheck(void);
static bool DjiTest_FlightControlMonitoredTakeoff(void);
static bool DjiTest_FlightControlCheckActionStarted(E_DjiFcSubscriptionDisplayMode mode);
static bool DjiTest_FlightControlMonitoredLanding(void);
static bool DjiTest_FlightControlGoHomeAndConfirmLanding(void);
static T_DjiTestFlightControlVector3f DjiTest_FlightControlQuaternionToEulerAngle(T_DjiFcSubscriptionQuaternion quat);
static T_DjiTestFlightControlVector3f
DjiTest_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(T_DjiFcSubscriptionPositionFused target,
                                                            T_DjiFcSubscriptionPositionFused origin,
                                                            dji_f32_t targetHeight,
                                                            dji_f32_t originHeight);
static T_DjiTestFlightControlVector3f
DjiTest_FlightControlVector3FSub(T_DjiTestFlightControlVector3f vectorA, T_DjiTestFlightControlVector3f vectorB);
static int DjiTest_FlightControlSignOfData(dji_f32_t data);
static void DjiTest_FlightControlHorizCommandLimit(dji_f32_t speedFactor, dji_f32_t *commandX, dji_f32_t *commandY);
static dji_f32_t DjiTest_FlightControlVectorNorm(T_DjiTestFlightControlVector3f v);
static T_DjiReturnCode
DjiTest_FlightControlJoystickCtrlAuthSwitchEventCallback(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData);
static bool DjiTest_FlightControlMoveByPositionOffset(T_DjiTestFlightControlVector3f offsetDesired,
                                                      float yawDesiredInDeg,
                                                      float posThresholdInM,
                                                      float yawThresholdInDeg);
static T_DjiReturnCode DjiTest_FlightControlInit(void);
static T_DjiReturnCode SAV_FlightControlInit(void);//declare here
static T_DjiReturnCode DjiTest_FlightControlDeInit(void);
static void DjiTest_FlightControlTakeOffLandingSample(void);
static void DjiTest_FlightControlPositionControlSample(void);
static void DjiTest_FlightControlGoHomeForceLandingSample(void);
static void DjiTest_FlightControlVelocityControlSample(void);
static void DjiTest_FlightControlArrestFlyingSample(void);
static void DjiTest_FlightControlSample(E_DjiTestFlightCtrlSampleSelect flightCtrlSampleSelect);
static void SAV_SubscriptionandControlSample(void);

static pthread_t logger_thread = 0;
static pthread_t flightcontrol_thread = 0;

void* logger_loop(void*arg);
void* fcontrol_loop(void*arg);

/* Exported functions definition ---------------------------------------------*/
T_DjiReturnCode DjiTest_FlightControlRunSample(E_DjiTestFlightCtrlSampleSelect flightCtrlSampleSelect)
{
    T_DjiReturnCode returnCode;

    USER_LOG_DEBUG("Init flight Control Sample");
    DjiTest_WidgetLogAppend("Init flight Control Sample");

 /* defaut codes
    returnCode = DjiTest_FlightControlInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight Control sample failed,error code:0x%08llX", returnCode);
        return returnCode;
    }
*/

    /*all subscriptions are done here, during initialize part, need to moniter CPU usage*/
    returnCode = SAV_FlightControlInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init SAV sample failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    DjiTest_FlightControlSample(flightCtrlSampleSelect);

    USER_LOG_DEBUG("Deinit Flight Control Sample");
    DjiTest_WidgetLogAppend("Deinit Flight Control Sample");
    returnCode = DjiTest_FlightControlDeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit Flight Control sample failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    return returnCode;
}


T_DjiReturnCode Sav_FlightControl_Logger_Sample(void)
{

    T_DjiReturnCode returnCode;

    USER_LOG_DEBUG("Start fc and logger Sample");
    DjiTest_WidgetLogAppend("Start fc and logger Sample");

    returnCode = SAV_FlightControlInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init subscription or environment failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    //add logger loop and control loop here
    if(pthread_create(&logger_thread,NULL,logger_loop,NULL)){
        printf("create logger task fail.\n");
    }

    if(pthread_create(&flightcontrol_thread,NULL,fcontrol_loop,NULL)){
        printf("create fcontrol task fail.\n");
    }


    while (1) {
        printf("main task is running.\n");
        sleep(1);
    }

    USER_LOG_DEBUG("Deinit Flight Control Sample");
    DjiTest_WidgetLogAppend("Deinit Flight Control Sample");
    returnCode = DjiTest_FlightControlDeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit Flight Control sample failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    return returnCode;

}

/* Private functions definition-----------------------------------------------*/
T_DjiReturnCode DjiTest_FlightControlInit(void)
{
    T_DjiReturnCode returnCode;
    T_DjiFlightControllerRidInfo ridInfo = {0};

    s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    ridInfo.latitude = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude = 10;

    returnCode = DjiFlightController_Init(ridInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight controller module failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init data subscription module failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    /*! subscribe fc data */
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic flight status failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic display mode failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic avoid data failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    } else if (returnCode == DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE) {
        USER_LOG_WARN("Subscribe topic quaternion duplicate");
    } else {
        USER_LOG_ERROR("Subscribe topic quaternion failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic position fused failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude fused failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude of home point failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
        DjiTest_FlightControlJoystickCtrlAuthSwitchEventCallback);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
        USER_LOG_ERROR("Register joystick control authority event callback failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode SAV_FlightControlInit(void)
{
    /*subscription : 1. acceleration 200Hz 
                     2. quaterion --> attitude 100Hz 
                     3. vel&pos 50Hz 
                     4. battery 5-10Hz
                     5. RCstick 100Hz 
                     6. to do: blade or patch info(vel&pos) from stereo camera
                     7. thrust 50Hz retrive from esc data
    */
    T_DjiReturnCode returnCode;
    T_DjiFlightControllerRidInfo ridInfo = {0};

    s_osalHandler = DjiPlatform_GetOsalHandler();
    if (!s_osalHandler) return DJI_ERROR_SYSTEM_MODULE_CODE_UNKNOWN;

    ridInfo.latitude = 22.542812;
    ridInfo.longitude = 113.958902;
    ridInfo.altitude = 10;

    //initialize by defaut
    returnCode = DjiFlightController_Init(ridInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init flight controller module failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    //initialize subscription
    returnCode = DjiFcSubscription_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Init data subscription module failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    /*! subscribe fc data */
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic flight status failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_10_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic display mode failed, error code:0x%08llX", returnCode);
        return returnCode;
    }

    //ultra sonic height, used during landing, 10hz
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic avoid data failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    //acceleration in IMU frame, m/s2, highest frequency, 200hz, used for determine contact
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_200_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic acceleration failed,error code:0x%08llX", returnCode);
        return returnCode;
    }


    //attitude, need to be converted in to roll/pitch/yaw, 100hz, might be used to determine contact
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_100_HZ,
                                                  NULL);

    if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    } else if (returnCode == DJI_ERROR_SUBSCRIPTION_MODULE_CODE_TOPIC_DUPLICATE) {
        USER_LOG_WARN("Subscribe topic quaternion duplicate");
    } else {
        USER_LOG_ERROR("Subscribe topic quaternion failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    //position, lat/lon/alt/sat number, 50Hz
    //this data is strongly base on GPS signal, everytime we use this should check if GPS signal is good, satellite number > 12
    //need to do: enable RTK for M350!!!
    /* E_DjiFlightControllerRtkPositionEnableStatus == 1*/
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic position fused failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    //altitude above sea level, along with altitude, 50Hz
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude fused failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    //velocity in m/s, used for safety check, 50Hz
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic velocity failed,error code:0x%08llX", returnCode);
        return returnCode;
    }
    
    //RC stick XYZR and connection state, used for safety check, 50Hz
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic RC failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    //battery info, used for safety check, 5Hz
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic BATTERY failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    /*calculate average throttle from esc data */
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ESC_DATA,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic ESC failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    //home LAT/LON when last takeoff, lowest frequency
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_HOME_POINT_INFO,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude of home point failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    //home altitude above sea level recorded when last takeoff, lowest frequency
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                  DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
                                                  NULL);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe topic altitude of home point failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    //essentiel for all modes
    returnCode = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
        DjiTest_FlightControlJoystickCtrlAuthSwitchEventCallback);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
        USER_LOG_ERROR("Register joystick control authority event callback failed,error code:0x%08llX", returnCode);
        return returnCode;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

T_DjiReturnCode DjiTest_FlightControlDeInit(void)
{
    T_DjiReturnCode returnCode;

    returnCode = DjiFlightController_DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit flight controller module failed, error code:0x%08llX",
                       returnCode);
        return returnCode;
    }

    returnCode = DjiFcSubscription_DeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Deinit data subscription module failed, error code:0x%08llX",
                       returnCode);
        return returnCode;
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void DjiTest_FlightControlTakeOffLandingSample()
{
    T_DjiReturnCode returnCode;

    USER_LOG_INFO("Flight control takeoff-landing sample start");
    DjiTest_WidgetLogAppend("Flight control takeoff-landing sample start");
    USER_LOG_INFO("--> Step 1: Obtain joystick control authority.");
    DjiTest_WidgetLogAppend("--> Step 1: Obtain joystick control authority.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 2: Take off\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Take off\r\n");
    if (!DjiTest_FlightControlMonitoredTakeoff()) {
        USER_LOG_ERROR("Take off failed");
        goto out;
    }
    USER_LOG_INFO("Successful take off\r\n");
    DjiTest_WidgetLogAppend("Successful take off\r\n");
    s_osalHandler->TaskSleepMs(4000);

    USER_LOG_INFO("--> Step 3: Landing\r\n");
    DjiTest_WidgetLogAppend("--> Step 3: Landing\r\n");
    if (!DjiTest_FlightControlMonitoredLanding()) {
        USER_LOG_ERROR("Landing failed");
        goto out;
    }
    USER_LOG_INFO("Successful landing\r\n");
    DjiTest_WidgetLogAppend("Successful landing\r\n");

    USER_LOG_INFO("--> Step 4: Release joystick authority");
    DjiTest_WidgetLogAppend("--> Step 4: Release joystick authority");
    returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }

out:
    USER_LOG_INFO("Flight control takeoff-landing sample end");
    DjiTest_WidgetLogAppend("Flight control takeoff-landing sample end");
}

void DjiTest_FlightControlPositionControlSample()
{
    T_DjiReturnCode returnCode;

    USER_LOG_INFO("Flight control move-by-position sample start");
    DjiTest_WidgetLogAppend("Flight control move-by-position sample start");

    USER_LOG_INFO("--> Step 1: Obtain joystick control authority.");
    DjiTest_WidgetLogAppend("--> Step 1: Obtain joystick control authority.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 2: Take off\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Take off\r\n");
    if (!DjiTest_FlightControlMonitoredTakeoff()) {
        USER_LOG_ERROR("Take off failed");
        goto out;
    }
    USER_LOG_INFO("Successful take off\r\n");
    DjiTest_WidgetLogAppend("Successful take off\r\n");

    USER_LOG_INFO("--> Step 3: Move to north:0(m), east:6(m), up:6(m) , yaw:30(degree) from current point");
    DjiTest_WidgetLogAppend("--> Step 3: Move to north:0(m), east:6(m), up:6(m) , yaw:30(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {0, 6, 6}, 30, 0.8, 1)) {
        USER_LOG_ERROR("Move to north:0(m), east:6(m), up:6(m) , yaw:30(degree) from current point failed");
        goto out;
    };

    USER_LOG_INFO("--> Step 4: Move to north:6(m), east:0(m), up:-3(m) , yaw:-30(degree) from current point");
    DjiTest_WidgetLogAppend(
        "--> Step 4: Move to north:6(m), east:0(m), up:-3(m) , yaw:-30(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {6, 0, -3}, -30, 0.8, 1)) {
        USER_LOG_ERROR("Move to north:6(m), east:0(m), up:-3(m) , yaw:-30(degree) from current point failed");
        goto out;
    };

    USER_LOG_INFO("--> Step 5: Move to north:-6(m), east:-6(m), up:0(m) , yaw:0(degree) from current point");
    DjiTest_WidgetLogAppend("--> Step 5: Move to north:-6(m), east:-6(m), up:0(m) , yaw:0(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {-6, -6, 0}, 0, 0.8, 1)) {
        USER_LOG_ERROR("Move to north:-6(m), east:-6(m), up:0(m) , yaw:0(degree) from current point failed");
        goto out;
    }

    USER_LOG_INFO("--> Step 6: Landing\r\n");
    DjiTest_WidgetLogAppend("--> Step 6: Landing\r\n");
    if (!DjiTest_FlightControlMonitoredLanding()) {
        USER_LOG_ERROR("Landing failed");
        goto out;
    }
    USER_LOG_INFO("Successful landing\r\n");
    DjiTest_WidgetLogAppend("Successful landing\r\n");

    USER_LOG_INFO("--> Step 7: Release joystick authority");
    DjiTest_WidgetLogAppend("--> Step 7: Release joystick authority");
    returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }

out:
    USER_LOG_INFO("Flight control move-by-position sample end");
    DjiTest_WidgetLogAppend("Flight control move-by-position sample end");
}

void DjiTest_FlightControlGoHomeForceLandingSample()
{
    T_DjiReturnCode returnCode;

    USER_LOG_INFO("Flight control go-home-force-landing sample start");
    DjiTest_WidgetLogAppend("Flight control go-home-force-landing sample start");

    USER_LOG_INFO("--> Step 1: Obtain joystick control authority");
    DjiTest_WidgetLogAppend("--> Step 1: Obtain joystick control authority");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 2: Take off\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Take off\r\n");
    if (!DjiTest_FlightControlMonitoredTakeoff()) {
        USER_LOG_ERROR("Take off failed");
        goto out;
    }
    USER_LOG_INFO("Successful take off\r\n");
    DjiTest_WidgetLogAppend("Successful take off\r\n");

    USER_LOG_INFO("--> Step 3: Move to north:0(m), east:0(m), up:30(m) , yaw:0(degree) from current point");
    DjiTest_WidgetLogAppend("--> Step 3: Move to north:0(m), east:0(m), up:30(m) , yaw:0(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {0, 0, 30}, 0, 0.8, 1)) {
        USER_LOG_ERROR("Move to north:0(m), east:0(m), up:30(m) , yaw:0(degree) from current point failed");
        goto out;
    }

    USER_LOG_INFO("--> Step 4: Move to north:10(m), east:0(m), up:0(m) , yaw:0(degree) from current point");
    DjiTest_WidgetLogAppend("--> Step 4: Move to north:10(m), east:0(m), up:0(m) , yaw:0(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {10, 0, 0}, 0, 0.8, 1)) {
        USER_LOG_ERROR("Move to north:10(m), east:0(m), up:0(m) , yaw:0(degree) from current point failed");
        goto out;
    }

    USER_LOG_INFO("--> Step 5: Set aircraft current position as new home location!");
    DjiTest_WidgetLogAppend("--> Step 5: Set aircraft current position as new home location!");
    returnCode = DjiFlightController_SetHomeLocationUsingCurrentAircraftLocation();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set aircraft current position as new home location failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO("--> Step 6: Set go home altitude to 50(m)\r\n");
    DjiTest_WidgetLogAppend("--> Step 6: Set go home altitude to 50(m)\r\n");
    returnCode = DjiFlightController_SetGoHomeAltitude(50);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set go home altitude to 50(m) failed, error code: 0x%08X", returnCode);
        goto out;
    }

    /*! get go home altitude */
    E_DjiFlightControllerGoHomeAltitude goHomeAltitude;
    returnCode = DjiFlightController_GetGoHomeAltitude(&goHomeAltitude);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get go home altitude failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current go home altitude is %d m\r\n", goHomeAltitude);
    DjiTest_WidgetLogAppend("Current go home altitude is %d m\r\n", goHomeAltitude);

    USER_LOG_INFO("--> Step 7: Move to north:20(m), east:0(m), up:0(m) , yaw:0(degree) from current point");
    DjiTest_WidgetLogAppend("--> Step 7: Move to north:20(m), east:0(m), up:0(m) , yaw:0(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {20, 0, 0}, 0, 0.8, 1)) {
        USER_LOG_ERROR("Move to north:20(m), east:0(m), up:0(m) , yaw:0(degree) from current point failed");
        goto out;
    }

    USER_LOG_INFO("--> Step 8: Go home and confirm force landing\r\n");
    DjiTest_WidgetLogAppend("--> Step 8: Go home and confirm force landing\r\n");
    if (!DjiTest_FlightControlGoHomeAndConfirmLanding()) {
        USER_LOG_ERROR("Go home and confirm force landing failed");
        goto out;
    }
    USER_LOG_INFO("Successful go home and confirm force landing\r\n");
    DjiTest_WidgetLogAppend("Successful go home and confirm force landing\r\n");

    USER_LOG_INFO("-> Step 9: Release joystick authority");
    DjiTest_WidgetLogAppend("-> Step 9: Release joystick authority");
    returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }

out:
    USER_LOG_INFO("Flight control go-home-force-landing sample end");
    DjiTest_WidgetLogAppend("Flight control go-home-force-landing sample end");
}

void DjiTest_FlightControlVelocityControlSample()
{
    T_DjiReturnCode returnCode;

    USER_LOG_INFO("Flight control move-by-velocity sample start");
    DjiTest_WidgetLogAppend("Flight control move-by-velocity sample start");

    USER_LOG_INFO("--> Step 1: Obtain joystick control authority");
    DjiTest_WidgetLogAppend("--> Step 1: Obtain joystick control authority");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 2: Take off\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Take off\r\n");
    if (!DjiTest_FlightControlMonitoredTakeoff()) {
        USER_LOG_ERROR("Take off failed");
        goto out;
    }
    USER_LOG_INFO("Successful take off\r\n");
    DjiTest_WidgetLogAppend("Successful take off\r\n");

    USER_LOG_INFO(
        "--> Step 3: Move with north:0(m/s), east:0(m/s), up:5(m/s), yaw:0(deg/s) from current point for 2s!");
    DjiTest_WidgetLogAppend(
        "--> Step 3: Move with north:0(m/s), east:0(m/s), up:5(m/s), yaw:0(deg/s) from current point for 2s!");
    DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {0, 0, 5.0}, 0, 2000);

    USER_LOG_INFO("--> Step 4: Emergency brake for 2s");
    DjiTest_WidgetLogAppend("--> Step 4: Emergency brake for 2s");
    returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);
    returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO(
        "--> Step 5: Move with north:-1.5(m/s), east:2(m/s), up:0(m/s), yaw:20(deg/s) from current point for 2s!");
    DjiTest_WidgetLogAppend(
        "--> Step 5: Move with north:-1.5(m/s), east:2(m/s), up:0(m/s), yaw:20(deg/s) from current point for 2s!");
    DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {-1.5, 2, 0}, 20, 2000);

    USER_LOG_INFO("--> Step 6: Emergency brake for 2s");
    DjiTest_WidgetLogAppend("--> Step 6: Emergency brake for 2s");
    returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);
    returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO(
        "--> Step 7: Move with north:3(m/s), east:0(m/s), up:0(m/s), yaw:0(deg/s) from current point for 2.5s!");
    DjiTest_WidgetLogAppend(
        "--> Step 7: Move with north:3(m/s), east:0(m/s), up:0(m/s), yaw:0(deg/s) from current point for 2.5s!");
    DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {3, 0, 0}, 0, 2500);

    USER_LOG_INFO("--> Step 8: Emergency brake for 2s");
    DjiTest_WidgetLogAppend("--> Step 8: Emergency brake for 2s");
    returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);
    returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO(
        "--> Step 9: Move with north:-1.6(m/s), east:-2(m/s), up:0(m/s), yaw:0(deg/s) from current point for 2.2s!");
    DjiTest_WidgetLogAppend(
        "--> Step 9: Move with north:-1.6(m/s), east:-2(m/s), up:0(m/s), yaw:0(deg/s) from current point for 2.2s!");
    DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {-1.6, -2, 0}, 0, 2200);

    USER_LOG_INFO("--> Step 10: Emergency brake for 2s");
    DjiTest_WidgetLogAppend("--> Step 10: Emergency brake for 2s");
    returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);
    returnCode = DjiFlightController_CancelEmergencyBrakeAction();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Cancel emergency brake action failed, error code: 0x%08X", returnCode);
        goto out;
    }

    USER_LOG_INFO("--> Step 11: Landing\r\n");
    DjiTest_WidgetLogAppend("--> Step 11: Landing\r\n");
    if (!DjiTest_FlightControlMonitoredLanding()) {
        USER_LOG_ERROR("Landing failed");
        goto out;
    }
    USER_LOG_INFO("Successful landing\r\n");
    DjiTest_WidgetLogAppend("Successful landing\r\n");

    USER_LOG_INFO("--> Step 12: Release joystick authority");
    DjiTest_WidgetLogAppend("--> Step 12: Release joystick authority");
    returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }

out:
    USER_LOG_INFO("Flight control move-by-velocity sample end");
    DjiTest_WidgetLogAppend("Flight control move-by-velocity sample end");
}

void DjiTest_FlightControlArrestFlyingSample()
{
    T_DjiReturnCode returnCode;

    USER_LOG_INFO("Flight control arrest-flying sample start");
    DjiTest_WidgetLogAppend("Flight control arrest-flying sample start");

    USER_LOG_INFO("--> Step 1: Enable arrest-flying");
    DjiTest_WidgetLogAppend("--> Step 1: Enable arrest-flying");
    returnCode = DjiFlightController_ArrestFlying();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Enable arrest-flying failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);

    //you can replace with takeoff to test in air.
    USER_LOG_INFO("--> Step 2: Turn on motors\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Turn on motors\r\n");
    returnCode = DjiFlightController_TurnOnMotors();
    if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on motors successfully, but arrest-flying failed");
        s_osalHandler->TaskSleepMs(4000);
        USER_LOG_INFO("--> Step 3: Turn off motors\r\n");
        DjiTest_WidgetLogAppend("--> Step 3: Turn off motors\r\n");
        returnCode = DjiFlightController_TurnOffMotors();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Turn off motors failed, error code: 0x%08X", returnCode);
        }
        goto out;
    }

    USER_LOG_INFO("Turn on motors failed.Arrest-flying successfully\r\n");
    DjiTest_WidgetLogAppend("Turn on motors failed.Arrest-flying successfully\r\n");
    s_osalHandler->TaskSleepMs(2000);

    USER_LOG_INFO("--> Step 3: Disable arrest-flying");
    DjiTest_WidgetLogAppend("--> Step 3: Disable arrest-flying");
    returnCode = DjiFlightController_CancelArrestFlying();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Disable arrest-flying failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);

    USER_LOG_INFO("--> Step 4: Turn on motors\r\n");
    DjiTest_WidgetLogAppend("--> Step 4: Turn on motors\r\n");
    returnCode = DjiFlightController_TurnOnMotors();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on motors failed and disable arrest-flying failed, error code: 0x%08X", returnCode);
        goto out;
    } else {
        USER_LOG_INFO("Turn on motors successfully and disable arrest-flying successfully\r\n");
        s_osalHandler->TaskSleepMs(4000);
        USER_LOG_INFO("--> Step 5: Turn off motors");
        DjiTest_WidgetLogAppend("--> Step 5: Turn off motors");
        returnCode = DjiFlightController_TurnOffMotors();
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Turn off motors failed, error code: 0x%08X", returnCode);
        }
    }

out:
    USER_LOG_INFO("Flight control arrest-flying sample end");
    DjiTest_WidgetLogAppend("Flight control arrest-flying sample end");
}

void DjiTest_FlightControlSetGetParamSample()
{
    T_DjiReturnCode returnCode;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalVisualObstacleAvoidanceStatus;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus horizontalRadarObstacleAvoidanceStatus;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsVisualObstacleAvoidanceStatus;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus upwardsRadarObstacleAvoidanceStatus;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus downloadsVisualObstacleAvoidanceStatus;
    E_DjiFlightControllerGoHomeAltitude goHomeAltitude;
    E_DjiFlightControllerRtkPositionEnableStatus rtkEnableStatus;
    E_DjiFlightControllerRCLostAction rcLostAction;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;

    returnCode = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
    }

    USER_LOG_INFO("Flight control set-get-param sample start");
    DjiTest_WidgetLogAppend("Flight control set-get-param sample start");

    /*! Turn on horizontal vision avoid enable */
    USER_LOG_INFO("--> Step 1: Turn on horizontal visual obstacle avoidance");
    DjiTest_WidgetLogAppend("--> Step 1: Turn on horizontal visual obstacle avoidance");
    returnCode = DjiFlightController_SetHorizontalVisualObstacleAvoidanceEnableStatus(
        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on horizontal visual obstacle avoidance failed, error code: 0x%08X", returnCode);
        goto out;
    };
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 2: Get horizontal horizontal visual obstacle status\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Get horizontal horizontal visual obstacle status\r\n");
    returnCode = DjiFlightController_GetHorizontalVisualObstacleAvoidanceEnableStatus(
        &horizontalVisualObstacleAvoidanceStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get horizontal visual obstacle avoidance failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current horizontal visual obstacle avoidance status is %d\r\n",
                  horizontalVisualObstacleAvoidanceStatus);
    s_osalHandler->TaskSleepMs(1000);

    /*! Turn on horizontal radar avoid enable */
    USER_LOG_INFO("--> Step 3: Turn on horizontal radar obstacle avoidance");
    DjiTest_WidgetLogAppend("--> Step 3: Turn on horizontal radar obstacle avoidance");
    if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK) {
        returnCode = DjiFlightController_SetHorizontalRadarObstacleAvoidanceEnableStatus(
            DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Turn on horizontal radar obstacle avoidance failed, error code: 0x%08X", returnCode);
            goto out;
        };
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 4: Get horizontal radar obstacle avoidance status\r\n");
    DjiTest_WidgetLogAppend("--> Step 4: Get horizontal radar obstacle avoidance status\r\n");
    if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK) {
        returnCode = DjiFlightController_GetHorizontalRadarObstacleAvoidanceEnableStatus(
            &horizontalRadarObstacleAvoidanceStatus);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Get horizontal radar obstacle avoidance failed, error code: 0x%08X", returnCode);
            goto out;
        }
        USER_LOG_INFO("Current horizontal radar obstacle avoidance status is %d\r\n",
                      horizontalRadarObstacleAvoidanceStatus);
    }
    s_osalHandler->TaskSleepMs(1000);

    /*! Turn on upwards vision avoid enable */
    USER_LOG_INFO("--> Step 5: Turn on upwards visual obstacle avoidance.");
    DjiTest_WidgetLogAppend("--> Step 5: Turn on upwards visual obstacle avoidance.");
    returnCode = DjiFlightController_SetUpwardsVisualObstacleAvoidanceEnableStatus(
        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on upwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
        goto out;
    };
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 6: Get upwards visual obstacle avoidance status\r\n");
    DjiTest_WidgetLogAppend("--> Step 6: Get upwards visual obstacle avoidance status\r\n");
    returnCode = DjiFlightController_GetUpwardsVisualObstacleAvoidanceEnableStatus(
        &upwardsVisualObstacleAvoidanceStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get upwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current upwards visual obstacle avoidance status is %d\r\n", upwardsVisualObstacleAvoidanceStatus);
    s_osalHandler->TaskSleepMs(1000);

    /*! Turn on upwards radar avoid enable */
    USER_LOG_INFO("--> Step 7: Turn on upwards radar obstacle avoidance.");
    DjiTest_WidgetLogAppend("--> Step 7: Turn on upwards radar obstacle avoidance.");
    if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK) {
        returnCode = DjiFlightController_SetUpwardsRadarObstacleAvoidanceEnableStatus(
            DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Turn on upwards radar obstacle avoidance failed, error code: 0x%08X", returnCode);
            goto out;
        }
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 8: Get upwards radar obstacle avoidance status\r\n");
    DjiTest_WidgetLogAppend("--> Step 8: Get upwards radar obstacle avoidance status\r\n");
    if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M300_RTK ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30 ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M30T ||
        aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M350_RTK) {
        returnCode = DjiFlightController_GetUpwardsRadarObstacleAvoidanceEnableStatus(
            &upwardsRadarObstacleAvoidanceStatus);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Get upwards radar obstacle avoidance failed, error code: 0x%08X", returnCode);
            goto out;
        }
        USER_LOG_INFO("Current upwards radar obstacle avoidance status is %d\r\n", upwardsRadarObstacleAvoidanceStatus);
        s_osalHandler->TaskSleepMs(1000);
    }

    /*! Turn on downwards vision avoid enable */
    USER_LOG_INFO("--> Step 9: Turn on downwards visual obstacle avoidance.");
    DjiTest_WidgetLogAppend("--> Step 9: Turn on downwards visual obstacle avoidance.");
    returnCode = DjiFlightController_SetDownwardsVisualObstacleAvoidanceEnableStatus(
        DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Turn on downwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 10: Get downwards visual obstacle avoidance status\r\n");
    DjiTest_WidgetLogAppend("--> Step 10: Get downwards visual obstacle avoidance status\r\n");
    returnCode = DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(
        &downloadsVisualObstacleAvoidanceStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get downwards visual obstacle avoidance failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current downwards visual obstacle avoidance status is %d\r\n",
                  downloadsVisualObstacleAvoidanceStatus);
    s_osalHandler->TaskSleepMs(1000);

    /*! Set new go home altitude */
    USER_LOG_INFO("--> Step 11: Set go home altitude to 50(m)");
    DjiTest_WidgetLogAppend("--> Step 11: Set go home altitude to 50(m)");
    returnCode = DjiFlightController_SetGoHomeAltitude(50);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set go home altitude to 50(m) failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    /*! get go home altitude */
    USER_LOG_INFO("--> Step 12: Get go home altitude\r\n");
    DjiTest_WidgetLogAppend("--> Step 12: Get go home altitude\r\n");
    returnCode = DjiFlightController_GetGoHomeAltitude(&goHomeAltitude);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get go home altitude failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current go home altitude is %d m\r\n", goHomeAltitude);
    s_osalHandler->TaskSleepMs(2000);

    /*! Set rtk enable */
    USER_LOG_INFO("--> Step 13: Set rtk enable status");
    DjiTest_WidgetLogAppend("--> Step 13: Set rtk enable status");
    returnCode = DjiFlightController_SetRtkPositionEnableStatus(DJI_FLIGHT_CONTROLLER_ENABLE_RTK_POSITION);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Set rtk enable failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(1000);

    USER_LOG_INFO("--> Step 14: Get rtk enable status\r\n");
    DjiTest_WidgetLogAppend("--> Step 14: Get rtk enable status\r\n");
    returnCode = DjiFlightController_GetRtkPositionEnableStatus(&rtkEnableStatus);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get rtk enable failed, error code: 0x%08X", returnCode);
        goto out;
    }
    USER_LOG_INFO("Current rtk enable status is %d\r\n", rtkEnableStatus);
    s_osalHandler->TaskSleepMs(1000);

    /*! Set rc lost action */
    if (aircraftInfoBaseInfo.aircraftType != DJI_AIRCRAFT_TYPE_M300_RTK &&
        aircraftInfoBaseInfo.aircraftType != DJI_AIRCRAFT_TYPE_M350_RTK) {
        USER_LOG_INFO("--> Step 15: Set rc lost action");
        DjiTest_WidgetLogAppend("--> Step 15: Set rc lost action");
        returnCode = DjiFlightController_SetRCLostAction(DJI_FLIGHT_CONTROLLER_RC_LOST_ACTION_GOHOME);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Set rc lost action failed, error code: 0x%08X", returnCode);
            goto out;
        }
        s_osalHandler->TaskSleepMs(1000);

        USER_LOG_INFO("--> Step 16: Get rc lost action\r\n");
        DjiTest_WidgetLogAppend("--> Step 16: Get rc lost action\r\n");
        returnCode = DjiFlightController_GetRCLostAction(&rcLostAction);
        if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
            USER_LOG_ERROR("Get rc lost action failed, error code: 0x%08X", returnCode);
            goto out;
        }
        USER_LOG_INFO("Current rc lost action is %d\r\n", rcLostAction);
        DjiTest_WidgetLogAppend("Current rc lost action is %d\r\n", rcLostAction);
        s_osalHandler->TaskSleepMs(1000);
    }

out:
    USER_LOG_INFO("Flight control set-get-param sample end");
    DjiTest_WidgetLogAppend("Flight control set-get-param sample end");
}

void SAV_SubscriptionandControlSample()
{
    /*subscription : 1. acceleration 200Hz 
                     2. quaterion 100Hz 
                     3. vel&pos 50Hz 
                     4. battery 10Hz
                     5. RCstick 50Hz 
                     6. blade or patch info(vel&pos) from stereo camera
                     7. thrust 50Hz

      control :      1  obtain joystick control
                     2. takeoff
            Phase1   3. set joystick mode, mainly pos_ctrl
                     4. control the drone to fly towards the desired position(assume as the blade tip)
            Phase2   5. change joystick mode, velocity mode mainly, need for more tests
                     6. fly forwards according to the given velocity commandes
            Phase3   7. fly backwards 
                     8. RTL
                     9. release joystick control authority

      safety check:  1. velocity check
                     2. attitude check
                     3. thrust check
                     4. acceleration check(contact)

    All control&safety check running in 30-50 Hz, depends on CPU usage(core stability).
    */
    T_DjiReturnCode returnCode;
    int flag_break_by_rc = false;
    int flag_high_attitude = false;
    int flag_high_velocity = false;
    int flag_high_throttle = false;


    float  X_acc_average = 0.0f;
    float  Y_acc_average = 0.0f;
    float  Z_acc_average = 0.0f;
    int flag_contacted_maybe = false;
    int flag_drone_blade_in_contact = false;
    int contact_maybe_counter = 0;


    int start_single_measurement = false;
    int flag_close_to_blade = false;

    /*
    todo: make all these control steps in a big loop and check in each loop if the pilot wants to takeover control authority.
        while((!mission_done or !RC_take_authority or !strong_wind)&&slower_than_50Hz)
        {
            1. update drone status;

            2. check if:
                i.     attitude& velocity& throttle exceeded bounds --> (strong_wind == true) and break;
                ii.    RC takes over control --> (RC_take_authority == true) and break;
                iii.   acceleration check --> contacted == true;
                iiii.  start a single measurement command by pilot --> start_single_measure == true;
                iiiii. Done command from pilot --> (mission_done == true) and break;
        
            3. control:
                if(far away from blade)
                    {   control phase1 --> 
                        check flags, if safe, do approach, trajectory generation
                        else stabilze for a moment and report}
                elseif(close to the blade && start_single_measure) 
                    {   control phase2 --> 
                        check flags, 
                        if (safe){
                            
                            if(!contacted) {
                                do fly slowly towards the patch until contact } 
                            elseif(contacted) {
                                backoff until distace > 1m 
                                start_single_measure == false}
                        else {

                            backoff until distance > 1m 
                            start_single_measure = false}
                        
                }
                elseif(close to the blade && !start_single_measure){ stabilize }
                else(reserved case) {stabilize and wait}
        }

        if(mission_done && RC_connected)  release RC authority ;

        if(RC_take_authority) release RC authority;

        if(strong_wind && RC_connected) fly backwards for 5 sec -> stabilize and warn pilot 
    */



    /*Firstly fetch values from subscribed topic, determine if it's safe for auto control */
    while{
        //DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT, 10Hz

        //acceleration in IMU frame, m/s2, highest frequency, 200hz, used for determine contact
        T_DjiFcSubscriptionAccelerationRaw Acc_raw = DjiTest_FlightControlGetValueOfAcceleration();
        float acc_x = Acc_raw.x;
        float acc_y = Acc_raw.y;
        float acc_z = Acc_raw.z;

        //need to fix with more log data, filter coefficients to be determined
        if((fabs(acc_x - X_acc_average) > 0.01f ) || (fabs(acc_y - Y_acc_average) > 0.01f ) 
            ||(fabs(acc_y - Y_acc_average) > 0.01f )){
           
            flag_contacted_maybe = true;
            contact_maybe_counter += 2;
        }
        else if(flag_contacted_maybe){
            
            if(contact_maybe_counter <=1 ){
                flag_contacted_maybe = false; 
            }else{
                contact_maybe_counter -= 1;
            }
           
        }
        else{
           
        }

        if(flag_contacted_maybe && contact_maybe_counter >= 20){
            flag_drone_blade_in_contact = true;
        }

        X_acc_average = X_acc_average*0.9f + acc_x*0.1f;
        Y_acc_average = Y_acc_average*0.9f + acc_y*0.1f;
        Z_acc_average = Z_acc_average*0.9f + acc_z*0.1f;


        //attitude, need to be converted in to roll/pitch/yaw, 100hz, might be used to determine contact
        T_DjiFcSubscriptionQuaternion currentQuaternion = DjiTest_FlightControlGetValueOfQuaternion();
        T_DjiTestFlightControlVector3f eulerAngle = DjiTest_FlightControlQuaternionToEulerAngle(currentQuaternion);
        float pitchInDeg = 57.3f*eulerAngle.x;
        float rollInDeg = 57.3f*eulerAngle.y;
        float yawInDeg = 57.3f*eulerAngle.z;

        //position, lat/lon/alt/sat number, 50Hz
        //this data is strongly base on GPS signal, everytime we use this should check if GPS signal is good, satellite number > 12
        //need to do: enable RTK for M350!!!
        /* E_DjiFlightControllerRtkPositionEnableStatus == 1*/
        T_DjiFcSubscriptionPositionFused currentGPSPosition = DjiTest_FlightControlGetValueOfPositionFused();
        float lat_curr = currentGPSPosition.latitude;
        float lon_curr = currentGPSPosition.longitude;
        float curr_alt = currentGPSPosition.altitude;
        float satNum = currentGPSPosition.visibleSatelliteNumber;
            

        //altitude above sea level, along with altitude, 50Hz


        //velocity in m/s, used for safety check, 50Hz
        T_DjiFcSubscriptionVelocity currentVelocity = DjiTest_FlightControlGetValueOfVelocity();
        float vel_N = currentVelocity.data.x;
        float vel_E = currentVelocity.data.y;
        float vel_U = currentVelocity.data.z;

        //RC stick XYZR and connection state, used for safety check, 50Hz
        T_DjiFcSubscriptionRCWithFlagData RC_infos = DjiTest_FlightControlGetValueOfRC();
        float stick_pitch = RC_infos.pitch;
        float stick_roll = RC_infos.roll;
        float stick_yaw = RC_infos.yaw;
        float stick_throttle = RC_infos.throttle;
        float gnd_signal = RC_infos.flag.groundConnected;

        //battery info, used for safety check, 5Hz
        T_DjiFcSubscriptionWholeBatteryInfo currBattery = DjiTest_FlightControlGetValueOfBattery();
        float voltage_whole = currBattery.voltage;
        float percent_whole = currBattery.percentage;

        /*calculate average throttle from esc data, here we use rotation speed instead, unit in RPM */
        T_DjiFcSubscriptionEscData ESC_all = DjiTest_FlightControlGetValueOfESCData();
        float speed0 = ESC_all.esc[0].speed;
        float speed1 = ESC_all.esc[1].speed;
        float speed2 = ESC_all.esc[2].speed;
        float speed3 = ESC_all.esc[3].speed;
        float speed_average = (speed0+speed1+speed2+speed3)/4;

        //home altitude above sea level recorded when last takeoff, lowest frequency

    }
    USER_LOG_INFO("SAV sample start");
    DjiTest_WidgetLogAppend("SAV sample start");



    /* first step as without authority, we can' start the sample*/
    USER_LOG_INFO("--> Step 1: Obtain joystick control authority.");
    DjiTest_WidgetLogAppend("--> Step 1: Obtain joystick control authority.");
    returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }
    s_osalHandler->TaskSleepMs(100);

    /*takeoff, in real flight we don't need it for recent future */
    USER_LOG_INFO("--> Step 2: Take off\r\n");
    DjiTest_WidgetLogAppend("--> Step 2: Take off\r\n");
    if (!DjiTest_FlightControlMonitoredTakeoff()) {
        USER_LOG_ERROR("Take off failed");
        goto out;
    }
    USER_LOG_INFO("Successful take off\r\n");
    DjiTest_WidgetLogAppend("Successful take off\r\n");
    s_osalHandler->TaskSleepMs(5000);

    /* fly towards blade(virtuel), this depends on how the desired position is given, relative to drone or absolute in NED frame?
       here we assume the position is given by absolute NED frame
    */

   //move fast
    USER_LOG_INFO("--> Phase1.1: Move to north:10(m), east:20(m), up:20(m) , yaw:15(degree) from current point");
    DjiTest_WidgetLogAppend("--> Phase1.2: Move to north:10(m), east:20(m), up:20(m) , yaw:15(degree) from current pointt");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {10, 20, 20}, 15, 0.8, 1)) {
        USER_LOG_ERROR("Phase1.1 failed");
        goto out;
    };

    //move in another direction
    USER_LOG_INFO("--> Phase1.2: Move to north:2(m), east:3(m), up:0.5(m) , yaw:-10(degree) from current point");
    DjiTest_WidgetLogAppend(
        "--> Phase1.2: Move to north:2(m), east:3(m), up:0.5(m) , yaw:-10(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {2, 3, 0.5}, -10, 0.8, 1)) {
        USER_LOG_ERROR("Phase1.2 failed");
        goto out;
    };

    //adjust slightly for re-positioning, target on patch right now
    USER_LOG_INFO("--> Phase1.3: Move to north:0.05(m), east:-0.12(m), up:0.02(m) , yaw:0.5(degree) from current point");
    DjiTest_WidgetLogAppend("--> Phase1.3: Move to north:0.05(m), east:-0.12(m), up:0.02(m) , yaw:0.5(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {0.05, -0.12, 0.02}, 0.5, 0.1, 1)) {
        USER_LOG_ERROR("Phase1.3 failed");
        goto out;
    }
    s_osalHandler->TaskSleepMs(2000);

    /*here comes the phase2
    1. Assume the patch is right before the drone and stable. In real flight, could make a PID here to calculate desired velocity.
    to do: 1. Assume the patch is vibrating, the target position is moving
           2. Apply wind to see if velocity control works or if we need switch back to position control
           3. Apply pitch and velocity bounds to see if it's dangerous
           4. The ending condition should be a maximum time or high acceleration detection 
    */
    USER_LOG_INFO("--> Phase2.1: (0.2 m/s, 0 m/s, 0m/s, 0°/s) Move forwards for 10 seconds");
    DjiTest_WidgetLogAppend("--> Phase2 - Vel1: (0.2 m/s, 0 m/s, 0m/s, 0°/s) Move forwards for 10 seconds");
    SAV_ControlVelocity_Yawrate_BodyCoord((T_DjiTestFlightControlVector3f) {0.2, 0, 0}, 0, 10000);


    //if contacted, fly backwards faster for 3 seconds
    USER_LOG_INFO("--> Phase2.2: (-1 m/s, 0 m/s, 0m/s, 0°/s) Move backwards for 3 seconds");
    DjiTest_WidgetLogAppend("--> Phase2.2: (-1 m/s, 0 m/s, 0m/s, 0°/s) Move backwards for 3 seconds");
    SAV_ControlVelocity_Yawrate_BodyCoord((T_DjiTestFlightControlVector3f) {-1, 0, 0}, 0, 3000);


    //switch to position control mode and RTL
    //to do: complet RTL

    // turn yaw
    USER_LOG_INFO("--> Phase3.1:  yaw:180(degree) turn the yaw backwards");
    DjiTest_WidgetLogAppend(" --> Phase3.1:  yaw:180(degree) turn the yaw backwards");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {0, 0, 0}, 180, 0.8, 5)) {
        USER_LOG_ERROR("Phase3.1 failed");
        goto out;
    };
    s_osalHandler->TaskSleepMs(2000);


    // return fast
    USER_LOG_INFO("--> Phase3.2: Move to north:-10(m), east:-20(m), up:-20(m) , yaw:0(degree) from current point");
    DjiTest_WidgetLogAppend("--> Phase3.2: Move to north:-10(m), east:-20(m), up:-20(m) , yaw:0(degree) from current point");
    if (!DjiTest_FlightControlMoveByPositionOffset((T_DjiTestFlightControlVector3f) {-10, -20, -20}, 0, 0.8, 1)) {
        USER_LOG_ERROR("Phase3.2 failed");
        goto out;
    };

    //replace auto-landing by a full RTL, always aorks, 20240301
    USER_LOG_INFO("--> Phase4: RTL\r\n");
    DjiTest_WidgetLogAppend("--> Phase4: RTL\r\n");
    if (!DjiTest_FlightControlGoHomeAndConfirmLanding()) {
        USER_LOG_ERROR("RTL failed");
        goto out;
    }
    USER_LOG_INFO("Successful RTL\r\n");
    DjiTest_WidgetLogAppend("Successful RTL\r\n");

    USER_LOG_INFO("--> Phase5 - ending: Release joystick authority");
    DjiTest_WidgetLogAppend("--> Phase5 - ending: Release joystick authority");
    returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
        goto out;
    }

out:
    USER_LOG_INFO("SAV sample end");
    DjiTest_WidgetLogAppend("SAV sample end");
    
}

void DjiTest_FlightControlSample(E_DjiTestFlightCtrlSampleSelect flightCtrlSampleSelect)
{
    switch (flightCtrlSampleSelect) {
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_LANDING: {
            DjiTest_FlightControlTakeOffLandingSample();
            break;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_POSITION_CTRL_LANDING: {
            DjiTest_FlightControlPositionControlSample();
            break;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_GO_HOME_FORCE_LANDING: {
            DjiTest_FlightControlGoHomeForceLandingSample();
            break;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_TAKE_OFF_VELOCITY_CTRL_LANDING: {
            DjiTest_FlightControlVelocityControlSample();
            break;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_ARREST_FLYING: {
            DjiTest_FlightControlArrestFlyingSample();
            break;
        }
        case E_DJI_TEST_FLIGHT_CTRL_SAMPLE_SELECT_SET_GET_PARAM: {
            DjiTest_FlightControlSetGetParamSample();
            break;
        }
        case SAV_SUB_AND_CTRL_SAMPLE: {
            SAV_SubscriptionandControlSample();
            break;
        }
        default:
            break;
    }
}

uint8_t DjiTest_FlightControlGetDisplayModeIndex(E_DjiFcSubscriptionDisplayMode displayMode)
{
    uint8_t i;

    for (i = 0; i < sizeof(s_flightControlDisplayModeStr) / sizeof(T_DjiTestFlightControlDisplayModeStr); i++) {
        if (s_flightControlDisplayModeStr[i].displayMode == displayMode) {
            return i;
        }
    }

    return i;
}

T_DjiFcSubscriptionFlightStatus DjiTest_FlightControlGetValueOfFlightStatus(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionFlightStatus flightStatus;
    T_DjiDataTimestamp flightStatusTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_FLIGHT,
                                                      (uint8_t *) &flightStatus,
                                                      sizeof(T_DjiFcSubscriptionFlightStatus),
                                                      &flightStatusTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic flight status error, error code: 0x%08X", djiStat);
        flightStatus = 0;
    } 

    return flightStatus;
}

T_DjiFcSubscriptionDisplaymode DjiTest_FlightControlGetValueOfDisplayMode(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionDisplaymode displayMode;
    T_DjiDataTimestamp displayModeTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_STATUS_DISPLAYMODE,
                                                      (uint8_t *) &displayMode,
                                                      sizeof(T_DjiFcSubscriptionDisplaymode),
                                                      &displayModeTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic display mode error, error code: 0x%08X", djiStat);
        displayMode = 0;
    } 

    return displayMode;
}

T_DjiFcSubscriptionHeightFusion DjiTest_FlightControlGetValueOfHeightFusion(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionHeightFusion heightFusion = {0};
    T_DjiDataTimestamp timestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_HEIGHT_FUSION,
                                                      (uint8_t *) &heightFusion,
                                                      sizeof(T_DjiFcSubscriptionHeightFusion),
                                                      &timestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic height fusion error, error code: 0x%08X", djiStat);
    } 

    return heightFusion;
}

T_DjiFcSubscriptionAccelerationRaw DjiTest_FlightControlGetValueOfAcceleration(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionAccelerationRaw Acceleration_raw = {0};
    T_DjiDataTimestamp Acc_rawTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW,
                                                      (uint8_t *) &Acceleration_raw,
                                                      sizeof(T_DjiFcSubscriptionAccelerationRaw),
                                                      &Acc_rawTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic acceleration error, error code: 0x%08X", djiStat);
    } 

    return Acceleration_raw;
}

T_DjiFcSubscriptionQuaternion DjiTest_FlightControlGetValueOfQuaternion(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionQuaternion quaternion = {0};
    T_DjiDataTimestamp quaternionTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION,
                                                      (uint8_t *) &quaternion,
                                                      sizeof(T_DjiFcSubscriptionQuaternion),
                                                      &quaternionTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic quaternion error, error code: 0x%08X", djiStat);
    } 

    return quaternion;
}

T_DjiFcSubscriptionPositionFused DjiTest_FlightControlGetValueOfPositionFused(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionPositionFused positionFused = {0};
    T_DjiDataTimestamp positionFusedTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED,
                                                      (uint8_t *) &positionFused,
                                                      sizeof(T_DjiFcSubscriptionPositionFused),
                                                      &positionFusedTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic position fused error, error code: 0x%08X", djiStat);
    } 

    return positionFused;
}

T_DjiFcSubscriptionVelocity DjiTest_FlightControlGetValueOfVelocity(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionVelocity rt_Velocity = {0};
    T_DjiDataTimestamp rt_VelocityTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY,
                                                      (uint8_t *) &rt_Velocity,
                                                      sizeof(T_DjiFcSubscriptionVelocity),
                                                      &rt_VelocityTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic real-time velocity  error, error code: 0x%08X", djiStat);
    } 

    return rt_Velocity;
}

T_DjiFcSubscriptionWholeBatteryInfo DjiTest_FlightControlGetValueOfBattery(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionWholeBatteryInfo battInfo = {0};
    T_DjiDataTimestamp battInfoTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_BATTERY_INFO,
                                                      (uint8_t *) &battInfo,
                                                      sizeof(T_DjiFcSubscriptionWholeBatteryInfo),
                                                      &battInfoTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic whole battery info error, error code: 0x%08X", djiStat);
    } 

    return battInfo;
}

T_DjiFcSubscriptionEscData DjiTest_FlightControlGetValueOfESCData(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionEscData  ESC_all = {0};
    T_DjiDataTimestamp ESC_allTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ESC_DATA,
                                                      (uint8_t *) &ESC_all,
                                                      sizeof(T_DjiFcSubscriptionEscData),
                                                      &ESC_allTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic whole battery info error, error code: 0x%08X", djiStat);
    } 

    return ESC_all;
}

T_DjiFcSubscriptionRCWithFlagData DjiTest_FlightControlGetValueOfRC(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionRCWithFlagData  RC_all = {0};
    T_DjiDataTimestamp RC_allTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA,
                                                      (uint8_t *) &RC_all,
                                                      sizeof(T_DjiFcSubscriptionRCWithFlagData),
                                                      &RC_allTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic whole battery info error, error code: 0x%08X", djiStat);
    } 

    return RC_all;
}

dji_f32_t DjiTest_FlightControlGetValueOfRelativeHeight(void)
{
    T_DjiReturnCode djiStat;
    T_DjiFcSubscriptionAltitudeFused altitudeFused = 0;
    T_DjiFcSubscriptionAltitudeOfHomePoint homePointAltitude = 0;
    dji_f32_t relativeHeight = 0;
    T_DjiDataTimestamp relativeHeightTimestamp = {0};

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_OF_HOMEPOINT,
                                                      (uint8_t *) &homePointAltitude,
                                                      sizeof(T_DjiFcSubscriptionAltitudeOfHomePoint),
                                                      &relativeHeightTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic altitude of home point error, error code: 0x%08X", djiStat);
        return -1;
    } 

    djiStat = DjiFcSubscription_GetLatestValueOfTopic(DJI_FC_SUBSCRIPTION_TOPIC_ALTITUDE_FUSED,
                                                      (uint8_t *) &altitudeFused,
                                                      sizeof(T_DjiFcSubscriptionAltitudeFused),
                                                      &relativeHeightTimestamp);

    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Get value of topic altitude fused error, error code: 0x%08X", djiStat);
        return -1;
    } 
    relativeHeight = altitudeFused - homePointAltitude;

    return relativeHeight;
}

bool DjiTest_FlightControlMotorStartedCheck(void)
{
    int motorsNotStarted = 0;
    int timeoutCycles = 20;

    while (DjiTest_FlightControlGetValueOfFlightStatus() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_ON_GROUND &&
           DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ENGINE_START &&
           motorsNotStarted < timeoutCycles) {
        motorsNotStarted++;
        s_osalHandler->TaskSleepMs(100);
    }
    return motorsNotStarted != timeoutCycles ? true : false;
}

bool DjiTest_FlightControlTakeOffInAirCheck(void)
{
    int stillOnGround = 0;
    int timeoutCycles = 110;

    while (DjiTest_FlightControlGetValueOfFlightStatus() != DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
           (DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF ||
            DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF) &&
           stillOnGround < timeoutCycles) {
        stillOnGround++;
        s_osalHandler->TaskSleepMs(100);
    }

    return stillOnGround != timeoutCycles ? true : false;
}

bool takeoffFinishedCheck(void)
{
    while (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_TAKEOFF ||
           DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ASSISTED_TAKEOFF) {
        s_osalHandler->TaskSleepMs(1000);
    }

    return (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
            DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) ? true : false;
}

bool DjiTest_FlightControlLandFinishedCheck(void)
{
    while (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING ||
           DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
        s_osalHandler->TaskSleepMs(1000);
    }

    return (DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
            DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) ? true : false;
}

bool DjiTest_FlightControlCheckActionStarted(E_DjiFcSubscriptionDisplayMode mode)
{
    int actionNotStarted = 0;
    int timeoutCycles = 20;

    while (DjiTest_FlightControlGetValueOfDisplayMode() != mode && actionNotStarted < timeoutCycles) {
        actionNotStarted++;
        s_osalHandler->TaskSleepMs(100);
    }

    if (actionNotStarted == timeoutCycles) {
        USER_LOG_ERROR("%s start failed, now flight is in %s.",
                       s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(mode)].displayModeStr,
                       s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(
                           DjiTest_FlightControlGetValueOfDisplayMode())].displayModeStr);
        return false;
    } else {
        USER_LOG_INFO("Now flight is in %s.",
                      s_flightControlDisplayModeStr[DjiTest_FlightControlGetDisplayModeIndex(mode)].displayModeStr);
        return true;
    }
}

bool DjiTest_FlightControlMonitoredTakeoff(void)
{
    T_DjiReturnCode djiStat;

    //! Start takeoff
    djiStat = DjiFlightController_StartTakeoff();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Request to take off failed, error code: 0x%08X", djiStat);
        return false;
    }

    //! Motors start check
    if (!DjiTest_FlightControlMotorStartedCheck()) {
        USER_LOG_ERROR("Takeoff failed. Motors are not spinning.");
        return false;
    } else {
        USER_LOG_INFO("Motors spinning...");
    }
    //! In air check
    if (!DjiTest_FlightControlTakeOffInAirCheck()) {
        USER_LOG_ERROR("Takeoff failed. Aircraft is still on the ground, but the "
                       "motors are spinning");
        return false;
    } else {
        USER_LOG_INFO("Ascending...");
    }
    //! Finished takeoff check
    if (!takeoffFinishedCheck()) {
        USER_LOG_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI GO.");
        return false;
    }

    return true;
}

bool DjiTest_FlightControlMonitoredLanding(void)
{
    T_DjiReturnCode djiStat;
    /*! Step 1: Start landing */
    djiStat = DjiFlightController_StartLanding();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start landing failed, error code: 0x%08X", djiStat);
        return false;
    }

    /*! Step 2: check Landing start*/
    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)) {
        USER_LOG_ERROR("Fail to execute Landing action!");
        return false;
    } else {
        /*! Step 3: check Landing finished*/
        if (!DjiTest_FlightControlLandFinishedCheck()) {
            USER_LOG_ERROR("Landing finished, but the aircraft is in an unexpected mode. "
                           "Please connect DJI Assistant.");
            return false;
        }
    }

    return true;
}

bool DjiTest_FlightControlGoHomeAndConfirmLanding(void)
{
    T_DjiReturnCode djiStat;
    T_DjiAircraftInfoBaseInfo aircraftInfoBaseInfo;
    E_DjiFlightControllerObstacleAvoidanceEnableStatus enableStatus;

    djiStat = DjiFlightController_GetDownwardsVisualObstacleAvoidanceEnableStatus(&enableStatus);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get downwards visual obstacle avoidance enable status error");
    }

    djiStat = DjiAircraftInfo_GetBaseInfo(&aircraftInfoBaseInfo);
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("get aircraft base info error");
    }

    /*! Step 1: Start go home */
    USER_LOG_INFO("Start go home action");
    djiStat = DjiFlightController_StartGoHome();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start to go home failed, error code: 0x%08X", djiStat);
        return false;;
    }

    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME)) {
        return false;
    } else {
        while (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
               DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_NAVI_GO_HOME) {
            s_osalHandler->TaskSleepMs(1000);// waiting for this action finished
        }
    }

    /*! Step 2: Start landing */
    USER_LOG_INFO("Start landing action");

    /*! bug fix, add start landing action */
    djiStat = DjiFlightController_StartLanding();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Start landing failed, error code: 0x%08X", djiStat);
        return false;
    }

    if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING)) {
        USER_LOG_ERROR("Fail to execute Landing action");
        return false;
    } else {
        while (DjiTest_FlightControlGetValueOfDisplayMode() == DJI_FC_SUBSCRIPTION_DISPLAY_MODE_AUTO_LANDING &&
               DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR) {
            T_DjiFcSubscriptionHeightFusion heightFusion = DjiTest_FlightControlGetValueOfHeightFusion();
            s_osalHandler->TaskSleepMs(1000);
            if (aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3E ||
                aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3T ||
                aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3D ||
                aircraftInfoBaseInfo.aircraftType == DJI_AIRCRAFT_TYPE_M3TD) {
                if ((dji_f64_t) 0.45 < heightFusion && heightFusion < (dji_f64_t) 0.55) {
                    break;
                }
            } else {
                if ((dji_f64_t) 0.65 < heightFusion && heightFusion < (dji_f64_t) 0.75) {
                    break;
                }
            }
        }
    }

    /*! Step 4: Confirm Landing */
    USER_LOG_INFO("Start confirm Landing and avoid ground action");
    djiStat = DjiFlightController_StartConfirmLanding();
    if (djiStat != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Fail to execute confirm landing avoid ground action, error code: 0x%08X", djiStat);
        return false;
    }

    if (enableStatus == DJI_FLIGHT_CONTROLLER_ENABLE_OBSTACLE_AVOIDANCE) {
        if (!DjiTest_FlightControlCheckActionStarted(DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING)) {
            return false;
        } else {
            while (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
                   DjiTest_FlightControlGetValueOfDisplayMode() ==
                   DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING) {
                s_osalHandler->TaskSleepMs(1000);
            }
        }
    } else {
        while (DjiTest_FlightControlGetValueOfFlightStatus() == DJI_FC_SUBSCRIPTION_FLIGHT_STATUS_IN_AIR &&
               DjiTest_FlightControlGetValueOfDisplayMode() ==
               DJI_FC_SUBSCRIPTION_DISPLAY_MODE_FORCE_AUTO_LANDING) {
            s_osalHandler->TaskSleepMs(1000);
        }
    }

    /*! Step 5: Landing finished check*/
    if (DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_P_GPS ||
        DjiTest_FlightControlGetValueOfDisplayMode() != DJI_FC_SUBSCRIPTION_DISPLAY_MODE_ATTITUDE) {
        USER_LOG_INFO("Successful landing");
    } else {
        USER_LOG_ERROR("Landing finished, but the aircraft is in an unexpected mode. "
                       "Please connect DJI Assistant.");
        return false;
    }

    return true;
}

T_DjiTestFlightControlVector3f DjiTest_FlightControlQuaternionToEulerAngle(const T_DjiFcSubscriptionQuaternion quat)
{
    T_DjiTestFlightControlVector3f eulerAngle;
    double q2sqr = quat.q2 * quat.q2;
    double t0 = -2.0 * (q2sqr + quat.q3 * quat.q3) + 1.0;
    double t1 = (dji_f64_t) 2.0 * (quat.q1 * quat.q2 + quat.q0 * quat.q3);
    double t2 = -2.0 * (quat.q1 * quat.q3 - quat.q0 * quat.q2);
    double t3 = (dji_f64_t) 2.0 * (quat.q2 * quat.q3 + quat.q0 * quat.q1);
    double t4 = -2.0 * (quat.q1 * quat.q1 + q2sqr) + 1.0;
    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;
    eulerAngle.x = asin(t2);
    eulerAngle.y = atan2(t3, t4);
    eulerAngle.z = atan2(t1, t0);
    return eulerAngle;
}

T_DjiTestFlightControlVector3f
DjiTest_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(const T_DjiFcSubscriptionPositionFused target,
                                                            const T_DjiFcSubscriptionPositionFused origin,
                                                            const dji_f32_t targetHeight,
                                                            const dji_f32_t originHeight)
{
    T_DjiTestFlightControlVector3f deltaNed;
    double deltaLon = target.longitude - origin.longitude;
    double deltaLat = target.latitude - origin.latitude;
    deltaNed.x = deltaLat * s_earthCenter;
    deltaNed.y = deltaLon * s_earthCenter * cos(target.latitude);
    deltaNed.z = targetHeight - originHeight;

    return deltaNed;
}

T_DjiTestFlightControlVector3f DjiTest_FlightControlVector3FSub(const T_DjiTestFlightControlVector3f vectorA,
                                                                const T_DjiTestFlightControlVector3f vectorB)
{
    T_DjiTestFlightControlVector3f result;
    result.x = vectorA.x - vectorB.x;
    result.y = vectorA.y - vectorB.y;
    result.z = vectorA.z - vectorB.z;
    return result;
}

int DjiTest_FlightControlSignOfData(dji_f32_t data)
{
    return data < 0 ? -1 : 1;
}

void DjiTest_FlightControlHorizCommandLimit(dji_f32_t speedFactor, dji_f32_t *commandX, dji_f32_t *commandY)
{
    if (fabs(*commandX) > speedFactor)
        *commandX = speedFactor * DjiTest_FlightControlSignOfData(*commandX);
    if (fabs(*commandY) > speedFactor)
        *commandY = speedFactor * DjiTest_FlightControlSignOfData(*commandY);
}

dji_f32_t DjiTest_FlightControlVectorNorm(T_DjiTestFlightControlVector3f v)
{
    return sqrt(pow(v.x, 2) + pow(v.y, 2) + pow(v.z, 2));
}

bool
DjiTest_FlightControlMoveByPositionOffset(const T_DjiTestFlightControlVector3f offsetDesired, float yawDesiredInDeg,
                                          float posThresholdInM, float yawThresholdInDeg)
{
    int timeoutInMilSec = 20000;
    int controlFreqInHz = 50;  // Hz
    int cycleTimeInMs = 1000 / controlFreqInHz;
    int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;    // 10 cycles
    int withinControlBoundsTimeReqmt = 100 * cycleTimeInMs;  // 100 cycles
    int elapsedTimeInMs = 0;
    int withinBoundsCounter = 0;
    int outOfBounds = 0;
    int brakeCounter = 0;
    int speedFactor = 2;

    //! get origin position and relative height(from home point)of aircraft.
    T_DjiFcSubscriptionPositionFused originGPSPosition = DjiTest_FlightControlGetValueOfPositionFused();
    dji_f32_t originHeightBaseHomePoint = DjiTest_FlightControlGetValueOfRelativeHeight();
    if (originHeightBaseHomePoint == -1) {
        USER_LOG_ERROR("Relative height is invalid!");
        return false;
    }

    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };
    DjiFlightController_SetJoystickMode(joystickMode);

    while (elapsedTimeInMs < timeoutInMilSec) {
        T_DjiFcSubscriptionPositionFused currentGPSPosition = DjiTest_FlightControlGetValueOfPositionFused();
        T_DjiFcSubscriptionQuaternion currentQuaternion = DjiTest_FlightControlGetValueOfQuaternion();
        dji_f32_t currentHeight = DjiTest_FlightControlGetValueOfRelativeHeight();
        if (originHeightBaseHomePoint == -1) {
            USER_LOG_ERROR("Relative height is invalid!");
            return false;
        }

        float yawInRad = DjiTest_FlightControlQuaternionToEulerAngle(currentQuaternion).z;
        //! get the vector between aircraft and origin point.

        T_DjiTestFlightControlVector3f localOffset = DjiTest_FlightControlLocalOffsetFromGpsAndFusedHeightOffset(
            currentGPSPosition,
            originGPSPosition,
            currentHeight,
            originHeightBaseHomePoint);
        //! get the vector between aircraft and target point.
        T_DjiTestFlightControlVector3f offsetRemaining = DjiTest_FlightControlVector3FSub(offsetDesired, localOffset);

        T_DjiTestFlightControlVector3f positionCommand = offsetRemaining;
        DjiTest_FlightControlHorizCommandLimit(speedFactor, &positionCommand.x, &positionCommand.y);

        T_DjiFlightControllerJoystickCommand joystickCommand = {positionCommand.x, positionCommand.y,
                                                                offsetDesired.z + originHeightBaseHomePoint,
                                                                yawDesiredInDeg};
        DjiFlightController_ExecuteJoystickAction(joystickCommand);

        if (DjiTest_FlightControlVectorNorm(offsetRemaining) < posThresholdInM &&
            fabs(yawInRad / s_degToRad - yawDesiredInDeg) < yawThresholdInDeg) {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
        } else {
            if (withinBoundsCounter != 0) {
                //! 2. Start incrementing an out-of-bounds counter
                outOfBounds += cycleTimeInMs;
            }
        }
        //! 3. Reset withinBoundsCounter if necessary
        if (outOfBounds > outOfControlBoundsTimeLimit) {
            withinBoundsCounter = 0;
            outOfBounds = 0;
        }
        //! 4. If within bounds, set flag and break
        if (withinBoundsCounter >= withinControlBoundsTimeReqmt) {
            break;
        }
        s_osalHandler->TaskSleepMs(cycleTimeInMs);
        elapsedTimeInMs += cycleTimeInMs;
    }

    while (brakeCounter < withinControlBoundsTimeReqmt) {
        s_osalHandler->TaskSleepMs(cycleTimeInMs);
        brakeCounter += cycleTimeInMs;
    }

    if (elapsedTimeInMs >= timeoutInMilSec) {
        USER_LOG_ERROR("Task timeout!");
        return false;
    }

    return true;
}

void DjiTest_FlightControlVelocityAndYawRateCtrl(const T_DjiTestFlightControlVector3f offsetDesired, float yawRate,
                                                 uint32_t timeMs)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    uint32_t originTime = 0;
    uint32_t currentTime = 0;
    uint32_t elapsedTimeInMs = 0;
    osalHandler->GetTimeMs(&originTime);
    osalHandler->GetTimeMs(&currentTime);
    elapsedTimeInMs = currentTime - originTime;
    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };

    DjiFlightController_SetJoystickMode(joystickMode);
    T_DjiFlightControllerJoystickCommand joystickCommand = {offsetDesired.x, offsetDesired.y, offsetDesired.z,
                                                            yawRate};

    while (elapsedTimeInMs <= timeMs) {
        DjiFlightController_ExecuteJoystickAction(joystickCommand);
        osalHandler->TaskSleepMs(2);
        osalHandler->GetTimeMs(&currentTime);
        elapsedTimeInMs = currentTime - originTime;
    }
}

void SAV_ControlVelocity_Yawrate_BodyCoord(const T_DjiTestFlightControlVector3f offsetDesired, float yawRate,
                                                 uint32_t timeMs)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    uint32_t originTime = 0;
    uint32_t currentTime = 0;
    uint32_t elapsedTimeInMs = 0;
    osalHandler->GetTimeMs(&originTime);
    osalHandler->GetTimeMs(&currentTime);
    elapsedTimeInMs = currentTime - originTime;
    
    /*
    Mode chosen reasons based on assumption before field test:
    1. For horizontal and vertical movement in measurement phase, velocity control could be better than position control.
       Patch position is detected, following speed could be adjusted by doing Desired_vel = (Desired_pos - Current_pos)*PID.
       Apply limits to Desired_vel as [-0.15m/s, 0.15m/s]. Apply attitude check as (if pitch > 10°, brake slowly).
       Attitude control could be faster! However could be more easy to get drift by wind.
       Thus velocity control + vel bounds + attitude limit check. 
    2. Yaw angle use rate control, wouldn't exerce extra force when yaw drfited by wind.
    3. Use Body frame as it's very close to the blade surface, target position is always given relatively.
    */

    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE, 
        DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_BODY_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };

    DjiFlightController_SetJoystickMode(joystickMode);
    T_DjiFlightControllerJoystickCommand joystickCommand = {offsetDesired.x, offsetDesired.y, offsetDesired.z,
                                                            yawRate};

    while (elapsedTimeInMs <= timeMs) {
        DjiFlightController_ExecuteJoystickAction(joystickCommand);
        osalHandler->TaskSleepMs(2);
        osalHandler->GetTimeMs(&currentTime);
        elapsedTimeInMs = currentTime - originTime;
    }
}

T_DjiReturnCode
DjiTest_FlightControlJoystickCtrlAuthSwitchEventCallback(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData)
{
    switch (eventData.joystickCtrlAuthoritySwitchEvent) {
        case DJI_FLIGHT_CONTROLLER_MSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_MSDK) {
                USER_LOG_INFO("[Event]Msdk request to obtain joystick ctrl authority\r\n");
            } else {
                USER_LOG_INFO("[Event]Msdk request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_INTERNAL_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_INTERNAL) {
                USER_LOG_INFO("[Event]Internal request to obtain joystick ctrl authority\r\n");
            } else {
                USER_LOG_INFO("[Event]Internal request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_OSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (eventData.curJoystickCtrlAuthority == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK) {
                USER_LOG_INFO("[Event] Request to obtain joystick ctrl authority\r\n");
            } else {
                USER_LOG_INFO("[Event] Request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_RC_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc lost\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_NOT_P_MODE_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc for rc is not in P mode\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_SWITCH_MODE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc switching mode\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_PAUSE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc pausing\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_REQUEST_GO_HOME_GET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to rc request for return\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_GO_HOME_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc for low battery return\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_LANDING_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc for low battery land\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_OSDK_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT:
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to sdk lost\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_NERA_FLIGHT_BOUNDARY_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            USER_LOG_INFO("[Event]Current joystick ctrl authority is reset to rc due to near boundary\r\n");
            break;
        default:
            USER_LOG_INFO("[Event]Unknown joystick ctrl authority event\r\n");
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

void* logger_loop(void* arg)
{
    dji_f64_t pitch, yaw, roll;

    TEST* data = (TEST *)getQuaternionaddress();

   while(1){

    pitch = (dji_f64_t) asinf(-2 * data->quaternion.q1 * data->quaternion.q3 + 2 * data->quaternion.q0 * data->quaternion.q2) * 57.3;
    roll = (dji_f64_t) atan2f(2 * data->quaternion.q2 * data->quaternion.q3 + 2 * data->quaternion.q0 * data->quaternion.q1,
                             -2 * data->quaternion.q1 * data->quaternion.q1 - 2 * data->quaternion.q2 * data->quaternion.q2 + 1) *57.3;
    yaw = (dji_f64_t) atan2f(2 * data->quaternion.q1 * data->quaternion.q2 + 2 * data->quaternion.q0 * data->quaternion.q3,
                             -2 * data->quaternion.q2 * data->quaternion.q2 - 2 * data->quaternion.q3 * data->quaternion.q3 + 1) *57.3;

    USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", data->timestamp.millisecond,
                          data->timestamp.microsecond);
    USER_LOG_INFO("euler angles: pitch = %.2f roll = %.2f yaw = %.2f.\r\n", pitch, roll, yaw);


            
            
    usleep(10000);
   }

}
void* fcontrol_loop(void* arg)
{
    
    dji_f64_t pitch, yaw, roll;

    TEST* data = (TEST *)getQuaternionaddress();

   while(1){

    pitch = (dji_f64_t) asinf(-2 * data->quaternion.q1 * data->quaternion.q3 + 2 * data->quaternion.q0 * data->quaternion.q2) * 57.3;
    roll = (dji_f64_t) atan2f(2 * data->quaternion.q2 * data->quaternion.q3 + 2 * data->quaternion.q0 * data->quaternion.q1,
                             -2 * data->quaternion.q1 * data->quaternion.q1 - 2 * data->quaternion.q2 * data->quaternion.q2 + 1) * 57.3;
    yaw = (dji_f64_t) atan2f(2 * data->quaternion.q1 * data->quaternion.q2 + 2 * data->quaternion.q0 * data->quaternion.q3,
                             -2 * data->quaternion.q2 * data->quaternion.q2 - 2 * data->quaternion.q3 * data->quaternion.q3 + 1) *
          57.3;

            printf("quaternion: %f %f %f %f.\n", data->quaternion.q0, data->quaternion.q1, data->quaternion.q2,
                          data->quaternion.q3);
            printf("timestamp: millisecond %u microsecond %u.\n", data->timestamp.millisecond,
                          data->timestamp.microsecond);

            printf("euler angles: pitch = %.2f roll = %.2f yaw = %.2f.\n", pitch, roll, yaw);

            usleep(100000);


   }

}

/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
