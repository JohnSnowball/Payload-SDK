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


static void *Quaternion_data_address = NULL;
static void *Acceleration_data_address = NULL;
static void *Position_data_address = NULL;
static void *Velocity_data_address = NULL;
static void *RC_stick_data_address = NULL;




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
        s_osalHandler->TaskSleepMs(1000);
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


    returnCode = All_Topic_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Error during data subscription, error code:0x%08llX", returnCode);
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

T_DjiReturnCode All_Topic_Init(void)
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

    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_QUATERNION, DJI_DATA_SUBSCRIPTION_TOPIC_100_HZ,
                                               Sav_Quaternion_data_Callback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe Quaternion failed, error code:0x%08llX", returnCode);
        return returnCode;
    }
    Quaternion_data_address = malloc(sizeof(quatrenion_data_node));


    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_ACCELERATION_RAW, DJI_DATA_SUBSCRIPTION_TOPIC_200_HZ,
                                               Sav_Acceleration_data_Callback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe Acceleration failed, error code:0x%08llX", returnCode);
        return returnCode;
    }
    Acceleration_data_address = malloc(sizeof(acceleration_data_node));

    //position, lat/lon/alt/sat number, 50Hz
    //position data is strongly base on GPS signal, everytime we use this should check if GPS signal is good, satellite number > 12
    //need to do: always enable RTK for M350!!!
    /* E_DjiFlightControllerRtkPositionEnableStatus == 1*/
    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_POSITION_FUSED, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               Sav_Position_data_Callback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe Position failed, error code:0x%08llX", returnCode);
        return returnCode;
    }
    Position_data_address = malloc(sizeof(position_data_node));


    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_VELOCITY, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               Sav_Velocity_data_Callback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe Velocity failed, error code:0x%08llX", returnCode);
        return returnCode;
    }
    Velocity_data_address = malloc(sizeof(velocity_data_node));


    returnCode = DjiFcSubscription_SubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RC_WITH_FLAG_DATA, DJI_DATA_SUBSCRIPTION_TOPIC_50_HZ,
                                               Sav_RC_stick_data_Callback);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        USER_LOG_ERROR("Subscribe RC sticks failed, error code:0x%08llX", returnCode);
        return returnCode;
    }
    RC_stick_data_address = malloc(sizeof(rc_stick_data_node));


    return returnCode;

}

void* get_Quaternion_data_address(void)
{
    return Quaternion_data_address;
}

void* get_Acceleration_data_address(void)
{
    return Acceleration_data_address;
}

void* get_Position_data_address(void)
{
    return Position_data_address;
}

void* get_Velocity_data_address(void)
{
    return Velocity_data_address;
}

void* get_RC_stick_data_address(void)
{
    return RC_stick_data_address;
}

static T_DjiReturnCode Sav_Quaternion_data_Callback(const uint8_t *data, uint16_t dataSize,const T_DjiDataTimestamp *timestamp)
{
    memcpy(&((quatrenion_data_node*)Quaternion_data_address)->quaternion,data,dataSize);
    memcpy(&((quatrenion_data_node*)Quaternion_data_address)->timestamp,timestamp,sizeof(T_DjiDataTimestamp));

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

static T_DjiReturnCode Sav_Acceleration_data_Callback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    memcpy(&((acceleration_data_node*)Acceleration_data_address)->acc,data,dataSize);
    memcpy(&((acceleration_data_node*)Acceleration_data_address)->timestamp,timestamp,sizeof(T_DjiDataTimestamp));

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

}

static T_DjiReturnCode Sav_Position_data_Callback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    memcpy(&((position_data_node*)Position_data_address)->pos,data,dataSize);
    memcpy(&((position_data_node*)Position_data_address)->timestamp,timestamp,sizeof(T_DjiDataTimestamp));

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

}


static T_DjiReturnCode Sav_Velocity_data_Callback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    memcpy(&((velocity_data_node*)Velocity_data_address)->vel,data,dataSize);
    memcpy(&((velocity_data_node*)Velocity_data_address)->timestamp,timestamp,sizeof(T_DjiDataTimestamp));

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}


static T_DjiReturnCode Sav_RC_stick_data_Callback(const uint8_t *data, uint16_t dataSize, const T_DjiDataTimestamp *timestamp)
{
    memcpy(&((rc_stick_data_node*)RC_stick_data_address)->rc,data,dataSize);
    memcpy(&((rc_stick_data_node*)RC_stick_data_address)->timestamp,timestamp,sizeof(T_DjiDataTimestamp));

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;

}

void* logger_loop(void* arg)
{
    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    uint32_t elapsedTimeInMs = 0;
    uint32_t loopstartTimeInMs = 0;
    uint32_t loopendTimeInMs = 0;
    uint32_t iteration_times = 0;
    
    
    dji_f64_t pitch, yaw, roll;
    T_DjiFcSubscriptionAccelerationRaw acc_raw;
    T_DjiFcSubscriptionPositionFused pos_fused;
    T_DjiFcSubscriptionVelocity vel_fused;
    T_DjiFcSubscriptionRCWithFlagData rc_sticks_flag;

    quatrenion_data_node* quat_pack = (quatrenion_data_node*)get_Quaternion_data_address();
    acceleration_data_node* acc_pack = (acceleration_data_node*)get_Acceleration_data_address();
    position_data_node* pos_pack = (position_data_node*)get_Position_data_address();
    velocity_data_node* vel_pack = (velocity_data_node*)get_Velocity_data_address();
    rc_stick_data_node* rc_pack = (rc_stick_data_node*)get_RC_stick_data_address();

    osalHandler->TaskSleepMs(1000);//wait for 1 sec for initialization


    while(1){

        osalHandler->GetTimeMs(&loopstartTimeInMs);
        

        pitch = (dji_f64_t) asinf(-2 * quat_pack->quaternion.q1 * quat_pack->quaternion.q3 + 2 * quat_pack->quaternion.q0 * quat_pack->quaternion.q2) * 57.3;
        roll = (dji_f64_t) atan2f(2 * quat_pack->quaternion.q2 * quat_pack->quaternion.q3 + 2 * quat_pack->quaternion.q0 * quat_pack->quaternion.q1,
                                -2 * quat_pack->quaternion.q1 * quat_pack->quaternion.q1 - 2 * quat_pack->quaternion.q2 * quat_pack->quaternion.q2 + 1) *57.3;
        yaw = (dji_f64_t) atan2f(2 * quat_pack->quaternion.q1 * quat_pack->quaternion.q2 + 2 * quat_pack->quaternion.q0 * quat_pack->quaternion.q3,
                                -2 * quat_pack->quaternion.q2 * quat_pack->quaternion.q2 - 2 * quat_pack->quaternion.q3 * quat_pack->quaternion.q3 + 1) *57.3;

        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", quat_pack->timestamp.millisecond, quat_pack->timestamp.microsecond);
        USER_LOG_INFO("attitude: pitch = %.2f roll = %.2f yaw = %.2f.\r\n", pitch, roll, yaw);


        acc_raw.x = acc_pack->acc.x;
        acc_raw.y = acc_pack->acc.y;
        acc_raw.z = acc_pack->acc.z;
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", acc_pack->timestamp.millisecond, acc_pack->timestamp.microsecond);
        USER_LOG_INFO("acceleration: accx = %.3f accy = %.3f accz = %.3f.\r\n", acc_raw.x, acc_raw.y, acc_raw.z);



        pos_fused.latitude = pos_pack->pos.latitude;
        pos_fused.longitude = pos_pack->pos.longitude;
        pos_fused.altitude = pos_pack->pos.altitude;
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", pos_pack->timestamp.millisecond, pos_pack->timestamp.microsecond);
        USER_LOG_INFO("position: lat = %.6f lon = %.6f alt = %.6f.\r\n", pos_fused.latitude, pos_fused.longitude, pos_fused.altitude);


        vel_fused.data.x = vel_pack->vel.data.x;
        vel_fused.data.y = vel_pack->vel.data.y;
        vel_fused.data.z = vel_pack->vel.data.z;
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", vel_pack->timestamp.millisecond, vel_pack->timestamp.microsecond);
        USER_LOG_INFO("velocity: velx = %.3f vely = %.3f velz = %.3f.\r\n", vel_fused.data.x, vel_fused.data.y, vel_fused.data.z);

        rc_sticks_flag.pitch = rc_pack->rc.pitch;
        rc_sticks_flag.roll = rc_pack->rc.roll;
        rc_sticks_flag.yaw = rc_pack->rc.yaw;
        rc_sticks_flag.throttle = rc_pack->rc.throttle;
        rc_sticks_flag.flag.groundConnected = rc_pack->rc.flag.groundConnected;
        rc_sticks_flag.flag.skyConnected = rc_pack->rc.flag.skyConnected;
        rc_sticks_flag.flag.logicConnected = rc_pack->rc.flag.logicConnected;
        USER_LOG_INFO("timestamp: millisecond %u microsecond %u.", rc_pack->timestamp.millisecond, rc_pack->timestamp.microsecond);
        USER_LOG_INFO("RC stick: RCpitch = %.2f RCroll = %.2f RCyaw = %.2f RCthr = %.2f connected = %d.\r\n", 
                            rc_sticks_flag.pitch, rc_sticks_flag.roll, rc_sticks_flag.yaw, rc_sticks_flag.throttle, rc_sticks_flag.flag.logicConnected);


        iteration_times +=1;
        osalHandler->GetTimeMs(&loopendTimeInMs);
        elapsedTimeInMs += loopendTimeInMs - loopstartTimeInMs;

        if(iteration_times%100 == 1){
            USER_LOG_INFO("100 logger loops take %u microsecond", elapsedTimeInMs);
        }

        osalHandler->TaskSleepMs(20);// how to eliminate repeated points in the log?
    }

}
void* fcontrol_loop(void* arg)
{
    /*
    todo: make all these control steps in a big loop and check in each loop if the pilot wants to takeover control authority.
        while((!mission_done or !RC_take_authority or !strong_wind)&&slower_than_50Hz)
        {
            1. update drone status;

            2. check if:
                1. attitude& velocity& throttle exceeded bounds --> (strong_wind == true) and break;
                2. RC takes over control --> (RC_take_authority == true) and break;
                3. acceleration check --> contacted == true;
                4. start a single measurement command by pilot --> start_single_measure == true;
                5. Done command from pilot --> (mission_done == true) and break;
                6. Check if Joystick is still in control or disrupted by pilot, JoystickCtrlAuthorityEventInfo;
                7. Re-enter or not initialized?

                if break:
                first ExecuteEmergencyBrakeAction; for 0.x sec?
                second fly backwards slowly, for x sec?
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

    T_DjiReturnCode returnCode;

    T_DjiOsalHandler *osalHandler = DjiPlatform_GetOsalHandler();
    uint32_t elapsedTimeInMs = 0;
    uint32_t loopstartTimeInMs = 0;
    uint32_t loopendTimeInMs = 0;
    uint32_t iteration_times = 0;


    dji_f64_t pitch, yaw, roll;
    T_DjiFcSubscriptionAccelerationRaw acc_raw, acc_average_filtered;
    uint8_t is_acc_filter_initialized = false;
    uint8_t is_joystick_auth_obtained = false;
    uint8_t is_takeoff_finished = false;
    uint8_t initialized_counter = 0;
    uint8_t flag_contacted_maybe = false;
    uint8_t flag_drone_blade_in_contact = false; 
    uint8_t flag_break_by_RC = false;
    uint8_t maybe_contact_counter = 0;
    T_DjiFcSubscriptionPositionFused pos_fused;
    T_DjiFcSubscriptionVelocity vel_fused;
    T_DjiFcSubscriptionRCWithFlagData rc_sticks_flag;


    quatrenion_data_node* quat_pack = (quatrenion_data_node*)get_Quaternion_data_address();
    acceleration_data_node* acc_pack = (acceleration_data_node*)get_Acceleration_data_address();
    position_data_node* pos_pack = (position_data_node*)get_Position_data_address();
    velocity_data_node* vel_pack = (velocity_data_node*)get_Velocity_data_address();
    rc_stick_data_node* rc_pack = (rc_stick_data_node*)get_RC_stick_data_address();


    T_DjiFlightControllerJoystickMode joystickMode = {
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_VERTICAL_POSITION_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_YAW_ANGLE_CONTROL_MODE,
        DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
        DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
    };//default fly mode


    osalHandler->TaskSleepMs(1000);//wait for 1 sec for initialization


    while(1){

        osalHandler->GetTimeMs(&loopstartTimeInMs);

        pitch = (dji_f64_t) asinf(-2 * quat_pack->quaternion.q1 * quat_pack->quaternion.q3 + 2 * quat_pack->quaternion.q0 * quat_pack->quaternion.q2) * 57.3;
        roll = (dji_f64_t) atan2f(2 * quat_pack->quaternion.q2 * quat_pack->quaternion.q3 + 2 * quat_pack->quaternion.q0 * quat_pack->quaternion.q1,
                                -2 * quat_pack->quaternion.q1 * quat_pack->quaternion.q1 - 2 * quat_pack->quaternion.q2 * quat_pack->quaternion.q2 + 1) * 57.3;
        yaw = (dji_f64_t) atan2f(2 * quat_pack->quaternion.q1 * quat_pack->quaternion.q2 + 2 * quat_pack->quaternion.q0 * quat_pack->quaternion.q3,
                                -2 * quat_pack->quaternion.q2 * quat_pack->quaternion.q2 - 2 * quat_pack->quaternion.q3 * quat_pack->quaternion.q3 + 1) * 57.3;


        acc_raw.x = acc_pack->acc.x;
        acc_raw.y = acc_pack->acc.y;
        acc_raw.z = acc_pack->acc.z;


        /* Determine if contacted, filter coefficients to be determined with precise log data*/
        if(!is_acc_filter_initialized){
            acc_average_filtered = acc_raw;
            initialized_counter += 1;

            if(initialized_counter >= 100){
                is_acc_filter_initialized = true;
                initialized_counter = 0;
            }

        }else{
            /*how to differ normal accelerating or turbulence or contact? Vibration characteristics: duration & amplitude & direction.
              other than an average filter, a SVM(supported vector machine) could be a good idea to make classification.*/
            if((fabs(acc_raw.x - acc_average_filtered.x) > 0.01f ) || (fabs(acc_raw.y - acc_average_filtered.y) > 0.01f ) 
                ||(fabs(acc_raw.z - acc_average_filtered.z) > 0.01f )){
            
                flag_contacted_maybe = true;
                maybe_contact_counter += 2;//increase faster?
            }
            else if(flag_contacted_maybe){
                
                if(maybe_contact_counter <=1){
                    flag_contacted_maybe = false; 
                    flag_drone_blade_in_contact = false;
                }else{
                    maybe_contact_counter -= 1;//recover slowly?
                }
            }
            else{
            
            }

            if(!flag_drone_blade_in_contact && flag_contacted_maybe && maybe_contact_counter >= 200){
                flag_drone_blade_in_contact = true;
                USER_LOG_INFO("contacted: millisecond %u microsecond %u.", acc_pack->timestamp.millisecond, acc_pack->timestamp.microsecond);
            }
                              
            acc_average_filtered.x = 0.05f*acc_raw.x + 0.95f*acc_average_filtered.x;
            acc_average_filtered.y = 0.05f*acc_raw.y + 0.95f*acc_average_filtered.y;
            acc_average_filtered.z = 0.05f*acc_raw.z + 0.95f*acc_average_filtered.z;
            USER_LOG_INFO("timestamp: millisecond %u.", loopstartTimeInMs);
            USER_LOG_INFO("filtered acc: accx = %.3f accy = %.3f accz = %.3f.\r\n", acc_average_filtered.x, acc_average_filtered.y, acc_average_filtered.z);
        }
        

        pos_fused.latitude = pos_pack->pos.latitude;
        pos_fused.longitude = pos_pack->pos.longitude;
        pos_fused.altitude = pos_pack->pos.altitude;


        vel_fused.data.x = vel_pack->vel.data.x;
        vel_fused.data.y = vel_pack->vel.data.y;
        vel_fused.data.z = vel_pack->vel.data.z;

                
        rc_sticks_flag.pitch = rc_pack->rc.pitch;
        rc_sticks_flag.roll = rc_pack->rc.roll;
        rc_sticks_flag.yaw = rc_pack->rc.yaw;
        rc_sticks_flag.throttle = rc_pack->rc.throttle;
        rc_sticks_flag.flag.groundConnected = rc_pack->rc.flag.groundConnected;
        rc_sticks_flag.flag.skyConnected = rc_pack->rc.flag.skyConnected;
        rc_sticks_flag.flag.logicConnected = rc_pack->rc.flag.logicConnected;


        /*first check if we breaks*/
        if(rc_sticks_flag.throttle >= 0.8f){
            flag_break_by_RC = true;
        }

        /*brake stabilize and release control to RC*/
        if(flag_break_by_RC && is_joystick_auth_obtained){
            USER_LOG_INFO("break by RC");
            DjiTest_WidgetLogAppend("break by RC");
            returnCode = DjiFlightController_ExecuteEmergencyBrakeAction();
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Emergency brake failed, error code: 0x%08X", returnCode);
                return false;
            }


            USER_LOG_INFO("Release joystick authority");
            DjiTest_WidgetLogAppend("Release joystick authority");
            returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Release joystick authority failed, error code: 0x%08X", returnCode);
                return false;
            }
            is_joystick_auth_obtained = false;
            osalHandler->TaskSleepMs(100000);//sleep like forever
        }


        /*get joystick authority*/
        if(!is_joystick_auth_obtained){
            USER_LOG_INFO("Obtain joystick control authority.");
            DjiTest_WidgetLogAppend("Obtain joystick control authority.");
            returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
            if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
                USER_LOG_ERROR("Obtain joystick authority failed, error code: 0x%08X", returnCode);
                return false;
            }
            is_joystick_auth_obtained = true;
            s_osalHandler->TaskSleepMs(1000);
        }

        /*takeoff, just for simulation*/
        if(!is_takeoff_finished){
            USER_LOG_INFO("Take off\r\n");
            DjiTest_WidgetLogAppend("Take off\r\n");

            if (!DjiTest_FlightControlMonitoredTakeoff()) {
                USER_LOG_ERROR("Take off failed");
                return false;
            }

            is_takeoff_finished = true;
            USER_LOG_INFO("Successful take off\r\n");
            DjiTest_WidgetLogAppend("Successful take off\r\n");
            s_osalHandler->TaskSleepMs(4000);
        }
        /*end of takeoff*/

        /*mini control loop*/
        if(is_takeoff_finished){
            /*switch to velocity control mode*/
            T_DjiFlightControllerJoystickMode joystickMode = { 
                DJI_FLIGHT_CONTROLLER_HORIZONTAL_VELOCITY_CONTROL_MODE,
                DJI_FLIGHT_CONTROLLER_VERTICAL_VELOCITY_CONTROL_MODE,
                DJI_FLIGHT_CONTROLLER_YAW_ANGLE_RATE_CONTROL_MODE,
                DJI_FLIGHT_CONTROLLER_HORIZONTAL_GROUND_COORDINATE,
                DJI_FLIGHT_CONTROLLER_STABLE_CONTROL_MODE_ENABLE,
            };

            DjiFlightController_SetJoystickMode(joystickMode);
            T_DjiFlightControllerJoystickCommand joystickCommand = {0.1f, 1.1f, 0.5f, 0.1f};

            /*execution each loop when activated*/
            DjiFlightController_ExecuteJoystickAction(joystickCommand);
        }

        /*time record*/
        iteration_times +=1;
        osalHandler->GetTimeMs(&loopendTimeInMs);
        elapsedTimeInMs += loopendTimeInMs - loopstartTimeInMs;

        if(iteration_times%100 == 1){
            USER_LOG_INFO("100 control loops take %u microsecond", elapsedTimeInMs);
        }
        /*time record*/

        osalHandler->TaskSleepMs(20);//try 20 -> 15 -> 10 -> 5 -> 2(?)

    }

}


/****************** (C) COPYRIGHT DJI Innovations *****END OF FILE****/
