#include <openvr.h>
#include <cmath>
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace vr;

IVRSystem *vr_pointer = nullptr;
geometry_msgs::Pose pose;
tf2::Quaternion rot, orig, q_new;


struct _TrackerData
{
    int deviceId = -1; // Device ID according to the SteamVR system
    HmdVector3_t pos;
    bool isValid;
};
typedef struct _TrackerData TrackerData;

HmdVector3_t GetPosition(HmdMatrix34_t matrix)
{
    HmdVector3_t vector;

    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];

    return vector;
}

HmdQuaternion_t GetRotation(HmdMatrix34_t matrix)
{
    HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

    return q;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nh;
    ros::Publisher tracker_pub = nh.advertise<geometry_msgs::Pose>("tracker_pose", 1000);
    ros::Rate loop_rate(10);

    EVRInitError eError = VRInitError_None;
    vr_pointer = VR_Init(&eError, VRApplication_Background);
    if (eError != VRInitError_None)
    {
        vr_pointer = nullptr;
        printf("Unable to init VR runtime: %s \n", VR_GetVRInitErrorAsEnglishDescription(eError));
        exit(EXIT_FAILURE);
    }

    TrackedDevicePose_t trackedDevicePose;
    VRControllerState_t controllerState;
    HmdQuaternion_t rotation;
    unsigned int deviceId = 0;

    for (; deviceId < k_unMaxTrackedDeviceCount; deviceId++)
    {
        ETrackedDeviceClass eClass = vr_pointer->GetTrackedDeviceClass(deviceId);

        if (eClass == ETrackedDeviceClass::TrackedDeviceClass_GenericTracker)
        {
            break;
        }
    }

    // Found tracker
    while (ros::ok())
    {
        if (vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, deviceId, &controllerState, sizeof(controllerState), &trackedDevicePose))
        {
            TrackerData pT = {};
            pT.deviceId = deviceId;
            pT.pos = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
            rotation = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
            pT.isValid = trackedDevicePose.bPoseIsValid;
            if (pT.isValid)
            {
                // converting y up to z up
                std::cout << "X: " << pT.pos.v[0] << ", Y: " << -pT.pos.v[2] << ", Z: " << pT.pos.v[1] << ", qX: " << rotation.x << ", qY: " << rotation.y << ", qZ: " << rotation.z << ", qW: " << rotation.w << std::endl;
                pose.position.x = pT.pos.v[0];
                pose.position.y = -pT.pos.v[2];
                pose.position.z = pT.pos.v[1];

                // Rotate quaternion -90 about X axis
                rot.setRPY(-1.57079632679,0,0);
                tf2::convert(pose.orientation, orig);
                q_new = rot*orig;
                q_new.normalize();
                tf2::convert(q_new, pose.orientation);
                
                pose.orientation.x = rotation.x;
                pose.orientation.y = rotation.y;
                pose.orientation.z = rotation.z;
                pose.orientation.w = rotation.w;

                tracker_pub.publish(pose);
                ros::spinOnce();
                loop_rate.sleep();
            }
            else
            {
                std::cout << "invalid" << std::endl;
            }
        }
    }

    return 0;
}