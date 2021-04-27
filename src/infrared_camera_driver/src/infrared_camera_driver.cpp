#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>

#include <map>
#include <string>
#include <utility>

#include <image_transport/image_transport.h>
#include "stdio.h" // C Standard Input/Output library.
#include "XCamera.h" // Xeneth SDK main header.
#include "XFooters.h" // Xeneth frame footer header.

#include <chrono>

#define CRLF    "\r\n"


//the seral of cameras

const long CAM_REAR = 14076;
const long CAM_REAR_LEFT = 14079;
const long CAM_REAR_RIGHT = 14078;
const long CAM_FRONT = 14081;
const long CAM_FRONT_LEFT = 14080;
const long CAM_FRONT_RIGHT = 14077;

long CAM_ACTIVATED = CAM_REAR;
const std::string  node_name = "infrared_camera_driver";

int device_discovery(std::vector<std::string> &camera_list)
{
    ErrCode errorCode = I_OK;
    unsigned int deviceCount = 0;
    //std::vector<std::string> camera_list;
    /*  We start by retrieving the number of discovered devices. In this sample discovery is performed on all available protocols.
     *  For this simply pass the XEF_EnableAll flag with the XCD_EnumerateDevices function.
     *  To only retrieve the device count the first argument must be set to NULL.
     *  Note that when no flags are passed at all, meaning the flags argument value is 0, no enumeration will happen. */

    if ((errorCode = XCD_EnumerateDevices(NULL, &deviceCount, XEF_EnableAll)) != I_OK) {
        printf("An error occurred while enumerating the devices. errorCode: %i\n", errorCode);
        return -1;
    }

    if (deviceCount == 0) {
        printf("Enumeration was a success but no devices were found!\n");
        return 0;
    }

    /*  At this point we know how much devices are present in the environment.
     *  Now allocate the XDeviceInformation array to accommodate all the discovered devices using
     *  the device count to determine the size needed. Once allocated we call the enumerate
     *  devices method again but now instead of passing null as the initial argument use the new
     *  allocated buffer. For the flags argument we no longer use a protocol enable flag but make use
     *  of the XEF_UseCached flag. On discovery devices are cached internally and as such we are able to
     *  retrieve the device information structures instantly when calling XCD_EnumerateDevices for a second time.
     *  Note that it is not required to first check device count, allocate structure and retrieve cached devices.
     *  A user could allocate one or more device structure and immediately pass this with the initial call to XCD_EnumerateDevices.
     *  XCD_EnumerateDevices will not exceed the supplied deviceCount and when less devices were discovered than the initial deviceCount
     *  this argument is updated with the new count. */

    XDeviceInformation *devices = new XDeviceInformation[deviceCount];
    if ((errorCode = XCD_EnumerateDevices(devices, &deviceCount, XEF_UseCached)) != I_OK) {
        printf("Error while retrieving the cached device information structures. errorCode: %i\n", errorCode);
        delete [] devices;
        return -1;
    }

    /*  All discovered devices are now available in our local array and we are now able
     *  to iterate the list and output each item in the array */

    for(unsigned int i = 0; i < deviceCount; i++) {
        XDeviceInformation * dev = &devices[i];
        if(0xF122!=dev->pid || CAM_ACTIVATED != dev->serial)
        {
            //std::cout<<"pid of the cam: "<<dev->pid<<std::endl;
            continue;
        }

        camera_list.push_back(dev->url);
        printf("device[%lu] %s @ %s (%s) \n", i, dev->name, dev->address, dev->transport);
        printf("PID: %4X\n", dev->pid);
        printf("Serial: %lu\n", dev->serial);
        printf("URL: %s\n", dev->url);
        printf("State: %s\n\n", dev->state == XDS_Available ? "Available" : dev->state == XDS_Busy ? "Busy" : "Unreachable");
    }

    if(0==deviceCount)
    {
        std::cout<<"The serial of the infrared camera: "<< CAM_ACTIVATED<<" is error, please check again!!!"<<std::endl;
    }

    delete [] devices;
    return deviceCount;
}

bool HandleError(ErrCode errCode, const char * msg)
{
    const int sz = 2048;
    char err_msg[sz];

    XC_ErrorToString(errCode, err_msg, sz);
    printf("%s: %s (%i)" CRLF, msg, err_msg, errCode);

    return I_OK == errCode;
}


bool SetupIPAdress(XCHANDLE handle, const char* str_ip)
{
    ErrCode errCode = I_OK;
    std::cout<<"Configuring camera IP adress..."<<std::endl;

    errCode = XC_SetPropertyValueL(handle, "GevCurrentIPConfigurationPersistentIP", 1, "");
    if (!HandleError(errCode, " * Enable persistent IP"))
        return false;


    errCode = XC_SetPropertyValueE(handle, "GevPersistentIPAddress", str_ip);
    if (!HandleError(errCode, " * Set IP adress"))
        return false;


    errCode = XC_SetPropertyValueE(handle, "GevPersistentSubnetMask", "255.255.0.0");
    if (!HandleError(errCode, " * Set subnet mask"))
        return false;

    //errCode = XC_SetPropertyValueE(handle, "GevPersistentDefaultGateway", "255.255.0.0");
    //if (!HandleError(errCode, " * Set gateway"))
    //    return false;

    printf(CRLF);
    return true;

}


/*
 *  In SetupShutterControl_F027 we disable the automatic shutter correction.
 *  When this is set to enabled it is possible that triggers being received
 *  during a calibrate cycle are not processed by the camera.
 *  To make sure the image does not drift and stays corrected the camera has
 *  to be occasionally calibrated by stopping / starting the acquisition or
 *  executing the "Calibrate"-property by setting its value to 1.
 */

bool SetupShutterControl(XCHANDLE handle, bool isAuto = false) {

    ErrCode errCode = I_OK;

    printf("Configuring camera to disable the automatic shutter control: " CRLF);

    /*
     *  AutoCorrectionEnabled = Disabled (0), Enabled (1)
     * -----------------------------------------------------------------
     *  Disable the automatic internal calibration.
     */

    errCode = XC_SetPropertyValueL(handle, "AutoCorrectionEnabled", isAuto, "");
    if (!HandleError(errCode, " * Disable auto correction"))
        return false;

    printf(CRLF);
    return true;
}


/*
 *  In SetupExternalTriggeredMode_F027 we configure the camera in an
 *  external trigger mode where images will be captured on the rising edge
 *  of a square wave signal. For this we have to set the trigger
 *  properties accordingly to reflect the working mode we are after.
 */

bool SetupExternalTriggeredMode(XCHANDLE handle)
{
//    if(!SetupShutterControl(handle))
//    {
//        std::cout<<" error in close free running!"<<std::endl;
//        return false;
//    }

    ErrCode errCode = I_OK;

    printf("Configuring camera in external triggered mode with rising edge activation" CRLF);

    /*
     *  TriggerOutEnable = Disabled (0), Enabled (1)
     * -----------------------------------------------------------------
     *  Although the TriggerDirection works as a multiplexer for the
     *  trigger connection on the camera, it is still good practice to
     *  make sure the trigger output block is completely disabled by setting
     *  its value to 0.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerOutEnable", 0, "");
    if (!HandleError(errCode, " * Disable trigger output"))
        return false;


    /*
     *  TriggerDirection = TriggerInput (0), TriggerOutput (1)
     * ---------------------------------------------------------------------
     *  The trigger direction will have to be configured such that the
     *  trigger connection on the camera is in a sinking (input) configuration.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerDirection", 0, "");
    if (!HandleError(errCode, " * Set trigger direction"))
        return false;


    /*
     *  TriggerInMode = FreeRunning (0), ExternalTriggered (1)
     * -----------------------------------------------------------------
     *  Set the TriggerInMode to ExternalTriggered to put the camera
     *  in triggered mode. FreeRunning-mode means the camera is
     *  continuously triggered by an internal source.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerInMode", 1, "");
    if (!HandleError(errCode, " * Set trigger input mode"))
        return false;


    /*
     *  TriggerInSensitivity = Level (0), Edge (1)
     * -----------------------------------------------------------------
     *  We are performing edge triggers.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerInSensitivity", 1, "");
    if (!HandleError(errCode, " * Set trigger input sensitivity"))
        return false;


    /*
     *  TriggerInPolarity = LowFalling (0), HighRising (1)
     * -----------------------------------------------------------------
     *  And we want to capture a frame on the rising edge.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerInPolarity", 1, "");
    if (!HandleError(errCode, " * Set trigger input polarity"))
        return false;


    /*
     *  TriggerInDelay = 0
     * -----------------------------------------------------------------
     *  This configures the time in microseconds between the trigger
     *  event and the moment the camera actually starts integrating a
     *  new frame. In this use case no delay is required.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerInDelay", 0, "");
    if (!HandleError(errCode, " * Set trigger input delay"))
        return false;


    /*
     *  TriggerInSkip = 0
     * -----------------------------------------------------------------
     *  Skip n triggers before allowing the trigger to capture a frame.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerInSkip", 0, "");
    if (!HandleError(errCode, " * Set trigger input skip"))
        return false;


    /*
     *  TriggerInTiming = Optimal (0), Custom (1)
     * -----------------------------------------------------------------
     *  In the optimal mode, the sensor is constantly running and acquiring
     *  images preventing the *  sensor to cool down due to an irregular or
     *  too slow external trigger rate. If an external trigger is active,
     *  the next frame from the sensor is sent out on the interface.
     *  The latency between a trigger pulse and the actual acquisition start
     *  is in the worst case 1 frame time. In this mode, the trigger input
     *  delay should be considered as a minimum trigger input delay.
     *  As soon as an external trigger pulse is detected, a timer is
     *  started to implement the trigger input delay. When the timer expires,
     *  a triggeframeBufferr pulse is generated. The next frame will be sent out.
     *
     *  In the custom mode, the sensor only acquires frames when an
     *  external trigger is active. This might degrade the image quality
     *  if the frequency of the external triggers is not high enough or
     *  when irregular external triggers are applied.
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerInTiming", 0, "");
    if (!HandleError(errCode, " * Set trigger input timing"))
        return false;


    /*
     *  TriggerInEnable = Disabled (0), Enabled (1)
     * -----------------------------------------------------------------
     *  Enable the trigger input block
     */

    errCode = XC_SetPropertyValueL(handle, "TriggerInEnable", 1, "");
    if (!HandleError(errCode, " * Enable trigger input"))
        return false;

    printf(CRLF);
    return true;
}

void publish_msg(XCHANDLE handle, image_transport::Publisher& pub, ros::NodeHandle nh)
{
    // Variables
    ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
    dword *frameBuffer = 0; // 32-bit buffer to store the capture frame.
    dword frameSize = 0; // The width of the camera's image.

    sensor_msgs::ImagePtr msg;


    //Select which part of the dynamical range needs to be mapped on the image.
    //XMsgSetTROIParms msg;
    //msg.pctlo = 0.;  // between 0 and 1
    //msg.pcthi = 1.0;  // between 0 and 1
    //XC_MsgImageFilter(handle, -1, XMsgSetTROI, &msg);

    // Determine framesize for a 32-bit buffer
    int width = XC_GetWidth(handle);   // 640
    int height = XC_GetHeight(handle); // 480
    frameSize = XC_GetWidth(handle) * XC_GetHeight(handle);

    // Initialize the 32-bit buffer.
    frameBuffer = new dword[frameSize];
    cv::Mat image = cv::Mat(height, width, CV_8UC3);

    // ... grab a frame from the camera - FT_32_BPP_BGRA
    //printf("Grabbing a frame - FT_32_BPP_BGRA.\n");
    if ((errorCode = XC_GetFrame(handle, FT_32_BPP_BGRA, XGF_Blocking, frameBuffer, frameSize * 4 /* bytes per pixel */)) != I_OK)
    {
        printf("Problem while fetching frame, errorCode %lu\n", errorCode);
    }

    else
    {
        //cv::Mat image = cv::Mat::zeros(height, width, CV_8U);


        for(int i = 0;i<height;i++)
        {
            for(int j = 0;j<width;j++)
            {
                dword pixel = frameBuffer[i*width+j];
                byte r = (pixel >> 0)  & 0xff;
                byte g = (pixel >> 8)  & 0xff;
                byte b = (pixel >> 16) & 0xff;
                byte a = (pixel >> 24) & 0xff;

                //std::cout<<" bgr: " <<cv::Vec3b(b, g, r)<<std::endl;
                image.at<cv::Vec3b>(i, j) = cv::Vec3b(b, g, r);

            }
        }

        auto header = std_msgs::Header();
        header.stamp = ros::Time::now();
        msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        pub.publish(msg);


        // //// test code
        // // serial number
        // long serial=0;
        // XC_GetPropertyValueL(handle, "_CAM_SER", &serial);
        // std::cout<<"cam: " <<serial<<std::endl;
        // //std::cout<<"pub: "<<pub<<std::endl;
        // //ros::spinOnce();
    }


    //printf("Clearing buffers.\n");
    if (frameBuffer != 0)
    {
        delete [] frameBuffer;
        frameBuffer = 0;
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;


    int serial_param = 0;
    ros::param::get("~serial", serial_param);
    CAM_ACTIVATED = (long) serial_param;

    image_transport::ImageTransport it(nh);
    //device_discovery();
//    image_transport::Publisher camera_rear_pub = it.advertise("/infrared_camera/rear/image_color", 1);
//    image_transport::Publisher camera_rear_left_pub = it.advertise("/infrared_camera/rear_left/image_color", 1);
//
//    image_transport::Publisher camera_rear_right_pub = it.advertise("/infrared_camera/rear_right/image_color", 1);
//
//    image_transport::Publisher camera_front_pub = it.advertise("/infrared_camera/front/image_color", 1);
//
//    image_transport::Publisher camera_front_left_pub = it.advertise("/infrared_camera/front_left/image_color", 1);
//    image_transport::Publisher camera_front_right_pub = it.advertise("/infrared_camera/front_right/image_color", 1);

    image_transport::Publisher camera_pub = it.advertise("/image_color", 1);

    ErrCode errorCode = 0;
    std::vector<std::string> camera_list;
    device_discovery(camera_list);

    std::vector<XCHANDLE> camera_handle_list;
    std::map<image_transport::Publisher, XCHANDLE> cam_ros_pub2handle_map;
     
    XCHANDLE handle = 0;

    for(auto begin=camera_list.cbegin();begin!=camera_list.cend(); begin++)
    {
        //std::cout<<(*begin).c_str()<<std::endl;
        //XCHANDLE handle = 0; // Handle to the camera
        //ErrCode errorCode = 0; // Used to store returned errorCodes from the SDK functions.
        handle = XC_OpenCamera((*begin).c_str());
        camera_handle_list.push_back(handle);
        //cam_ros_pub2handle_map.insert(std::make_pair(camera_pub, handle));

//
//        // get serial number
//        long serial=0;
//        XC_GetPropertyValueL(handle, "_CAM_SER", &serial);
//
//        if(CAM_FRONT==serial)
//        {
//            cam_ros_pub2handle_map.insert(std::make_pair(camera_front_pub, handle));
//        }
//
//        else if(CAM_FRONT_RIGHT==serial)
//        {
//            cam_ros_pub2handle_map.insert(std::make_pair(camera_front_right_pub, handle));
//        }
//
//        else if(CAM_FRONT_LEFT==serial)
//        {
//            cam_ros_pub2handle_map.insert(std::make_pair(camera_front_left_pub, handle));
//        }
//
//        else if(CAM_REAR==serial)
//        {
//            cam_ros_pub2handle_map.insert(std::make_pair(camera_rear_pub, handle));
//        }
//
//        else if(CAM_REAR_LEFT==serial)
//        {
//            cam_ros_pub2handle_map.insert(std::make_pair(camera_rear_left_pub, handle));
//        }
//
//        else if(CAM_REAR_RIGHT==serial)
//        {
//            cam_ros_pub2handle_map.insert(std::make_pair(camera_rear_right_pub, handle));
//        }
//
//
//        else
//        {
//            std::cout<<"The serial: "<< serial <<" is error!"<<std::endl;
//        }


        // When the connection is initialised, ...
        if (XC_IsInitialised(handle))
        {

            // SetupShutterControl(handle);
            // set triggered mode
            if(!SetupExternalTriggeredMode(handle))
            {
                std::cout<< "Setting triggered mode Error! "<<std::endl;
            }

            // ... start capturing
            printf("%s Start capturing.\n", (*begin).c_str());
            if ((errorCode = XC_StartCapture(handle)) != I_OK) {
                printf("%s Could not start capturing, errorCode: %lu\n", (*begin).c_str());
            }

            else if (XC_IsCapturing(handle)) // When the camera is capturing ...
            {
                // Load the color profile delivered with this sample.
                if ((errorCode = XC_LoadColourProfile(handle, "../../../Resources/Colour_profiles/Thermal Blue.png")) !=
                    I_OK) {
                    printf("Problem while loading the desired colorprofile, errorCode: %lu \n", errorCode);
                }

                // Set the colourmode so that the last loaded colorprofile is used.
                XC_SetColourMode(handle, ColourMode_Profile);
            }


        }

        else
        {
            printf("%s Initialization failed\n", (*begin).c_str());
        }
    }

    while(nh.ok())
    {

//        publish_msg(cam_ros_pub2handle_map[camera_rear_pub], camera_rear_pub, nh);
//        publish_msg(cam_ros_pub2handle_map[camera_rear_left_pub], camera_rear_left_pub, nh);
//        blish_msg(cam_ros_pub2handle_map[camera_front_left_pub], camera_front_left_pub, nh);
//
//        publish_msg(cam_ros_pub2handle_map[camera_front_pub], camera_front_pub, nh);
//        publish_msg(cam_ros_pub2handle_map[camera_front_right_pub], camera_front_right_pub, nh);
//        publish_msg(cam_ros_pub2handle_map[camera_rear_right_pub], camera_rear_right_pub, nh);

        publish_msg(handle, camera_pub, nh);
        // auto end = cam_ros_pub2handle_map.end();
        // for(auto begin = cam_ros_pub2handle_map.begin(); begin!=end; begin++)
        // {
        //     //std::cout<<"first: "<<(*begin).first<<std::endl;
        //     publish_msg((*begin).second, (*begin).first, nh);
        
        // }
    }


    //// close cameras

    for(auto begin=camera_handle_list.begin();begin!=camera_handle_list.end(); begin++)
    {
        // // When the camera is still capturing ...
        XCHANDLE handle = *begin;
        if(XC_IsCapturing(handle))
        {
            // ... stop capturing.
            printf("Stop capturing.\n");
            if ((errorCode = XC_StopCapture(handle)) != I_OK)
            {
                printf("Could not stop capturing, errorCode: %lu\n", errorCode);
            }
        }

        // When the handle to the camera is still initialised ...
        if (XC_IsInitialised(handle))
        {
            // .. close the connection.
            printf("Closing connection to camera.\n");
            XC_CloseCamera(handle);
        }
    }

    //ros::spinOnce();
    return 0;

}
