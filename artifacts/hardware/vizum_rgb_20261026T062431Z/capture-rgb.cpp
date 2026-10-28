#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <unistd.h>
#include <vector>
#include "VZNL_Common.h"
#include "VZNL_RGBConfig.h"

// 只从确认的设备读取一次独立 RGB 图像；程序没有点云和激光扫描调用。
int main() {
  std::setvbuf(stdout,nullptr,_IONBF,0);
  VzNL_SetLogLevel(keNLLogLevel_None,keNLLogType_None);
  SVzNLConfigParam config{};
  int rc=VzNL_Init(&config);
  std::printf("init=%d\n",rc);
  if(rc!=0)return 1;
  struct SDKCleanup { ~SDKCleanup(){VzNL_Destroy();} } sdk_cleanup;
  rc=VzNL_ResearchDevice(keSearchDeviceFlag_EthLaserRobotEye);
  if(rc!=0)return 1;
  int count=0;
  rc=VzNL_GetEyeCBDeviceInfo(nullptr,&count);
  if(rc!=0||count<=0||count>256)return 1;
  std::vector<SVzNLEyeCBInfo> devices(count);
  rc=VzNL_GetEyeCBDeviceInfo(devices.data(),&count);
  if(rc!=0||count<0||count>int(devices.size()))return 1;
  for(int i=0;i<count;++i){
    const auto& device=devices[i];
    if(std::strncmp(device.byServerIP,"192.168.1.20",sizeof device.byServerIP)!=0)continue;
    if(std::strncmp(device.szDeviceID,"Ik256431",sizeof device.szDeviceID)!=0)return 1;
    SVzNLOpenDeviceParam params;
    params.bStopStream=VzFalse;
    int error=0;
    std::puts("Opening 192.168.1.20 / Ik256431...");
    VZNLHANDLE handle=VzNL_OpenDevice(&device,&params,&error);
    std::printf("open_error=%d handle_valid=%d\n",error,int(handle!=nullptr));
    if(error!=0||!handle)return 1;
    struct DeviceCleanup {
      VZNLHANDLE handle;
      ~DeviceCleanup(){std::printf("close=%d\n",VzNL_CloseDevice(handle));}
    } device_cleanup{handle};
    error=0;
    const auto support=VzNL_IsSupportRGBCamera(handle,&error);
    std::printf("rgb_support_query=%d supported=%d\n",error,int(support));
    SVzNLEyeDeviceInfoEx info{};
    info.sEyeCBInfo.nSize=sizeof info;
    rc=VzNL_GetDeviceInfo(handle,&info.sEyeCBInfo);
    std::printf("extended_info_query=%d capability_mask=%u supports_center=%u pupil_distance_raw=%u\n",
                rc,info.sCapability.nCapabilityMask,info.sCapability.bIsSupportRGBSensor,info.nPupilDistance);
    error=0;
    const auto enabled=VzNL_IsEnableRGB(handle,&error);
    const int enable_query=error;
    std::printf("rgb_enable_query=%d enabled=%d\n",enable_query,int(enabled));
    struct EnableCleanup {
      VZNLHANDLE handle;
      VzBool original;
      bool changed=false;
      ~EnableCleanup(){
        if(!changed)return;
        const int restore=VzNL_EnableRGB(handle,original);
        int error=0;
        const auto actual=VzNL_IsEnableRGB(handle,&error);
        std::printf("rgb_enable_restore=%d verify=%d actual=%d original=%d\n",restore,error,int(actual),int(original));
      }
    } enable_cleanup{handle,enabled};
    if(support==VzTrue && enable_query==0 && enabled==VzFalse){
      enable_cleanup.changed=true;
      rc=VzNL_EnableRGB(handle,VzTrue);
      std::printf("temporary_enable_rgb=%d\n",rc);
      if(rc!=0)return 1;
    }
    SVzVideoResolution resolution{};
    rc=VzNL_GetRGBResolution(handle,&resolution);
    std::printf("rgb_resolution_query=%d width=%u height=%u\n",rc,resolution.nFrameWidth,resolution.nFrameHeight);
    error=0;
    const auto exposure=VzNL_GetRGBExpose(handle,&error);
    std::printf("rgb_exposure_query=%d value_raw=%u\n",error,exposure);
    error=0;
    const auto automatic=VzNL_IsEnableRGBAutoExpose(handle,&error);
    std::printf("rgb_auto_exposure_query=%d enabled=%d\n",error,int(automatic));
    error=0;
    const auto trigger=VzNL_GetRGBTriggerMode(handle,&error);
    std::printf("rgb_trigger_query=%d mode=%d\n",error,int(trigger));
    SVzNLROIRect roi{};
    rc=VzNL_GetRGBROI(handle,&roi);
    std::printf("rgb_roi_query=%d rect=%d,%d,%d,%d\n",rc,roi.left,roi.right,roi.top,roi.bottom);
    VzNL_SetOutputImageFormat(keVzNLImageType_RGB888);
    SVzNLImageData* image=nullptr;
    struct ImageCleanup {
      SVzNLImageData** image;
      ~ImageCleanup(){if(*image)std::printf("release_image=%d\n",VzNL_ReleaseImage(image));}
    } image_cleanup{&image};
    rc=VzNL_GetRGBImage(handle,&image);
    std::printf("rgb_snapshot_result=%d image_valid=%d\n",rc,int(image!=nullptr));
    if(rc!=0){
      char message[256]{};
      VzNL_GetErrorInfo(rc,message);
      std::printf("rgb_snapshot_error=%.*s\n",int(sizeof message),message);
      return 1;
    }
    if(!image||!image->pBuffer)return 1;
    std::printf("rgb_image: width=%u height=%u bits=%u channels=%u type=%d bytes=%u\n",
                image->nWidth,image->nHeight,unsigned(image->byBitDepth),unsigned(image->nChannels),
                int(image->eImageType),image->nBufferSize);
    const auto pixels=std::size_t(image->nWidth)*image->nHeight;
    if(!pixels||pixels>100000000||image->byBitDepth!=8||image->nChannels!=3||
       (image->eImageType!=keVzNLImageType_RGB888&&image->eImageType!=keVzNLImageType_BGR888)||
       image->nBufferSize!=pixels*3){
      std::puts("Unsupported native RGB image layout; no format assumption applied.");
      return 1;
    }
    std::vector<unsigned char> output(image->pBuffer,image->pBuffer+image->nBufferSize);
    if(image->eImageType==keVzNLImageType_BGR888)
      for(std::size_t offset=0;offset<output.size();offset+=3)std::swap(output[offset],output[offset+2]);
    const auto directory=std::filesystem::path("/tmp/vizum-camera-setup")/("rgb-"+std::to_string(getpid()));
    if(!std::filesystem::create_directory(directory))return 1;
    const auto path=directory/"rgb.ppm";
    FILE* file=std::fopen(path.c_str(),"wbx");
    if(!file)return 1;
    std::fprintf(file,"P6\n%u %u\n255\n",image->nWidth,image->nHeight);
    const auto written=std::fwrite(output.data(),1,output.size(),file);
    std::fclose(file);
    if(written!=output.size())return 1;
    std::printf("saved=%s\n",path.c_str());
    return 0;
  }
  std::puts("Confirmed target not found.");
  return 1;
}
