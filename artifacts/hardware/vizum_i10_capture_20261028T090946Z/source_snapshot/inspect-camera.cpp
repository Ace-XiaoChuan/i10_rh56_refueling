#include <cstdio>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <string>
#include <unistd.h>
#include <vector>
#include "VZNL_Common.h"
#include "VZNL_DetectLaser.h"
#include "VZNL_DetectConfig.h"
#include "VZNL_SwingMotor.h"

// 仅连接已确认相机；默认查询，显式选项用于二维预览或一次静态扫描。
int main(int argc, char** argv) {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  VzNL_SetLogLevel(keNLLogLevel_None, keNLLogType_None);
  SVzNLConfigParam config{};
  int rc = VzNL_Init(&config);
  std::printf("init=%d\n", rc);
  if (rc != 0) return 1;
  struct Cleanup { ~Cleanup() { VzNL_Destroy(); } } cleanup;
  rc = VzNL_ResearchDevice(keSearchDeviceFlag_EthLaserRobotEye);
  std::printf("search=%d\n", rc);
  if (rc != 0) return 1;
  int count = 0;
  rc = VzNL_GetEyeCBDeviceInfo(nullptr, &count);
  if (rc != 0 || count <= 0 || count > 256) return 1;
  std::vector<SVzNLEyeCBInfo> devices(count);
  rc = VzNL_GetEyeCBDeviceInfo(devices.data(), &count);
  if (rc != 0 || count < 0 || count > int(devices.size())) return 1;
  for (int i = 0; i < count; ++i) {
    const auto& device = devices[i];
    if (std::strncmp(device.byServerIP, "192.168.1.20", sizeof device.byServerIP) != 0)
      continue;
    if (std::strncmp(device.szDeviceID, "Ik256431", sizeof device.szDeviceID) != 0) {
      std::puts("Target serial mismatch; connection cancelled.");
      return 1;
    }
    SVzNLOpenDeviceParam params;
    params.bStopStream = VzFalse;
    int error = 0;
    std::puts("Opening confirmed target with bStopStream=false...");
    VZNLHANDLE handle = VzNL_OpenDevice(&device, &params, &error);
    std::printf("open_error=%d handle_valid=%d\n", error, int(handle != nullptr));
    if (error != 0 || handle == nullptr) return 1;
    struct DeviceCleanup {
      VZNLHANDLE handle;
      ~DeviceCleanup() { std::printf("close=%d\n", VzNL_CloseDevice(handle)); }
    } device_cleanup{handle};
    SVzNLVersionInfo version{};
    rc = VzNL_GetVersion(handle, &version);
    std::printf("version_query=%d mask=%ld\n", rc, version.sCap.nMaskCap);
    if (rc == 0) {
      std::printf("sdk=%.*s app=%.*s camera=%.*s hardware=%.*s firmware=%.*s\n",
                  int(sizeof version.szSDKVersion), version.szSDKVersion,
                  int(sizeof version.szAppVersion), version.szAppVersion,
                  int(sizeof version.szCameraVersion), version.szCameraVersion,
                  int(sizeof version.szVizumEyeVersion), version.szVizumEyeVersion,
                  int(sizeof version.szFwVersion), version.szFwVersion);
    }
    SVzVideoResolution resolution{};
    rc = VzNL_GetResolution(handle, &resolution);
    std::printf("resolution_query=%d width=%u height=%u\n", rc,
                resolution.nFrameWidth, resolution.nFrameHeight);
    error = 0;
    auto pupil = VzNL_GetPupilDistance(handle, &error);
    std::printf("pupil_distance_query=%d value=%u\n", error, pupil);
    error = 0;
    auto capturing = VzNL_IsCapturing(handle, &error);
    std::printf("capture_status_query=%d capturing=%d\n", error, int(capturing));
    error = 0;
    auto detecting = VzNL_IsAutoDetecting(handle, &error);
    std::printf("detect_status_query=%d detecting=%d\n", error, int(detecting));
    EVzNLExposeMode exposure_mode{};
    unsigned int exposure_time = 0;
    rc = VzNL_GetConfigEyeExpose(handle, &exposure_mode, &exposure_time);
    const int exposure_query_result = rc;
    std::printf("exposure_query=%d mode=%d time_raw=%u\n", rc, int(exposure_mode), exposure_time);
    SVzNLROIRect left_roi{}, right_roi{};
    const int roi_query = VzNL_GetConfigDetectROI(handle, &left_roi, &right_roi);
    std::printf("roi_query=%d left=%d,%d,%d,%d right=%d,%d,%d,%d\n", roi_query,
                left_roi.left, left_roi.right, left_roi.top, left_roi.bottom,
                right_roi.left, right_roi.right, right_roi.top, right_roi.bottom);
    error = 0;
    const auto trigger = VzNL_GetTriggerMode(handle, &error);
    std::printf("trigger_query=%d mode=%d\n", error, int(trigger));
    double near_distance = 0, far_distance = 0;
    rc = VzNL_GetSwingMotorWorkRange(handle, &near_distance, &far_distance);
    std::printf("scan_work_range_query=%d near_raw=%.9g far_raw=%.9g\n", rc, near_distance, far_distance);
    if (argc == 2 && std::strcmp(argv[1], "--cloud") == 0) {
      // 采用相机现有扫描配置；不镜像、不映射到 RGB、不修改机械臂状态。
      const int tool = VzNL_BeginDetectLaser(handle);
      std::printf("create_laser_detection_tool=%d\n", tool);
      if (tool != 0) return 1;
      struct ToolCleanup {
        VZNLHANDLE handle;
        ~ToolCleanup() { VzNL_EndDetectLaser(handle); std::puts("destroy_laser_detection_tool=called"); }
      } tool_cleanup{handle};
      const auto directory = std::filesystem::path("/tmp/vizum-camera-setup") /
                             ("cloud-" + std::to_string(getpid()));
      if (!std::filesystem::create_directory(directory)) return 1;
      SVzNLImageData* color_image = nullptr;
      unsigned int count = 0;
      void* raw_points = nullptr;
      const auto host_ns = [] {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      };
      std::printf("scan_start_host_unix_ns=%lld\n", (long long)host_ns());
      const int scan = VzNL_PointCloudSnapShort(handle, keResultDataType_PointF,
                         keFlipType_None, VzFalse, VzFalse, &color_image, &count, &raw_points);
      std::printf("scan_end_host_unix_ns=%lld\n", (long long)host_ns());
      std::printf("scan_result=%d point_count=%u point_pointer_valid=%d\n", scan, count, int(raw_points != nullptr));
      if (scan != 0) {
        char message[256]{};
        VzNL_GetErrorInfo(scan, message);
        std::printf("scan_error_message=%.*s\n", int(sizeof message), message);
      }
      bool saved = false;
      if (scan == 0 && raw_points && count > 0 && count <= 20000000) {
        static_assert(sizeof(SVzNL3DPointF) == 12, "Expected three packed float32 coordinates");
        const auto path = directory / "cloud_sdk_native.pcd";
        FILE* file = std::fopen(path.c_str(), "wbx");
        if (file) {
          std::fprintf(file, "# SDK native XYZ; no flip; no RGB map; units documented in observation record\n"
                             "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                             "WIDTH %u\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %u\nDATA binary\n",count,count);
          const auto written = std::fwrite(raw_points, sizeof(SVzNL3DPointF), count, file);
          std::fclose(file);
          saved = written == count;
          if (saved) std::printf("saved=%s\n", path.c_str());
        }
      }
      if (color_image) {
        std::printf("scan_image: width=%u height=%u bits=%u channels=%u type=%d bytes=%u\n",
                    color_image->nWidth,color_image->nHeight,unsigned(color_image->byBitDepth),
                    unsigned(color_image->nChannels),int(color_image->eImageType),color_image->nBufferSize);
        // 保留原始图像字节和布局，避免未经确认的格式转换或 RGB 配准假设。
        const auto path = directory / "scan_image_sdk_native.bin";
        FILE* file = std::fopen(path.c_str(), "wbx");
        if (file) {
          const auto written = std::fwrite(color_image->pBuffer,1,color_image->nBufferSize,file);
          std::fclose(file);
          if (written == color_image->nBufferSize) std::printf("saved=%s\n",path.c_str());
        }
      }
      std::printf("scan_release=%d\n", VzNL_ReleasePointCloudSnapShortResult(&color_image,&raw_points));
      return scan == 0 && saved ? 0 : 1;
    }
    const bool full_frame = argc == 2 && std::strcmp(argv[1], "--snapshot-full") == 0;
    const bool temporary_exposure = full_frame || (argc == 2 && std::strcmp(argv[1], "--snapshot-exposure") == 0);
    if (argc == 2 && (std::strcmp(argv[1], "--snapshot") == 0 || temporary_exposure)) {
      // 预览可临时调曝光和全画幅 ROI，退出前恢复并读回；不控制激光或摆动机构。
      struct ROICleanup {
        VZNLHANDLE handle;
        SVzNLROIRect left, right;
        bool changed = false;
        ~ROICleanup() {
          if (!changed) return;
          const int restored = VzNL_ConfigDetectROI(handle, &left, &right);
          SVzNLROIRect actual_left{}, actual_right{};
          const int verify = VzNL_GetConfigDetectROI(handle, &actual_left, &actual_right);
          auto same = [](const SVzNLROIRect& a, const SVzNLROIRect& b) {
            return a.left==b.left && a.right==b.right && a.top==b.top && a.bottom==b.bottom;
          };
          const bool matched = verify == 0 && same(left, actual_left) && same(right, actual_right);
          std::printf("roi_restore=%d verify=%d matches_original=%d\n", restored, verify, int(matched));
        }
      } roi_cleanup{handle,left_roi,right_roi};
      struct ExposureCleanup {
        VZNLHANDLE handle;
        EVzNLExposeMode mode;
        unsigned int time;
        bool changed = false;
        ~ExposureCleanup() {
          if (!changed) return;
          const int restore = VzNL_ConfigEyeExpose(handle, mode, time);
          EVzNLExposeMode actual_mode{};
          unsigned int actual_time = 0;
          const int verify = VzNL_GetConfigEyeExpose(handle, &actual_mode, &actual_time);
          std::printf("exposure_restore=%d verify=%d mode=%d time_raw=%u expected_mode=%d expected_time=%u\n",
                      restore, verify, int(actual_mode), actual_time, int(mode), time);
        }
      } exposure_cleanup{handle, exposure_mode, exposure_time};
      if (temporary_exposure) {
        if (exposure_query_result != 0) return 1;
        exposure_cleanup.changed = true;
        const int update = VzNL_ConfigEyeExpose(handle, keVzNLExposeMode_Fix, 10000);
        std::printf("temporary_preview_exposure=%d time_raw=10000\n", update);
        if (update != 0) return 1;
      }
      if (full_frame) {
        if (roi_query != 0 || resolution.nFrameWidth == 0 || resolution.nFrameHeight == 0) return 1;
        SVzNLROIRect full{0, int(resolution.nFrameWidth), 0, int(resolution.nFrameHeight)};
        roi_cleanup.changed = true;
        const int update = VzNL_ConfigDetectROI(handle, &full, &full);
        std::printf("temporary_full_frame_roi=%d width=%u height=%u\n", update,
                    resolution.nFrameWidth,resolution.nFrameHeight);
        if (update != 0) return 1;
      }
      SVzNLImageData* left = nullptr;
      SVzNLImageData* right = nullptr;
      rc = VzNL_GetEyeImage(handle, &left, &right, 5000);
      std::printf("snapshot_query=%d\n", rc);
      if (rc == 0) {
        const auto directory = std::filesystem::path("/tmp/vizum-camera-setup") /
                               ("preview-" + std::to_string(getpid()));
        if (!std::filesystem::create_directory(directory)) return 1;
        auto save = [&](const char* name, const SVzNLImageData* image) {
          if (!image) return;
          std::printf("%s: width=%u height=%u bits=%u channels=%u type=%d bytes=%u\n",
                      name, image->nWidth, image->nHeight, unsigned(image->byBitDepth),
                      unsigned(image->nChannels), int(image->eImageType), image->nBufferSize);
          const auto pixels = std::size_t(image->nWidth) * image->nHeight;
          const bool gray = image->nChannels == 1 && image->eImageType == keVzNLImageType_GRAY;
          const bool rgb = image->nChannels == 3 &&
                           (image->eImageType == keVzNLImageType_RGB888 ||
                            image->eImageType == keVzNLImageType_BGR888);
          const auto bytes = pixels * image->nChannels;
          if (!image->pBuffer || !pixels || image->byBitDepth != 8 ||
              (!gray && !rgb) || image->nBufferSize != bytes) {
            std::puts("Image layout is unsupported or has unknown stride; not saved.");
            return;
          }
          std::vector<unsigned char> output(image->pBuffer, image->pBuffer + bytes);
          if (image->eImageType == keVzNLImageType_BGR888) {
            for (std::size_t offset = 0; offset < bytes; offset += 3)
              std::swap(output[offset], output[offset + 2]);
          }
          const auto path = directory / (std::string(name) + (gray ? ".pgm" : ".ppm"));
          FILE* file = std::fopen(path.c_str(), "wbx");
          if (!file) return;
          std::fprintf(file, "%s\n%u %u\n255\n", gray ? "P5" : "P6", image->nWidth, image->nHeight);
          const auto written = std::fwrite(output.data(), 1, bytes, file);
          std::fclose(file);
          if (written == bytes) std::printf("saved=%s\n", path.c_str());
        };
        save("left", left);
        save("right", right);
      }
      if (left) VzNL_ReleaseImage(&left);
      if (right) VzNL_ReleaseImage(&right);
      if (rc != 0) return 1;
    }
    return 0;
  }
  std::puts("Confirmed target not found.");
  return 1;
}
