#include <cstdio>
#include <vector>
#include "VZNL_Common.h"
#include "VZNL_EyeConfig.h"

// 只搜索并读取设备列表，不绑定、打开、采集或修改设备配置。
int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  SVzNLConfigParam config{};
  VzNL_SetLogLevel(keNLLogLevel_None, keNLLogType_None);
  const int init = VzNL_Init(&config);
  std::printf("SDK init result: %d\n", init);
  if (init != 0) return 1;
  const int search = VzNL_ResearchDevice(keSearchDeviceFlag_EthLaserRobotEye);
  std::printf("Device search result: %d\n", search);
  int count = 0;
  const int query = VzNL_GetEyeCBDeviceInfo(nullptr, &count);
  std::printf("Device list result: %d; count: %d\n", query, count);
  if (query == 0 && count > 0 && count <= 256) {
    std::vector<SVzNLEyeCBInfo> devices(count);
    const int result = VzNL_GetEyeCBDeviceInfo(devices.data(), &count);
    std::printf("Read devices result: %d\n", result);
    if (result == 0 && count >= 0 && count <= static_cast<int>(devices.size())) {
      for (int i = 0; i < count; ++i) {
        const auto& d = devices[i];
        std::printf("name=%.*s ip=%.*s mac=%.*s valid=%d description=%.*s\n",
                    int(sizeof d.szDeviceName), d.szDeviceName,
                    int(sizeof d.byServerIP), d.byServerIP,
                    int(sizeof d.byMAC), d.byMAC, int(d.bValidDevice),
                    int(sizeof d.szDesc), d.szDesc);
        std::printf("guid=%.*s device_id=%.*s\n",
                    int(sizeof d.szGUID), d.szGUID,
                    int(sizeof d.szDeviceID), d.szDeviceID);
        SVzNLEthernetEyeConfigInfo details{};
        const int info_result = VzNL_GetEthernetEyeConfigInfo(&d, &details);
        std::printf("Network information result: %d\n", info_result);
        if (info_result == 0) {
          std::printf("vendor=%.*s product=%.*s serial=%.*s ip_type=%d\n",
                      int(sizeof details.vendor), details.vendor,
                      int(sizeof details.productName), details.productName,
                      int(sizeof details.serialID), details.serialID,
                      int(details.eEthernetIPType));
        }
        int in_use_result = 0;
        const auto in_use = VzNL_IsDeviceInUse(&d, &in_use_result);
        std::printf("Device in use: %d; query result: %d\n", int(in_use), in_use_result);
      }
    }
  }
  VzNL_Destroy();
  return search == 0 && query == 0 ? 0 : 1;
}
