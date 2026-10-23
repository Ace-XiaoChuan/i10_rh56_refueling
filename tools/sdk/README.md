# AUBO SDK 开发包

本机使用官方 `aubo_sdk-0.23.1-Linux_x86_64+71831e1`，与现场软件显示的 SDK 版本一致。厂家开发包保持原样，共 190 个文件；共享库、调试库和下载压缩包保留在本机，Git 保存本说明和逐文件 SHA-256 清单。

官方入口：[SDK 下载目录](https://download.aubo-robotics.cn/sdk/)、[本版本 Linux x86_64 ZIP](https://download.aubo-robotics.cn/sdk/aubo_sdk-0.23.1-Linux_x86_64+71831e1.zip)、[开发准备](https://developer.aubo-robotics.cn/guide_sdk/4_dev_preparation/2_installation.html)。下载后在项目根目录解压：

```bash
curl -fL --output /tmp/aubo_sdk-0.23.1.zip \
  'https://download.aubo-robotics.cn/sdk/aubo_sdk-0.23.1-Linux_x86_64+71831e1.zip'
unzip /tmp/aubo_sdk-0.23.1.zip -d tools/sdk
```

预期目录为 `tools/sdk/aubo_sdk-0.23.1-Linux_x86_64+71831e1/`。文件摘要记录在 [manifest_0.23.1_linux_x86_64.json](manifest_0.23.1_linux_x86_64.json)，由当前已验证的本地包生成；可据此核对重新下载的内容，不把清单视为厂家数字签名。

编译与只读查询见[检查工具](../aubo_sdk_probe/README.md)。接口以此版本 `include/` 头文件和随包示例为准；在线 API 文档可能已更新到更高版本。
