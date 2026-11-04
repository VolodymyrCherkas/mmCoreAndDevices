#include "EmergentCamera.h"
#include "ModuleInterface.h"

#include <cstring>

const char* g_EmergentCameraName = "EmergentCamera";

MODULE_API void InitializeModuleData()
{
   RegisterDevice(g_EmergentCameraName, MM::CameraDevice,
                  "Emergent Vision Technologies monochrome camera (eSDK)");
}

MODULE_API MM::Device* CreateDevice(const char* deviceName)
{
   if (deviceName != nullptr && std::strcmp(deviceName, g_EmergentCameraName) == 0)
      return new EmergentCamera();
   return nullptr;
}

MODULE_API void DeleteDevice(MM::Device* device)
{
   delete device;
}
