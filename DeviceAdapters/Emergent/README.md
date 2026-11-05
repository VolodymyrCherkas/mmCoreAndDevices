# EmergentCamera for Micro-Manager

Micro-Manager device adapter for Emergent monochrome cameras. Supports Snap,
Live, sequences, exposure, binning, hardware ROI, gain and trigger/GPIO controls.
Camera settings can be saved and restored between Micro-Manager sessions.

**Developer:** Dr. Volodymyr Cherkas

**Company:** [Advanced Optical Devices](https://www.ao-devices.com)

**Manufacturer:** [Emergent Vision Technologies](https://www.emergentvisiontec.com)

Advanced Optical Devices is a system integrator and Emergent reseller in
Poland and the European Union.

## Compatibility

**macOS is not supported, including Intel Macs and Apple silicon.**

| Platform | Status |
| --- | --- |
| Windows x64 | Hardware-tested with the configuration below |
| Linux x86_64, Ubuntu 24.04 | Experimental; cross-compiled and statically checked; Linux runtime and camera tests pending |
| macOS | Not supported |

Tested with **Micro-Manager 2.0.3 (20261008), Windows x64, Emergent eSDK
4.08.06**, and an **HE-25000SBL-T-M** camera with firmware **3.35**. Other
camera models and SDK versions have not been validated with the Windows binary.

The Linux binary uses **Emergent eSDK 4.08.05 for Ubuntu 24.04 x64**.
Both binaries require **Micro-Manager Device API 75 / Module API 10**.

## Windows installation

1. Install [Micro-Manager](https://micro-manager.org/Download_Micro-Manager_Latest_Release)
   for Windows x64 and the compatible
   [Emergent eSDK](https://www.emergentvisiontec.com/downloads/).
   This binary was tested with eSDK 4.08.06; obtain that version from Emergent
   if it is not listed.
2. Configure the camera and network interface using Emergent's instructions.
   Confirm that the camera captures in eCapture, then close eCapture and Micro-Manager.
3. Download [mmgr_dal_EmergentCamera.dll](mmgr_dal_EmergentCamera.dll) and copy it
   into the folder containing `ImageJ.exe`, normally:

   ```text
   C:\Program Files\Micro-Manager-2.0\
   ```

4. Start Micro-Manager. Open **Devices > Hardware Configuration Wizard**, add
   **EmergentCamera / EmergentCamera**, and select the discovered camera.
   Verify **Camera IP Address**, or choose **Manual IP** and enter its actual
   address. The adapter's fallback address, `192.168.207.207`, is not a universal
   camera address.
5. Select the device as the default camera, finish the wizard, and save the
   hardware `.cfg` file. For the first **Snap/Live** test, use
   **Trigger - Mode = Internal** and **PTP - Mode = Off**.

If the adapter is unavailable, check CoreLog, install the
[Microsoft Visual C++ v14 x64 runtime](https://learn.microsoft.com/en-us/cpp/windows/latest-supported-vc-redist),
and ensure these SDK folders are on `PATH` (adjust for your SDK location), then
restart Micro-Manager:

```text
C:\Program Files\EVT\eSDK\bin
C:\Program Files\EVT\eSDK\genicam\bin\Win64_x64
C:\Program Files\EVT\eSDK\ffmpeg\bin
```

## Linux installation (experimental)

1. Install Linux x86_64 Micro-Manager with the matching device/module APIs,
   and [Emergent eSDK](https://www.emergentvisiontec.com/downloads/)
   **4.08.05 for Ubuntu 24.04 x64**. Use its supported kernel and NIC drivers;
   the supplied SDK package targets kernel **6.17.0**. Confirm capture using
   the vendor software, then close it.
2. Download [libmmgr_dal_EmergentCamera.so.0](libmmgr_dal_EmergentCamera.so.0)
   and place it beside Micro-Manager's other `libmmgr_dal_*.so.0` adapters.
   Keep the `.so.0` filename.
3. Make the SDK libraries available before starting Micro-Manager. For the
   default SDK installation, run:

   ```bash
   export LD_LIBRARY_PATH="/opt/EVT/eSDK/lib:/opt/EVT/eSDK/genicam/bin/Linux64_x64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
   ```

4. Run `ldd` on the downloaded adapter and resolve any `not found` dependencies.
   Start Micro-Manager from that terminal and configure the camera as described
   in Windows steps 4-5 above.

## Settings

Exposure and binning are available in the main window. Other controls are in
**Devices > Device Property Browser**. Frame-rate limit defaults to **Off**;
an explicitly saved On preference is restored on the next startup.

`Emergent.ini` is created beside the adapter binary and stores settings separately for
each camera serial number. Settings are saved on a **clean close/device
unload** and restored at initialization. To save immediately, stop Live and
set **Settings - Save = Save now**. Check **Settings - Status** for the result.

If the adapter folder is not writable, set **Settings - File** to an
absolute path in a writable folder during hardware configuration.
**Settings - Remember = Off** disables automatic save/restore. The `.cfg`
file still selects hardware and camera IP addresses.

## Notes

- Supported output formats are unpacked Mono8, Mono10 and Mono12. The latter
  two use 16-bit image buffers. Colour and packed formats are not implemented.
- On the tested camera, GPO0/GPO1 are fixed outputs and GPI2/GPI3 are fixed
  inputs. Defaults are GPI2 acquisition start and GPO1 exposure active,
  active high. Saved settings override defaults. See the
  [manufacturer's GPIO pinout](https://docs.emergentvisiontec.com/cameras/gpio-ports-schematics)
  before connecting external equipment.
- PTP diagnostics are exposed, but master lock and synchronization accuracy
  have not been verified. Scheduled PTP acquisition is not implemented.
