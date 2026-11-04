# EmergentCamera Micro-Manager device adapter

This adapter targets the installed Emergent Vision Technologies eSDK and a
monochrome GigE camera selected by IP address. The default IP is
`192.168.207.207`. Its interfaces were checked against the installed
Micro-Manager 2.0 core (`MMCore 12.5.0`, Device API 75, Module API 10).

Implemented functionality:

- automatic camera discovery plus manual-IP selection;
- snap acquisition and Micro-Manager sequence acquisition;
- exposure in milliseconds (translated to the eSDK `Exposure` value in
  microseconds);
- analog gain in dB and automatic gain control;
- hardware ROI with the camera's width, height, and offset increments;
- writable ROI center coordinates that calculate and align the hardware offsets;
- hardware binning when the camera exposes a writable `Bin` node;
- independent 8/10-bit ADC selection and Mono8/10/12 transport format;
- hardware horizontal and vertical image reversal;
- continuous and hardware MultiFrame acquisition;
- PGA gain, analog offset, automatic gain/exposure tuning, auto-control ROI,
  LUT, frame-rate, and GPIO controls;
- internal, software, and external trigger modes;
- GPI trigger input/edge/end-exposure/delay controls and GPO source/polarity
  controls;
- live read-only sensor, camera/FPGA, and network-PHY temperatures;
- native unpacked `Mono8`, `Mono10`, and `Mono12` output;
- one-component 8-bit or 16-bit Micro-Manager image buffers;
- clean stream/buffer teardown and detailed eSDK errors in CoreLog.

The connected camera was probed with the vendor API while this adapter was
written:

- model: `HE-25000SBL-T-M`;
- firmware: `3.26`;
- sensor: `5328 x 4608`;
- supported native mono formats: `Mono8`, `Mono10`, and `Mono12`;
- ROI increments: 8 pixels horizontally and 16 pixels vertically;
- exposure range: 0.094 to 244.205 ms.
- analog gain range: 0 to 30.1019 dB;
- trigger inputs: `GPI_2` and `GPI_3`;
- trigger outputs: `GPO_0` and `GPO_1`;
- hardware sensor binning: not available (`Bin` is disabled by this model's
  firmware), so Micro-Manager correctly reports read-only `Binning=1`.

## Camera properties and display order

The pre-initialization **Camera** property lists every camera found by eSDK as
model, serial number, and IP address. Select **Manual IP** to use **Camera IP
Address** directly.

Micro-Manager stores device properties alphabetically rather than in adapter
creation order. Runtime names therefore begin with stable numeric section and
item prefixes so the Property Browser always presents them in this order:

1. **01 - Camera ID / Temperature**: read-only discovery summary, model, serial,
   firmware, sensor, camera, and network-PHY values;
2. **02 - Acquisition / Exposure / Readout**: acquisition mode, dynamic or
   exact frame-rate control, exposure and auto-exposure, ADC mode, and pixel
   type;
3. **03 - Gain / Auto Control ROI**: analog and PGA gain, offset, automatic-gain
   controls, and its metering region;
4. **04 - ROI**: binning, offsets, dimensions, and axis-centering commands;
5. **05 - Trigger / GPIO**: trigger mode and timing, then GPO 0, GPO 1, GPI 2,
   and GPI 3 with every line's related controls adjacent;
6. **09 - Other**: hardware image reversal and LUT controls.

In software mode the adapter issues `TriggerSoftware` for each snap or sequence
frame. In external mode it waits for the selected GPI trigger, with a 60-second
per-frame timeout. `Internal` uses the camera's free-running acquisition.

The temperature fields are read-only and refresh whenever Micro-Manager updates
device properties. Set **04.20 - ROI - Center X** or **04.21 - ROI - Center Y**
to `Center now` to center that ROI axis on the sensor. The command immediately
returns to `Idle`; the aligned result is shown in **04.10 - ROI - Offset X** or
**04.11 - ROI - Offset Y**. Micro-Manager's adapter API has no native button
property type, so these are repeatable momentary command dropdowns in the
standard Property Browser.

ADC mode and pixel type are independent. The ADC controls sensor conversion
depth; `PixelType` controls the transported Mono8/10/12 representation. The
adapter preserves the active ROI when either setting changes.

**02.03 - Acquisition - Frame Rate Limit** controls how
**02.04 - Acquisition - Frame Rate (fps)** is applied. `Off` requests the camera's current maximum
frame rate. The adapter queries and reapplies that dynamic maximum whenever
exposure or readout geometry changes, so shorter exposures can run faster.
`On` applies the exact requested rate, clamped only when the exposure/readout
combination makes it physically unattainable. The displayed rate and its upper
limit refresh after every exposure change.

When **02.01 - Acquisition - Mode** is `MultiFrame`, a finite Micro-Manager sequence
programs `AcquisitionFrameCount` to the requested sequence length. This camera
supports 1–1000 frames per hardware MultiFrame acquisition. Continuous mode is
used for unlimited sequences.

The adapter exposes the camera's available, safe imaging/acquisition controls.
Raw registers, firmware/reset commands, transport/network configuration, and
duplicate raw representations such as `GainRaw`/`OffsetSigned` are intentionally
not exposed because changing them can disconnect the camera or conflict with
the higher-level properties.

## Repository contents

```text
EmergentCamera/
  EmergentCamera.cpp
  EmergentCamera.h
  Module.cpp
  EmergentCamera.vcxproj
  EmergentCamera.vcxproj.filters
prebuilt/windows-x64/
  mmgr_dal_EmergentCamera.dll
README.md
.gitignore
```

This repository contains the adapter source, its Visual Studio project, and a
precompiled Windows x64 Release adapter DLL.
Micro-Manager headers, libraries, and shared build settings come from a separate
`mmCoreAndDevices` checkout. The Emergent eSDK headers, import libraries, and
runtime must be installed separately. Neither dependency is bundled here.
The adapter project is designed to build inside the Micro-Manager source tree,
as described below; it does not build independently from this repository root.

## Install the precompiled adapter (Windows x64)

The existing Release build is included at
[`prebuilt/windows-x64/mmgr_dal_EmergentCamera.dll`](prebuilt/windows-x64/mmgr_dal_EmergentCamera.dll).

1. Install the classic Emergent eSDK runtime and the required camera/NIC drivers.
2. Close Micro-Manager.
3. Copy the included `mmgr_dal_EmergentCamera.dll` into the installation
   directory of your 64-bit Micro-Manager, for example
   `C:\Program Files\Micro-Manager-2.0`.
4. Make the eSDK runtime discoverable using the `PATH` instructions below,
   restart Micro-Manager, and follow **Configure and test**.

The included DLL was copied from the existing Windows x64 Release build; it
was not rebuilt during project extraction. The exact eSDK version used for
that build was not recorded. The eSDK runtime is installed separately.

## Build (Windows x64)

1. Install Visual Studio 2022 with **Desktop development with C++**. The
   adapter project uses toolset `v143`.
2. Obtain the current
   [mmCoreAndDevices source](https://github.com/micro-manager/mmCoreAndDevices).
3. Copy this repository's `EmergentCamera` directory to
   `mmCoreAndDevices\DeviceAdapters\EmergentCamera`.
4. Open `mmCoreAndDevices\micromanager.sln`, choose **x64** and **Release**, and
   add `EmergentCamera.vcxproj` to the solution (Add > Existing Project).
5. Build the `EmergentCamera` project. It defaults to the SDK at
   `C:\Program Files\EVT\eSDK`; override the MSBuild property `ESDK_ROOT` if
   needed.
6. Copy the generated `mmgr_dal_EmergentCamera.dll` into
   `C:\Program Files\Micro-Manager-2.0`.

The SDK runtime must be discoverable when ImageJ/Micro-Manager starts. The
eSDK installer normally configures this. If the Hardware Configuration Wizard
shows the adapter as unavailable, add these directories to the system `PATH`
and restart Micro-Manager:

```text
C:\Program Files\EVT\eSDK\bin
C:\Program Files\EVT\eSDK\genicam\bin\Win64_x64
C:\Program Files\EVT\eSDK\ffmpeg\bin
```

## Configure and test

1. Close eCapture; only one application should control the camera.
2. Start Micro-Manager and open **Devices > Hardware Configuration Wizard**.
3. Add **EmergentCamera / EmergentCamera**.
4. Leave **Camera IP Address** at `192.168.207.207` and initialize the device.
5. Set it as the Core camera, then test **Snap** and **Live** first with
   **02.21 - Readout - Pixel Type** set to `Mono8` and a short exposure.

`Mono10` and `Mono12` are returned as 16-bit little-endian grayscale buffers;
the significant bit depth reported to Micro-Manager is 10 or 12 respectively.
Packed formats are deliberately omitted because the camera provides their
unpacked equivalents, avoiding a conversion and extra buffer copy.

For loading/runtime errors, enable **Tools > Options > Debug Logging** and
inspect the Micro-Manager CoreLog. eSDK failures include their numeric error
code in the log.
