# RTK GPS

[Real Time Kinematic (RTK)](https://en.wikipedia.org/wiki/Real_Time_Kinematic) can increase the accuracy of GNSS/GPS systems to centimeter-level, allowing PX4 to be used in applications like precision surveying, where pinpoint accuracy is essential.

RTK GPS devices with a UBlox FP9 chip can also be used as an alternative to a compass for providing heading information.

Some RTK devices can output yaw as well as position infomation (i.e. an alternative way to get yaw from RTK).

:::note
RTK positioning and RTK GPS heading with U-blox FP9 modules can be used in parallel (this requires at least three RTK modules).
:::

This topic covers:
- [Supported RTK Devices](../gps_compass/rtk_gps.md#supported-rtk-devices)
- [Positioning Setup/Configuration](#positioning-setup-configuration)
- [RTK FP9 GPS Heading Setup/Configuration](#rtk-gps-heading-setup-configuration-u-blox-f9p)
- [RTK GPS that Output Yaw](#rtk-gps-that-output-yaw)


## Supported RTK Devices

PX4 supports the [u-blox M8P](https://www.u-blox.com/en/product/neo-m8p), [u-blox F9P](https://www.u-blox.com/en/product/zed-f9p-module) and the [Trimble MB-Two](https://www.trimble.com/Precision-GNSS/MB-Two-Board.aspx) GPS, and products that incorporate them.

The following RTK-compatible devices have been tested.
* [CUAV C-RTK GPS](../gps_compass/rtk_gps_cuav_c-rtk.md)
* [Drotek XL RTK GPS](../gps_compass/rtk_gps_drotek_xl.md)
* [Femtones MINI2 Receiver](../gps_compass/rtk_gps_fem_mini2.md)
* [Freefly RTK GPS](../gps_compass/rtk_gps_freefly.md) (F9P)
* [Here+ RTK GPS](../gps_compass/rtk_gps_hex_hereplus.md)
* [Holybro H-RTK F9P GNSS](../gps_compass/rtk_gps_holybro_h-rtk-f9p.md)
* [Holybro H-RTK M8P GNSS](../gps_compass/rtk_gps_holybro_h-rtk-m8p.md)
* [SparkFun GPS-RTK2 Board - ZED-F9P](https://www.sparkfun.com/products/15136) (www.sparkfun.com)
* [SIRIUS RTK GNSS ROVER (F9P)](https://store-drotek.com/911-1010-sirius-rtk-gnss-rover-f9p.html#/158-sensor-no_magnetometer) (store-drotek.com)
* [Trimble MB-Two](../gps_compass/rtk_gps_trimble_mb_two.md)

:::note
Some RTK modules can only be used in a particular role (base or rover), while others can be used interchangeably.
Only devices that include a u-blox F9P module can be used for [RTK GPS Heading Setup/Configuration](#rtk-gps-heading-setup-configuration-u-blox-f9p).
:::


## Positioning Setup/Configuration

RTK positioning requires a *pair* of [RTK GPS devices](#supported-rtk-devices): a "base" for the ground station and a "rover" for the vehicle.

In addition you will need:
- A *laptop/PC* with QGroundControl (QGroundControl for Android/iOS do not support RTK)
- A vehicle with a WiFi or Telemetry radio link to the laptop.

:::note
*QGroundControl* with a base module can theoretically enable RTK GPS for multiple vehicles/rover modules.
At time of writing this use case has not been tested.
:::


### Hardware Setup

#### Rover RTK Module (Vehicle)

Connect the vehicle-based module to the flight controller's GPS port (in the same way as any other GPS module).

The actual cables/connectors required will depend on the flight controller and selected RTK module (see [documentation for the selected device](#supported-rtk-devices) for more information).

#### Base RTK Module (Ground)

Connect the base module to *QGroundControl* via USB.
The base module must not be moved while it is being used.

:::tip
Choose a position where the base module won't need to be moved, has a clear view of the sky, and is well separated from any buildings.
Often it is helpful to elevate the base GPS, by using a tripod or mounting it on a roof.
:::

#### Telemetry Radio/WiFi

The vehicle and ground control laptop must be connected via [wifi or a radio telemetry link](../telemetry/README.md).

The link *must* use the MAVLink 2 protocol as it makes more efficient use of the channel.
This should be set by default, but if not, follow the [MAVLink2 configuration instructions](#mavlink2) below.


### RTK Connection Process

The RTK GPS connection is essentially plug and play:

1. Start *QGroundControl* and attach the base RTK GPS via USB to the ground station.
   The device is recognized automatically.
1. Start the vehicle and make sure it is connected to *QGroundControl*.

   :::tip
   *QGroundControl* displays an RTK GPS status icon in the top icon bar while an RTK GPS device is connected (in addition to the normal GPS status icon).
   The icon is red while RTK is being set up, and then changes to white once RTK GPS is active.
   You can click the icon to see the current state and RTK accuracy.
   :::
1. *QGroundControl* then starts the RTK setup process (known as "Survey-In").

   Survey-In is a startup procedure to get an accurate position estimate of the base station.
   The process typically takes several minutes (it ends after reaching the minimum time and accuracy specified in the [RTK settings](#rtk-gps-settings)).

   You can track the progress by clicking the RTK GPS status icon.
   
   ![survey-in](../../assets/qgc/setup/rtk/qgc_rtk_survey-in.png)

1. Once Survey-in completes:

   - The RTK GPS icon changes to white and *QGroundControl* starts to stream position data to the vehicle:
   
     ![RTK streaming](../../assets/qgc/setup/rtk/qgc_rtk_streaming.png)

   - Vehicle GPS switches to RTK mode.
     The new mode is displayed in the *normal* GPS status icon (`3D RTK GPS Lock`):

     ![RTK GPS Status](../../assets/qgc/setup/rtk/qgc_rtk_gps_status.png)


### Optional PX4 Configuration

The following settings may need to be changed (using *QGroundControl*).

#### RTK GPS settings

The RTK GPS settings are specified in the *QGroundControl* [General Settings](https://docs.qgroundcontrol.com/en/SettingsView/General.html#rtk_gps) (**SettingsView > General Settings > RTK GPS**).

![RTK GPS Setup](../../assets/qgc/setup/rtk/settings_view_general_rtk_gps.jpg)

These settings define the minimum duration and minimum accuracy for completing the RTK GPS setup process (known as "Survey-In).

:::tip
You can save and reuse a base position in order to save time: perform Survey-In once, select *Use Specified Base Position* and press **Save Current Base Position** to copy in the values for the last survey.
The values will then persist across QGC reboots until they are changed.
:::

#### MAVLink2

The MAVLink2 protocol must be used because it makes more efficient use of lower-bandwidth channels.
This should be enabled by default on recent builds.

To ensure MAVLink2 is used:
* Update the telemetry module firmware to the latest version (see [QGroundControl > Setup > Firmware](https://docs.qgroundcontrol.com/en/SetupView/Firmware.html)).
* Set [MAV_PROTO_VER](../advanced_config/parameter_reference.md#MAV_PROTO_VER) to 2 (see [QGroundControl Setup > Parameters](https://docs.qgroundcontrol.com/en/SetupView/Parameters.html))


#### Tuning

You may also need to tune some parameters as the default parameters are tuned assuming a GPS accuracy in the order of meters, not centimeters.
For example, you can decrease [EKF2_GPS_V_NOISE](../advanced_config/parameter_reference.md#EKF2_GPS_V_NOISE) and [EKF2_GPS_P_NOISE](../advanced_config/parameter_reference.md#EKF2_GPS_P_NOISE) to 0.2.


#### Dual Receivers

A second GPS receiver can be used as a backup (either RTK or non RTK).
See the [EKF2 GPS Configuration](../advanced_config/tuning_the_ecl_ekf.md#gps) section.

<!--
- Video demonstration would be nice.
- something that shows positioning of base, connection of RTK rover, survey in process. Some sort of short precision survey.
-->


### Vehicle Setup Example

The airframe build topic [DJI Flamewheel 450 with distance sensor and RTK GPS](../frames_multicopter/dji_flamewheel_450.md) describes an airframe setup with the Here+ RTK GPS and a Pixhawk 3 Pro.


## RTK GPS Heading Setup/Configuration (U-blox F9P)

Two u-blox F9P devices mounted on a vehicle can be used to accurately compute a heading angle (i.e. an alternative to compass-based heading estimation).
The two GPS devices in this scenario are referred to as the *Moving Base* and *Rover*.

### Supported Devices

This is the subset of devices above that have a U-blox F9P:
* [Freefly RTK GPS](../gps_compass/rtk_gps_freefly.md) (F9P)
* [Holybro H-RTK F9P GNSS](../gps_compass/rtk_gps_holybro_h-rtk-f9p.md)
* [SparkFun GPS-RTK2 Board - ZED-F9P](https://www.sparkfun.com/products/15136) (www.sparkfun.com)
* [SIRIUS RTK GNSS ROVER (F9P)](https://store-drotek.com/911-1010-sirius-rtk-gnss-rover-f9p.html#/158-sensor-no_magnetometer) (store-drotek.com)


### Configuration/Setup

:::tip
The general setup is described  in: [ZED-F9P Moving base applications (Application note)](https://www.u-blox.com/sites/default/files/ZED-F9P-MovingBase_AppNote_%28UBX-19009093%29.pdf).
:::

In overview:
- The UART2 of the GPS devices need to be connected together (TXD2 of the "Moving Base" to RXD2 of the "Rover")
- Connect UART1 on each of the GPS to (separate) unused UART's on the autopilot, and configure both of them as GPS with baudrate set to `Auto`.
  The mapping is as follows:
  - Main GPS = Rover
  - Secondary GPS = Moving Base
- Set [GPS_UBX_MODE](../advanced_config/parameter_reference.md#GPS_UBX_MODE) to `Heading` (1)
- Reboot and wait until both devices have GPS reception.
  `gps status` should then show the Main GPS going into RTK mode, which means the heading angle is available.

:::note
If using RTK with a fixed base station the secondary GPS will show the RTK state w.r.t. the base station.
:::

Ideally the two antennas should be identical, on the same level/horizontal plane and oriented the same way, and on an identical ground plane size and shape ([Application note](https://www.u-blox.com/sites/default/files/ZED-F9P-MovingBase_AppNote_%28UBX-19009093%29.pdf), section *System Level Considerations*)). 

Otherwise the antennas can be positioned as needed, but the [GPS_YAW_OFFSET](../advanced_config/parameter_reference.md#GPS_YAW_OFFSET) must be configured.
This offset is the angle made by the *baseline* (the line between the two GPS modules) relative to the vehicle x-axis (front/back axis, as shown [here](../config/flight_controller_orientation.md#calculating-orientation)).

:::note
The application note does not state the minimal required separation between modules.
50cm has been used in test vehicles running PX4.
:::

To activate heading fusion for the attitude estimation, set the [EKF2_AID_MASK](../advanced_config/parameter_reference.md#EKF2_AID_MASK) parameter to enable *GPS yaw fusion*.


## RTK GPS that Output Yaw 

Some RTK GPS units (i.e. with multiple antennas) can output a yaw angle.
This can be used instead of the heading from the magnetic compass, and as an alternative to [RTK Heading with Two FP9 modules](#rtk-gps-heading-setup-configuration-u-blox-f9p).

###  Supported Devices

* [Trimble MB-Two](../gps_compass/rtk_gps_trimble_mb_two.md)

### Configuration

[RTK Positioning](#positioning-setup-configuration) should be set up as normal.

Then enable yaw by setting bit position 7 in [EKF2_AID_MASK](../advanced_config/parameter_reference.md#EKF2_AID_MASK) to 1 (add 128 to the parameter value).

