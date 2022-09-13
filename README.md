# ELMO

This repository contains drivers and tools for the tabletop robot Elmo.

The drivers are implemented in ROS. Alongside the ROS workspace there is a companion application implemented in PyQT5 to control the robot. The application communicates with the robot through an REST Api.

The robot features the following devices:

    - Raspberry PI 4 Running PiOS Buster;
    - Battery
    - 13x13 Led matrix;
    - Touch sensor in front of the led matrix;
    - Pan/Tilt mechanism for neck movement;
    - Touchscreen;
    - USB Camera with microphone;
    - 5 Touch sensors on top of head.

On the back of the robot there is a panel with a button to turn the robot On and Off, as well as a plug to connect a charger and a status led.

## First usage

Upon receiving the robot, plug the charger into the charging port on the back of the robot and press the power button.

After the robot powers on it will launch it's programs and attempt to connect to a Wifi, if one is configured.

If the connection to a wifi network is not successfull, the robot will become a wifi hotspot called Elmo, with password "asdf", without quotation marks.

You can connect to the hotspot and run the companion application.

The companion application will start by opening a dialog window while scanning the network for robots. For each robot found, a button will appear. Clicking a button will connect the application with the robot, closing the dialog.

In the main application window there are several panels you can use to:

    - control the chest LEDs;
    - change the screen;
    - play Audio files;
    - toggle behaviours;
    - inspect touch sensors;
    - control the pan/tilt motors;
    - upload and delete multimedia files;
    - upload wifi credentials;
    - shutdown;

## Companion app

The companion app source code is available inside the catkin_elmo/app folder.

The application was built using PyQt5, and the layout was created with QT Designer 5.

The following scripts are available in this folder:

    - rebuild_ui.sh -> converts the main_window.ui file, produced with QT Designer, into main_window_ui.py, which is used by the application file;
    - app.py -> runs the main application;
    - dev.sh -> rebuilds the ui before running the main application;
    - build.sh -> attempts to build an executable file for the application, using pyinstaller;

The application communicates with robots using the robot_client.py module.

The network is scanned using UDP Broadcast requests. After a connection is established, the app will communicate with the robot using an HTTP Rest API.

## REST API

A REST API is exposed by the module catkin_elmo/src/elmo/src/robot_server.py

The module exposed the following endpoints:

    [GET] /status -> returns the state of the robot
        accepts: {}
        returns: { success: boolean, message: string }
    [POST] /command -> updates the state
        accepts: { op: string, ...}
        returns: { success: boolean, message: string }

The robot status is an object with the following description:

    {
        pan: int -> current pan value
        tilt: int -> current tilt value
        pan_min: int -> minimum pan value
        pan_max: int -> maximum pan value
        tilt_min: int -> minimum tilt value
        tilt_max: int -> maximum til value
        pan_torque: boolean -> is torque enabled for pan
        tilt_torque: boolean -> is torque enabled for tilt
        pan_temperature: int -> pan temperature
        tilt_temperature: int -> tilt temperature
        touch_chest: boolean -> is chest touch sensor active
        touch_head_n: boolean -> is head north touch sensor active
        touch_head_s: boolean -> is head south touch sensor active
        touch_head_e: boolean -> is head east touch sensor active
        touch_head_w: boolean -> is head west touch sensor active
        touch_threshold: int -> touch sensor calibration threshold
        behaviour_test_motors: boolean -> is test motors behaviour active
        behaviour_test_leds: boolean -> is test leds behaviour active
        speech_list: string[] -> speech files
        sound_list: string[] -> sound files
        image_list: string[] -> image files
        icon_list: string[] -> icon files
        volume: int -> current volume percentage
        video_port: string -> port of video server for camera
        video_path: string -> path of video server for camera
        multimedia_port: int -> port of multimedia server
        image_address: string -> path of image files in multimedia server
        icon_address: string -> path of icon files in multimedia server
        sound_address: string -> path of sound files in multimedia server
        speech_address: string -> path of speech files in multimedia server
    }

    The available robot commands are the following:

        {
            "op": "enable_behaviour",
            "name": string -> name of behaviour to enable,
            "control": boolean -> True to enable, False to disable
        }

        {
            "op": "update_touch_threshold",
            "threshold": int -> the new threshold
        }

        {
            "op": "set_pan_torque",
            "control": boolean -> True to enable torque
        }

        {
            "op": "set_tilt_torque",
            "control": boolean -> True to enable torque
        }

        {
            "op": "set_pan",
            "angle": boolean -> angle to set pan motor
        }

        {
            "op": "set_tilt",
            "angle": boolean -> angle to set tilt motor
        }

        {
            "op": "update_motor_limits",
            "pan_min": int -> minimum value for pan,
            "pan_max": int -> maximum value for pan,
            "tilt_min": int -> minimum value for tilt,
            "tilt_max": int -> maximum value for tilt,
        }

        {
            "op": "play_sound",
            "name": string -> name of sound file to play,
        }

        {
            "op": "play_speech",
            "name": string -> name of speech file to play,
        }

        {
            "op": "set_volumme",
            "volume": int -> volume to set,
        }

        {
            "op": "set_screen",
            "image": string -> name of image file to set, if any,
            "text": string -> text to show, if any,
            "url": string -> url to load, if any
            "camera": boolean -> True to show camera feed
        }

        {
            "op": "update_leds",
            "colors": int[13*13][3] -> rgb values (max 255) for each led, in row major order
        }

        {
            "op": "update_leds_icon",
            "name": string -> name of icon file to set
        }

        {
            "op": "update_wifi_credentials",
            "ssid": string -> SSID of network,
            "password": string -> password of network
        }

        {
            "op": "shutdown",
        }

## Developing Elmo

When developing modules for Elmo, it may be correct to use the ROS API. In order to facilitate development, the module catkin_elmo/src/elmo/src/robot.py exposes several classes to access the underlying ROS system.

## Notes

The camera driver is not handled by the ROS usb_cam package, as is common. Instead, the program motion was installed, which exposes the camera feed via http.

The default network manager for Pi OS Buster is not used, instead nmcli is running the show. The script catkin_elmo/src/elmo/configure_hotspot.sh shows how to configure it.