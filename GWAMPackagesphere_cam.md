# usb\_cam #

This Package contains the Sphere camera video functionality.

  * Author: Román Navarro
  * License: BSD

**The usb\_cam manages the communication with the camera Logitech SphereCam**

This node process the video images and publishes them.

**Starting the usb\_cam**

`roslaunch logitech_usb_webcam low_res.launch`


## Interaction with the Usbcam Node ##

### The usb\_cam publishes the following topics (Default: 15Hz) ###

/logitech\_usb\_webcam/camera\_info

/logitech\_usb\_webcam/image\_raw

/logitech\_usb\_webcam/image\_raw/compressed

/logitech\_usb\_webcam/image\_raw/compressed/parameter\_descriptions

/logitech\_usb\_webcam/image\_raw/compressed/parameter\_updates

/logitech\_usb\_webcam/image\_raw/theora

/logitech\_usb\_webcam/image\_raw/theora/parameter\_descriptions

/logitech\_usb\_webcam/image\_raw/theora/parameter\_updates

### The usb\_cam offers the following services ###

/logitech\_usb\_webcam/get\_loggers

/logitech\_usb\_webcam/image\_raw/compressed/set\_parameters

/logitech\_usb\_webcam/image\_raw/theora/set\_parameters

/logitech\_usb\_webcam/set\_logger\_level