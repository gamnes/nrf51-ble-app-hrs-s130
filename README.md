nRF51-ble-app-hrs-s130
======================
This example shows how a S130 application can be set up with the same structure as the SDK examples. The example is built on ble_app_hrs_c (S120), and ble_app_hrs (S110) has been added to the project. The functionality of the application is explained in main.c.
Requirements
------------
* nRF51 series SDK 6.1.0 
* S130 0.5.0-1 alpha 
* nRF51822 Development kit (nRF6310 motherboard + development kit)
  
The project may need modifications to work with other versions or other boards. 
 
To compile it, clone the repository in the \nrf51822\Board\nrf6310 folder. 
  
In addition, you should **add the S130 header files in a new folder called "s130" in the following folder**: ..\nrf51822\Include  

About this project
------------------
This application is one of several applications that has been built by the support team at Nordic Semiconductor, as a demo of some particular feature or use case. It has not necessarily been thoroughly tested, so there might be unknown issues. It is hence provided as-is, without any warranty.
  
However, in the hope that it still may be useful also for others than the ones we initially wrote it for, we've chosen to distribute it here on GitHub. 
  
The application is built to be used with the official nRF51 SDK, that can be downloaded from https://www.nordicsemi.no, provided you have a product key for one of our kits.
 
Please post any questions about this project on https://devzone.nordicsemi.com.
