# TetonBrainWorksHB_UI3_Dev2
Tilta Saddle User Interface
Processor: nRF52832

The program is executable only if the files are copied to the following folder: /nRF5_SDK_17.0.2_d674dde/examples/My_nRF/Projects/ where the folder name, My_nRF_Projects is arbitrary.
be sure to copy hb_ui_3.emProject and hb_ui_3.emSession to the folder: /nRF5_SDK_17.0.2_d674dde/examples/My_nRF_Projects/hb_ui_3/pca10040/s132/ses
In original SDK, the nRF5 SDK path for main.c was /nRF5_SDK_17.0.2_d674dde/examples/My_nRF_Projects/hb_ui_3/main.c
The base software is ble_app_blinky.
Four user-activated external switches are read and their states are transmitted to a Central Device via BLE.
The board support package was not used.
