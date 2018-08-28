# 5DoF_FI
Firmware of the STM32 Nucleof303k8

1. Copy the folders of libraries from the repository to ~/Arduino/libraries

2. Add the supported boards to the arduino IDE, by going to File->Preferences and in the field "Additional Boards URL" enter: https://github.com/stm32duino/BoardManagerFiles/raw/master/STM32/package_stm_index.json

3. Go to tools->board:"XX"->Boards Manager, look for STM32 Cores by ST-Microelectronics ( we are using version 1.3.0)

4. Small hack: In the file boards.txt (~/.arduino15/packages/STM32/hardware/stm32/1.3.0) include the flags in the .build.mcu of the nucleo board :

-mfpu=fpv4-sp-d16 -mfloat-abi=hard

like so:     Nucleo_32.menu.pnum.NUCLEO_F303K8.build.mcu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard 


3. FYI, the calibration values for the foot platforms are: 

    Right Leg: 
    #define ENCODERSCALE1 0.350F/7360.0F #define ENCODERSCALE2 0.293F/7560.0F #define ENCODERSCALE3 90.F/2140.0F
    int CS4 = D6; //! CS4 -> Lateral , int CS5 = D9; //! CS5  -> Dorsi/Plantar Flexion , int CS6 = D10; //! CS6 -> Flexion/Extension of the Leg       
    #define ENCODERSIGN1  1 , #define ENCODERSIGN2, #define ENCODERSIGN3 -1

    Left leg:
    #define ENCODERSCALE1 0.350F/7360.0F/2.244126428F/0.9495171863F #define ENCODERSCALE2 0.293F/7560.0F/2.170616483F #define ENCODERSCALE3 90.F/2140.0F/2.0644889F 
    #define ENCODERSIGN1  1 , #define ENCODERSIGN2, #define ENCODERSIGN3 -1
    int CS1 = D6; -> Lateral , int CS2 = D9; -> Dorsi/Plantar Flexion , int CS3 = D10;-> Flexion/Extension of the Leg
    
