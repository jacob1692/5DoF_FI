# 5DoF_FI
Firmware of the STM32 Nucleof303k8

1. Copy the folders of libraries from the repository to ~/Arduino/libraries

2. In the file boards.txt (~/.arduino15/packages/STM32/hardware/stm32/1.3.0) include the flags in the .build.mcu of the nucleo board :

-mfpu=fpv4-sp-d16 -mfloat-abi=hard

like so:     Nucleo_32.menu.pnum.NUCLEO_F303K8.build.mcu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard 


3. FYI, the calibration values for the foot platforms are: 

    Right Leg: #define ENCODERSCALE1 0.350F/7360.0F   
               #define ENCODERSCALE2 0.293F/7560.0F
               #define ENCODERSCALE3 90.F/2140.0F
               
               int CS4 = D6; //! CS4 -> Lateral 
               int CS5 = D9; //! CS5  -> Dorsi/Plantar Flexion
               int CS6 = D10; //! CS6 -> Flexion/Extension of the Leg
               
               #define ENCODERSIGN1  1
               #define ENCODERSIGN2  
               #define ENCODERSIGN3 -1
