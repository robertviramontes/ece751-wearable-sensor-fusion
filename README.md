# ece751-wearable-sensor-fusion
Repository for ECE751 project on wearable sensor fusion for heart rate measurement.

# Pre-Requisites
## Matlab
Install MATLAB and the Simulink Coder and Embedded Coder packages. Those should automatically bring in all the packages they require to support them. 

You should also install the Nucleo hardaware support package. [Mathworks provides instructions for this](https://www.mathworks.com/help/releases/R2021b/supportpkg/nucleo/ug/intro.html).

## Nulceo Board Drivers
The Nucleo board requires drivers to interact with MATLAB. They can be downloaded directly from STM. [Mathworks has documentation](https://www.mathworks.com/help/releases/R2021b/supportpkg/nucleo/ug/install-drives-for-simulink-coder-support-package-for-stmicroelectronics-nucleo-boards.html) on what to do with the drivers once you've downloaded them.

# Getting Started
## Hold To Blink Demo
The [hold_to_blink.slx](src/examples/hold_to_blink.slx) is a small demo that can be used to ensure your MATLAB environment is set up correctly. This should blink the green user LED (LD2) when the blue button is held down. After opening the Simulink model, it can be deployed to the board by navigating to the Hardware tab and clicking "Build, Deploy, and Start".
![image](https://user-images.githubusercontent.com/17224381/139503508-1a3c439e-b555-469d-a9e1-5aab41f5c8ad.png)



# Tips
## Built-In Pin Viewer
When editing a block that interacts with board IO, there is a helpful "View pin map" button that will bring up a board layout!
![image](https://user-images.githubusercontent.com/17224381/139503670-f9633ce8-ceb6-4045-a5f4-4b0e6f498373.png)
