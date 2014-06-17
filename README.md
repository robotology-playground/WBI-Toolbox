WBI Toolbox (WBI-T) - Simulink Wrapper for Whole Body Control
-------------------------------------------------------------

This document contains basic instructions on how to install this toolbox, *tips and tricks* to do so and a walkthrough to get you started using it. Simulink blocks consist of S-functions (http://goo.gl/1GuHVd) which allow C/C++ user specific code compiled as Matlab Executable (MEX) files, thus extending the capabilities of the Simulink environment. In other words, MEX files have been created linking YARP, iCub, **iDynTree** (a more efficient and generic YARP-based robot dynamics library than its predecessor iDyn - http://goo.gl/BnGzKr) and CoDyCo, wrapping the **Whole Body Interface** described in http://goo.gl/dBWO3k. The following video shows CoDyCo's 1st year results on iCub in which the top level controller has been implemented with the WBI-Toolbox and runs at a 10ms rate!


<p align="center">
<a href="https://www.youtube.com/watch?v=jaTEbCsFp_M
" target="_blank"><img src="http://img.youtube.com/vi/jaTEbCsFp_M/0.jpg" 
alt="iCub balancing via external force control" width="480" height="360" border="10" /></a>
</p>



###### Main Goal ######
> The library should allow non-programming experts or those researchers just getting acquainted with Whole Body Control to more easily deploy controllers either on simulation or a real YARP-based robotic platform, as well as to analyze their performance and take advantage of the innumerable MATLAB and Simulink toolboxes. We like to call it "rapid controller prototyping" after which a proper YARP module should be made for hard real time performance and final deployment.


###### Requirements
* Matlab V. 7.1+ and Simulink (Tested with Matlab R2014a, R2013a, R2012a/b, R2011a)
* YARP (-IMPORTANT- compiled as shared library, currently a default yarp configuration option)
* CoDyCo (https://github.com/robotology-playground/codyco-superbuild)
* iCub (https://github.com/robotology/icub-main)
* Gazebo Simulator (http://gazebosim.org/)
* gazebo_yarp_plugins (https://github.com/robotology/gazebo_yarp_plugins).

**Operating Systems supported: Linux, MAC OS X, Windows.**

**Note: The following instructions are for Linux distributions, but it works similarly on the other operating systems.**

###### Compiling the Toolbox MEX Files
The WBI-Toolbox can be compiled through the CoDyCo project (https://github.com/robotology-playground/codyco-superbuild). This is the easiest and recommended way to do so. In the following steps assume that `$CODYCO_SUPERBUILD_DIR` points to the `/build` directory of your CoDyCo installation and `$CODYCO_SUPERBUILD_ROOT` to the corresponding root directory of your installation. In case you are using the simulator, make sure that the iCub models are being loaded and the `gazebo_yarp_plugins` properly working. This is easy to verify as you need only to launch a `yarpserver` followed by Gazebo and load the desired model, be it iCub (fixed) or iCub. If the robot does not fall under the effect of gravity, it means the plugins are working and you can go ahead with the installation of the Toolbox.

- **Check the matlab configuration.** Before going ahead with the compilation of the library, make sure that you have MATLAB and Simulink properly installed and running. Then, check that the MEX compiler for MATLAB is setup and working. For this you can try compiling some of the MATLAB C code examples as described in [http://www.mathworks.com/help/matlab/ref/mex.html#btz1tb5-12]. 

- **Compiling the WBI Toolbox.** Before compiling, you need to configure the project via CMake. A few flags need to be taken into account when doing this. When using the Gazebo simulator do:

```bash
   cd $CODYCO_SUPERBUILD_DIR
   cd build
   ccmake ../ -DCODYCO_USES_WBI_TOOLBOX:BOOL=YES -DCODYCO_USES_URDFDOM:BOOL=YES -DICUBWBI_USE_EXTERNAL_TORQUE_CONTROL:BOOL=NO
```
When compiling the Toolbox to be used with the real robot set the flag -DICUBWBI_USE_EXTERNAL_TORQUE_CONTROL:BOOL=YES. Then as usual type c to configure until no stars (*) show up and g to generate. Finally, to compile type make.


###### Installing the WBI-Toolbox
- **Installation.** There are a number of ways to install the Toolbox. They all consist in ensuring that the MEX files you just compiled are found in MATLAB's path, along with the Toolbox itself and its icons. We try to make your life easier and prepared an installation script that can be found under the name `startup_wbitoolbox.m` in `${CODYCO_SUPERBUILD_ROOT}/codyco/WBIToolbox` which automatically takes into account where you installed the WBIToolbox as specified by the variable `CMAKE_INSTALL_PREFIX`. You can see the default value of this variable by going to `${CODYCO_SUPERBUILD_DIR}/codyco/WBIToolbox` and typing `ccmake ./` to see the CMake default options for the Toolbox. In this way after compilation, running `startup_wbitoolbox.m` should automatically add the desired directories to MATLAB's path. It will also give you further instructions if you desire to permanently install it as to not run the script every time you want to use the Toolbox.

If for some reason the installation fails or you want to do this manually, the directories you need to add to the path are `${CODYCO_SUPERBUILD_DIR}/build/install/mex` (assuming the default CMake installation directory) and that for the controllers, models and Toolbox itself, i.e. `${CODYCO_SUPERBUILD_ROOT}/codyco/WBIToolbox/controllers` by doing

```bash
    addpath([getenv(CODYCO_SUPERBUILD_DIR)  /build/install/mex])
    addpath([getenv(CODYCO_SUPERBUILD_ROOT) /codyco/WBIToolbox/controllers])
```
You can also create a .m file with these two lines and launch MATLAB from terminal as:
```bash
    matlab -r yourStartupFile
```

WBI-Toolbox is discrete in principle and your simulation should be discrete as well. By going to Simulation > Configuration Parameters > Solver you should change the solver options to `Fixed Step` and use a `discrete (no continuous states)` solver.

- **Test the Library.** In `$CODYCO_SUPERBUILD_ROOT/src/simulink/controllers` you can find some models for testing (more on this in the README of the aforementioned directory). In order to test that the library is working correctly and properly linking YARP you can try launching a `yarpserver`, after which you can go to the controllers directory in MATLAB and open yarpwrite.mdl. Before starting the simulation, give a name to the YARP port where you want to write by double clicking the block and editing the mask that pops up. 

- **For MAC OS X Users.** It has been reported that on MAC OS you need to define the place where you want MATLAB to find at runtime dynamic libraries for YARP, in case you have compiled YARP in a directory different from the default one. This can be added in `${MATLAB_ROOT}/bin/.matlab7rc.sh`. 
```bash
    chmod +w .matlab7rc.sh
    LDPATH_SUFFIX = 'YOUR_ENV_DYLD_LIBRARY_PATH'
    chmod -w .matlab7rc.sh
```
- **Additional notes.** In case Matlab has trouble finding a specific library, a workaround is to launch it preloading the variable `LD_PRELOAD` (or `DYLD_INSERT_LIBRARIES` on Mac OS X) with the location of the missing library. On Linux you might also have trouble with libstdc++.so since Matlab comes with its own. To use your system's libstdc++ you would need to launch Matlab as:

`LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6.0.19   matlab`

###### Using the Simulink Library
Internally, the toolbox uses YARP's ResourceFinder (http://goo.gl/4zAS6r). When you first pull this repository you will download some default .ini files for each robot for which we have used the Toolbox (i.e. iCubGenova01, iCubGenova03, icubGazeboSim). These .ini files can be found in `${CODYCO_SUPERBUILD_ROOT}/codyco/WBIToolbox/libraries/wbInterface/conf/wbit`. If you want to use this toolbox with a different robot, you can create a new .ini file in which you set the following parameters:

- **robot**:     robot name (i.e. icubGazeboSim, iCubGenova01, etc).
- **local**:     prefix of the YARP ports that the WBI will open.
- **headV**:     [int] head version of your robot.
- **legsV**:     [int] legs version of your robot.
- **feetFT**:    [bool] Is the robot endowed with force/torque sensors for its feet?
- **uses_urdf**: [bool] Is your robot fixed to root or standing on the floor? (for icubGazeboSim this would mean whether you are using the `iCub (fixed)` or `iCub` models)
- **urdf**:      When using the icubGazeboSim you need to specify the exact location of the urdf model of the robot as found in `/icub-model-generator/generated/`. These models can be downloaded from the repository https://github.com/robotology-playground/icub-model-generator. This step is still succeptible to changes in the near future.

You will find a few controllers and models that have already been used with the iCub simulator and the real robot as found in `${CODYCO_SUPERBUILD_ROOT}/codyco/WBIToolbox/controllers`. 

###### Current Controllers
Our most recent controllers and other Simulink diagrams can be found in `${CODYCO_SUPERBUILD_ROOT}/codyco/WBIToolbox/controllers`. In there you can find:

- **torqueBalancing/controllerWithHandControl.slx**  This is the latest iCub's COM controller. The one used for the video in the beginning of this document.

- **wholeBodyImpedance/impedanceControl.mdl** This is a whole body impedance controller which sets all joints in impedance where the equilibrium pose is the initial one before running the controller. You can additionally perturb the system applying external wrenches on the robot links. Go to a terminal and enter `yarp rpc /icubGazeboSim/applyExternalWrench/rpc:i` then type `help` for additional information on how to apply wrenches on the robot and thus test its compliant behavior.

###### Tested OS
Linux, Windows, MAC OS X

###### To Do List
- [ ] Yarp read should read bottles!
- [ ] Documentation (Functions, etc)
- [ ] ZMP block.
- [x] ~~Debug incompatibilities with Gazebo (at the c++ whole body interface level)~~ - with Francesco Romano
- [x] ~~Compile the Soft Real Time mex as another module of the library. Possibly make our own.~~
- [x] ~~Modify YarpRead module so that you can specify the port you wanna read from and where you want it to connect. Connection should be done inside the block.~~
- [x] ~~Restructure code for wbInterface~~
- [x] ~~Expose computeMass() and generalizedBiasForces()~~
- [x] ~~Debug computeMass() and generalizedBiasForces()~~
- [x] ~~Check minimum jerk generator.~~
- [x] ~~Reproduce COM Controller as a Force Controlled version.~~
- [x] ~~Documentation (Installation)~~
- [x] ~~How to properly get dynamic libraries linked at runtime on MAC OS X.~~
- [x] ~~Divide blocks into subgroups (actuators, estimators, etc) and put them all together as a real Simulink Library :D~~
- [x] ~~Create icons for each block in the library.~~
 
