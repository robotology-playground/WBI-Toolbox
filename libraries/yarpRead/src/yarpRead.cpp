/*
 * Copyright (C) 2013-2015 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Jorhabib Eljaik Gomez, Francesco Romano
 * email: jorhabib.eljaik@iit.it, francesco.romano@iit.it
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace std;


#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME yarpRead

// PARAMETERS MASK
#define NPARAMS 6                              // Number of input parameters

#define PARAM_IDX_1 0                           // port name
#define PARAM_IDX_2 1                           // Size of the port you're reading
#define PARAM_IDX_3 2                           // boolean for blocking reading
#define PARAM_IDX_4 3                           // boolean to stream timestamp
#define PARAM_IDX_5 4                           // Autoconnect boolean
#define PARAM_IDX_6 5                           // Error on missing port if autoconnect is on boolean
#define GET_OPT_SIGNAL_SIZE mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_2))    // Get first input parameter from mask
#define GET_OPT_BLOCKING  mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_3))
#define GET_OPT_TIMESTAMP mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_4))
#define GET_OPT_AUTOCONNECT mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_5))
#define GET_OPT_ERROR_ON_MISSING_PORT mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_6))

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
#include "simstruc.h"

#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))


// Function: MDL_CHECK_PARAMETERS
// Input parameters for locomotionController in Simulink

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
}
#endif  /*MDL_CHECK_PARAMETERS*/

#include <algorithm>

// Function: mdlInitializeSizes ===============================================
// Abstract:
//    The sizes information is used by Simulink to determine the S-function
//    block's characteristics (number of inputs, s, states, etc.).
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NPARAMS);
#if defined(MATLAB_MEX_FILE)
    if(ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)){
        mdlCheckParameters(S);
        if(ssGetErrorStatus(S)!=NULL){
            return;
        }
    } else{
        return; // Parameter mismatch reported by Simulink
    }
#endif

    // Parameter mismatch will be reported by Simulink
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        ssSetErrorStatus(S,"Number of paramaeters different from those defined");
        return;
    }

    // Specify I/O
    // INPUTS
    if(!ssSetNumInputPorts(S,0)) return;
    // OUTPUTS
    int timestamp = (int)GET_OPT_TIMESTAMP;
    int signalSize = (int)GET_OPT_SIGNAL_SIZE;
    int autoconnect = (int)GET_OPT_AUTOCONNECT;

    if (signalSize < 0) {
        ssSetErrorStatus(S,"Signal size must be non negative");
        return;
    }

    int numberOfOutputPorts = 1;
    if (timestamp) numberOfOutputPorts++; //timestamp is the second port
    if (!autoconnect) numberOfOutputPorts++; //!autoconnect => additional port with 1/0 depending on the connection status

    if (!ssSetNumOutputPorts(S, numberOfOutputPorts)) return;

    ssSetOutputPortWidth(S, 0, signalSize);
    ssSetOutputPortDataType(S, 0, SS_DOUBLE);

    int portIndex = 1;
    if (timestamp) {
        ssSetOutputPortWidth(S, portIndex, 2);
        ssSetOutputPortDataType(S, portIndex, SS_DOUBLE);
        portIndex++;
    }
    if (!autoconnect) {
        ssSetOutputPortWidth(S, portIndex, 1);
        ssSetOutputPortDataType(S, portIndex, SS_INT8); //use double anyway. Simplifies simulink stuff
        portIndex++;
    }

    ssSetNumSampleTimes(S, 1);

    // Reserve place for C++ object
    ssSetNumPWork(S, 1); //buffered port
    // IWork vector for booleans
    ssSetNumIWork(S, 3); //0 : blocking
                         //1 : timestamp
                         //2 : autoconnect

    ssSetSimStateCompliance(S, USE_CUSTOM_SIM_STATE); //??

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_ALLOW_INPUT_SCALAR_EXPANSION |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR |
                 SS_OPTION_CALL_TERMINATE_ON_EXIT);

}

// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct *S)
{
    // The sampling time of this SFunction must be inherited so that the Soft Real Time sblock can be used.
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    // ssSetSampleTime(S, 0, 10.0);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you
//   have states that should be initialized once, this is the place
//   to do it.
#define MDL_START
static void mdlStart(SimStruct *S)
{
    // ######### YARP INITIALIZATION STUFF ##################
    Network::init();
#ifndef NDEBUG
    mexPrintf("YARP NETWORK INITIALIZED\n");
#endif

    if (!Network::checkNetwork() || !Network::initialized()){
        ssSetErrorStatus(S,"YARP server wasn't found active!! \n");
        return;
    }
#ifndef NDEBUG
    mexPrintf("Yarp network running\n");
#endif

    int_T autoconnect = GET_OPT_AUTOCONNECT;
    int_T errorOnMissingPort = GET_OPT_ERROR_ON_MISSING_PORT;
#ifndef NDEBUG
    mexPrintf("Autoconnect option is %d\n", autoconnect);
#endif
    ssGetIWork(S)[2] = autoconnect;

    int_T buflen, status;
    char *buffer = NULL;

    buflen = (1 + mxGetN(ssGetSFcnParam(S, PARAM_IDX_1))) * sizeof(mxChar);
    buffer = static_cast<char*>(mxMalloc(buflen));
    status = mxGetString((ssGetSFcnParam(S, PARAM_IDX_1)), buffer, buflen);
    if (status) {
        ssSetErrorStatus(S,"Cannot retrieve string from port name parameter");
        return;
    }

    std::string sourcePortName;
    std::string destinationPortName;

    if (autoconnect) {
        sourcePortName = buffer;
        destinationPortName = "...";
    } else {
        destinationPortName = buffer;
    }
    mxFree(buffer);
    buffer = NULL;

    //######## CHECKING INPUT PARAMETERS ############

    BufferedPort<Vector> *port;
    port = new BufferedPort<Vector>();
    ssGetPWork(S)[0] = port;

    if (!port || !port->open(destinationPortName)) {
        ssSetErrorStatus(S,"Error while opening yarp port");
        return;
    }

#ifndef NDEBUG
    mexPrintf("Opened port %s\n", port->getName().c_str());
#endif

    int_T blocking = GET_OPT_BLOCKING;
#ifndef NDEBUG
    mexPrintf("Blocking option is %d\n", blocking);
#endif
    ssGetIWork(S)[0] = blocking;

    int_T timestamp = GET_OPT_TIMESTAMP;
#ifndef NDEBUG
    mexPrintf("Timestamp option is %d\n", timestamp);
#endif
    ssGetIWork(S)[1] = timestamp;

    if (autoconnect) {
        if (!Network::connect(sourcePortName, port->getName())) {
            mexPrintf("Failed to connect %s to %s\n", sourcePortName.c_str(), port->getName().c_str());
            if (errorOnMissingPort) {
                ssSetErrorStatus(S,"ERROR connecting ports!");
            }
            return;
        }
#ifndef NDEBUG
        mexPrintf("Connected %s to %s\n", sourcePortName.c_str(), port->getName().c_str());
#endif
    }
}

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs(SimStruct *S, int_T tid)
{
    BufferedPort<Vector> *port = static_cast<BufferedPort<Vector>*>(ssGetPWork(S)[0]);
    int_T blocking = ssGetIWork(S)[0];
    int_T shouldReadTimestamp = ssGetIWork(S)[1];
    int_T isAutoconnect = ssGetIWork(S)[2];
    int timeStampPortIndex = 1;
    int connectionStatusPortIndex = 1;

    Vector *v = port->read(blocking); // Read from the port.  Waits until data arrives.
    if (v)
    {
        if (shouldReadTimestamp) {
            connectionStatusPortIndex++;
            yarp::os::Stamp timestamp;
            port->getEnvelope(timestamp);

            real_T *pY1 = ssGetOutputPortRealSignal(S, timeStampPortIndex);
            pY1[0] = (real_T)(timestamp.getCount());
            pY1[1] = (real_T)(timestamp.getTime());
        }
        real_T *signal = ssGetOutputPortRealSignal(S, 0);
        int_T widthPort = ssGetOutputPortWidth(S, 0);
        for (int i = 0; i < std::min(widthPort, (int_T)v->size()); i++) {
            signal[i] = (*v)[i];
        }

        if (!isAutoconnect) {
            unsigned char *statusPort = (unsigned char*)ssGetOutputPortSignal(S, connectionStatusPortIndex);
            statusPort[0] = 1; //somebody wrote in the port => the port is connected
            //TODO implement a sort of "timeout" paramter
            //At the current state this is equal to timeout = inf
            //Otherwise we can check the timeout and if nobody sent data in the last X secs
            //we set the port to zero again
        }
    }
}

static void mdlTerminate(SimStruct *S)
{
    // IF YOU FORGET TO DESTROY OBJECTS OR DEALLOCATE MEMORY, MATLAB WILL CRASH.
    // Retrieve and destroy C++ object
    if (ssGetPWork(S)) { //This is not created in compilation
        BufferedPort<Vector> *toPort = static_cast<BufferedPort<Vector>*>(ssGetPWork(S)[0]);
        if (toPort) {
            toPort->close();
            delete toPort; //remove port
        }
        Network::fini();
    }
    fprintf(stderr,"Everything was closed correctly\n");
}

// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
