#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  ySynchronizer

#define RATE(S) ssGetSFcnParam(S,0)
#define SAMPLING_TIME 1

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"

/*
 *  Include the standard ANSI C header for handling time functions:
 *  ---------------------------------------------------------------
 */
#include <time.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <thrift/ClockServer.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>


struct InternalData
{
    struct {
        std::string clientPortName;
        std::string serverPortName;

        unsigned numberOfSteps;
    } configuration;

    yarp::os::Port clientPort;
    gazebo::ClockServer clockServer;
};
typedef struct InternalData InternalData;

static void mdlInitializeSizes(SimStruct *S)
{

    ssSetNumSFcnParams(S, 1);  /* 1 Input parameters: Time Scale Factor, e.g. 1 for real time simulation */

    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, 1);
    ssSetNumDWork (S, 1);
    ssSetDWorkWidth (S, 0, 1);
    ssSetDWorkDataType (S, 0, SS_INTEGER);
    ssSetOptions(S, 0);

    ssSetOptions (S,
                  SS_OPTION_WORKS_WITH_CODE_REUSE        |
                  SS_OPTION_EXCEPTION_FREE_CODE          |   //we must be sure that every function we call never throws an exception
                  SS_OPTION_ALLOW_INPUT_SCALAR_EXPANSION |
                  SS_OPTION_USE_TLC_WITH_ACCELERATOR     |
                  SS_OPTION_PLACE_ASAP                   |   // This options makes the ySynchronizer execute AS SOON AS POSSIBLE
                  SS_OPTION_CALL_TERMINATE_ON_EXIT);

    yDebug("[mdlInitializeSizes] Options set for ySynchronizer\n\n");

}

#define MDL_INITIALIZE_SAMPLE_TIMES
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_START
static void mdlStart(SimStruct *S)
{
    int_T* flag = (int_T*) ssGetDWork (S, 0);
    flag[0] = 0;

    yarp::os::Network::init();
    yInfo("YARP Network initialized by ySynchronizer\n");

    if (!yarp::os::Network::checkNetwork() || !yarp::os::Network::initialized()) {
        yError("[mdlStart] ySynchronizer did not find YARP server active\n");
        ssSetErrorStatus(S,"mdlStart >> YARP server wasn't found active!! \n");
        return;
    }

    //  During startup I need this block to pause the simulation, then execute last
    //  and step the simulation by Ts.
    InternalData *internalData = new InternalData();

    internalData->configuration.clientPortName = "/ySynchronizer/clock:o"; //compose this with the module name?
    internalData->configuration.serverPortName = "/clock/rpc";

    double period = 0.01;
    //TODO read period from parameter here

    internalData->clientPort.open(internalData->configuration.clientPortName);
    if (!yarp::os::Network::connect(internalData->configuration.clientPortName, internalData->configuration.serverPortName)) {
        yError("mdlStart : ERR Problems connecting with /clock/rpc. Has gazebo been launched with the clock plugin?\n");
        ssSetErrorStatus(S, "Problems connecting with /clock/rpc. Has gazebo been launched with the clock plugin?");
    }

    internalData->clockServer.yarp().attachAsClient(internalData->clientPort);

    double stepSize = internalData->clockServer.getStepSize();
    yInfo("[ySynchronizer::mdlStart] >> clockServer client retrieved stepSize = %lf\n", stepSize);

    //check if stepSize/period is an integer.
    internalData->configuration.numberOfSteps = period / stepSize;

//     internalData->clockServer.pauseSimulation();

    ssGetPWork(S)[0] = internalData;

    // Initialization is over.
    flag[0]     = 1;

    yInfo("ySynchronizer::mdlStart >> Finished startup\n");
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    InternalData *internalData = static_cast<InternalData*>(ssGetPWork(S)[0]);
    int_T* flag = (int_T*) ssGetDWork (S, 0);
    if (flag[0]) {
        // Pause only once in mdlOutputs
        yInfo("[ySynhcronizer] PAUSED THE SIMULATION DURING THE FIRST SIMULATION LOOP\n");
        internalData->clockServer.pauseSimulation();
        flag[0] = 0;    
    }
    internalData->clockServer.stepSimulationAndWait(internalData->configuration.numberOfSteps);
}

static void mdlTerminate(SimStruct *S)
{
    if (ssGetNumPWork(S) > 0 && ssGetPWork(S)) {
        InternalData *internalData = static_cast<InternalData*>(ssGetPWork(S)[0]);
        if (internalData) {
            yInfo("[mdlTerminate][ySynchronizer] Before issuing continueSimulation command\n");
            internalData->clockServer.continueSimulation();
            if(!yarp::os::Network::disconnect(internalData->configuration.clientPortName, internalData->configuration.serverPortName)) {
                yError("[mdlTerminate][ySynchronizer] Could not disconnect src %s from dest %s \n", internalData->configuration.clientPortName.c_str(), internalData->configuration.serverPortName.c_str());
            }
            internalData->clientPort.close();
            delete internalData;
            internalData = 0;
        }
    }
    yarp::os::Network::fini();
    yInfo("[mdlTerminate][ySynchronizer] reached end of this simulation\n");
}

/*
 *  Required S-function trailer:
 *  ----------------------------
 */
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
