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
    
    //     if (!ssSetNumInputPorts(S, 0)) return;
    
    //     if (!ssSetNumOutputPorts(S, 2)) return;
    //     ssSetOutputPortWidth(S, 0, 1);	    		// Robot joint angular positions in radians
    //     ssSetOutputPortWidth(S, 1, 1);
    //     ssSetOutputPortDataType(S, 0, 0);
    //     ssSetOutputPortDataType(S, 1, 0);
    
    ssSetNumSampleTimes(S, 1);
    ssSetNumPWork(S, 1);
    ssSetOptions(S, 0);
    
    ssSetOptions (S,
                  SS_OPTION_WORKS_WITH_CODE_REUSE        |
                  SS_OPTION_EXCEPTION_FREE_CODE          |   //we must be sure that every function we call never throws an exception
                  SS_OPTION_ALLOW_INPUT_SCALAR_EXPANSION |
                  SS_OPTION_USE_TLC_WITH_ACCELERATOR     |
                  SS_OPTION_CALL_TERMINATE_ON_EXIT);
    
    fprintf (stderr, "[mdlInitializeSizes] : Options set\n\n");
    
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
    yarp::os::Network::init();
    printf("YARP Network initialized\n");
    
    if (!yarp::os::Network::checkNetwork() || !yarp::os::Network::initialized()) {
        ssSetErrorStatus(S,"mdlStart >> YARP server wasn't found active!! \n");
        return;
    }
    
    //  During startup I need this block to pause the simulation, then execute last
    //  and step the simulation by Ts.
    InternalData *internalData = new InternalData();
    
//    yarp::os::Port *clientPort;
//    clientPort = new yarp::os::Port();
//    
//    gazebo::ClockServer *clockServer;
//    clockServer = new gazebo::ClockServer();
    
    internalData->configuration.clientPortName = "/ySynchronizer/clock:o"; //compose this with the module name?
    
    internalData->configuration.serverPortName = "/clock/rpc";
//    if (exists serverPortParameter)
//        internalData->configuration.serverPortName = readParameter();
    
    bool resetTimeAtStart = false;
//    if (exists resetTimeAtStart parameter)
//        resetTimeAtStart = read boolean variable;
    
    double period = 0.01;
    //read period from parameter here
    
    internalData->clientPort.open(internalData->configuration.clientPortName);
    if (!yarp::os::Network::connect(internalData->configuration.clientPortName, internalData->configuration.serverPortName)) {
        fprintf(stderr,"mdlStart : ERR Problems connecting with /clock/rpc. Has gazebo been launched with the clock plugin?\n");
    }
    
    internalData->clockServer.yarp().attachAsClient(internalData->clientPort);
    
    double stepSize = internalData->clockServer.getStepSize();
    fprintf(stderr,"[mdlStart] : clockServer client retrieved stepSize = %lf", stepSize);
    
    //check if stepSize / period is an integer.
    internalData->configuration.numberOfSteps = period / stepSize;
    
    internalData->clockServer.pauseSimulation();
    
    if (resetTimeAtStart) {
        internalData->clockServer.resetSimulationTime();
    }
    
    ssGetPWork(S)[0] = internalData;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    InternalData *internalData = static_cast<InternalData*>(ssGetPWork(S)[0]);
    internalData->clockServer.stepSimulationAndWait(internalData->configuration.numberOfSteps);
}

static void mdlTerminate(SimStruct *S)
{
    //     UNUSED_ARG(S); /* unused input argument */
    if (ssGetNumPWork(S) > 0 && ssGetPWork(S)) {
        InternalData *internalData = static_cast<InternalData*>(ssGetPWork(S)[0]);
        if (internalData) {
            yarp::os::Network::disconnect(internalData->configuration.clientPortName, internalData->configuration.serverPortName);
            internalData->clientPort.close();
            
            delete internalData;
            internalData = 0;
        }
    }
    yarp::os::Network::fini();
    fprintf(stderr,"mdlTerminate : Reached end of this simulation\n");
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
