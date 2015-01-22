#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  realTimeSynchronizer

//accessors macro
#define setCounter(S_, counter_) ssSetIWorkValue(S_, 0, counter_)
#define getCounter(S_) ssGetIWorkValue(S_, 0)
#define getInitialTime(S_) ssGetRWorkValue(S_, 1)
#define setInitialTime(S_, initialTime_) ssSetRWorkValue(S_, 1, initialTime_)
#define getThreadPeriod(S_) ssGetRWorkValue(S_, 0)
#define setThreadPeriod(S_, threadPeriod_) ssSetRWorkValue(S_, 0, threadPeriod_)


#include <stdlib.h>
/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#include <mex.h>

#include <yarp/os/Time.h>

#define MDL_CHECK_PARAMETERS   /* Change to #undef to remove function */
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
    const mxArray *parameter = ssGetSFcnParam(S,0);
    if (mxGetNumberOfElements(parameter) != 1) {
        ssSetErrorStatus(S,"Parameter to S-function must be a scalar");
        return;
    } else if (mxGetPr(parameter)[0] < 0) {
        ssSetErrorStatus(S, "Parameter to S-function must be nonnegative");
        return;
    }
}
#endif /* MDL_CHECK_PARAMETERS */

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 1);  /* Number of expected parameters */

    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;
#if defined(MATLAB_MEX_FILE)
    mdlCheckParameters(S);
    if(ssGetErrorStatus(S) != NULL) return;
#endif


    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 0)) return;

    if (!ssSetNumOutputPorts(S, 0)) return;

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 2);
    ssSetNumIWork(S, 1);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE
                 | SS_OPTION_PLACE_ASAP
                 | SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE
                 | SS_OPTION_USE_TLC_WITH_ACCELERATOR);

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
    yarp::os::Time::turboBoost();
    //read the thread period parameter
    double* threadPeriod = mxGetPr(ssGetSFcnParam(S, 0));
    if (!threadPeriod || *threadPeriod <= 0) {
        ssSetErrorStatus(S, "Thread period has not been specified.");
        return;
    }
    //use this period
    setThreadPeriod(S, *threadPeriod);
    setCounter(S, 0);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    int_T counter = getCounter(S);
    double initialTime = 0;
    if (counter == 0) {
        initialTime = yarp::os::Time::now();
        setInitialTime(S, initialTime);
    } else {
        initialTime = getInitialTime(S);
    }
    double threadPeriod = getThreadPeriod(S);

    //read current time
    double currentTime = yarp::os::Time::now() - initialTime;
    double desiredTime = counter * threadPeriod;

    double sleepPeriod = desiredTime - currentTime;

    //sleep for the remaining time
    if (sleepPeriod > 0)
        yarp::os::Time::delay(sleepPeriod);

    setCounter(S, counter + 1);
}

static void mdlTerminate(SimStruct *S)
{
    UNUSED_ARG(S);
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
