#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME  realTimeSynchronizer

#include <stdlib.h>
/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#include <mex.h>

/*
 *  Include the standard ANSI C header for handling time functions:
 *  ---------------------------------------------------------------
 */
#include <time.h>


static void mdlInitializeSizes(SimStruct *S)
{
   ssSetNumSFcnParams(S, 1);  /* Number of expected parameters */

   if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) return;

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

   if (!ssSetNumInputPorts(S, 0)) return;

   if (!ssSetNumOutputPorts(S, 0)) return;
   
   ssSetNumSampleTimes(S, 1);
   ssSetNumRWork(S, 0);
   ssSetNumIWork(S, 0);
   ssSetNumPWork(S, 1);
   ssSetNumModes(S, 0);
   ssSetNumNonsampledZCs(S, 0);
   ssSetOptions(S, 0);
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
  //read the thread period parameter
    double* threadPeriodInput = mxGetPr(ssGetSFcnParam(S, 0));
    if (!threadPeriodInput || *threadPeriodInput <= 0) {
        //TODO: return an error
        return;
    }
    //use this period
    //convert into timespec structure
    timespec *threadPeriod = (timespec*)malloc(sizeof(timespec));
    if (!threadPeriod) {
        //TODO: return an error
        return;
    }
    threadPeriod->tv_sec = 0;
    threadPeriod->tv_nsec = 10e+6; //10ms
    ssSetPWorkValue(S, 0, threadPeriod);
}

//Hp: end > start
static inline timespec computeDiffTime(timespec start, timespec end)
{
    timespec result;
    if (end.tv_nsec - start.tv_nsec < 0) {
        result.tv_sec = end.tv_sec - start.tv_sec - 1;
        result.tv_nsec = 1e+9 + end.tv_nsec - start.tv_nsec;
    } else {
        result.tv_sec = end.tv_sec - start.tv_sec;
        result.tv_nsec = end.tv_nsec - start.tv_nsec;
    }
    return result;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    static short firstLoop = 1;
    static timespec previousTime;
    static timespec firstTime;
    int returnValue = 0;
    if (firstLoop) {
        //initialization logic
        firstLoop = 0;
        returnValue = clock_gettime(CLOCK_REALTIME, &previousTime);
        firstTime = previousTime;
    }
    timespec *threadPeriod = (timespec*)ssGetPWorkValue(S, 0);
    
    //read current time
    timespec currentTime;
    returnValue = clock_gettime(CLOCK_REALTIME, &currentTime);
    
    timespec timeSinceStart = computeDiffTime(firstTime, currentTime);
    mexPrintf("Time is %ld-%ld\n", timeSinceStart.tv_sec, (long int)((long double)timeSinceStart.tv_nsec * 1e-6));
    
    //compute diff time
    timespec diffTime = computeDiffTime(previousTime, currentTime);
    mexPrintf("%ld-%ld - %ld-%ld => %ld-%ld\n", currentTime.tv_sec, currentTime.tv_nsec, previousTime.tv_sec, previousTime.tv_nsec, diffTime.tv_sec, diffTime.tv_nsec);
    diffTime.tv_sec = threadPeriod->tv_sec - diffTime.tv_sec;
    diffTime.tv_nsec = threadPeriod->tv_nsec - diffTime.tv_nsec;
    

    //sleep for the remaining time
    timespec unsleptAmount;
    mexPrintf("Sleeping for %ld[s]-%ld[ms]-%ld[us]\n", diffTime.tv_sec, (diffTime.tv_nsec/1000000), (diffTime.tv_nsec/1000));
//     diffTime.tv_sec = 0;
//     diffTime.tv_nsec = 600e+6;
    
//     timespec tempPrev, tempAfter;
//     returnValue = clock_gettime(CLOCK_REALTIME, &tempPrev);
    returnValue = nanosleep(&diffTime, &unsleptAmount);
//     returnValue = clock_gettime(CLOCK_REALTIME, &tempAfter);
//     timespec tempDiff = computeDiffTime(tempPrev, tempAfter);
//     mexPrintf("----------------TEMP%ld-%ld\n", tempDiff.tv_sec, tempDiff.tv_nsec);
    
    if (returnValue < 0) {
        //an error occurred
        //TODO: handle this case
        mexPrintf("Waken up\n");
        mexPrintf("Remaining %ld-%ld\n", unsleptAmount.tv_sec, unsleptAmount.tv_nsec);
    }
    else 
        mexPrintf("Correctly waken up\n");
    
    
    previousTime = currentTime;

}

static void mdlTerminate(SimStruct *S)
{
    timespec *threadPeriod = (timespec*)ssGetPWorkValue(S, 0);
    if (threadPeriod) {
        free(threadPeriod);
        threadPeriod = NULL;
    }
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
