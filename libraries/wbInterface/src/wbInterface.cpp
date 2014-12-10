/*
 * Copyright (C) 2013 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Jorhabib Eljaik Gomez, Francesco Nori
 * email: jorhabib.eljaik@iit.it
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

// #define CODYCO_USES_URDFDOM 1


#include "wbInterface.h"
#include <wbiIcub/wholeBodyInterfaceIcub.h>
// MASK PARAMETERS --------------------------------------
#define NPARAMS            4                                // Number of input parameters
#define BLOCK_TYPE_IDX     0                                // Index number for first input parameter
#define BLOCK_TYPE_PARAM ssGetSFcnParam(S,BLOCK_TYPE_IDX)   // Get first input parameter from mask
#define STRING_PARAM_IDX   1
#define LOCAL_PARAM_IDX    2
#define LINKNAME_PARAM_IDX 3
// END MASK PARAMETERS -----------------------------------

#define VERBOSE   0
#define TIMING    0



YARP_DECLARE_DEVICES (icubmod)

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;

using namespace std;
using namespace Eigen;

using namespace wbi;
using namespace wbiIcub;



//-----------------------------------------------------------------------------------------------------------------------//
// START ROBOTSTATUS IMPLEMENTATION -------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------//
//These variables must be initialized in this .cpp file and in this particular way for being static member variables of a class
int  robotStatus::creationCounter = 0;
int* robotStatus::tmpContainer    = NULL;
int  counterClass::count          = 0;
bool robotStatus::icub_fixed      = false;
yarp::os::ConstString robotStatus::worldRefFrame = "l_sole";


robotStatus::robotStatus() {
    creationCounter++;
//     fprintf (stderr, "robotStatus::robotStatus >> ROBOTSTATUS instantiated %d times\n ", creationCounter);
    wbInterface = NULL;
}
//=========================================================================================================================
robotStatus::~robotStatus() {
    if (tmpContainer != NULL) {
        creationCounter--;
        fprintf (stderr, "robotStatus::~robotStatus >> wbInterface in destructor: %p \n", wbInterface);
        if (wbInterface->close()) {
            delete wbInterface;
            fprintf (stderr, "robotStatus::~robotStatus >> wbInterface has been closed and deleted correctly. %d to go \n", creationCounter);
            wbInterface = NULL;
        } else {
            fprintf (stderr, "robotStatus::~robotStatus >> ERROR: wbInterface couldn't close correctly");
        }

        tmpContainer = NULL;
    }
}
//=========================================================================================================================
void robotStatus::setRobotName (string rn) {
    robotName = rn;
}
//=========================================================================================================================
void robotStatus::setmoduleName (string mn) {
    moduleName = mn;
}
//=========================================================================================================================
void robotStatus::setParamLink (string lk) {
    linkName = lk;
}
//=========================================================================================================================
std::string robotStatus::getParamLink () {
    return linkName;
}
//=========================================================================================================================
int robotStatus::decreaseCounter() {
    creationCounter--;
    return creationCounter;
}
//=========================================================================================================================
void robotStatus::resetCounter() {
    creationCounter = 0;
}
//=========================================================================================================================
bool robotStatus::robotConfig() {
    if (VERBOSE) fprintf (stderr, "robotStatus::robotConfig >> Configuring...\n");
    if (tmpContainer != NULL) {
        wbInterface = (wholeBodyInterface*) tmpContainer;
#ifdef DEBUG
        fprintf (stderr, "[robotStatus::robotConfig] Copying wholeBodyInterface POINTER!\n");
#endif
    } else {
        ResourceFinder rf;
        rf.setVerbose (true);
        rf.setDefaultContext ("wbit");

        rf.setDefaultConfigFile (string (robotName + ".ini").c_str());
        rf.configure (0, 0);

        ConstString robotNamefromConfigFile = rf.find ("robot").asString();
        ConstString localNamefromConfigFile = rf.find ("local").asString();
        int         headVfromConfigFile     = rf.find ("headV").asInt();
        int         legsVfromConfigFile     = rf.find ("legsV").asInt();
        bool        feetFTfromConfigFile    = rf.find ("feetFT").asBool();
        bool        uses_urdf               = rf.find ("uses_urdf").asBool();
        std::string urdf_file               = rf.find ("urdf").asString();
        worldRefFrame                       = rf.find ("worldRefFrame").asString();
        if (rf.check ("icub_fixed")) {
            if (rf.find ("icub_fixed").isBool())
                icub_fixed                  = rf.find ("icub_fixed").asBool();
            else {
                fprintf (stderr, "ERROR [robotStatus::robotConfig] Wrong icub_fixed in the config file!\n");
                return false;
            }
        } else {
            fprintf (stderr, "ERROR [robotStatus::robotConfig] No option icub_fixed in the config file!\n");
            return false;
        }
        if (icub_fixed)
            fprintf (stderr, "[robotStatus::robotConfig] icub_fixed is true!\n");
        else
            fprintf (stderr, "[robotStatus::robotConfig] icub_fixed is false!\n");

// #ifdef DEBUG
        fprintf (stderr, "[robotStatus::robotConfig] After reading from config file, params are \n");
        fprintf (stderr, "[robotStatus::robotConfig] robot name:    %s \n", robotNamefromConfigFile.c_str());
        fprintf (stderr, "[robotStatus::robotConfig] urdf file:     %s \n", urdf_file.c_str());
        fprintf (stderr, "[robotStatus::robotConfig] local name:    %s \n", localNamefromConfigFile.c_str());
        fprintf (stderr, "[robotStatus::robotConfig] head version:  %i \n", headVfromConfigFile);
        fprintf (stderr, "[robotStatus::robotConfig] legs version:  %i \n", legsVfromConfigFile);
        fprintf (stderr, "[robotStatus::robotConfig] feet version:  %i \n", feetFTfromConfigFile);
        fprintf (stderr, "[robotStatus::robotConfig] world reference frame: %s \n", worldRefFrame.c_str());
// #endif

        //---------------- CREATION WHOLE BODY INTERFACE ---------------------/
#ifdef CODYCO_USES_URDFDOM
        iCub::iDynTree::iCubTree_version_tag icub_version =
            iCub::iDynTree::iCubTree_version_tag (headVfromConfigFile, legsVfromConfigFile, feetFTfromConfigFile, uses_urdf, urdf_file);
#else
        iCub::iDynTree::iCubTree_version_tag icub_version =
            iCub::iDynTree::iCubTree_version_tag (headVfromConfigFile, legsVfromConfigFile, feetFTfromConfigFile);
#endif

        wbInterface = new icubWholeBodyInterface (localNamefromConfigFile.c_str(), robotNamefromConfigFile.c_str(), icub_version);


#ifdef DEBUG
        fprintf (stderr, "[robotStatus::robotConfig] New wbInterface created\n");
#endif

        tmpContainer = (int*) wbInterface;

#ifdef DEBUG
        fprintf (stderr, "[robotStatus::robotConfig] icubWholeBodyInterface has been created %p \n", wbInterface);
#endif
        //---------------- CONFIGURATION WHOLE BODY INTERFACE ----------------/
        // Add main iCub joints
        wbInterface->addJoints (ICUB_MAIN_JOINTS);

        if(robotNamefromConfigFile == "icub"){
            fprintf (stderr, "WARNING [robotStatus::robotConfig] Configuring WBI for the real robot. Please check the jointTorqueControl module is up and running.\n");
            if(yarp::os::NetworkBase::exists(string("/jtc/info:o").c_str()))
                fprintf (stderr, "[robotStatus::robotConfig] The module jointTorqueControl is running. Proceeding with configuration of the interface...\n");
            else {
                fprintf (stderr, "ERROR [robotStatus::robotConfig] The jointTorqueControl module is not running. Please launch it before running your controller. \n");
                return false;
            }
        }

#ifdef WBI_ICUB_COMPILE_PARAM_HELP
	if(!yarp::os::NetworkBase::exists(string("/jtc/info:o").c_str())){
	  fprintf (stderr, "ERROR [robotStatus::robotConfig] This module is trying to use the jointTorqueControl but it was not found active. Type jointTorqueControl --help for more information.\n");
	  return false;
	}
	else {
        yarp::os::Value trueValue;
        trueValue.fromString ("true");
        ( (icubWholeBodyInterface*) wbInterface)->setActuactorConfigurationParameter (icubWholeBodyActuators::icubWholeBodyActuatorsUseExternalTorqueModule, trueValue);
        ( (icubWholeBodyInterface*) wbInterface)->setActuactorConfigurationParameter (icubWholeBodyActuators::icubWholeBodyActuatorsExternalTorqueModuleAutoconnect, trueValue);
        ( (icubWholeBodyInterface*) wbInterface)->setActuactorConfigurationParameter (icubWholeBodyActuators::icubWholeBodyActuatorsExternalTorqueModuleName, Value ("jtc"));
	}
#endif

        if (!wbInterface->init()) {
            fprintf (stderr, "ERROR [robotStatus::robotConfig] Initializing Whole Body Interface!\n");
            return false;
        } else {
            fprintf (stderr, "[robotStatus::robotConfig] Whole Body Interface correctly initialized, yayyy!!!!\n");
        }

        // Put robot in position mode so that in won't fall at startup assuming it's balanced in its startup position
        if (VERBOSE) fprintf (stderr, "[robotStatus::robotConfig] About to set control mode\n");
        setCtrlMode (CTRL_MODE_POS);
    }

    // Initializing private variables. This must be done regardless of the new creation of wbInterface
    actJnts = wbInterface->getJointList().size();
    qRad.resize (actJnts, 0.0);
    dqJ.resize (actJnts); dqJ.setZero();

    ddqJ.resize (actJnts, 0.0);
    tauJ.resize (actJnts, 0.0);
    tauJ_out.resize (actJnts, 0.0);


    return true;

}
//=========================================================================================================================
bool robotStatus::robotInit (int btype, int link) {

    // To initialize x_pose which will be used to compute either
    //forward kinematics or jacobians, it is necessary to know for what
    //it is desired. So far, we have it only for right and left leg and
    //center of mass
    /** There might be additional block types that should be considered here. Check out*/
    if (btype == 2 || btype == 3) {
        const char* linkName = "";
        int default_size = 0;
        int linkID = 0;
        printf ("READ LINK IS: %d \n", link);
        switch (link) {
        case 0:
            linkName = "r_sole";
            default_size = DEFAULT_X_LINK_SIZE.size();
            break;
        case 1:
            linkName = "l_sole";
            default_size = DEFAULT_X_LINK_SIZE.size();
            break;
        case 2:
            linkName = "com";
            default_size = DEFAULT_X_LINK_SIZE.size();
            break;
        case 3:
            linkName = "r_gripper";
            default_size = DEFAULT_X_LINK_SIZE.size();
            break;
        case 4:
            linkName = "l_gripper";
            default_size = DEFAULT_X_LINK_SIZE.size();
            break;
        default:
            fprintf (stderr, "ERROR: No link has been specified for this block\n");
            return false;
        }
        getLinkId (linkName, linkID);
        // Output of forward kinematics and jacobian
        x_pose.resize (default_size, 0.0);
    }

    if(btype == 17) {
        x_pose.resize (DEFAULT_X_LINK_SIZE.size());
    }

    // TODO This variable JfootR must be changed with a more appropriate name
    JfootR.resize (NoChange, ROBOT_DOF + 6);

    // dot{J}dot{q}
    dJdq.resize (6, 0);

    dxB.resize (6, 0);

    ddxB.resize (6, 0);

    grav.resize (3, 1);
    grav[0] = grav[1] = 0;
    grav[2] = -9.81;
#ifdef DEBUG
    fprintf (stderr, "robotStatus::robotInit: reached here after setting gravity!\n");
#endif

    // Generalized bias forces term.
    hterm.resize (6 + ROBOT_DOF, 0);



    // Should the mass matrix be resized here??? In the future if the number of DOFS or limbs for which the interface will
    // be used are input parameters, all variables could be resized here and by default leave ROBOT_DOF.

    // rotation to align foot Z axis with gravity, Ha=[0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1]
    Ha.R = Rotation (0, 0, 1, 0, -1, 0, 1, 0, 0);

    EEWrench.resize (6, 0);

    return true;
}
//=========================================================================================================================
void robotStatus::getLinkId (const char* linkName, int& lid) {
    char comlink[] = "com";
    if (strcmp (linkName, comlink) != 0) { // !=0 means that they're different
#ifdef DEBUG
        printf("robotStatus::getLinkId has params: %s, %i\n", linkName, lid);
#endif
        wbInterface->getLinkId (linkName, lid);
    } else {
        lid = wbi::iWholeBodyModel::COM_LINK_ID;
    }
}
//=========================================================================================================================
//---------------- This is especifically for the COM ----------------
/**linkName in this method is not used. Check whether I'm actually using it */
int robotStatus::getLinkId (const char* linkName) {
    comLinkId = iWholeBodyModel::COM_LINK_ID;
    return comLinkId;
}
//=========================================================================================================================
bool robotStatus::world2baseRototranslation (double* q) {
    if (!icub_fixed) {
        int LINK_FOOT_WRF;
        getLinkId (worldRefFrame.c_str(), LINK_FOOT_WRF);
        wbInterface->computeH (q, Frame(), LINK_FOOT_WRF, H_base_wrfLink);
        H_base_wrfLink = H_base_wrfLink * Ha;
    } else {
        int LINK_ROOT;
        getLinkId ("root_link", LINK_ROOT);
        wbInterface->computeH (q, Frame(), LINK_ROOT, H_base_wrfLink);
    }
    H_base_wrfLink.setToInverse().get4x4Matrix (H_w2b.data());
#ifdef DEBUG
    fprintf (stderr, "robotStatus::world2baseRototranslation >> Ha             : %s \n", Ha.toString().c_str());
    fprintf (stderr, "robotStatus::world2baseRototranslation >> H_base_wrfLink : %s \n", H_base_wrfLink.toString().c_str());
    fprintf (stderr, "robotStatus::world2baseRototranslation >> qRad           : %s \n", qRad.toString().c_str());
#endif
    xBase.set4x4Matrix (H_w2b.data());
#ifdef DEBUG
    fprintf (stderr, "robotStatus::world2baseRototranslation >> xbase          : %s \n", xBase.toString().c_str());
#endif
    return true;
}
//=========================================================================================================================
void robotStatus::setWorldReferenceFrame (const char* wrf) {
    worldRefFrame = wrf;

}
//=========================================================================================================================
bool robotStatus::robotJntAngles (bool blockingRead) {
#ifdef DEBUG
    fprintf (stderr, "robotStatus::robotJntAngles >> About to estimate joint positions\n");
#endif
    return wbInterface->getEstimates (ESTIMATE_JOINT_POS, qRad.data(), -1.0, blockingRead);
}
//=========================================================================================================================
bool robotStatus::robotJntVelocities (bool blockingRead) {
    return wbInterface->getEstimates (ESTIMATE_JOINT_VEL, dqJ.data(), -1.0, blockingRead);
}
//=========================================================================================================================
bool robotStatus::robotJntAccelerations (bool blockingRead) {
    return wbInterface->getEstimates (ESTIMATE_JOINT_ACC, ddqJ.data(), -1.0, blockingRead);
}
//=========================================================================================================================
bool robotStatus::robotJntTorques (bool blockingRead) {
    return wbInterface->getEstimates (ESTIMATE_JOINT_TORQUE, tauJ.data(), -1.0, blockingRead);
}
//=========================================================================================================================
// bool robotStatus::robotEEWrenches(int &linkId) {
//   return wbInterface->getEstimate(ESTIMATE_EXTERNAL_FORCE_TORQUE,);
// }
//=========================================================================================================================
Vector robotStatus::forwardKinematics (int& linkId) {
    if (robotJntAngles (false)) {
        if (world2baseRototranslation (qRad.data())) {
            footLinkId = linkId;
#ifdef DEBUG
            fprintf (stderr, "robotStatus::forwardKinematics >> Forward kinematics will be computed with LinkId: %d and x_pose: %s \n", footLinkId, x_pose.toString().c_str());
            fprintf (stderr, "robotStatus::forwardKinematics >> and qRad: %s \n xBase: %s \n ",qRad.toString().c_str(), xBase.toString().c_str());
#endif

            bool ans = wbInterface->forwardKinematics (qRad.data(), xBase, footLinkId, x_pose.data());
            if (ans) {
#ifdef DEBUG
                fprintf (stderr, "robotStatus::forwardKinematics >> Forward Kinematics computed \n");
                fprintf (stderr, "robotStatus::forwardKinematics >> pose: %s \n", x_pose.toString().c_str());
#endif
                return x_pose;
            }
//            else
//            {
//                x_pose.zero();
//                return x_pose;
//            }
        } else {
            fprintf (stderr, "ERROR [robotStatus::forwardKinematics] computing world 2 base rototranslation in robotStatus::forwardKinematics!\n");
        }
    } else {
        fprintf (stderr, "ERROR [robotStatus::forwardKinematics] acquiring robot joint angles in robotStatus::forwardKinematics\n");
    }
    x_pose.zero();
    return x_pose;
}
//=========================================================================================================================
JacobianMatrix robotStatus::jacobian (int& lid) {
    if (robotJntAngles (false)) {
        if (world2baseRototranslation (qRad.data())) {
            bool ans = wbInterface->computeJacobian (qRad.data(), xBase, lid, JfootR.data());
            if (ans) {
#ifdef DEBUG
                fprintf (stderr, "robotStatus::jacobian >> Base pos: %s\n", xBase.toString().c_str());
#endif
                return JfootR;
            }
        } else {
            fprintf (stderr, "ERROR [robotStatus::jacobian] computing world to base rototranslation \n");
        }
    } else {
        fprintf (stderr, "ERROR [robotStatus::jacobian] getting robot joint angles to compute Jacobians \n");
    }
    JfootR.setZero();
    return JfootR;
}
//=========================================================================================================================
Vector robotStatus::getEncoders() {
    return qRad;
}
//=========================================================================================================================
VectorXd robotStatus::getJntVelocities() {
    return dqJ;
}
//=========================================================================================================================
Vector robotStatus::getJntAccelerations() {
    return  ddqJ;
}
//=========================================================================================================================
Vector robotStatus::getJntTorques() {
    return tauJ;
}
//=========================================================================================================================
Vector robotStatus::getEEWrench() {
    return EEWrench;
}
//=========================================================================================================================
bool robotStatus::setCtrlMode (ControlMode ctrl_mode) {
    if (wbInterface->setControlMode (ctrl_mode)) {
        return true;
    } else {
        fprintf (stderr, "ERROR [robotStatus::setCtrlMode] >> Control mode could not be set\n");
        return false;
    }
}
//=========================================================================================================================
bool robotStatus::setCtrlMode(ControlMode ctrl_mode, int dof, double constRefSpeed)
{
    if(ctrl_mode == wbi::CTRL_MODE_POS) {
        yarp::sig::Vector refSpeed(dof, constRefSpeed);
        if(!wbInterface->setControlParam(wbi::CTRL_PARAM_REF_VEL, refSpeed.data())) {
            fprintf(stderr,"[ERR] robotStatus::setCtrlMode : Error setting reference speed\n");
            return false;
        }
    }
    if (wbInterface->setControlMode (ctrl_mode)) {
        return true;
    } else {
        fprintf (stderr, "ERROR [robotStatus::setCtrlMode] >> Control mode could not be set\n");
        return false;
    }

}

//=========================================================================================================================
void robotStatus::setdqDes (Vector dqD) {
#ifdef DEBUG
    fprintf (stderr, "robotStatus::setdqDes >> control reference to be sent is: \n%s\n", dqD.toString().c_str());
#endif
    if (!wbInterface->setControlReference (dqD.data()))
        fprintf (stderr, "ERROR [robotStatus::setdqDes] control reference could not be set.\n");
}
//=========================================================================================================================
bool robotStatus::inverseDynamics (double* qrad_input, double* dq_input, double* ddq_input, double* tauJ_computed) {
    bool ans = false;
    if (world2baseRototranslation (qrad_input)) {
        Vector qrad_base (16);
        qrad_base.resize (16, 0.0);
        Vector dq_base (6)  ;
        dq_base.resize (6, 0.0);
        Vector ddq_base (6) ;
        ddq_base.resize (6, 0.0);

        double* qrad_robot = &qrad_input[16];
        double* dq_robot   = &dq_input[6];
        double* ddq_robot  = &ddq_input[6];

        for (unsigned int i = 0; i < qrad_base.size(); i++) {
            qrad_base[i] = * (qrad_input + i);
        }

        for (unsigned int i = 0; i < dq_base.size(); i++)
            dq_base[i] = * (dq_input + i);
        for (unsigned int i = 0; i < ddq_base.size(); i++)
            ddq_base[i] = * (ddq_input + i);

#ifdef WORLD2BASE_EXTERNAL
        wbi::Frame qBaseFrame = wbi::Frame (qrad_base.data());
#ifdef DEBUG
        fprintf (stderr, "robotStatus::inverseDynamics >> This is the rototranslation \
	    matrix for the base that has been read: \n%s\n", qBaseFrame.toString().c_str());
        fprintf (stderr, "robotStatus::inverseDynamics >> Reading external world to base rototranslation\n");
#endif
        ans = wbInterface->inverseDynamics (qrad_robot, qBaseFrame, dq_robot, dq_base.data(), ddq_robot, ddq_base.data(), grav.data(), tauJ_computed);
#else
#ifdef DEBUG
        fprintf (stderr, "robotStatus::inverseDynamics >> Computing world to base rototranslation\n");
#endif
        ans = wbInterface->inverseDynamics (qrad_robot, xBase, dq_robot, dq_base.data(), ddq_robot, ddq_base.data(), grav.data(), tauJ_computed);
#endif

#ifdef DEBUG
        fprintf (stderr, "robotStatus::inverseDynamics qrad_robot: >> \n");
        for (int i = 0; i < 25; i++)
            fprintf (stderr, "%f ", qrad_robot[i]);
        fprintf (stderr, "\n");
        fprintf (stderr, "robotStatus::inverseDynamics >> qrad_base: (%f, %f, %f) \n", qrad_base[0], qrad_base[1], qrad_base[2]);
        fprintf (stderr, "robotStatus::inverseDynamics >> qrad_robot: (%f, %f, %f ...) \n", qrad_robot[0], qrad_robot[1], qrad_robot[2]);
#endif
    }

    return ans;
}
//=========================================================================================================================
bool robotStatus::dynamicsMassMatrix (double* qrad_input) {
    bool ans = false;
    if (robotJntAngles (false)) {
#ifdef DEBUG
        fprintf (stderr, "robotStatus::dynamicsMassMatrix >> robotJntAngles computed\n");
#endif
        if (world2baseRototranslation (qRad.data())) {
            Vector qrad_base (16);
            qrad_base.resize (16, 0.0);
            double* qrad_robot = &qrad_input[16];

            for (unsigned int i = 0; i < qrad_base.size(); i++) {
                qrad_base[i] = * (qrad_input + i);
            }

            wbi::Frame qBaseFrame = wbi::Frame (qrad_base.data());
#ifdef DEBUG
            fprintf (stderr, "robotStatus::dynamicsMassMatrix >> This is the rototranslation \
	  matrix for the base that has been read: \n%s\n", qBaseFrame.toString().c_str());
#endif

#ifdef WORLD2BASE_EXTERNAL
#ifdef DEBUG
            fprintf (stderr, "robotStatus::dynamicsMassMatrix >> world2baseRototranslation read\n");
#endif
            ans = wbInterface->computeMassMatrix (qrad_robot, qBaseFrame, massMatrix.data());
#else
#ifdef DEBUG
            fprintf (stderr, "robotStatus::dynamicsMassMatrix >> world2baseRototranslation computed\n");
#endif
            ans = wbInterface->computeMassMatrix (qrad_robot, xBase, massMatrix.data());
#endif
        }
    }
    return ans;
}
//=========================================================================================================================
MassMatrix robotStatus::getMassMatrix() {
    return massMatrix;
}
//=========================================================================================================================
Vector robotStatus::dynamicsGenBiasForces (double* qrad_input, double* dq_input) {
    bool ans = false;
    // grav is a 3x1 dim array
    Vector qrad_base (16);
    qrad_base.resize (16, 0.0);
    Vector dq_base (6)  ;
    dq_base.resize (6, 0.0);

    double* qrad_robot = &qrad_input[16];
    double* dq_robot   = &dq_input[6];

    for (unsigned int i = 0; i < qrad_base.size(); i++)
        qrad_base[i] = * (qrad_input + i);

    for (unsigned int i = 0; i < dq_base.size(); i++)
        dq_base[i] = * (dq_input + i);

    if (robotJntAngles (false)) {
#ifdef DEBUG
        fprintf (stderr, "robotStatus::dynamicsGenBiasForces >> robotJntAngles computed for dynamicsGenBiasForces\n");
#endif
        if (robotJntVelocities (false)) {
#ifdef DEBUG
            fprintf (stderr, "robotStatus::dynamicsGenBiasForces >> robotJntVelocities computed for dynamicsGenBiasForces\n");
#endif
            if (world2baseRototranslation (qRad.data())) {
#ifdef DEBUG
                fprintf (stderr, "robotStatus::dynamicsGenBiasForces >> world2baseRototranslation computed for dynamicsGenBiasForces\n");
#endif
                if (robotBaseVelocity()) {

#ifdef WORLD2BASE_EXTERNAL
                    wbi::Frame qBaseFrame = wbi::Frame (qrad_base.data());
#ifdef DEBUG
                    fprintf (stderr, "robotStatus::dynamicsGenBiasForces >> This is the rototranslation matrix for the base that has been read: \n%s\n", qBaseFrame.toString().c_str());
#endif
                    ans = wbInterface->computeGeneralizedBiasForces (qrad_robot, qBaseFrame, dq_robot, dq_base.data(), grav.data(), hterm.data());
#else

#ifdef DEBUG
                    printf ("computeGeneralizedBiasForces, computed with: \n");
                    printf ("qrad_robot: \n");
                    for (unsigned int i = 0; i < ROBOT_DOF; i++)
                        printf ("%f ", qrad_robot[i]);
                    printf ("\n");
                    printf ("xBase: \n%s\n", xBase.toString().c_str());
                    printf ("dq_robot: \n");
                    for (unsigned int i = 0; i < ROBOT_DOF; i++)
                        printf ("%f ", dq_robot[i]);
                    printf ("dq_base: \n%s\n", dq_base.toString().c_str());
                    printf ("grav: \n%s\n", grav.toString().c_str());
#endif
                    ans = wbInterface->computeGeneralizedBiasForces (qrad_robot, xBase, dq_robot, dq_base.data(), grav.data(), hterm.data());
#endif
                }
            }
        }
    }
    if (ans) {
#ifdef DEBUG
        fprintf (stderr, "robotStatus::dynamicsGenBiasForces >> h term: %s\n", hterm.toString().c_str());
#endif
        return hterm;
    } else {
        fprintf (stderr, "ERROR [robotStatus::dynamicsGenBiasForces] Generalized bias forces were not successfully computed\n");
        hterm.resize (ROBOT_DOF + 6, 0);
        return hterm;
    }
}
//=========================================================================================================================
bool robotStatus::robotBaseVelocity() {
//       ans = wbInterface->getEstimate(ESTIMATE_BASE_VEL, )
    return true;
}
//=========================================================================================================================
bool robotStatus::dynamicsDJdq (int& linkId, double* qrad_input, double* dq_input) {
    bool ans = false;
    if (robotJntAngles (false)) {
#ifdef DEBUG
        fprintf (stderr, "robotStatus::dynamicsDJdq >> robotJntAngles computed for dynamicsDJdq\n");
#endif
        if (robotJntVelocities (false)) {
#ifdef DEBUG
            fprintf (stderr, "robotStatus::dynamicsDJd >> robotJntVelocities computed for dynamicsDJdq\n");
#endif
            if (world2baseRototranslation (qRad.data())) {
                footLinkId = linkId;
                if (!robotBaseVelocity()) {
                    fprintf (stderr, "ERROR: robotStatus::dynamicsDJdq >> robotBaseVelocity failed in robotStatus::dynamicsDJd\n");
                    return false;
                }
                // This method does not use yet the last parameter xpose.
#ifdef DEBUG
                fprintf (stderr, "robotStatus::dynamicsDJd >> link ID is: %d", footLinkId);
#endif

                Vector qrad_base (16);
                qrad_base.resize (16, 0.0);
                Vector dq_base (6)  ;
                dq_base.resize (6, 0.0);

                double* qrad_robot = &qrad_input[16];
                double* dq_robot   = &dq_input[6];

                for (unsigned int i = 0; i < qrad_base.size(); i++) {
                    qrad_base[i] = * (qrad_input + i);
                }

                for (unsigned int i = 0; i < dq_base.size(); i++)
                    dq_base[i] = * (dq_input + i);

#ifdef WORLD2BASE_EXTERNAL
                wbi::Frame qBaseFrame = wbi::Frame (qrad_base.data());
#ifdef DEBUG
                fprintf (stderr, "robotStatus::dynamicsGenBiasForces >> This is the rototranslation matrix for the base that has been read: \n%s\n", qBaseFrame.toString().c_str());
#endif
                ans = wbInterface->computeDJdq (qrad_robot, qBaseFrame, dq_robot, dq_base.data(), footLinkId, dJdq.data());
#else
                ans = wbInterface->computeDJdq (qrad_robot, xBase, dq_robot, dq_base.data(), footLinkId, dJdq.data());
#endif
            }
        }
    }
    return ans;
}
//=========================================================================================================================
Vector robotStatus::getDJdq() {
    return dJdq;
}
//=========================================================================================================================
bool robotStatus::centroidalMomentum (double* qrad_input, double* dq_input, double* h) {
    bool ans = false;
    if (robotJntAngles (false)) {
        if (world2baseRototranslation (qRad.data())) {

            //Extract robot and base joint angles and velocities
            Vector qrad_base (16);
            qrad_base.resize (16, 0.0);
            Vector dq_base (6)  ;
            dq_base.resize (6, 0.0);

            double* qrad_robot = &qrad_input[16];
            double* dq_robot   = &dq_input[6];

            for (unsigned int i = 0; i < qrad_base.size(); i++) {
                qrad_base[i] = * (qrad_input + i);
            }

#ifdef WORLD2BASE_EXTERNAL
            wbi::Frame qBaseFrame = wbi::Frame (qrad_base.data());
            ans = wbInterface->computeCentroidalMomentum (qrad_robot, qBaseFrame, dq_robot, dq_base.data(), h);
#else
            #ifdef DEBUG
                fprintf (stderr, "robotStatus::centroidalMomentum >> Calling the main function \n");
            #endif
            ans = wbInterface->computeCentroidalMomentum (qrad_robot, xBase, dq_robot, dq_base.data(), h);
#endif
        }
    }
    return ans;
}
//=========================================================================================================================
/** Returns joints limits in radians.*/
bool robotStatus::getJointLimits (double* qminLims, double* qmaxLims, const int jnt) {
    bool ans = false;
    if (!wbInterface->getJointLimits (qminLims, qmaxLims, jnt)) {
        fprintf (stderr, "ERROR [robotStatus::getJointLimits] wbInterface->getJointLimits failed\n");
        return ans;
    } else {
        ans = true;
        return ans;
    }
}
//==========================================================================================================================
bool robotStatus::robotEEWrenches (wbi::LocalId LID) {
    bool ans = false;
    if (robotJntAngles (false)) {
        if (world2baseRototranslation (qRad.data())) {
            if ( ( (icubWholeBodyInterface*) wbInterface)->setWorldBasePosition (xBase)) {
                if (wbInterface->getEstimate (ESTIMATE_EXTERNAL_FORCE_TORQUE, LID, EEWrench.data())) {
#ifdef DEBUG
                    printf ("End effector wrench retrieved: \n %s", EEWrench.toString().c_str());
#endif
                    ans = true;
                    return ans;
                }
            }
        }
    }
    return ans;
}
//==========================================================================================================================
bool robotStatus::addEstimate() {
    if (wbInterface->addEstimate (ESTIMATE_EXTERNAL_FORCE_TORQUE, wbi::LocalId (iCub::skinDynLib::LEFT_LEG, 8)))
        return true;
    else
        return false;
}
//------------------------------------------------------------------------------------------------------------------------//
// END robotStatus implementation ----------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------//




//------------------------------------------------------------------------------------------------------------------------//
// COUNTERCLASS IMPLEMENTATION -------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------//

counterClass::counterClass() {
    count++;
}
//=========================================================================================================================
int counterClass::getCount() {
    return count;
}

// -----------------------------------------------------------------------------------------------------------------------//
// END COUNTERCLASS IMPLEMENTATION----------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------//





// -----------------------------------------------------------------------------------------------------------------------//
// START SIMULINK MDL IMPLEMENTATION--------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------------------------------------//
#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
    !mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))


// Function: MDL_CHECK_PARAMETERS
// Input parameters for locomotionController in Simulink

#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters (SimStruct* S) {
    {
        if (!IS_PARAM_DOUBLE (BLOCK_TYPE_PARAM)) {
            ssSetErrorStatus (S, "mdlCheckParameters: 1st parameter to S-function should be a number indicating the type of block");
            return;
        }
    }
}
#endif  /*MDL_CHECK_PARAMETERS*/

// Function: mdlInitializeSizes ===============================================
// Abstract:
//    The sizes information is used by Simulink to determine the S-function
//    block's characteristics (number of inputs, s, states, etc.).
static void mdlInitializeSizes (SimStruct* S) {
#ifdef DEBUG
    fprintf (stderr, "mdlInitializeSizes >> STARTED\n");
#endif

    ssSetNumSFcnParams (S, NPARAMS);
#if defined(MATLAB_MEX_FILE)
    if (ssGetSFcnParamsCount (S) == ssGetNumSFcnParams (S)) {                        // Then we have the usual case of val, robotName, localName
        mdlCheckParameters (S);
        if (ssGetErrorStatus (S) != NULL) {
            return;
        } else {
            fprintf (stderr, "mdlInitializeSizes >> BLOCK TYPE IS: %d\n", static_cast<int> (mxGetScalar (ssGetSFcnParam (S, BLOCK_TYPE_IDX))));
        }
    } else {
        printf ("There was an error assessing number of parameters\n");
        return;
    }
#endif


    // Specify I/O
    if (!ssSetNumInputPorts (S, 5)) return;
    ssSetInputPortWidth (S, 0, 1);                          //INPUT for BLOCK TYPE
    ssSetInputPortWidth (S, 1, ROBOT_DOF);                  //INPUT for dqDes (control reference, for setting positions, velocities or torques)
    ssSetInputPortWidth (S, 2, ROBOT_DOF + 16);         //INPUT for q (input angles different maybe from current ones)
    ssSetInputPortWidth (S, 3, ROBOT_DOF + 6);      //INPUT for dq (input joint velocities maybe different from current ones)
    ssSetInputPortWidth (S, 4, ROBOT_DOF + 6);      //INPUT for ddq (input joint accelerations maybe different from current ones)
    ssSetInputPortDataType (S, 0, SS_INT8);                 //Input data type
    ssSetInputPortDataType (S, 1, SS_DOUBLE);
    ssSetInputPortDataType (S, 2, SS_DOUBLE);
    ssSetInputPortDataType (S, 3, SS_DOUBLE);
    ssSetInputPortDataType (S, 4, SS_DOUBLE);
    ssSetInputPortDirectFeedThrough (S, 0, 1);              //The input will be used in the output
    ssSetInputPortDirectFeedThrough (S, 1, 1);
    ssSetInputPortDirectFeedThrough (S, 2, 1);
    ssSetInputPortDirectFeedThrough (S, 3, 1);
    ssSetInputPortDirectFeedThrough (S, 4, 1);
    if (!ssSetNumOutputPorts (S, 13)) return;
    ssSetOutputPortWidth (S, 0, ROBOT_DOF);                 // Robot joint angular positions in radians
    ssSetOutputPortWidth (S, 1, ROBOT_DOF);                 // Robot joint angular velocities in radians
    ssSetOutputPortWidth (S, 2, 7);                         // foot or COM pose from fwdKinematics.
    ssSetOutputPortWidth (S, 3, 186);                       // 6 x (N+6) Jacobians for a specific link
    ssSetOutputPortWidth (S, 4, (ROBOT_DOF + 6) * (ROBOT_DOF + 6)); // Mass matrix of size (N+6 x N+6)
    ssSetOutputPortWidth (S, 5, ROBOT_DOF + 6);             // Generalized bias forces of size (N+6 x 1)
    ssSetOutputPortWidth (S, 6, 6);                     // dot{J}dot{q} term
    ssSetOutputPortWidth (S, 7, ROBOT_DOF);             // Joint accelerations
    ssSetOutputPortWidth (S, 8, ROBOT_DOF);             // Joint torques
    ssSetOutputPortWidth (S, 9, ROBOT_DOF);             // Compute torques with inverse dynamics
    ssSetOutputPortWidth (S, 10, ROBOT_DOF);            // Min joint limits;
    ssSetOutputPortWidth (S, 11, ROBOT_DOF);            // Max joint limits;
    ssSetOutputPortWidth (S, 12, 6);                //Centroidal momentum
    ssSetOutputPortDataType (S, 0, 0);
    ssSetOutputPortDataType (S, 1, 0);
    ssSetOutputPortDataType (S, 2, 0);
    ssSetOutputPortDataType (S, 3, 0);
    ssSetOutputPortDataType (S, 4, 0);
    ssSetOutputPortDataType (S, 5, 0);
    ssSetOutputPortDataType (S, 6, 0);
    ssSetOutputPortDataType (S, 7, 0);
    ssSetOutputPortDataType (S, 8, 0);
    ssSetOutputPortDataType (S, 9, 0);
    ssSetOutputPortDataType (S, 10, 0);
    ssSetOutputPortDataType (S, 11, 0);
    ssSetOutputPortDataType (S, 12, 0);

    ssSetNumSampleTimes (S, 1);

    // Reserve place for C++ object
    ssSetNumPWork (S, 3);

    ssSetNumDWork (S, 3);
    ssSetDWorkWidth (S, 0, 1);
    ssSetDWorkWidth (S, 1, 2);
    ssSetDWorkWidth (S, 2, 1);
    ssSetDWorkDataType (S, 0, SS_DOUBLE);
    ssSetDWorkDataType (S, 1, SS_INTEGER);
    ssSetDWorkDataType (S, 2, SS_INTEGER);

    ssSetSimStateCompliance (S, USE_CUSTOM_SIM_STATE);

    ssSetOptions (S,
                  SS_OPTION_WORKS_WITH_CODE_REUSE |
                  SS_OPTION_EXCEPTION_FREE_CODE | //we must be sure that every function we call never throws an exception
                  SS_OPTION_ALLOW_INPUT_SCALAR_EXPANSION |
                  SS_OPTION_USE_TLC_WITH_ACCELERATOR);

    // TODO For FUTURE WORK this flag should be called and debug by correctly programming mdlTerminate.
    //SS_OPTION_CALL_TERMINATE_ON_EXIT); //this calls the terminate function even in case of errors
#ifdef DEBUG
    fprintf (stderr, "mdlInitializeSizes >> FINISHED\n\n");
#endif

}


// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes (SimStruct* S) {
#ifdef DEBUG
    fprintf (stderr, "mdlInitializeSampleTimes >> STARTED\n");
#endif
    // The sampling time of this SFunction must be inherited so that the Soft Real Time sblock can be used.
    ssSetSampleTime (S, 0, INHERITED_SAMPLE_TIME);
    // ssSetSampleTime(S, 0, 10.0);
    ssSetOffsetTime (S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance (S);
#ifdef DEBUG
    fprintf (stderr, "mdlInitializeSampleTimes >> FINISHED\n\n");
#endif
}

// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you
//   have states that should be initialized once, this is the place
//   to do it.
#define MDL_START
static void mdlStart (SimStruct* S) {
//     fprintf (stderr, "mdlStart >> STARTED\n");
    // Counts the times the blocks have been STARTED
    counterClass counter;
    double* minJntLimits = new double[ROBOT_DOF];
    double* maxJntLimits = new double[ROBOT_DOF];
    ssGetPWork (S) [1] = &minJntLimits[0];
    ssGetPWork (S) [2] = &maxJntLimits[0];

    int_T* flagConfig = (int_T*) ssGetDWork (S, 2);
    flagConfig[0] = 0;

//     fprintf (stderr, "mdlStart >> Publicly stating that a new child has been born: %d \n", counter.getCount());

    // Objects creation
    // ------------------- START YARP INITIALIZATION -----------------------
    Network::init();

    if (!Network::checkNetwork() || !Network::initialized()) {
        ssSetErrorStatus (S, "ERROR: [mdlStart] YARP server wasn't found active!! \n");
        return;
    } else {
        fprintf (stderr, "mdlStart >> YARP is up and running!!\n");
    }
    // -------------------- END YARP INITALIZATION STUFF --------------------

    robotStatus* robot = new robotStatus();


    char* cString = mxArrayToString (ssGetSFcnParam (S, STRING_PARAM_IDX));
    if (!cString) {
        ssSetErrorStatus (S, "mdlStart >> Cannot retrieve string from parameter 1.\n");
        return;
    }
//     fprintf (stderr, "mdlStart >> The string being passed for robotName is - %s\n ", cString);
    std::string robot_name (cString);
    mxFree (cString);
    cString = 0;

    cString = mxArrayToString (ssGetSFcnParam (S, LOCAL_PARAM_IDX));
    if (!cString) {
        ssSetErrorStatus (S, "mdlStart >> Cannot retrieve string from parameter 2.\n");
        return;
    }
    std::string local_name (cString);
    mxFree (cString);
    cString = 0;

    real_T block_type = mxGetScalar (ssGetSFcnParam (S, BLOCK_TYPE_IDX));
//     fprintf (stderr, "mdlStart >> BLOCK TYPE MASK PARAMETER: %f\n", block_type);

    cString = mxArrayToString (ssGetSFcnParam (S, LINKNAME_PARAM_IDX));
    if (!cString) {
        ssSetErrorStatus (S, "mdlStart >> Cannot retrieve string from parameter 4.\n");
        return;
    }
    printf("mdlStart >> The link passed for parametric blocks is: %s\n", cString);
    std::string param_link_name (cString);
    mxFree (cString);
    cString = 0;

    // This will help determining the kind of block we'll be using
    real_T* x = (real_T*) ssGetDWork (S, 0);
    x[0]      = block_type;

    // Control flag for end effector wrench add method
    int_T* flag = (int_T*) ssGetDWork (S, 1);
    flag[0]     = 1;


    switch (static_cast<int> (block_type)) {
    case 0:
        printf ("mdlStart >> This block will retrieve joints angles\n");
        break;
    case 1:
        printf ("mdlStart >> This block will retrieve joints velocities\n");
        break;
    case 2:
        printf ("mdlStart >> This block will retrieve parametric forward kinematics\n");
        break;
    case 3:
        printf ("mdlStart >> This block will retrieve parametric Jacobians\n");
        break;
    case 4:
        printf ("mdlStart >> This block will set velocities\n");
        break;
    case 5:
        printf ("mdlStart >> This block will set positions\n");
        break;
    case 6:
        printf ("mdlStart >> This block will set torques\n");
        break;
    case 7:
        printf ("mdlStart >> This block will compute generalized bias forces from dynamics\n");
        break;
    case 8:
        printf ("mdlStart >> This block will compute mass matrix from dynamics\n");
        break;
    case 9:
        printf ("mdlStart >> This block will compute dJdq\n");
        break;
    case 10:
        printf ("mdlStart >> This block will retrieve joint accelerations\n");
        break;
    case 11:
        printf ("mdlStart >> This block will retrieve joint torques\n");
        break;
    case 12:
        printf ("mdlStart >> This block will compute inverse dynamics\n");
        break;
    case 13:
        printf ("mdlStart >> This block will retrieve joint limits\n");
        break;
    case 14:
        printf ("mdlStart >> This block will retrieve the centroidal momentum\n");
        break;
    case 15:
        printf ("mdlStart >> This block will retrieve end-effector wrenches for legs or arms\n");
        break;
    case 16:
        printf ("mdlStart >> This block will attach the world reference frame to the specified link\n");
        break;
    case 17:
        printf ("mdlStart >> This block will compute forward kinematics for the specified link\n");
        break;
    case 18:
        printf ("mdlStart >> This  block will compute the Jacobian matrix for the specified link\n");
        break;
    case 19:
        printf ("mdlStart >> This block will compute the product dJdq for the specified link\n");
        break;
    default:
        ssSetErrorStatus (S, "ERROR [mdlStart] The type of this block has not been defined yet\n");
    }

    // INPUT PARAMETER FOR PARAMETRIC FORWARD KINEMATICS and JACOBIANS
    InputPtrsType           u = ssGetInputPortSignalPtrs (S, 0);
    InputInt8PtrsType   uPtrs = (InputInt8PtrsType) u;

//     fprintf (stderr, "mdlStart >> An object robot of type wholeBodyInterface has been created\n");
    robot->setmoduleName (local_name);
    robot->setRobotName (robot_name);
    robot->setParamLink (param_link_name);

// NOTE: If I don't do this here when I call ssSetErrorStatus in the following validation it will try to retrieve from PWork that wouldn't exist yet
// in a few words, always remember to have initialized all the objects you instantiate before the first ssSetErrorStatus as it will call mdlTerminate

    ssGetPWork (S) [0] = robot; // update PWork

    if(robot_name == "icubGazeboSim"){
        if(yarp::os::NetworkBase::exists(string("/"+robot_name+"/torso/state:o").c_str()))
            printf ("iCub on the Gazebo simulator has been found active. Proceeding with configuration of the interface...\n");
        else{
            ssSetErrorStatus (S, "ERROR [mdlStart] >> The simulator is not running... ");
            return;
        }
    } else{
        std::size_t pos = robot_name.find ("iCub");
        if(pos != std::string::npos){ //iCubGenova0X is being used
            if(yarp::os::NetworkBase::exists(string("/icub/torso/state:o").c_str()))
                printf("!!! You're using the real iCub platform. Proceeding with configuration of the interface... \n");
            else{
                ssSetErrorStatus (S, "ERROR [mdlStart] >> The real platform is not reachable... ");
                return;
            }
        }
    }

    fprintf (stderr, "mdlStart >> About to configure robot... \n");
    bool res = robot->robotConfig();
    if (res) {
        flagConfig[0]     = 1;
        fprintf (stderr, "mdlStart >> Succesfully exited robotConfig.\n");
    }
    else {
        ssSetErrorStatus (S, "ERROR [mdlStart] in robotConfig.\n");
        return;
    }

    res = res && robot->robotInit (static_cast<int> (block_type), static_cast<int> (*uPtrs[0]));
    if (res == true)
        fprintf (stderr, "mdlStart >> Succesfully exited robotInit.\n");
    else {
        ssSetErrorStatus (S, "ERROR [mdlStart] in robotInit. \n");
	return;
    }

    if (!robot->getJointLimits (&minJntLimits[0], &maxJntLimits[0], -1)) {
        ssSetErrorStatus (S, "ERROR [mdlOutput] Joint limits could not be computed\n");
        return;
    }

    ssGetPWork (S) [0] = robot;
    ssGetPWork (S) [1] = &minJntLimits[0];
    ssGetPWork (S) [2] = &maxJntLimits[0];


    //--------------GLOBAL VARIABLES INITIALIZATION --------------
    fprintf (stderr, "mdlStart >> FINISHED\n\n");
}

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs (SimStruct* S, int_T tid) {
#ifdef DEBUG
    fprintf(stderr,"mdlOutputs: Entering ... \n");
#endif
    double tinit, tend;
    if (TIMING) tinit = Time::now();

    //Getting type of block
    real_T* block_type = (real_T*) ssGetDWork (S, 0);
    int btype = (int) block_type[0];
    int_T* flag = (int_T*) ssGetDWork (S, 1);
    int Flag = (int) flag[0];
    const char* linkName;
    int lid = 0;
    Vector dJdq; dJdq.resize (6, 0.0);

    robotStatus* robot = (robotStatus*) ssGetPWork (S) [0];
    bool blockingRead = false;
#ifdef DEBUG
    fprintf (stderr, "mdlOutputs: wbInterface pointers: %p %p for btype %d\n", robot->tmpContainer, robot->wbInterface, btype);
#endif


    // INPUT PARAMETER FOR FORWARD KINEMATICS
    InputPtrsType           u = ssGetInputPortSignalPtrs (S, 0);
    InputInt8PtrsType   uPtrs = (InputInt8PtrsType) u;
#ifdef DEBUG
    fprintf (stderr, "mdlOutputs: Input port value: %d \n", (int) (*uPtrs[0]));
#endif

    // This block will compute robot joint angles
    if (btype == 0) {
        Vector qrad (ROBOT_DOF);
        qrad.zero();
        if (robot->robotJntAngles (blockingRead)) {
            qrad = robot->getEncoders();
#ifdef DEBUG
            fprintf (stderr, "mdlOutputs: Angles have been retrieved:\n %s \n", qrad.toString().c_str());
#endif

            real_T* pY1 = (real_T*) ssGetOutputPortSignal (S, 0);
            int_T widthPort = ssGetOutputPortWidth (S, 0);
            for (int_T i = 0; i < widthPort; i++) {
                pY1[i] = qrad ( (int) i);
            }
        } else {
            fprintf (stderr, "ERROR: [mdlOutputs] Robot Joint Angles could not be computed\n");
        }
    }
//
    // This block will compute robot joint velocities
    if (btype == 1) {
#ifdef DEBUG
        fprintf (stderr, "mdlOutputs: About to send joint velocities to ports...\n");
#endif
        Eigen::VectorXd dotq;
        dotq.Zero (ROBOT_DOF);

        if (robot->robotJntVelocities (blockingRead)) {
            dotq = robot->getJntVelocities();
            real_T* pY2 = (real_T*) ssGetOutputPortSignal (S, 1);
            int_T widthPort = ssGetOutputPortWidth (S, 1);
            for (int_T i = 0; i < widthPort; i++) {
                pY2[i] = dotq ( (int) i);
            }
        } else {
            fprintf (stderr, "ERROR: [mdlOutputs] Robot joint velocities could not be computed\n");
        }
    }

    // This block will compute forward kinematics of the specified link
    if (btype == 2) {
        Vector xpose;
        switch ( (int) *uPtrs[0]) {
        case 0:
            linkName = "r_sole";
            break;
        case 1:
            linkName = "l_sole";
            break;
        case 2:
            linkName = "com";
            break;
        case 3:
            linkName = "r_gripper";
            break;
        case 4:
            linkName = "l_gripper";
            break;
        case 5:
            linkName = "head";
            break;
        default:
            fprintf (stderr, "ERROR: [mdlOutputs] No body part has been specified to compute forward kinematics\n");
        }
        robot->getLinkId (linkName, lid);
#ifdef DEBUG
       fprintf(stderr,"mdlOutputs: RIGHT BEFORE CALLING FORWARDKINEMATICS\n");
#endif
        xpose = robot->forwardKinematics (lid);

#ifdef DEBUG
       fprintf(stderr,"mdlOutputs: RIGHT AFTER CALLING FORWARDKINEMATICS\n");
#endif

        real_T* pY3 = (real_T*) ssGetOutputPortSignal (S, 2);
        for (int_T j = 0; j < ssGetOutputPortWidth (S, 2); j++) {
            pY3[j] = xpose ( (int) j);
        }
    }

    // This block will compute Jacobians for the specified link
    if (btype == 3) {
        JacobianMatrix jacob;
        switch ( (int) *uPtrs[0]) {
        case 0:
            linkName = "r_sole";
            break;
        case 1:
            linkName = "l_sole";
            break;
        case 2:
            linkName = "com";
            break;
        case 3:
            linkName = "r_gripper";
            break;
        case 4:
            linkName = "l_gripper";
            break;
        case 5:
            linkName = "head";
            break;
        default:
            fprintf (stderr, "ERROR: [mdlOutputs] No body part has been specified to compute jacobians\n");
        }
        robot->getLinkId (linkName, lid);

        jacob = robot->jacobian (lid);
#ifdef DEBUG
        fprintf (stderr, "mdlOutputs: Jacobians Computed Succesfully. Jacobian is: \n");
#endif

        real_T* pY4 = (real_T*) ssGetOutputPortSignal (S, 3);
        for (int_T j = 0; j < ssGetOutputPortWidth (S, 3); j++) {
            pY4[j] = jacob (j);
        }
    }

    // This block will set control references for the specified control mode
    if (btype == 4 || btype == 5 || btype == 6) {
        //GET INPUT dqDes
        InputRealPtrsType uPtrs1 = ssGetInputPortRealSignalPtrs (S, 1); //Get the corresponding pointer to "desired position port"
        int nu = ssGetInputPortWidth (S, 1);                            //Getting the amount of elements of the input vector/matrix
        Vector dqDestmp;
        dqDestmp.resize (ROBOT_DOF, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            dqDestmp (j) = (*uPtrs1[j]);
        }
        if (btype == 4) robot->setCtrlMode (CTRL_MODE_VEL);
        if (btype == 5) {
            if(!robot->setCtrlMode (CTRL_MODE_POS, ROBOT_DOF, CTRL_DEG2RAD*10.0)) {
                fprintf(stderr,"[ERR] Error setting position control mode\n");
                ssSetErrorStatus(S, "[ERR] Error setting position control mode");
                return;
            }
        }
        if (btype == 6) robot->setCtrlMode (CTRL_MODE_TORQUE);
        robot->setdqDes (dqDestmp);
    }

    Vector h;
    h.resize (ROBOT_DOF + 6, 0);
    // This block will compute the generalized bias force from the dynamics equation
    if (btype == 7) {
        int nu;
        //READ INPUT ANGLES
        InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs (S, 2); //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth (S, 2);                                //Getting the amount of elements of the input vector/matrix
        Vector qrad_in;
        qrad_in.resize (nu, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            qrad_in (j) = (*uPtrs2[j]);
        }

        //READ INPUT JOINT VELOCITIES
        InputRealPtrsType uPtrs3 = ssGetInputPortRealSignalPtrs (S, 3); //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth (S, 3);                                //Getting the amount of elements of the input vector/matrix
        Vector dqrad_in;
        dqrad_in.resize (nu, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            dqrad_in (j) = (*uPtrs3[j]);
        }

        h = robot->dynamicsGenBiasForces (qrad_in.data(), dqrad_in.data());
        real_T* pY5 = (real_T*) ssGetOutputPortSignal (S, 5);
        for (int_T j = 0; j < ssGetOutputPortWidth (S, 5); j++) {
            pY5[j] = h (j);
        }
    }

    MassMatrix massMatrix;
    // This block will return the mass matrix from the dynamics equation
    if (btype == 8) {
        int nu;
        //READ INPUT ANGLES
        InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs (S, 2); //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth (S, 2);                                //Getting the amount of elements of the input vector/matrix
        Vector qrad_in;
        qrad_in.resize (nu, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            qrad_in (j) = (*uPtrs2[j]);
        }
#ifdef DEBUG
        fprintf (stderr, "mdlOutputs: About to compute mass matrix\n");
#endif
        if (!robot->dynamicsMassMatrix (qrad_in.data())) {
            fprintf (stderr, "ERROR: [mdlOutputs] Mass matrix was not successfully computed\n");
        }
        massMatrix = robot->getMassMatrix();
        real_T* pY6 = (real_T*) ssGetOutputPortSignal (S, 4);
        for (int_T j = 0; j < ssGetOutputPortWidth (S, 4); j++) {
            pY6[j] = massMatrix (j);
        }
    }

    // This block will compute dJdq from the dynamics equation for the specified link
    if (btype == 9) {
#ifdef DEBUG
        fprintf (stderr, "mdlOutputs: About to compute dJdq\n");
#endif
        switch ( (int) *uPtrs[0]) {
        case 0:
            linkName = "r_sole";
            break;
        case 1:
            linkName = "l_sole";
            break;
        case 2:
            linkName = "com";
            break;
        case 3:
            linkName = "r_gripper";
            break;
        case 4:
            linkName = "l_gripper";
            break;
        case 5:
            linkName = "head";
            break;
        default:
            fprintf (stderr, "ERROR: [mdlOutputs] No body part has been specified to compute forward kinematics\n");
        }

        int nu;
        //READ INPUT ANGLES
        InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs (S, 2); //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth (S, 2);                                //Getting the amount of elements of the input vector/matrix
        Vector qrad_in;
        qrad_in.resize (nu, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            qrad_in (j) = (*uPtrs2[j]);
        }

        //READ INPUT JOINT VELOCITIES
        InputRealPtrsType uPtrs3 = ssGetInputPortRealSignalPtrs (S, 3); //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth (S, 3);                                //Getting the amount of elements of the input vector/matrix
        Vector dqrad_in;
        dqrad_in.resize (nu, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            dqrad_in (j) = (*uPtrs3[j]);
        }

        robot->getLinkId (linkName, lid);
        if (!robot->dynamicsDJdq (lid, qrad_in.data(), dqrad_in.data())) {
            fprintf (stderr, "ERROR: [mdlOutputs] dynamicsDJdq did not finish successfully\n");
        } else {
            dJdq = robot->getDJdq();
        }

        real_T* pY7 = (real_T*) ssGetOutputPortSignal (S, 6);
#ifdef DEBUG
        fprintf (stderr, "mdlOutputs: djdq computed is: %s \n", dJdq.toString().c_str());
#endif
        for (int_T j = 0; j < ssGetOutputPortWidth (S, 6); j++) {
            pY7[j] = dJdq (j);
        }
    }

    // This block will retrieve joint accelerations
    if (btype == 10) {
        Vector ddqJ;
        ddqJ.resize (ROBOT_DOF, 0);
        if (robot->robotJntAccelerations (blockingRead)) {
            ddqJ = robot->getJntAccelerations();
            //Stream joint accelerations
            real_T* pY8 = (real_T*) ssGetOutputPortSignal (S, 7);
            for (int_T j = 0; j < ssGetOutputPortWidth (S, 7); j++) {
                pY8[j] = ddqJ (j);
            }
        } else {
            fprintf (stderr, "ERROR: [mdlOutputs] Joint accelerations could not be retrieved\n");
        }
    }

    // This block will retrieve joint torques
    yarp::sig::Vector tauJ (ROBOT_DOF);
    if (btype == 11) {
        if (robot->robotJntTorques (blockingRead)) {
            tauJ = robot->getJntTorques();
            //Stream joint torques
            real_T* pY9 = (real_T*) ssGetOutputPortSignal (S, 8);
            for (int_T j = 0; j < ssGetOutputPortWidth (S, 8); j++) {
                pY9[j] = tauJ (j);
            }
        } else {
            fprintf (stderr, "ERROR: [mdlOutputs) Joint torques could not be retrieved\n");
        }
    }

    // This block will compute inverse dynamics
    if (btype == 12) {
        int nu;
        //READ INPUT ANGLES
        InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs (S, 2); //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth (S, 2);                                //Getting the amount of elements of the input vector/matrix
        Vector qrad_in;
        qrad_in.resize (nu, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            qrad_in (j) = (*uPtrs2[j]);
        }

        //READ INPUT JOINT VELOCITIES
        InputRealPtrsType uPtrs3 = ssGetInputPortRealSignalPtrs (S, 3); //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth (S, 3);                                //Getting the amount of elements of the input vector/matrix
        Vector dqrad_in;
        dqrad_in.resize (nu, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            dqrad_in (j) = (*uPtrs3[j]);
        }

        //READ INPUT JOINT ACCELERATIONS
        InputRealPtrsType uPtrs4 = ssGetInputPortRealSignalPtrs (S, 4); //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth (S, 4);                                //Getting the amount of elements of the input vector/matrix
        Vector ddqrad_in;
        ddqrad_in.resize (nu, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            ddqrad_in (j) = (*uPtrs4[j]);
        }

        yarp::sig::Vector tau_computed;
        tau_computed.resize (ROBOT_DOF + 6, 0.0);
        if (robot->inverseDynamics (qrad_in.data(), dqrad_in.data(), ddqrad_in.data(), tau_computed.data())) {
#ifdef DEBUG
            fprintf (stderr, "robotStatus::inverseDynamics >> Inverse dynamics has been computed correctly\n");
            fprintf (stderr, "robotStatus::inverseDynamics >> Angs: %s\n", qrad_in.toString().c_str());
            fprintf (stderr, "robotStatus::inverseDynamics >> Vels: %s\n",  dqrad_in.toString().c_str());
            fprintf (stderr, "robotStatus::inverseDynamics >> Accs: %s\n", ddqrad_in.toString().c_str());
            fprintf (stderr, "robotStatus::inverseDynamics >> Size of tau_computed is: \n%d\n", int (tau_computed.size()));
            fprintf (stderr, "robotStatus::inverseDynamics >> Computed torques are: \n%s\n", tau_computed.toString().c_str());
#endif
            //Stream computed joint torques by inverseDynamics
            real_T* pY10 = (real_T*) ssGetOutputPortSignal (S, 9);
            for (int_T j = 0; j < ssGetOutputPortWidth (S, 9); j++) {
                pY10[j] = tau_computed (j + 6);
            }
        }
    }

    // min/max joint limits
    if (btype == 13) {
        double* minJntLimits = 0;// new double[ROBOT_DOF];
        double* maxJntLimits = 0; //new double[ROBOT_DOF];
        // Gets joint limits for the entire body since we're still using ROBOT_DOF as default
        if (!ssGetPWork (S) [1] & !ssGetPWork (S) [2]) {
            ssSetErrorStatus (S, "ERROR [mdlOutput] Joint limits could not be computed\n");
        } else {
#ifdef DEBUG
//             fprintf(stderr,"minJntLimits are: \n%s\n maxJntLimits are: \n%s\n", minJntLimits.toString().c_str(), maxJntLimits.toString().c_str());
#endif
            minJntLimits = (double*) ssGetPWork (S) [1];
            maxJntLimits = (double*) ssGetPWork (S) [2];
            real_T* pY11 = (real_T*) ssGetOutputPortSignal (S, 10);
            real_T* pY12 = (real_T*) ssGetOutputPortSignal (S, 11);
            for (int_T j = 0; j < ssGetOutputPortWidth (S, 10); j++) {
                pY11[j] = minJntLimits[j];
            }
            for (int_T j = 0; j < ssGetOutputPortWidth (S, 11); j++) {
                pY12[j] = maxJntLimits[j];
            }
        }
    }

    // angular momentumminJntLimits
    if (btype == 14) {

        int nu;
        //READ INPUT ANGLES
        InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs (S, 2); //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth (S, 2);                                //Getting the amount of elements of the input vector/matrix
        Vector qrad_in;
        qrad_in.resize (nu, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            qrad_in (j) = (*uPtrs2[j]);
        }

        //READ INPUT JOINT VELOCITIES
        InputRealPtrsType uPtrs3 = ssGetInputPortRealSignalPtrs (S, 3); //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth (S, 3);                                //Getting the amount of elements of the input vector/matrix
        Vector dqrad_in;
        dqrad_in.resize (nu, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            dqrad_in (j) = (*uPtrs3[j]);
        }

        // Centroidal momentum
        Vector momentum (6);
        momentum.zero();
        if (!robot->centroidalMomentum (qrad_in.data(), dqrad_in.data(), momentum.data()))
            fprintf (stderr, "ERROR [mdlOutput] in robot->centroidalMomentum");
        real_T* pY13 = (real_T*) ssGetOutputPortSignal (S, 12);
        for (int_T j = 0; j < ssGetOutputPortWidth (S, 12); j++)
            pY13[j] = momentum[j];
    }

    // This block will retrieve force/torque estimates at the end effectors of the arms and legs of the robot
    if (btype == 15) {
        wbi::LocalId LID = wbi::LocalId (0, 0);
        switch ( (int) *uPtrs[0]) {
        case 0:
            linkName = "r_sole";
            LID = wbi::LocalId (iCub::skinDynLib::RIGHT_LEG, 8);
            break;
        case 1:
            linkName = "l_sole";
            LID = wbi::LocalId (iCub::skinDynLib::LEFT_LEG, 8);
            break;
        case 2:
            linkName = "r_gripper";
            LID = wbi::LocalId (iCub::skinDynLib::RIGHT_ARM, 8);
            break;
        case 3:
            linkName = "l_gripper";
            LID = wbi::LocalId (iCub::skinDynLib::LEFT_ARM, 8);
            break;
        default:
            fprintf (stderr, "ERROR: [mdlOutputs] No body part has been specified to compute end effectors wrenches\n");
        }

        if (flag) {
            if (robot->addEstimate()) {
#ifdef DEBUG
                printf ("mdlOutputs: addEstimate exited correctly");
#endif
                flag = 0;
            }
        }

        Vector tmp (6);
        tmp.zero();
        if (!robot->robotEEWrenches (LID)) {
            printf ("Error obtaining es for ... \n");
        } else {
            tmp = robot->getEEWrench();
        }

        // We're gonna use in this case the same six-dimensional output used for dJdq
        real_T* pY15 = (real_T*) ssGetOutputPortSignal (S, 6);
        for (int_T j = 0; j < ssGetOutputPortWidth (S, 6); j++)
            pY15[j] = tmp (j);
    }

    if (btype == 16) {
        InputRealPtrsType uPtrs0   = ssGetInputPortRealSignalPtrs (S, 0); //Get the input link
        InputInt8PtrsType uPtrsInt = (InputInt8PtrsType) uPtrs0;
        int extLink = static_cast<int> (*uPtrsInt[0]);
        const char* charLink = NULL;
        switch (extLink) {
        case 0:
            charLink = "l_sole";
            break;
        case 1:
            charLink = "r_sole";
            break;
        default:
            charLink = "l_sole";
        }
        robot->setWorldReferenceFrame (charLink);
    }


    // Parametric forward kinematics
    if (btype == 17) {
        Vector xpose;
        std::string tmpStr (robot->getParamLink());
        linkName = tmpStr.c_str();
        robot->getLinkId (linkName, lid);
#ifdef DEBUG
       fprintf(stderr,"mdlOutputs: RIGHT BEFORE CALLING FORWARDKINEMATICS\n");
#endif
        xpose = robot->forwardKinematics (lid);
#ifdef DEBUG
       fprintf(stderr,"mdlOutputs: RIGHT AFTER CALLING FORWARDKINEMATICS\n");
#endif
        real_T* pY3 = (real_T*) ssGetOutputPortSignal (S, 2);
        for (int_T j = 0; j < ssGetOutputPortWidth (S, 2); j++) {
            pY3[j] = xpose ( (int) j);
        }
    }

    // Parametric Jacobians
    if (btype == 18) {
        JacobianMatrix jacob;
        std::string tmpStr (robot->getParamLink());
        linkName = tmpStr.c_str();
#ifdef DEBUG
        printf("Parametric Jacobian will be computed for linkName: %s\n", linkName);
#endif
        robot->getLinkId (linkName, lid);

        jacob = robot->jacobian (lid);
#ifdef DEBUG
        fprintf (stderr, "mdlOutputs: Jacobians Computed Succesfully. Jacobian is: \n");
#endif

        real_T* pY4 = (real_T*) ssGetOutputPortSignal (S, 3);
        for (int_T j = 0; j < ssGetOutputPortWidth (S, 3); j++) {
            pY4[j] = jacob (j);
        }
    }

    // Parametric dJdq
    if (btype == 19) {
        std::string tmpStr (robot->getParamLink());
        linkName = tmpStr.c_str();
#ifdef DEBUG
        printf("Parametric dJdq will be computed for linkName: %s\n", linkName);
#endif
        robot->getLinkId(linkName, lid);

        int nu;
        //READ INPUT ANGLES
        InputRealPtrsType uPtrs2 = ssGetInputPortRealSignalPtrs (S, 2); //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth (S, 2);                                //Getting the amount of elements of the input vector/matrix
        Vector qrad_in;
        qrad_in.resize (nu, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            qrad_in (j) = (*uPtrs2[j]);
        }

        //READ INPUT JOINT VELOCITIES
        InputRealPtrsType uPtrs3 = ssGetInputPortRealSignalPtrs (S, 3); //Get the corresponding pointer to "input joint angles port"
        nu = ssGetInputPortWidth (S, 3);                                //Getting the amount of elements of the input vector/matrix
        Vector dqrad_in;
        dqrad_in.resize (nu, 0.0);
        for (int j = 0; j < nu; j++) {                                  //Reading inpute reference
            dqrad_in (j) = (*uPtrs3[j]);
        }

        if (!robot->dynamicsDJdq (lid, qrad_in.data(), dqrad_in.data())) {
            fprintf (stderr, "ERROR: [mdlOutputs] dynamicsDJdq did not finish successfully\n");
        } else {
            dJdq = robot->getDJdq();
        }

        real_T* pY7 = (real_T*) ssGetOutputPortSignal (S, 6);
#ifdef DEBUG
        fprintf (stderr, "mdlOutputs: djdq computed is: %s \n", dJdq.toString().c_str());
#endif
        for (int_T j = 0; j < ssGetOutputPortWidth (S, 6); j++) {
            pY7[j] = dJdq (j);
        }
    }

    if (TIMING) tend = Time::now();
    if (TIMING) fprintf (stderr, "Time elapsed: %f \n", tend - tinit);

}

// Function: mdlTerminate =====================================================
// Abstract:
//   In this function, you should perform any actions that are necessary
//   at the termination of a simulation.  For example, if memory was
//   allocated in mdlStart, this is the place to free it.
static void mdlTerminate (SimStruct* S) {
    fprintf(stderr,"Terminating...\n");
    // IF YOU FORGET TO DESTROY OBJECTS OR DEALLOCATE MEMORY, MATLAB WILL CRASH.
    // Retrieve and destroy C++ object
    robotStatus* robot = (robotStatus*) ssGetPWork (S) [0];
    double* minJntLimits = 0;//new double[ROBOT_DOF];
    double* maxJntLimits = 0;//new double[ROBOT_DOF];

    minJntLimits = (double*) ssGetPWork (S) [1];
    maxJntLimits = (double*) ssGetPWork (S) [2];

    // Was the interface configured?
    int_T* flagConfig = (int_T*) ssGetDWork (S, 2);

#ifdef DEBUG
    fprintf (stderr, "mdlTerminate: robot pointer: %p\n", robot);
#endif


    if (robot != NULL) {
        fprintf (stderr, "mdlTerminate >> Inside robot object %p \n", robot);
        if (robot->decreaseCounter() == 0) {
	    // NOTE This should be done only if the WBI has been configured and control mode was initially set.
	    if (flagConfig[0] == 1){
	      robot->setCtrlMode (CTRL_MODE_POS, ROBOT_DOF, 0.0);
	      printf ("ctrl mode set\n");
	    }
            delete robot;
            printf ("robot deleted\n");
            delete[] minJntLimits;
            printf ("minJntLimits deleted\n");
            delete[] maxJntLimits;
            printf ("maxJntLimits deleted\n");
            robotStatus::resetCounter();
            robot        = NULL;
            minJntLimits = NULL;
            maxJntLimits = NULL;
            ssSetPWorkValue (S, 0, NULL);
            ssSetPWorkValue (S, 1, NULL);
            ssSetPWorkValue (S, 2, NULL);
        }
    }
        Network::fini();
	fprintf(stderr,"mdlTerminate >> Leaving mdlTerminate...\n");
}

// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif


