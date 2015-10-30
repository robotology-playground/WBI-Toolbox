#include "SetPIDBlock.h"

#include <yarp/os/ResourceFinder.h>
#include <codyco/PIDList.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <sstream>

namespace wbitoolbox {
    using namespace codyco;

    const std::string TorquePIDInitialKey = "__ORIGINAL_PIDs__";
    const std::string TorquePIDDefaultKey = "__DEFAULT_PIDs__";

    static bool loadTorqueGainsFromFile(std::string filename,
                                        const PIDList &originalList,
                                        yarpWbi::yarpWholeBodyInterface& interface,
                                        PIDList &loadedPIDs);
    static bool fillPIDWithBottleDescription(const yarp::os::Bottle &bottle, yarp::dev::Pid &pid);


    bool loadGainsFromValue(const yarp::os::Value &gains,
                               PidMap &pidMap,
                               yarpWbi::yarpWholeBodyInterface& interface)
    {
        pidMap.clear();

        //Load original gains from controlboards and save them the original key.
        PIDList originalGains(interface.getDoFs());
        yarpWbi::yarpWholeBodyActuators *actuators = interface.wholeBodyActuator();
        actuators->getPIDGains(originalGains.pidList(), wbi::CTRL_MODE_TORQUE);
        pidMap.insert(PidMap::value_type(TorquePIDInitialKey, originalGains));

        //Now load additional gains
        bool result = true;
        if (gains.isString()) {
            PIDList pids(interface.getDoFs());
            result = loadTorqueGainsFromFile(gains.asString(), originalGains, interface, pids);
            pidMap.insert(PidMap::value_type(TorquePIDDefaultKey, pids));
        } else if (gains.isList()) {
            using namespace yarp::os;
            Bottle *list = gains.asList();

            //list of files. gains will be saved as integer-values
            for (int i = 0; i < list->size(); ++i) {
                if (!list->get(i).isString()) continue;
                std::string filename = list->get(i).asString();
                PIDList pids(interface.getDoFs());
                result = loadTorqueGainsFromFile(filename, originalGains, interface, pids);

                if (result) {
                    std::ostringstream num;
                    num << (i + 1);
                    pidMap.insert(PidMap::value_type(num.str(), pids));
                }
            }
        }
        return result;
    }


    bool setCurrentGains(const PidMap &pidMap, int map, yarpWbi::yarpWholeBodyInterface& interface)
    {
        std::stringstream num;
        num << map;
        return setCurrentGains(pidMap, num.str(), interface);
    }

    bool setCurrentGains(const PidMap &pidMap, std::string key, yarpWbi::yarpWholeBodyInterface& interface)
    {
        PidMap::const_iterator found = pidMap.find(key);
        //Treat one exception: pidMap with size == 2, the default can be set to either the string or the num 1
        if (found == pidMap.end() && key == TorquePIDDefaultKey && pidMap.size() == 2) {
            found = pidMap.find("1");
        }

        if (found == pidMap.end()) return false;
        yarpWbi::yarpWholeBodyActuators *actuators = interface.wholeBodyActuator();
        return actuators->setPIDGains(found->second.pidList(), wbi::CTRL_MODE_TORQUE); //to be rendered generic (torque, position, etc)
    }

    static bool loadTorqueGainsFromFile(std::string filename,
                                        const PIDList &originalList,
                                        yarpWbi::yarpWholeBodyInterface& interface,
                                        PIDList &loadedPIDs)
    {
        //List of list. Each element has a key: joint name, and a list of pairs: kp, kd, ki and its respective value
        using namespace yarp::os;
        yarp::os::ResourceFinder resourceFinder = yarp::os::ResourceFinder::getResourceFinderSingleton();
        Property file;
        file.fromConfigFile(resourceFinder.findFile(filename));

        Bottle externalList;
        externalList.fromString(file.toString());

        bool result = true;
        wbi::IDList jointList = interface.getJointList();
        for (int i = 0; i < externalList.size(); ++i) {
            if (!externalList.get(i).isList()) continue;
            Bottle *jointConfig = externalList.get(i).asList();
            if (jointConfig->size() < 2 || !jointConfig->get(0).isString()) continue;
            wbi::ID jointID(jointConfig->get(0).asString());
            int jointIndex = -1;
            if (!jointList.idToIndex(jointID, jointIndex)) continue;
            if (jointIndex < 0 || jointIndex >= jointList.size()) {
                yWarning("Specified joint %s index is outside joint list size", jointID.toString().c_str());
                continue;
            }

            loadedPIDs.pidList()[jointIndex] = originalList.pidList()[jointIndex];
            result = result && fillPIDWithBottleDescription(*jointConfig, loadedPIDs.pidList()[jointIndex]);
        }
        return result;
    }

    static bool fillPIDWithBottleDescription(const yarp::os::Bottle &bottle, yarp::dev::Pid &pid)
    {
        for (int i = 1; i < bottle.size(); ++i) {
            if (!bottle.get(i).isList()) continue;
            yarp::os::Bottle *gains = bottle.get(i).asList();
            if (gains->size() == 2 && gains->get(0).isString() && gains->get(1).isDouble()) {
                std::string gainType = gains->get(0).asString();
                double value = gains->get(1).asDouble();

                if (gainType == "kp") {
                    pid.setKp(value);
                } else if (gainType == "kd") {
                    pid.setKd(value);
                } else if (gainType == "ki") {
                    pid.setKi(value);
                }
            }
        }
        return true;
    }

}