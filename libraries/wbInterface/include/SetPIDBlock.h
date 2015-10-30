#ifndef WBITOOLBOX_SETPIDBLOCK_H
#define WBITOOLBOX_SETPIDBLOCK_H

namespace yarp {
    namespace os {
        class Value;
    }
    namespace dev {
        class Pid;
    }
}

namespace yarpWbi {
    class yarpWholeBodyInterface;
}

namespace codyco {
    class PIDList;
}

#include <map>

namespace wbitoolbox {
    typedef std::map<std::string, codyco::PIDList> PidMap;

    extern const std::string TorquePIDInitialKey;
    extern const std::string TorquePIDDefaultKey;

    //Utility functions for the SetPID block
    bool loadGainsFromValue(const yarp::os::Value &gains, PidMap &pidMap, yarpWbi::yarpWholeBodyInterface& interface);
    bool setCurrentGains(const PidMap &pidMap, std::string key, yarpWbi::yarpWholeBodyInterface& interface);
    bool setCurrentGains(const PidMap &pidMap, int map, yarpWbi::yarpWholeBodyInterface& interface);
}

#endif /* end of include guard: WBITOOLBOX_SETPIDBLOCK_H */
