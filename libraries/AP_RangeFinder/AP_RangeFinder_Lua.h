#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LUA_ENABLED

#include "AP_RangeFinder_Backend.h"

// Data timeout
#define AP_RANGEFINDER_LUA_TIMEOUT_MS 500

class AP_RangeFinder_Lua : public AP_RangeFinder_Backend
{
public:

    // constructor
    AP_RangeFinder_Lua(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // update state
    void update(void) override;

    // Get update from Lua script
    bool handle_script_msg(float dist_m) override;
    bool handle_script_msg_ex(float dist_m, float signal_quality_pct) override;

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_UNKNOWN;
    }

    // Return the last signal_quality that the driver returned
    bool get_signal_quality_pct(int8_t &quality_pct) const override WARN_IF_UNUSED;

private:

    float _distance_m;          // stored data from lua script:
    int8_t _signal_quality_pct; // stored data from lua script:
};

#endif  // AP_RANGEFINDER_LUA_ENABLED
