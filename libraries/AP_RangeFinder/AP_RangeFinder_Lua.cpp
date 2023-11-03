/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LUA_ENABLED

#include "AP_RangeFinder_Lua.h"
#include <AP_HAL/AP_HAL.h>

// constructor
AP_RangeFinder_Lua::AP_RangeFinder_Lua(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params), _distance_m(0.0f), _signal_quality_pct(-1)
{
    set_status(RangeFinder::Status::NoData);
}


// Set the distance based on a Lua Script
bool AP_RangeFinder_Lua::handle_script_msg(float dist_m)
{
    return handle_script_msg_ex(dist_m, -1.0f);
}


// Set the distance and signal_quality based on a Lua Script.
// If the signal_quality_pct argument is an int8_t from lua, then
// things don't work. Not sure why but using a float works better.
bool AP_RangeFinder_Lua::handle_script_msg_ex(float dist_m, float signal_quality_pct)
{
    state.last_reading_ms = AP_HAL::millis();
    _distance_m = dist_m;

    // Trying to set a bogus signal_quality is like having no signal_quality data.
    _signal_quality_pct = (signal_quality_pct >= 0.0f && signal_quality_pct <= 100.0f) ?
            static_cast<int8_t>(signal_quality_pct) : -1;
    return true;
}

bool AP_RangeFinder_Lua::get_signal_quality_pct(int8_t &quality_pct) const
{
    // Status and distance are updated via a periodic update routine. signal_quality
    // is returned via this overridden virtual method. It seems that there is
    // a good chance that the distance measurement and the signal_quaqlity value can
    // get out of sync. For example if a hardware driver collects both distance and
    // signal_quality and then a client queries the distance before the update routine
    // runs, then the client will get and old distance with a new signal_quality.
    // signal_quality should really be in the state structure along with distance.

    // Just return the signal_quality when asked.
    quality_pct = _signal_quality_pct;

    // A value of -1 (no signal_quality) causes a false return.
    return quality_pct >= 0;
}

// update the state of the sensor
void AP_RangeFinder_Lua::update(void)
{
    //Time out on incoming data; if we don't get new
    //data in 500ms, dump it
    if (AP_HAL::millis() - state.last_reading_ms > AP_RANGEFINDER_LUA_TIMEOUT_MS) {
        set_status(RangeFinder::Status::NoData);
        state.distance_m = 0.0f;
        _signal_quality_pct = -1; // no signal_quality also
    } else {

        // Move distance into the state structure. The status is updated based
        // on if the distance is outside of min and max. Note: signal_quality provides levels
        // of goodness for a Status::Good measurement. It is possible to have a Status::Good
        // measurement with a signal_quality of 0.
        state.distance_m = _distance_m;
        update_status();
    }
}

#endif  // AP_RANGEFINDER_LUA_ENABLED
