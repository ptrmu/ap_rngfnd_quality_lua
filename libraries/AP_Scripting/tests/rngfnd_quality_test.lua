


-- The UPDATE_PERIOD_MS is the time between when a distance is set and
-- it is read. There is a periodic task that copies the set distance to
-- a place it is read from. If UPDATE_PERIOD_MS is too short this periodic
-- task might not get a chance to run. A value of 25 seems to be too quick.
local UPDATE_PERIOD_MS = 50
local TIMEOUT_MS = 5000
local RANGEFINDER_TYPE_LUA = 36.0
local RANGEFINDER_ORIENTATION = 25 -- down


local function send(str)
    gcs:send_text(3, string.format("RQTL %s", str))
end
local function send_success()
    send("Complete: !!SUCCESS!!")
end
local function send_failure(err_str)
    send(string.format("Complete: !!FAILURE!!: %s", err_str))
end



---@type AP_RangeFinder_Backend_ud
local device_backend

local function dist_overflow_failfunc(dist_m_in, dist_cm_out, signal_quality_pct_in, signal_quality_pct_out)
    local max_dist_cm = rangefinder:max_distance_cm_orient(device_backend:orientation())
     if dist_m_in > max_dist_cm / 100.0 then
        if signal_quality_pct_in < 0 and signal_quality_pct_out ~= -1 then
            return true
        end
        if signal_quality_pct_in >= 0 and signal_quality_pct_out ~= 0 then
            return true
        end
        if math.abs(dist_cm_out - max_dist_cm) > 1.0e-8 then
            return true
        end
        return false
    end
    return
end

local function dist_underflow_failfunc(dist_m_in, dist_cm_out, signal_quality_pct_in, signal_quality_pct_out)
    local min_dist_cm = rangefinder:min_distance_cm_orient(device_backend:orientation())
    if dist_m_in < min_dist_cm / 100.0 then
        if signal_quality_pct_in < 0 and signal_quality_pct_out ~= -1 then
            return true
        end
        if signal_quality_pct_in >= 0 and signal_quality_pct_out ~= 0 then
            return true
        end
        if math.abs(dist_cm_out - min_dist_cm) > 1.0e-8 then
            return true
        end
        return false
    end
    return
end

local function dist_equal_failfunc(dist_m_in, dist_cm_out, signal_quality_pct_in, signal_quality_pct_out)
    if dist_cm_out ~= dist_m_in * 100.0 then
        return true
    end
    if signal_quality_pct_in < 0 and signal_quality_pct_out == -1 then
        return false
    end
    if signal_quality_pct_in > 100 and signal_quality_pct_out == -1 then
        return false
    end
    if math.floor(signal_quality_pct_in) == signal_quality_pct_out then
        return false
    end
    return true
end

local function simple_test_funcfact(dist_m_in, signal_quality_pct_in)
    return function(test_idx)
        if not device_backend:handle_script_msg_ex(dist_m_in, signal_quality_pct_in) then
            return
        end
        return function()
            local dist_cm_out, signal_quality_pct_out = rangefinder:distance_cm_orient(RANGEFINDER_ORIENTATION)

            send(string.format("Test %i dist in_m: %.2f out_cm: %.2f, signal_quality_pct in: %.1f out: %.1f",
                test_idx, dist_m_in, dist_cm_out, signal_quality_pct_in, signal_quality_pct_out))

            local failed

            failed = dist_overflow_failfunc(dist_m_in, dist_cm_out, signal_quality_pct_in, signal_quality_pct_out) 
            if failed ~= nil then
                if failed then
                    return string.format("Test %i failed dist_overflow", test_idx)
                end
                return
            end

            failed = dist_underflow_failfunc(dist_m_in, dist_cm_out, signal_quality_pct_in, signal_quality_pct_out) 
            if failed ~= nil then
                if failed then
                    return string.format("Test %i failed dist_underflow", test_idx)
                end
                return
            end

            failed = dist_equal_failfunc(dist_m_in, dist_cm_out, signal_quality_pct_in, signal_quality_pct_out) 
            if failed ~= nil then
                if failed then
                    return string.format("Test %i failed dist_equal", test_idx)
                end
                return
            end
        end
    end
end


local tests = {
    simple_test_funcfact(20.0, -1),
    simple_test_funcfact(21.0, 0),
    simple_test_funcfact(22.0, 50),
    simple_test_funcfact(23.0, 100),
    simple_test_funcfact(24.0, 101),
    simple_test_funcfact(25.0, -3),
    simple_test_funcfact(26.0, 10000),
    simple_test_funcfact(27.0, 1.5),
    simple_test_funcfact(28.0, -0.5),
    simple_test_funcfact(29.0, 99.5),
    simple_test_funcfact(-1.0, 50),
    simple_test_funcfact(-1.0, -1),
    simple_test_funcfact(500.0, 50),
    simple_test_funcfact(500.0, -1),
}

-- Record the start time so we can timeout if it takes to long to initialize.
local time_start_ms = millis():tofloat()

-- Forward declare the update functions.
local prepare_test
local begin_test
local eval_test

local eval_func
local test_idx = 0

prepare_test = function()
    -- Check for timeout while initializing
    if millis():tofloat() - time_start_ms > TIMEOUT_MS then
        send_failure("Timeout while trying to initialize")
        return
    end
    if Parameter('RNGFND1_TYPE'):get() ~= 36 then
        send_failure("LUA driver not enabled")
        return
    end
    if rangefinder:num_sensors() < 1 then
        send_failure("LUA driver not connected")
        return
    end
    device_backend = rangefinder:get_backend(0)
    if not device_backend then
        send_failure("Range Finder 1 does not exist")
        return
    end
    if (device_backend:type() ~= RANGEFINDER_TYPE_LUA) then
        send_failure("Range Finder 1 is not a LUA driver")
        return
    end


    -- Wait until the prearm check passes. This ensures that the system is mostly initialized
    -- before starting the tests.
    if not arming:pre_arm_checks() then
        return prepare_test, UPDATE_PERIOD_MS
    end

    -- start the test
    return begin_test()
end

begin_test = function()
    test_idx = test_idx + 1
    if test_idx > #tests then
        send_success()
        return
    end

    eval_func = tests[test_idx](test_idx)
    if not eval_func then
        send_failure(string.format("Test %i failed to initialize", test_idx))
    end

    return eval_test, UPDATE_PERIOD_MS
end

eval_test = function()
    local err_str = eval_func()
    if not err_str then
        return begin_test()
    end
    send_failure(string.format("Test %i failed with error: %s", test_idx, err_str))
    return
end

send("Loaded rngfnd_quality_test.lua")

return prepare_test, UPDATE_PERIOD_MS