-- Rawan four-motion demo mission helper
-- Version 3.6.5 climb-push-action-matrices
-- Keep Scenario Editor Animation tab defaultAnimation = "Rawan Idle".
-- In final demo mode, the DLL force-cycles visible states internally:
-- Rawan Idle -> Rawan Walk -> Rawan Push -> Rawan Climb.
-- In C++ tuning mode, change kRequestedAnimation below to isolate one state.

local function safePrint(message)
    if print ~= nil then
        print(message)
    end
end

-- For isolated tuning mode in C++, set one of:
-- "Rawan Idle", "Rawan Walk", "Rawan Push", or "Rawan Climb".
-- For final auto-cycle demo mode, leave this as "Rawan Idle".
local kRequestedAnimation = "Rawan Idle"

local function requestSelectedAnimation(entityId)
    if animation == nil or animation.setAnimation == nil then
        safePrint("[RawanMission] animation.setAnimation unavailable; relying on scenario defaultAnimation")
        return false
    end

    local ok, result = pcall(animation.setAnimation, entityId, kRequestedAnimation)
    if not ok then
        safePrint("[RawanMission] setAnimation " .. tostring(kRequestedAnimation) .. " crashed: " .. tostring(result))
        return false
    end

    safePrint("[RawanMission] requested " .. tostring(kRequestedAnimation) .. " -> " .. tostring(result))
    return result
end

function onInit(entityId)
    safePrint("[RawanMission] onInit entity=" .. tostring(entityId))
    requestSelectedAnimation(entityId)
end

function onStart(entityId)
    safePrint("[RawanMission] onStart entity=" .. tostring(entityId))
    requestSelectedAnimation(entityId)
end

function init(entityId)
    safePrint("[RawanMission] init entity=" .. tostring(entityId))
    requestSelectedAnimation(entityId)
end

function start(entityId)
    safePrint("[RawanMission] start entity=" .. tostring(entityId))
    requestSelectedAnimation(entityId)
end

function onTick(entityId, simulationTimeSeconds, deltaTimeSeconds)
    -- No external switching here. The DLL handles cycling or isolated tuning.
end

function tick(entityId, simulationTimeSeconds, deltaTimeSeconds)
    -- No external switching here. The DLL handles cycling or isolated tuning.
end

function update(entityId, simulationTimeSeconds, deltaTimeSeconds)
    -- No external switching here. The DLL handles cycling or isolated tuning.
end

function onUpdate(entityId, simulationTimeSeconds, deltaTimeSeconds)
    -- No external switching here. The DLL handles cycling or isolated tuning.
end

function onShutdown(entityId)
    safePrint("[RawanMission] onShutdown entity=" .. tostring(entityId))
end
