-- Rawan four-motion demo mission helper
-- Keep the scenario Animation tab defaultAnimation = "Rawan Idle".
-- The DLL now forces the visible cycle internally, so this script only requests
-- Rawan Idle once and does not fight the plugin by switching animation codes.

local function safePrint(message)
    if print ~= nil then
        print(message)
    end
end

local function requestIdle(entityId)
    if animation == nil or animation.setAnimation == nil then
        safePrint("[RawanMission] animation.setAnimation unavailable; relying on scenario defaultAnimation")
        return false
    end

    local ok, result = pcall(animation.setAnimation, entityId, "Rawan Idle")
    if not ok then
        safePrint("[RawanMission] setAnimation Rawan Idle crashed: " .. tostring(result))
        return false
    end

    safePrint("[RawanMission] requested Rawan Idle -> " .. tostring(result))
    return result
end

function onInit(entityId)
    safePrint("[RawanMission] onInit entity=" .. tostring(entityId))
    requestIdle(entityId)
end

function onTick(entityId, simulationTimeSeconds, deltaTimeSeconds)
    -- No external switching here. The DLL handles:
    -- Rawan Idle -> Rawan Walk -> Rawan Push -> Rawan Climb.
end

function onShutdown(entityId)
    safePrint("[RawanMission] onShutdown entity=" .. tostring(entityId))
end
