---@meta

---@class mission
mission = {}

---Returns the current target entity identifier from mission state.
Input: getCurrentTarget(entityId)
- entityId: string
Returns: string (target entity ID, or empty string).
---@overload fun(entityId: string): string
---@param ... any
---@return any
function mission.getCurrentTarget(...) end

---Returns objective identifiers from mission state as a JSON string.
Input: getObjectiveIds(entityId)
- entityId: string
Returns: string (JSON array of objective IDs, or '[]').
---@overload fun(entityId: string): string
---@param ... any
---@return any
function mission.getObjectiveIds(...) end

---Returns mission state as a serialized JSON payload.
Input: getState(entityId)
- entityId: string
Returns: string (JSON mission state, or '{}' if unavailable).
---@overload fun(entityId?: string): string
---@param ... any
---@return any
function mission.getState(...) end

---Returns the task list from mission state as a JSON string.
Input: getTaskList(entityId)
- entityId: string
Returns: string (JSON array of tasks, or '[]').
---@overload fun(entityId: string): string
---@param ... any
---@return any
function mission.getTaskList(...) end

---Checks whether the given entity has a valid mission component.
Input: isValid(entityId)
- entityId: string
Returns: boolean.
---@overload fun(entityId?: string): boolean
---@param ... any
---@return any
function mission.isValid(...) end

---Writes an informational log message from the mission script to the application logger.
Input: log(message)
- message: string (include entityId in the string from Lua if needed)
Returns: boolean (false if arguments are invalid).
---@overload fun(message: string): boolean
---@param ... any
---@return any
function mission.log(...) end

---Marks the mission status as 'completed'.
Input: markCompleted(entityId)
- entityId: string
Returns: boolean.
---@overload fun(entityId: string): boolean
---@param ... any
---@return any
function mission.markCompleted(...) end

---Marks the mission status as 'failed'.
Input: markFailed(entityId)
- entityId: string
Returns: boolean.
---@overload fun(entityId: string): boolean
---@param ... any
---@return any
function mission.markFailed(...) end

---Marks the mission status as 'running'.
Input: markRunning(entityId)
- entityId: string
Returns: boolean.
---@overload fun(entityId: string): boolean
---@param ... any
---@return any
function mission.markRunning(...) end

---Publishes a named mission event to the message bus.
Input: requestEvent(entityId, eventName)
- entityId: string
- eventName: string
Returns: boolean.
---@overload fun(entityId: string, eventName: string): boolean
---@param ... any
---@return any
function mission.requestEvent(...) end

---Sets the current target entity for the mission.
Input: setCurrentTarget(entityId, targetEntityId)
- entityId: string
- targetEntityId: string
Returns: boolean.
---@overload fun(entityId: string, targetEntityId: string): boolean
---@param ... any
---@return any
function mission.setCurrentTarget(...) end

---Records the last time the current target was observed.
Input: setLastSeenTargetTime(entityId, simulationTime)
- entityId: string
- simulationTime: number (seconds)
Returns: boolean.
---@overload fun(entityId: string, simulationTime: number): boolean
---@param ... any
---@return any
function mission.setLastSeenTargetTime(...) end

