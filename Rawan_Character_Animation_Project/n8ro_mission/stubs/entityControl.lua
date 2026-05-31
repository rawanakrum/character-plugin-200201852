---@meta

---@class entityControl
entityControl = {}

---Queues deferred creation of an entity by profile name.
Input: requestCreateFromProfile(entityProfileName)
- entityProfileName: string
Returns: string (runtime entity id), or empty string on failure.
---@overload fun(entityProfileName: string): string
---@param ... any
---@return any
function entityControl.requestCreateFromProfile(...) end

---Queues deferred deletion of an entity by scenario id.
Input: requestDelete(entityId)
- entityId: string
Returns: boolean.
---@overload fun(entityId: string): boolean
---@param ... any
---@return any
function entityControl.requestDelete(...) end

