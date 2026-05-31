---@meta

---@class animation
animation = {}

---Clears local joint override.
Input: clearJointOverride(entityId, jointId)
Returns: boolean.
---@overload fun(entityId: string, jointId: string): boolean
---@param ... any
---@return any
function animation.clearJointOverride(...) end

---Returns active animation code. Input: getActiveAnimation(entityId). Returns: string or nil.
---@overload fun(entityId: string): string
---@param ... any
---@return any
function animation.getActiveAnimation(...) end

---Sets entity active animation.
Input: setAnimation(entityId, animationCode) or setAnimation(entityId, animationCode, loop, speedScale)
Returns: boolean.
---@overload fun(entityId: string, animationCode: string, loop?: boolean, speedScale?: number): boolean
---@param ... any
---@return any
function animation.setAnimation(...) end

---Sets local joint rotation override in radians.
Input: setJointRotation(entityId, jointId, xRad, yRad, zRad)
Returns: boolean.
---@overload fun(entityId: string, jointId: string, xRad: number, yRad: number, zRad: number): boolean
---@param ... any
---@return any
function animation.setJointRotation(...) end

---Stops active animation for entity. Input: stop(entityId). Returns: boolean.
---@overload fun(entityId: string): boolean
---@param ... any
---@return any
function animation.stop(...) end

