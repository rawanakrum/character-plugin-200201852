---@meta

---@class entity
entity = {}

---Returns the NED acceleration of the entity as ax/ay/az values.
Input: getAcceleration(entityId)
- entityId: string
Returns: number, number, number (ax, ay, az; nil triplet on failure).
---@overload fun(entityId: string): number, number, number
---@param ... any
---@return any
function entity.getAcceleration(...) end

---Returns the body-to-NED orientation of the entity as heading/pitch/roll angles.
Input: getOrientation(entityId)
- entityId: string
Returns: number, number, number (headingDeg, pitchDeg, rollDeg; nil triplet on failure).
---@overload fun(entityId: string): number, number, number
---@param ... any
---@return any
function entity.getOrientation(...) end

---Returns the geodetic position of the entity as lat/lon/alt values.
Input: getPosition(entityId)
- entityId: string
Returns: number, number, number (latitudeDeg, longitudeDeg, altitudeMeters; nil triplet on failure).
---@overload fun(entityId: string): number, number, number
---@param ... any
---@return any
function entity.getPosition(...) end

---Returns the NED velocity of the entity as vx/vy/vz values.
Input: getVelocity(entityId)
- entityId: string
Returns: number, number, number (vx, vy, vz; nil triplet on failure).
---@overload fun(entityId: string): number, number, number
---@param ... any
---@return any
function entity.getVelocity(...) end

