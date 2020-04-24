# This message holds an orientation in a parameter-defined coordinate system

uint8 format # Format enum. Available values:
uint8 UNIT_VECTOR = 0
    # orientation defined by [x,y,z], one axis rotation invariant,
    # ||[x,y,z]|| = 1, frame is user defined
uint8 QUATERNION = 1
    # [x,y,z,w] form a unit quaternion, frame is user defined
uint8 LOCAL_NED_UNIT_VECTOR = 2
    # like UNIT_VECTOR, but x points north, y east, z down
uint8 LOCAL_NED_QUATERNION = 3
    # like QUATERNION, reference is defined as for LOCAL_NED_UNIT_VECTOR

float64 x
float64 y
float64 z
float64 w
