# This messages holds a point in a parameter-defined coordinate system

uint8 coordinate_system # Coordinate system enum. Available values:
uint8 CARTESIAN_FRAME = 0 # x/y/z
uint8 GEO_FRAME = 1 # x = latitude, [deg], range: [-90, 90], positive is north of equator
                    # y = longitude, [deg], range: [-180, 180], positive is east of prime meridian
                    # z = altitude, [m], positive is above the WGS 84 ellipsoid
uint8 GEO_FRAME_LOCAL_HEIGHT = 2 # like GEO_FRAME, but z has a user defined reference

float64 x
float64 y
float64 z
