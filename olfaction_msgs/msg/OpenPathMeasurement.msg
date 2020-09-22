# This message holds measurement data from an open path gas sensor (e.g. TDLAS)

# Status of the measurement (valid/invalid)
int8 status
int8 STATUS_VALID = 0
int8 STATUS_INVALID = -1 # neither concentration nor range valid
int8 STATUS_CONCENTRATION_INVALID = -2
int8 STATUS_RANGE_INVALID = -3
# If a field is invalid, its value has no meaning;
# position and orientation, however, must always be set.
# It is recommended to set concentration and/or range to -1
# if they are invalid. This is helpful e.g. for plotting.

# Integral concentration, a negative value signals no data
float64 concentration # [ppm * m]

# Range of the measurement, a negative value signals no data
float64 range # [m]

# Position at which the measurement was taken
FramePoint position

# Orientation of the sensor when the measurement was taken
FrameOrientation orientation
