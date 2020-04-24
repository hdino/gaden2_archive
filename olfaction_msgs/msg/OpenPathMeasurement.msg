# This message holds measurement data from an open path gas sensor (e.g. TDLAS)

# Integral concentration, a negative value signals no data
float64 concentration # [ppm * m]

# Range of the measurement, a negative value signals no data
float64 range # [m]

# Position at which the measurement was taken
FramePoint position

# Orientation of the sensor when the measurement was taken
FrameOrientation orientation
