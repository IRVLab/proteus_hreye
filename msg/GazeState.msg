# This message represents the gaze state of a "eye". Not the values of the the LED arrays, but the
# subjective state of the eye.

Header header #for managing timeseries data
int16 gaze_direction # The gaze direction in polar coordinates. -1 indicates no gaze direction, 0-360 in polar coordinates.
bool blink_state # Should the eyes blink?
bool widen_state # Should the eyes widen?
string color # A color to change the eyes to. Note that this is only respected for 1 second approximately.