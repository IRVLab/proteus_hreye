#This message represents the illumination state of HREye ringlight, comprised
#of 2 Neopixel rings (one 24, one 16), oriented with data in line "up"

Header header #for managing timeseries data
int16 hreye_index #indicates which ringlight this spec is for
std_msgs/ColorRGBA[40] state #40 RGBA values, with RGB filled out for each