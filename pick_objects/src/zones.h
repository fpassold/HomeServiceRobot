/*
* Definitions of constants associated with pickup and drop off zones
*
* The idea would be to use this same file with "pick_objects.cpp" to avoid having to edit 
* the 2 codes if the user changes the pickup and drop off zones.
* It would be even better to create a plain text file (fixed name) that the 2 programs: 
* "add_markers.cpp" and "pick_objects.cpp" reads to update at runtime (not at compile time) 
* the pickup and drop off zones.
* But I don't master C++ to this point.
* Fernando Passold, 03.06.2021
*/
const double pickup_zone_x = 2.8;
const double pickup_zone_y = -3.2;
const double drop_off_zone_x = -5.2;
const double drop_off_zone_y =  4.0;

