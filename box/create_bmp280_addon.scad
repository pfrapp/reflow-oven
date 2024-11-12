// Mount for the BMP280 temperature sensor (evaluation board).
// Should at some point be unified with the other boxes.
//
// BMP280 evaluation board
// - outer distances of holes: 13.30 mm
// - inner distances of holes: 7.20 mm
//
// --> Double hole diameter = 13.30 - 7.20 = 3.05 mm
// 2x M2 mit Unterlegscheibe (statt M3) wg. Positionstoleranz
//
// Distance between hole centeres: 0.5 * (13.30 + 7.20) = 10.25 mm
// (likely nominally 4 * 2.54 mm = 10.16 mm)
//
// Breite Auflageflaeche: 15 mm (mit Bohrungen mittig)
// Hoehe Auflageflaeche: 25 mm (Bohrungen jeweils am Rand)
//
// Hoehe des Bauteils: 20 mm


g_fn = 90;

g_spacing = 10.16;
g_holes_y = [-g_spacing / 2.0, g_spacing / 2.0];

// M2
g_M2_thread_diameter = 3.2;
g_M2_thread_depth    = 4.1;
g_M2_through_hole_diameter = 2.4;

module entity() {
    difference() {
        union() {   
            // Cube
            translate([-15.0/2+1,-25.0/2+1,10]) {
                minkowski() {
                sphere(r = 1, $fn = g_fn);
                linear_extrude(20-2, center=true)
                    square([15-2, 25-2]);
                }
            }
            linear_extrude(3, center=false)
                scale([1.0, 2.0, 1.0])
                    circle(r = 12, $fn = g_fn);
        }
        union() {
            // Through holes
            for (y = g_holes_y) {
                translate([0.0, y, 0.0])
                    cylinder(h = 20, d = g_M2_through_hole_diameter, $fn = g_fn);
            }
            // Holes for thread mount
            for (y = g_holes_y) {
                translate([0.0, y, 20.0 - g_M2_thread_depth])
                    cylinder(h = g_M2_thread_depth, d = g_M2_thread_diameter, $fn = g_fn);
            }
        }
    }
}



entity();


