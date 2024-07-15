module lid() {
    // 52 mm vom linken Ende des Relais bis zur
    // Bohrung (nicht Kante) auf der rechten Seite
    // des Boards.
    height = 2;
    difference() {
        union() {
            translate([0,-40,0])
                cube([96, 80, height]);
            // Verstrebungen
            for (y = [-24,24]) {
                translate([0,y,height])
                    cube([96,3,1.5*height]);
            }
            for (x = [5,75]) {
                translate([x,-40,height])
                    cube([3,80,1.5*height]);
            }
        }
        // Nur zur Ausrichtung der Loecher der Platine
//        for (y = g_bohrmuster_relay_board_y) {
//            translate([52,y,0])
//                cylinder(d=3.4, h=height, $fn=g_fn);
//        }
        // Loecher zur Befestigung an der Box
        for (y = [-35, 35]) {
            translate([91,y,0])
                cylinder(d=3.4, h=height, $fn=g_fn);
        }
        // Aussparung Sicherung
        translate([26,0,0])
            cylinder(d=20, h=height, $fn=g_fn);
    }
}
