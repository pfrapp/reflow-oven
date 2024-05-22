// Box for the reflow controller
//
// Aktor/Ansteuerung Board Dimension: 90 x 60
// Bohrmuster: 82 x 52
// Rand: 10 links (hin zum uC), 20 rechts (hin zum Ofen und Netzspannung)

// Sensor/Digital Thermocouple Board Dimesion: 70 x 50
// Bohrmuster: 62 x 42
// Hinweis: Die Bohrung oben rechts ist um ca. 0.3 mm in y-Richtung verschoben.


// Notwendige Aenderungen:
// - 20 mm mehr Platz hin zum Stromanschluss (/)
// - Loecher muessen deutlich hoeher sein (/)
// - Wand muss 10 mm hoeher sein (/)
// - Bohrungen fuer Deckel vorsehen (/)
// - Befestigung fuer uC vorsehen
// - Platz fuer Temperaturmessverstaerker vorsehen
// - Anschluss fuer Temperatursensor vorsehen


g_fn = 90;

// Nominell 8 mm (gemessen), plus 0.2 mm Toleranz wg. 3D Drucker
d_stromkabel = 8.0 + 0.2;

g_bohrmuster_relay_board_x = [-41, 41];
g_bohrmuster_relay_board_y = [-26, 26];

g_bohrmuster_thermo_board_x = [-31, 31];
g_bohrmuster_thermo_board_y = [-21, 21];

module box() {
    translate([-60,-40,0])
        difference() {
            cube([140,80,35]);
            translate([0,3,3])
                cube([135,74,32]);
            translate([0,0,3])
                cube([60,80,32]);
        }
}

module halter(pattern_x, pattern_y) {
    for (x = pattern_x) {
        for (y = pattern_y) {
            translate([x,y,0]) {
                cylinder(d=10, h=12, $fn=g_fn);
            }
        }
    }
}


module halter_loecher(pattern_x, pattern_y) {
    for (x = pattern_x) {
        for (y = pattern_y) {
            translate([x,y,0]) {
                translate([0,0,6.2])
                    cylinder(d=4.0, h=5.8, $fn=g_fn);
                cylinder(d=3.4, h=12, $fn=g_fn);
            }
        }
    }
}

module kabelfuehrung() {
    for (y = [-3,15]) {
        translate([70,y,25])
            rotate([0,90,0]) {
                cylinder(d=d_stromkabel, h=20, $fn=g_fn);
            }
    }
}

module quarter_cylinder(r, h) {
    difference() {
        cylinder(h=h, r=r, $fn=g_fn);
        translate([-r,0,0])
            cube([2*r, 2*r, 4*h], center=true);
        translate([0,-r,0])
            cube([2*r, 2*r, 4*h], center=true);
    }
}

module entity() {
    difference() {
        union() {
            box();
            translate([-5,0,0])
                halter(g_bohrmuster_relay_board_x, g_bohrmuster_relay_board_y);
            translate([-90,80,0])
                halter(g_bohrmuster_thermo_board_x, g_bohrmuster_thermo_board_y);
            translate([80,40,3])
                rotate([0,0,180])
                    quarter_cylinder(r=12, h=32);
            translate([80,-40,3])
                rotate([0,0,90])
                    quarter_cylinder(r=12, h=32);
        }
        translate([-5,0,0])
            halter_loecher(g_bohrmuster_relay_board_x, g_bohrmuster_relay_board_y);
        translate([-90,80,0])
            halter_loecher(g_bohrmuster_thermo_board_x, g_bohrmuster_thermo_board_y);
        translate([80,0,23])
            halter_loecher([-5], [-35, 35]);
        kabelfuehrung();
    }
}


//quarter_cylinder(r=10, h=20);

entity();


