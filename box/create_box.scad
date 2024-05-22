// Box for the reflow controller
//
// Aktor/Ansteuerung Board Dimension: 90 x 60
// Bohrmuster: 82 x 52
// Rand: 10 links (hin zum uC), 20 rechts (hin zum Ofen und Netzspannung)

// Sensor/Digital Thermocouple Board Dimesion: 70 x 50
// Bohrmuster: 62 x 42
// Hinweis: Die Bohrung oben rechts ist um ca. 0.3 mm in y-Richtung verschoben.


// Notwendige Aenderungen:
// - 20 mm mehr Platz hin zum Stromanschluss
// - Loecher muessen deutlich hoeher sein
// - Wand muss 10 mm hoeher sein
// - Bohrungen fuer Deckel vorsehen
// - Befestigung fuer uC vorsehen
// - Platz fuer Temperaturmessverstaerker vorsehen
// - Anschluss fuer Temperatursensor vorsehen


g_fn = 90;

// Nominell 8 mm (gemessen), plus 0.2 mm Toleranz wg. 3D Drucker
d_stromkabel = 8.0 + 0.2;

g_bohrmuster_relay_board_x = [-41, 41];
g_bohrmuster_relay_board_y = [-26, 26];

module box() {
    translate([-60,-40,0])
        difference() {
            cube([120,80,25]);
            translate([0,3,3])
                cube([115,74,22]);
            translate([0,0,3])
                cube([40,80,22]);
        }
}

module halter() {
    for (x = g_bohrmuster_relay_board_x) {
        for (y = g_bohrmuster_relay_board_y) {
            translate([x,y,0]) {
                cylinder(d=10, h=12, $fn=g_fn);
            }
        }
    }
}


module halter_loecher() {
    for (x = g_bohrmuster_relay_board_x) {
        for (y = g_bohrmuster_relay_board_y) {
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
        translate([50,y,15])
            rotate([0,90,0]) {
                cylinder(d=d_stromkabel, h=20, $fn=g_fn);
            }
    }
}

difference() {
    union() {
        box();
        translate([-5,0,0])
            halter();
    }
    translate([-5,0,0])
        halter_loecher();
    kabelfuehrung();
}


