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
// - Befestigung fuer uC vorsehen (/)
// - Platz fuer Temperaturmessverstaerker vorsehen (/)
// - Anschluss fuer Temperatursensor vorsehen (/)


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
            union() {
                cube([140,80,32]);
                translate([136,80,0])
                    cube([4,58,32]);
            }
            translate([0,2,1.5])
                cube([136,76,32.5]);
            translate([0,80,2])
                cube([136,115,32]);
            translate([0,0,2])
                cube([60,160,32]);
            translate([0,150,2])
                cube([140,45,35]);
            translate([42,100,0])
                cube([35,80,2]);
        }
}

module halter(pattern_x, pattern_y) {
    for (x = pattern_x) {
        for (y = pattern_y) {
            translate([x,y,0]) {
                cylinder(d=10, h=10, $fn=g_fn);
            }
        }
    }
}

module halter_hutschiene(pattern_x, pattern_y) {
    for (x = pattern_x) {
        for (y = pattern_y) {
            translate([x,y,0]) {
                cylinder(d=12, h=16, $fn=g_fn);
            }
        }
    }
}


module halter_loecher(pattern_x, pattern_y) {
    for (x = pattern_x) {
        for (y = pattern_y) {
            translate([x,y,0]) {
                translate([0,0,4.2])
                    cylinder(d=4.0, h=5.8, $fn=g_fn);
                cylinder(d=3.4, h=10, $fn=g_fn);
            }
        }
    }
}

module halter_loecher_hutschiene(pattern_x, pattern_y) {
    for (x = pattern_x) {
        for (y = pattern_y) {
            translate([x,y,0]) {
                translate([0,0,7.8])
                    cylinder(d=5.6, h=8.2, $fn=g_fn);
                cylinder(d=4.6, h=16, $fn=g_fn);
            }
        }
    }
}

module kabelfuehrung() {
    for (y = [-3,15]) {
        translate([70,y,23])
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

// Bohrungen fuer die Laborstecker.
// Durchmesser: 8 mm (daher 8.2 mm wg. Toleranz).
module bohrungen_laborstecker() {
    g_laenge = 90.0;
    g_wanddicke = 5;
    g_hoehe = 30;
    
    
    d_bohrung_laborstecker = 8.2;
    translate([0,g_wanddicke,g_hoehe/2])
        rotate([90,0,0])
            linear_extrude(g_wanddicke, center=false) {
                translate([g_laenge/4,0,0])
                    circle(d=d_bohrung_laborstecker, $fn=g_fn);
                translate([g_laenge/2,0,0])
                    circle(d=d_bohrung_laborstecker, $fn=g_fn);
            }
    // Beschriftung
    font_size = 4.5;
    text_depth = 2;
    translate([g_laenge/4+0*d_bohrung_laborstecker,0,3*g_hoehe/4])
        rotate([90,0,0])
            translate([0,0,-text_depth])
                linear_extrude(text_depth, center=false)
                    text("T(-)",
                        font = "Arial Black",
                        size = font_size,
                        valign = "bottom",
                        halign = "center");
    translate([g_laenge/2+0*d_bohrung_laborstecker,0,3*g_hoehe/4])
        rotate([90,0,0])
            translate([0,0,-text_depth])
                linear_extrude(text_depth, center=false)
                    text("T(+)",
                        font = "Arial Black",
                        size = font_size,
                        valign = "bottom",
                        halign = "center");
}

module verbindung_halter() {
    // Grundform
    grundform_points = [[-52, -38], [-52, 40], [-38, 60], [-38, 150],
                        [55, 150], [38, 110], [38, 60], [42, 40], [42, -38],
                        [30, -38], [30, 40], [25, 60], [25, 110], [35, 140],
                        [-25, 140], [-25, 55], [-40, 35], [-40, -38]
                       ];
    linear_extrude(3.0)
        minkowski() {
            polygon(grundform_points);
            circle(r=2, $fn = g_fn);
        }
}

module entity() {
    difference() {
        union() {
            box();
            translate([-5,0,0])
                halter(g_bohrmuster_relay_board_x, g_bohrmuster_relay_board_y);
            translate([0,80,0])
                halter(g_bohrmuster_thermo_board_x, g_bohrmuster_thermo_board_y);
            // Hutschiene
            translate([-30,145,0])
                halter_hutschiene([0, 75], [0]);
            translate([80,40,2])
                rotate([0,0,180])
                    quarter_cylinder(r=12, h=30);
            translate([80,-40,2])
                rotate([0,0,90])
                    quarter_cylinder(r=12, h=30);
            verbindung_halter();
        }
        translate([-5,0,0])
            halter_loecher(g_bohrmuster_relay_board_x, g_bohrmuster_relay_board_y);
        translate([0,80,0])
            halter_loecher(g_bohrmuster_thermo_board_x, g_bohrmuster_thermo_board_y);
        // Loecher zur Befestigung des Deckels
        translate([80,0,23])
            halter_loecher([-5], [-35, 35]);
        // Loecher zur Befestigung der Hutschiene
        translate([-30,145,0])
            halter_loecher_hutschiene([0, 75], [0]);
        kabelfuehrung();
        translate([80,40,0])
            rotate([0,0,90])
                bohrungen_laborstecker();
    }
}


entity();






