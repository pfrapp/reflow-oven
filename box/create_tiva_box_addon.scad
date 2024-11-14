// Box for the Tiva controller.
// Should at some point be unified with the other box.
//
// Tiva Board Dimension: 100 x 100
// Bohrmuster: 92 x 92


g_fn = 90;


g_bohrmuster_tiva_board_x = [-46, 46];
g_bohrmuster_tiva_board_y = [-46, 46];


module halter(pattern_x, pattern_y) {
    for (x = pattern_x) {
        for (y = pattern_y) {
            translate([x,y,0]) {
                cylinder(d=10, h=10, $fn=g_fn);
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


module verbindung_halter() {
    // Grundform
    grundform_points = [[-52, -52],
                        [-52, 52],
                        [52, 52],
                        [52, -52],
                        [0, -52],
                        [0, 42],
                        [-42, 42],
                        [-42, -42],
                        [42, -42],
                        [42, 42],
                        [0, 42],
                        [0, -52]
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
            translate([0,0,0])
                halter(g_bohrmuster_tiva_board_x, g_bohrmuster_tiva_board_y);
            verbindung_halter();
        }
        translate([0,0,0])
            halter_loecher(g_bohrmuster_tiva_board_x, g_bohrmuster_tiva_board_y);
    }
}


entity();







