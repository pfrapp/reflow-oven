// Stencil mount

// This part is used to fixate the stencil on the PCB mount.
// To be printed twice.

include <parameters.scad>

function get_fn(detailed) = detailed ? 90 : 24;
g_detailed = true;
g_fn = get_fn(g_detailed);

// wall_thickness_mount + stencil_space
stencil_width = wall_thickness_mount + stencil_space;

stencil_height = 3.0;
stencil_length = 50.0;


// L: Length of the Langloch
// h: its height
module langloch(L, h) {
    r = 0.5*g_M3_through_hole_diameter_large;
    linear_extrude(h, center=false) {
        circle(d = g_M3_through_hole_diameter_large, $fn=g_fn);
        translate([L,0,0])
            circle(d = g_M3_through_hole_diameter_large, $fn=g_fn);
        polygon([[0,-r], [L, -r], [L, r], [0, r]]);
    }
}


module stencil_mount() {
    difference() {
        cube([stencil_width, stencil_length, stencil_height]);
        translate([4,4,0])
            langloch(18-2*4, stencil_height);
        translate([4, 4+stencil_mount_hole_offset, 0])
            langloch(18-2*4, stencil_height);
    }
    
}

stencil_mount();
