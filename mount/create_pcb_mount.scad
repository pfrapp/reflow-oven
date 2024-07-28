// PCB mount

// Dimensions of the wheel speed sensor SMD-PCB
// according to the drawing: 64 mm x 79 mm
// (I double checked this via a measurement)
// Measured thickness: 1.65 mm (1.6 mm according to ordering)
//
// Stencil dimensions: 80 mm x 100 mm
// (measured, and likely given as custom size
// during ordering -- I forgot)
// Measured thickness: 0.15 mm

// space left: 16 / 2 = 8
// space top: 21 / 2 = 10.5

include <parameters.scad>

function get_fn(detailed) = detailed ? 90 : 24;
g_detailed = false;
g_fn = get_fn(g_detailed);


height_mount = 12.0;
height_pcb = 1.6;
// Height of the center (axis) of the holes
// for the horizontal screws that fixate
// the PCB
height_holes = height_mount - 2*height_pcb;

module pcb() {
    cube([64, 79, height_pcb]);
}

module pcb_mount() {
    difference() {
        cube([100, 120, height_mount]);
        translate([10,0,height_mount-4*height_pcb])
            cube([90,110,4*height_pcb]);
        translate([0,120,height_mount-4*height_pcb])
            rotate([0,0,-45])
                translate([-50,-5,0])
                    cube([100,10,4*height_pcb]);

    }
    translate([14, 120-50-14, 0])
        cube([50,50,height_mount-height_pcb]);
}

module screw_mount() {
    screw_mount_thickness = 10.0;
    screw_mount_height = 12.0;
    difference() {
        translate([0,0,0.5*screw_mount_height])
            cube([10,screw_mount_thickness,screw_mount_height], center=true);
        // Through hole
        translate([0,0,height_holes])
            rotate([270,0,0])
                translate([0,0,-0.5*screw_mount_thickness])
                    cylinder(h = screw_mount_thickness, d = g_M3_through_hole_diameter_large, $fn = g_fn);
        // Hole for mounting the threads
        translate([0,0,height_holes])
            rotate([270,0,0])
                translate([0,0,-0.5*screw_mount_thickness])
                    cylinder(h = g_M3_thread_depth, d = g_M3_thread_diameter, $fn = g_fn);
    }
}

translate([40,5,0])
    screw_mount();
translate([95,70,0])
    rotate([0,0,90])
        screw_mount();

pcb_mount();
translate([10,120-79-10,height_mount-height_pcb])
    color([0.0, 0.5, 0.0])
        pcb();

