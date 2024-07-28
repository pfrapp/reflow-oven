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

include <../parameters.scad>

height = 12.0;
height_pcb = 1.6;
// Height of the center (axis) of the holes
// for the horizontal screws that fixate
// the PCB
height_holes = height - 0.5*height_pcb;

module pcb() {
    cube([64, 79, height_pcb]);
}

module pcb_mount() {
    difference() {
        cube([100, 120, height]);
        translate([10,0,height-4*height_pcb])
            cube([90,110,4*height_pcb]);
        translate([0,120,height-4*height_pcb])
            rotate([0,0,-45])
                translate([-50,-5,0])
                    cube([100,10,4*height_pcb]);

    }
    translate([14, 120-50-14, 0])
        cube([50,50,height-height_pcb]);
}

pcb_mount();
translate([10,120-79-10,height-height_pcb])
    color([0.0, 0.5, 0.0])
        pcb();

