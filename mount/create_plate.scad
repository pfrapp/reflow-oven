// The plates are used to fixate the PCB.
// The screws are pressing against the plates.

include <parameters.scad>

module plate() {
    difference() {
        cube([15, 30, 4*height_pcb]);
        translate([0,0,3*height_pcb])
            cube([10, 30, height_pcb]);
    }
}

plate();
