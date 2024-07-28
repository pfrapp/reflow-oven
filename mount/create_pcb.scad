// Create the PCB (only for the sake of checking the assembly)

include <parameters.scad>

module pcb() {
    cube([64, 79, height_pcb]);
}

pcb();
