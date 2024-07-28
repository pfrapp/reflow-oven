// Create the PCB (only for the sake of checking the assembly)

height_pcb = 1.6;

module pcb() {
    cube([64, 79, height_pcb]);
}

pcb();
