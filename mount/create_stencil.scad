// Create the stencil (only for the sake of checking the assembly)

include <parameters.scad>

module stencil() {
    cube([80, 100, height_stencil]);
}

stencil();
