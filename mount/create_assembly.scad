// Create the overall assembly to check before printing.

height_mount = 12.0;
height_pcb = 1.6;
width_mount = 120.0;

// Thickness of the wall where the PCB is pushed against
wall_thickness_mount = 11.0;
// Space for the stencil (at the top and left)
stencil_space = 7.0;


translate([stencil_space+wall_thickness_mount,
           width_mount-79-stencil_space-wall_thickness_mount,
           height_mount-height_pcb])
    color([0.0, 0.5, 0.0])
        import("pcb.stl");

translate([stencil_space,
           width_mount-100-stencil_space,
           height_mount])
    color([0.7, 0.7, 0.7])
        import("stencil.stl");

color([0.0, 0.7, 0.8])
    import("pcb_mount.stl");

translate([0, 50-2, height_mount+0.15])
    color([0.5, 0.5, 0.0])
        import("stencil_mount.stl");

translate([24, 120, height_mount+0.15])
    rotate([0,0,-90])
        color([0.5, 0.5, 0.0])
            import("stencil_mount.stl");