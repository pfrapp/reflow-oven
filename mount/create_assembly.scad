// Create the overall assembly to check before printing.

include <parameters.scad>



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

translate([0, 50-6, height_mount+0.15])
    color([0.5, 0.5, 0.0])
        import("stencil_mount.stl");

translate([28, 120, height_mount+0.15])
    rotate([0,0,-90])
        color([0.5, 0.5, 0.0])
            import("stencil_mount.stl");

translate([77, 60, height_mount-4*height_pcb])
    color([0.5, 0.0, 0.5])
        import("plate.stl");

translate([25, 28, height_mount-4*height_pcb])
    rotate([0,0,-90])
        color([0.5, 0.0, 0.5])
            import("plate.stl");
