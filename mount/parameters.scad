// Screw threads.
// Note that the depth is increased by 0.1 mm as compared
// to the data sheet.
// The diameter corresponds to the d3 from the data sheet,
// and the depth correponds to the L.

// M2
g_M2_thread_diameter = 3.2;
g_M2_thread_depth    = 4.1;

// M3
g_M3_thread_diameter = 4.0;
g_M3_thread_depth    = 5.8;

// Through holes.
// See for instance https://de.wikipedia.org/wiki/Durchgangsbohrung
g_M2_through_hole_diameter = 2.4;
g_M3_through_hole_diameter = 3.4;
g_M3_through_hole_diameter_large = 3.6;

// Offset between holes in stencil mount
stencil_mount_hole_offset = 40.0;

// Outer dimension of the PCB mount part
height_mount = 10.0;
length_mount = 105.0;
width_mount = 120.0;

// Thickness of the wall where the PCB is pushed against.
// This is used for the margins of the stencil which
// is larger than the PCB.
wall_thickness_mount = 14.0;

// Space for the stencil (at the top and left).
// Actually this is space for the stencil mount.
stencil_space = 8.0;

// Height of the PCB
height_pcb = 1.6;

// Height of the stencil
height_stencil = 0.15;
