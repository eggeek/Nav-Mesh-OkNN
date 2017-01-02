# Utilities for polyanya

Useful utilities for parsing gridmaps, polymaps and meshes.

The spec for polymaps and meshes are contained within `spec`, and some example
maps have been included in `maps`.

# Tools

`gridmap2poly`: Converts grid maps into the polymap format.
Suitable for loading into Fade2D.
Takes a grid map from stdin.

`poly2mesh`: Converts polymaps into the mesh format. Suitable for search
algorithms.
Takes a polymap from stdin.  
Note that `poly2mesh` outputs the Fade2D license when the program runs.

`visualiser`: Writes a PostScript file representing an input polymap.
Takes a polymap file as the first argument.

Included is a basic `gridmap2mesh` script which converts a gridmap to a mesh,
and also strips the Fade2D license from `poly2mesh`.

# Compiling

This has only been tested on Arch Linux, but it should work for any new Linux
distribution.
Ensure you have [GMP](https://gmplib.org/) installed, and run `make all`.
All the utilities will be compiled.
