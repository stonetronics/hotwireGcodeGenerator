# hotwireGcodeGenerator
A program for generating XYUV gcode from arbitrary svg files

run with python3
dependencies: matplotlib, svgpathtools

This program lets you import 2 svg drawings that will be the path of the XY and UV plane of an 4-axis cnc hotwire cutter.
The svg files must contain a closed figure consisting of lines, arcs, etc that will be the toolpath.

The program uses "anchor points" to assign point in one plane to a point on the other plane. (imagine a triangle on one plane and a square on the other plane - which edges correspond?) The gcode will then be interpolated between the anchor points.

Additionally, the svgs can be offset and the geometry of the machine is taken into account. this is useful to get the exact shape of the svgs onto the sides of the foam block that is to be cut

feel free to edit
