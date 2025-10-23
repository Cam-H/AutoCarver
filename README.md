# AutoCarver
Automates the generation of tooling paths to create 3D sculptures via subtractive manufacturing techniques.

Core functionality to provide trimming and silhouette refinement is present, but further refinement
planning is still under development.

For a summary of the current state of the project, see:
https://cam-h.github.io/autocarver.html

## Current priorities for development
- Develop ProfileBuilder to develop profiles directly from the mesh, rather than the current approximation using image processing
- Identification of features for refinement
- Integration of D* or similar planning algorithms to improve path planning
- Debugging contact physics simulation to improve stability
- Miscellaneous TODO notes interspersed through the code
