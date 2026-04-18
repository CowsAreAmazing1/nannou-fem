# nannou-fem
Custom FEM implementation solving poissons equaion

### Setup Density Field
Closed parametric curves representing regions of a specified density.

### Triangulate
Fill the window with a controllable triangle mesh, with constraints defining the shape of the density regions.

### Solve
Set up a discretised density field on the mesh nodes, and display this if density mode is selected. Otherwise, Convert this to a sparse matrix representation to solve poisson's equation, by solving the resulting `Kx = f` equation for potential, and display this if potential mode is selected. Otherwise, compute $\nabla \phi$ for the acceleration field, and display this.

<img width="2999" height="1999" alt="image" src="https://github.com/user-attachments/assets/0d1ef2aa-53b6-4852-8e54-1cb31fd97682" />
