# BBW-Real-Time-Mesh_Deformation

In this project the so called "Bounded Biharmonic Weights" real-time mesh deformation technique (by Jacobson et al) is employed to develop a new module in Graphite interface.(Graphite is an open source platform for computer graphics and 3D modeling http://alice.loria.fr/index.php?option=com_content&view=article&id=22).
As the demo indicates, the mesh deformation method "Bounded Biharmonic Weights" outperforms "As Rigid As Possible" method in terms of Smoothness, Non-negativity, Shape-awareness, Locality and sparsity, etc. 
The "BBW-Real-Time-Mesh_Deformation" source code contains following steps:

- Triangulation of 2D images (using C++, Graphite interface)
- Minimization of the Laplacian energy through the use of bounded bi harmonic blending using MOSEK library as a sparse quadratic programming solver. (using C++, Graphite interface)

Demo:

[![BBW](https://img.youtube.com/vi/NqKzbHyBxoA/0.jpg)](https://www.youtube.com/watch?v=NqKzbHyBxoA)


