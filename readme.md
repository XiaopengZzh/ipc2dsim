### Description

This is a project for Geometry Method course at New York University. It is a 2D simulation of incremental potential contact or [IPC](https://ipc-sim.github.io).

In IPC method, the positions of elastic objects at each time step are updated by minimizing an incremental potential energy which contains elastic energy, barrier energy and frictional energy(IPC designed a conservative force to approximate this).

IPC minimizes this energy by Newton method. For each iteration update, [Tight Inclusion CCD](https://github.com/Continuous-Collision-Detection/Tight-Inclusion) helps me to find a step size $\alpha$ that ensures no penetration with step size less than this so that we will have much less sample point to search.

### Parameters

dhat :  the maximum separation distance where repulsion between objects exists.

kappa : a parameter to control barrier stiffness.

With the above two parameters, the barrier energy is defined as $\kappa \sum_{k\in C}b(d_k(x))$. We can see that $\frac{\partial E_{others}}{\partial x} + \kappa \frac{\partial E_{barrier}}{\partial x} = 0$, thus for the same scenario, with larger $\kappa$, we will have less barrier repulsion force.

E : Young's modulus, measuring stiffness of elastic material

nu : Poisson' ratio, measuring the ability to be deformed in the direction perpendicular to applied force.

All the above parameters are defined in phyScene.h

### Dependencies

This project requires Eigen, catch2, GLFW, OpenGL.

### Build

```sh
mkdir build
cd build
cmake [options] ..
make
```

This project has several tags to try.

```sh
./ipc2dsim "[development]"
```

Toggle off DISABLE_RENDER in macros.h file to enable rendering. Try different models in folder /model, by changing the filename in modelInit.cpp

```sh
./ipc2dsim "[unit tests]"
```

This runs a few unit tests, currently including "1tri1cube", "2cubes", "2triangles".(there will be more).

For larger model, the simulation can be quite slow. To export obj file sequence for making animation uses, try the following tag. (It's better to toggle on DISABLE_RENDER in macros.h first).
```sh
./ipc2dsim "[export obj sequence]"
```

### Reference

1. Incremental Potential Contact: Intersection- and Inversion-free Large Deformation Dynamics
2. A Large Scale Benchmark and an Inclusion-Based Algorithm for Continuous Collision Detection
