# Automated Parametric Posture Solver for Closed Kinematic Chains

## About the Project
In space robotics, tools like the Satellite Dynamics Toolbox (SDTlib) rely on Linear Fractional Transformations (LFTs) to model parametric uncertainties. However, closed kinematic chains introduce implicit non-rational constraints that prevent direct LFT modelling, requiring the computation of the exact constrained parameters to approximate the necessary rational constraints. This computation is often either extremely time-consuming or outright impossible. 

This research develops a parametric numerical solver to compute the constrained posture of both planar and spatial multi-loop mechanisms. By using a standardized JSON architecture and graph theory, the solver autonomously discovers independent kinematic loops and uses the Levenberg-Marquardt algorithm to solve parametrically obtained loop closure constraints. The custom solver was verified and validated against ESA’s Athena spacecraft hexapod platform achieving a high precision accuracy of 10^-16 m. By successfully solving the posture in a parametric manner, the custom solver effectively bridges the gap between complex physical multi-body systems and the approximation of LFTs.

## How to Run
This tool is built entirely in MATLAB. No external toolboxes are required.

1. Clone this repository to your local machine.
2. Open MATLAB and navigate to the repository folder.
3. Run `main.m` in the command window.
4. Use the GUI to select a mechanism from the `linkages/` directory, set your input joints, and launch the solver.

## Defining a Mechanism
Mechanisms are defined using a standardized `.json` architecture. Users only need to define the rigid bodies and the joints connecting them. The solver autonomously handles the topological mapping. 

For complex spatial mechanisms, please refer to the templates provided in the `linkages/` directory (good examples are the ATHENA hexapod or the 6-bar linkages). Appendices A and B from the report in the `docs/` repository can also be used as a reference for defining mechanisms using the `.json` structure.

## Documentation & Theory
For a deep dive into the underlying mathematics, including the Cycle Basis algorithm, the parametric constraint equations, and the Levenberg-Marquardt numerical optimization implementation, please see the full Master's Report located in the `docs/` folder.