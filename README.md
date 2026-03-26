# Automated Parametric Posture Solver for Closed Kinematic Chains

## About the Project
In space robotics, tools like the Satellite Dynamics Toolbox (SDTlib) [1] rely on Linear Fractional Transformations (LFTs) [2] to model parametric uncertainties. However, closed kinematic chains introduce implicit non-rational constraints that prevent direct LFT modelling, requiring the computation of the exact constrained parameters to approximate the necessary rational constraints. This computation is often either extremely time-consuming or outright impossible.

This research develops a parametric numerical solver to compute the constrained posture of both planar and spatial multi-loop mechanisms. By using a standardized JSON architecture and graph theory [3, 4], the solver autonomously discovers independent kinematic loops and uses the Levenberg-Marquardt algorithm [5] to solve parametrically obtained loop closure constraints. The custom solver was verified and validated against ESA's Athena spacecraft hexapod platform [6] achieving a high precision accuracy of $10^{-16}$ m. By successfully solving the posture in a parametric manner, the custom solver effectively bridges the gap between complex physical multi-body systems and the approximation of LFTs.

## Supervision
This project was developed at **ISAE-SUPAERO** under the guidance of the following tutors:

**José O. NEVES**, ISAE-SUPAERO

**Francesco SANFEDINO**, ISAE-SUPAERO

**Daniel ALAZARD**, ISAE-SUPAERO

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

## References
[1] Daniel Alazard and Francesco Sanfedino. Satellite Dynamics Toolbox library (SDTlib) - User’s guide. Tech. rep. Institut Supérieur de l’Aéronautique et de l’Espace, 2021.

[2] Kemin Zhou and John Comstock Doyle. Essentials of Robust Control. Vol. 104. Upper Saddle River, NJ: Prentice Hall, 1999. isbn: 9780137739790.

[3] Lung-Wen Tsai. Mechanism design: enumeration of kinematic structures according to function. CRC press, 2000.

[4] Andreas Müller. “Representation of the kinematic topology of mechanisms for kinematic analysis”. In: Mechanism and Machine Theory 90 (2015), pp. 88–105.

[5] TomomichiSugihara. “Solvability-unconcerned inverse kinematics by the Levenberg–Marquardt method”. In: IEEE Transactions on Robotics 27.5 (2011), pp. 984–991.

[6] Simon Görries. “ATHENA Space Telescope: Line of Sight Control with a Hexapod in the Loop”. MA thesis. Stockholm, Sweden: KTH Royal Institute of Technology, 2017.
