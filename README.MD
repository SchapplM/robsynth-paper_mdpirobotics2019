# Modeling Parallel Robot Kinematics for 3T2R and 3T3R Tasks Using Reciprocal Sets of Euler Angles


Authors:  
Moritz Schappler, Svenja Tappe, Prof. Dr.-Ing. Tobias Ortmaier

This code is supplemental material to the following submission to the MDPI Robotics Special Issue on Kinematics and Robot Design (KaRD 2019):

```bib
@Article{SchapplerTapOrt2019,
  author        = {Schappler, Moritz and Tappe, Svenja and Ortmaier, Tobias},
  title         = {Modeling Parallel Robot Kinematics for 3T2R and 3T3R Tasks Using Reciprocal Sets of Euler Angles},
  journal       = {Robotics (MDPI Journal)},
  year          = {2019},
  volume        = {8},
  number        = {3},
  issn          = {2218-6581},
  doi           = {10.3390/robotics8030068},
  issue         = {Kinematics and Robot Design II},
  publisher     = {Multidisciplinary Digital Publishing Institute},
}
```

## Abstract

Industrial manipulators and parallel robots are often used for tasks, such as drilling or milling, that require three translational, but only two rotational degrees of freedom ("3T2R").
While kinematic models for specific mechanisms for these tasks exist, a general kinematic model for parallel robots is still missing. 
This paper presents the definition of the rotational component of kinematic constraints equations for parallel robots based on two reciprocal sets of Euler angles for the end-effector orientation and the orientation residual. 
The method allows completely removing the redundant coordinate in 3T2R tasks and to solve the inverse kinematics for general serial and parallel robots with the gradient descent algorithm. 
The functional redundancy of robots with full mobility is exploited using nullspace projection.


## Contents

This repository contains a Maple worksheet and Matlab scripts to reproduce the equations and figures of the paper:  

* `equations/ikfr_pkm_paper_equations.mw`
  * creates latex code for equations (A24), (A25) and Appendix A.1 of the paper.
* `equations/equations_postprocess.sh` has to be used after running the Maple worksheet
* `calculations/Goldenberg_residual.m` checks the relation between the residual from this paper and the reference Goldenberg et al. 1985, discussed in appendix A.2
* `figures/pkm_3RRR_figure_gen.m`
  * Create figures for the 3RRR parallel robot in different configurations
  * Is used in Fig. 3 and Fig. 4 of the paper.
* `figures/serrob_traj.m` creates Fig. 6 and 7 of the paper
* `figures/parrob_traj.m` creates Fig. 8 of the paper
* `figures/serrob_ik_histogram_start_all.m` creates the data for the histograms in Fig. 9 and 10 of the paper
* `figures/serrob_ik_histogram_combinedfigure.m` creates Fig. 9 and 10 from the data


## Other Implementations and Further Tests

The method described in the paper is employed in the [robotics toolbox](https://github.com/SchapplM/robotics-toolbox) and is used in other related research projects as well, e.g. [the combined structural and dimensional robot synthesis](https://github.com/SchapplM/robsynth-paper_iftommdach2020).
The functionality of the gradient calculation and the inverse kinematics algorithm is tested to more extent in the test scripts `ParRob_constraints_test.m.m` and `ParRob_class_example_6UPS_3T2R.m` in the [robotics toolbox](https://github.com/SchapplM/robotics-toolbox).

## Prerequisites

The Maple worksheet has been tested with Maple version 2019
* For the worksheet `ikfr_pkm_paper_equations.mw`, download Maple dependencies containing procedures for Euler angle transformations:
  * https://github.com/SchapplM/robotics-toolbox (tested with rev. bcb2e77; 2020-02-15)
  * https://github.com/SchapplM/robsynth-paper_iftommwc2019_invkinfuncred (tested with rev. 134745a; 2020-02-23)
* Create one path-file for each dependencies as described in the files `calculations/reci_euler_paper_path.template` and `calculations/robotics_toolbox_path.template`
* See instructions at the top of the worksheet

The Matlab script has been tested with Matlab version R2019a
* Download additional dependencies
  * https://github.com/SchapplM/robsynth-serroblib (tested with rev. dff760b; 2020-02-16)
  * https://github.com/SchapplM/robsynth-modelgen (tested with rev. 0f11fd8; 2020-02-15)
  * https://github.com/SchapplM/matlab_toolbox (tested with rev. 366f149; 2020-02-15)
  * https://github.com/SchapplM/robotics-dep-ext (tested with rev. bc57a42; 2020-02-17)
  * https://github.com/SchapplM/robotics-dep-traj (tested with rev. 41ceefd; 2020-02-15)
* Set up the toolboxes to work and modify the following files according to the instructions
  * robsynth-serroblib/maplerepo_path.m.template (has to link to robsynth-modelgen)
  * robsynth-modelgen/robotics_repo_path.template (has to link to robsynth-robotics_toolbox)
* Run the path initialization scripts (`..._path_init.m`) of all downloaded software repos in Matlab. Put the run commands in the startup.m file of Matlab for easier use.

