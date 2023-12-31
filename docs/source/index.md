---
hide-toc: true
---

# Safe Navigation and Obstacle Avoidance Using Differentiable Optimization Based Control Barrier Functions

*[Bolun Dai](https://bolundai0216.github.io), Rooholla Khorrambakht, Prashanth Krishnamurthy, Vinícius Gonçalves,  
Anthony Tzes, Farshad Khorrami*  

---

**Abstract**: *Control barrier functions (CBFs) have been widely applied to safety-critical robotic applications. However, the construction of control barrier functions for robotic systems remains a challenging task. Recently, collision detection using differentiable optimization has provided a way to compute the minimum uniform scaling factor that results in an intersection between two convex shapes and to also compute the Jacobian of the scaling factor. In this paper, we propose a framework that uses this scaling factor, with an offset, to systematically define a CBF for obstacle avoidance tasks. We provide theoretical analyses of the continuity and continuous differentiability of the proposed CBF. We empirically evaluate the proposed CBF's behavior and show that the resulting optimal control problem is computationally efficient, which makes it applicable for real-time robotic control. We validate our approach, first using a 2D mobile robot example, then on the Franka-Emika Research 3 (FR3) robot manipulator both in simulation and experiment.*

---

## Results

This paper presents a systematic and computationally simple approach to construct CBFs using differentiable optimization based collision detectors. We showed continuous differentiability for strongly convex scaling functions when the gradients and Hessians exist and are continuous. We conjecture that the continuous differentiability can be generalized to one object being strongly convex and the second being only convex. We experimentally demonstrated the efficacy of our approach on a mobile robot in simulation and a 7-DOF robot manipulator in both simulations and on the real robot. The following videos show the simulation and experimental results.

```{eval-rst}
..  youtube:: MgFF3Xjnlpc
   :width: 100%
```

---

## Paper

Latest version: [here](https://arxiv.org/abs/2304.08586).  
*IEEE Robotics & Automation Letters (RA-L), 2023*

[![Paper PDF](_static/ra-l_paper.png)](https://arxiv.org/abs/2304.08586)

<a href="https://github.com/BolunDai0216/DifferentiableOptimizationCBF">
  <img src="_static/github-mark.png" alt="Link to GitHub Repo" width="50"/>
</a>

---

## Citation

To cite our paper, please use the following bibtex

```bibtex
@article{DaiKKGTK23,
  author       = {Bolun Dai and Rooholla Khorrambakht and Prashanth Krishnamurthy and Vin{\'{\i}}cius Gon{\c{c}}alves and Anthony Tzes and Farshad Khorrami},
  title        = {Safe Navigation and Obstacle Avoidance Using Differentiable Optimization Based Control Barrier Functions},
  journal      = {{IEEE} Robotics and Automation Letters},
  year         = {2023},
  volume       = {8},
  number       = {9},
  pages        = {5376-5383},
}
```

---

## Acknowledgements

This work was partially supported by the NYUAD Center for Artificial Intelligence and Robotics (CAIR), funded by Tamkeen under the NYUAD Research Institute Award CG010. We would like to thank Avadesh Meduri for his help in setting up the robot communication pipeline. We would also like to thank the contributors to the open-source software that we utilize for the simulations and robotic experiments.

---

## Contact

If you have any questions, please feel free to contact [Bolun Dai](mailto:bd1555@nyu.edu).

```{toctree}
:caption: 'Tutorials:'
:hidden:
:maxdepth: 2

install
tutorials/index
```