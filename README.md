# Risk Contours
Risk Contours ( Chance Constrained Sets ): Set of all design parameters that satisfy the probabilistic safety constraints.

Risk Contours turn probabilistic constraints into deterministic constraints.
<p align="center">
    Probabilistic Constraints ⮕ Deterministic Constraints
</p>

---




-  **Probabilistic Safety Constraint:**  

Safety constraints are defined in terms of the design parameters and uncertainties. The objective is to satisfy the safety constraints with high probability.

<p align="center">
    Probabilistic safety constraint = Probability( satisfying the safety constraint ) >= 1-Delta
</p>

- **Risk Contours:** 

Set of all design parameters that satisfy the probabilistic safety constraints.

<p align="center">
    Risk Contours (Delta) = { design parameters: Probability(satisfying the safety constraint ) >= 1-Delta }
</p>

---

**Example:** 

Uncertainty: An object with uncertain location, size, and geometry.

Probabilistic Safety Constraint: Probability of collision with the uncertain object should be low.

Risk Contours (Delta) = Set of all locations in the environment whose probability of collision with the uncertain object is less than or equal to Delta.

---

**Approach:** 

We provide an provable analytical method that uses statistics (moments) of the probability distributions of the uncertainties to construct the risk contours. Existing methods to construct such risk bounded sets either are limited to particular class of uncertainties and linear safety constriants or rely on expensive sampling-based methods. Hence, such methods are not suitable for online planning problems and can not assure the safety in the presence of nonlinear safety constraints. 

The provided approach deals with 
i) arbitrary probabilistic uncertainties, ii) nonlinear safety constraints, and iii) static and time varying safety constraints and is suitable for online planning problems.

**A)** Risk Contours for Multi-Modal Non-Gaussian Uncertainties

A-1) Eq(4): Static Safety Constraints ⮕  Eq(9): Static Risk Contours ⮕  Eq (10): Inner approximation of the static risk contour

A-2) Eq(5): Dynamic Safety Constraints ⮕ Eq (11): Dynamic Risk Contour ⮕ Eq (12): Inner approximation of the dynamic risk contour

[Ashkan Jasour, Weiqiao Han, Brian Williams,"Convex Risk Bounded Continuous-Time Trajectory Planning in Uncertain Nonconvex Environments", Robotics: Science and Systems (RSS), 2021.](http://www.roboticsproceedings.org/rss17/p069.pdf)


**B)** Risk Contours for Unimodal Non-Gaussian Uncertainties

Eq(2): Safety Constraints ⮕ Eq(5): Risk Contours ⮕ Eq (6): Inner approximation of the risk contour


[Weiqiao Han, Ashkan Jasour, Brian Williams,"Non-Gaussian Risk Bounded Trajectory Optimization for Stochastic Nonlinear Systems in Uncertain Environments", 39th IEEE Conference on Robotics and Automation (ICRA), 2022.](https://arxiv.org/pdf/2203.03038.pdf)


Other methods:

**C)** Semidefinite Programming-based Risk Contours

This approach uses polynomial indicator function of the safety set.

Eq(1): Safety Constraints ⮕ Eq(3): Risk Contours ⮕ Eq (10) and (13): Outer and Inner approximation of the risk contour


[Ashkan Jasour, Brian C. Williams, "Risk Contours Map for Risk Bounded Motion Planning under Perception Uncertainties", Robotics: Science and System (RSS), 2019.](http://www.roboticsproceedings.org/rss15/p56.pdf)

---






