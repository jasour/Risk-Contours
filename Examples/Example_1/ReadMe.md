

# Static Uncertain Obstacle


Uncertain object with uncertain size

* Uncertain Object: A circle with uncertain radius
i.e., {(x1,x2) : w1^2-(x1-0)^2-(x2-0)^2 >= 0 } where w1 is the uncertain radius with Uniform distribution on [0.3, 0.4]


* Probabilistic safety constraint:
Probability of collision with the uncertain object should be low, i.e., Probability( w1^2-(x1-0)^2-(x2-0)^2 >= 0 ) <= Delta

* Risk-bounded safe set:
Set of all locations that does not collide with the uncertain object with high probability,
i.e., { (x1,x2): Probability( w1^2-(x1-0)^2-(x2-0)^2 >= 0 ) <= Delta }




**Approach A**
<p align="center">
<img src="https://github.com/jasour/Risk-Contours/blob/main/Examples/Example_1/Plots/A_1.png" width="400" height="400" />
<img src="https://github.com/jasour/Risk-Contours/blob/main/Examples/Example_1/Plots/A_2.png" width="400" height="400" />
<p align = "center">


**Approach B**
<p align="center">
<img src="https://github.com/jasour/Risk-Contours/blob/main/Examples/Example_1/Plots/B_1.png" width="400" height="400" />
<img src="https://github.com/jasour/Risk-Contours/blob/main/Examples/Example_1/Plots/B_2.png" width="400" height="400" />
<p align = "center">



