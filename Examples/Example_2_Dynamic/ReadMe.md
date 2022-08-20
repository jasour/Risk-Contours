


# Dynamic Uncertain Obstacle



* Uncertain Moving Object: A circle-shaped obstacle with uncertain radius and uncertain trajectory

<p align="center">
{(x1,x2) : w1^2-(x1-px)^2-(x2-py)^2 >= 0 }
<p>
<p align="center">
px=2-t+t^2+0.2w2
<p>
<p align="center">
py=-1+4*t-t^2+0.1w3 
<p>
where w1 is the uncertain radius with Beta distribution, (px,py) is the uncertain trajectory of the center of the obstacle, t is time, w2 has a uniform distribution, and w3 has a normal distribution.



* Probabilistic safety constraint:
Probability of collision with the uncertain object should be low, 
<p align="center">
Probability( w1^2-(x1-px)^2-(x2-py)^2 >= 0 ) <= Delta
<p>

* Risk-bounded safe set:
Set of all locations that does not collide with the uncertain object with high probability,
<p align="center">
{ (x1,x2): Probability( w1^2-(x1-px)^2-(x2-py)^2 >= 0 ) <= Delta }
<p>




<p align="center">
<img src="https://github.com/jasour/Risk-Contours/blob/main/Examples/Example_2/plot.png" width="400" height="400" />
<p align = "center">
At each time t, for any point inside the risk contour (outside of the outer curve), probability of collision with the moving uncertain obstacle is less or equal to ∆.




