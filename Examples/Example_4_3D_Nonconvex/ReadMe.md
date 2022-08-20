
# Complex Uncertain Obstacle



* Uncertain object with uncertain geometry


<p align="center">
{(x1,x2) : g(x1,x2,w) >= 0 }  
<p>

where g(x1,x2,w)= 

and w is the uncertain parameter with a Beta distribution.


* Probabilistic safety constraint:
Probability of collision with the uncertain object should be low, 
<p align="center">
Probability( g(x1,x2,w) >= 0 ) <= Delta
<p>

* Risk-bounded safe set:
Set of all locations that does not collide with the uncertain object with high probability,
<p align="center">
{ (x1,x2): Probability( g(x1,x2,w) >= 0 ) <= Delta }
<p>

<p align="center">
<img src="https://github.com/jasour/Risk-Contours/blob/main/Examples/Example_4_3D_Nonconvex/plot.png" width="400" height="400" />
<p align = "center">

