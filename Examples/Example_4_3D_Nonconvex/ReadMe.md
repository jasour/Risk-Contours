
# Complex Uncertain Obstacle



* Uncertain object with uncertain geometry


<p align="center">
{(x1,x2) : g(x1,x2,w) >= 0 }  
<p>

where g(x1,x2,w)=- (21x1^5)/50 - (59x1^4x2)/50 - (47x1^4)/100 + (3x1^3x2^2)/10 - (57x1^3x2)/100 + (3x1^3)/5 - (13x1^2x2^3)/20 + (17x1^2x2^2)/100 + (187x1^2x2)/100 + (3x1^2)/50 + (69x1x2^4)/100 - (7x1x2^3)/50 - (17x1x2^2)/20 + (3x1x2)/5 - (21x1)/100 + x2^5/100 - (3x2^4)/50 - (7x2^3)/100 - (41x2^2)/100 - (2*x2)/25 - w/10 + 7/100
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

