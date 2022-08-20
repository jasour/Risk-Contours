
# Complex Uncertain Obstacle



* Uncertain object with uncertain geometry


<p align="center">
{(x1,x2) : g(x1,x2,w) >= 0 }  
<p>

where g(x1,x2,w)= 0.94-0.002.*x1-0.004.*x2-0.04.*x3-0.38.*x1.^2+0.04.*x1.*x2-0.31.*x2.^2-0.05.*x1.*x3-0.01.*x2.*x3-0.4.*x3.^2-0.1.*x1.^3-0.02.*x1.^2.*x2+0.09.*x1.*x2.^2-0.05.*x2.^3+0.14.*x1.^2.*x3-1.83.*x1.*x2.*x3+0.11.*x2.^2.*x3-0.1.*x1.*x3.^2+0.12.*x2.*x3.^2+0.34.*x3.^3-0.32.*x1.^4-0.13.*x1.^3.*x2+0.48.*x1.^2.*x2.^2+0.11.*x1.*x2.^3-0.34.*x2.^4+0.03.*x1.^3.*x3+0.01.*x1.^2.*x2.*x3-0.005.*x1.*x2.^2.*x3-0.05.*x2.^3.*x3+0.54.*x1.^2.*x3.^2-0.06.*x1.*x2.*x3.^2+0.48.*x2.^2.*x3.^2+0.008.*x1.*x3.^3+0.06.*x2.*x3.^3-0.3.*x3.^4+0.12.*x1.^5+0.005.*x1.^4.*x2-0.1.*x1.^3.*x2.^2+0.007.*x1.^2.*x2.^3+0.005.*x1.*x2.^4+0.071.*x2.^5-0.02.*x1.^4.*x3+0.73.*x1.^3.*x2.*x3-0.07.*x1.^2.*x2.^2.*x3+0.72.*x1.*x2.^3.*x3-0.20.*x2.^4.*x3+0.03.*x1.^3.*x3.^2-0.01.*x1.^2.*x2.*x3.^2+0.02.*x1.*x2.^2.*x3.^2-0.05.*x2.^3.*x3.^2-0.07.*x1.^2.*x3.^3+0.73.*x1.*x2.*x3.^3+0.09.*x2.^2.*x3.^3+0.03.*x1.*x3.^4-0.06.*x2.*x3.^4-0.31.*x3.^5-w;

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

