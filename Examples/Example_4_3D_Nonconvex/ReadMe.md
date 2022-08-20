
# Complex Uncertain Obstacle



* Uncertain object with uncertain geometry


<p align="center">
{(x1,x2) : g(x1,x2,x3,w) >= 0 }  
<p>

where g(x1,x2,x3,w)= 0.94-0.002x1-0.004x2-0.04x3-0.38x1^2+0.04x1x2-0.31x2^2-0.05x1x3-0.01x2x3-0.4x3^2-0.1x1^3-0.02x1^2x2+0.09x1x2^2-0.05x2^3+0.14x1^2x3-1.83x1x2x3+0.11x2^2x3-0.1x1x3^2+0.12x2x3^2+0.34x3^3-0.32x1^4-0.13x1^3x2+0.48x1^2x2^2+0.11x1x2^3-0.34x2^4+0.03x1^3x3+0.01x1^2x2x3-0.005x1x2^2x3-0.05x2^3x3+0.54x1^2x3^2-0.06x1x2x3^2+0.48x2^2x3^2+0.008x1x3^3+0.06x2x3^3-0.3x3^4+0.12x1^5+0.005x1^4x2-0.1x1^3x2^2+0.007x1^2x2^3+0.005x1x2^4+0.071x2^5-0.02x1^4x3+0.73x1^3x2x3-0.07x1^2x2^2x3+0.72x1x2^3x3-0.20x2^4x3+0.03x1^3x3^2-0.01x1^2x2x3^2+0.02x1x2^2x3^2-0.05x2^3x3^2-0.07x1^2x3^3+0.73x1x2x3^3+0.09x2^2x3^3+0.03x1x3^4-0.06x2x3^4-0.31x3^5-w-0.84;


and w is the uncertain parameter with a normal distribution.


* Probabilistic safety constraint:
Probability of collision with the uncertain object should be low, 
<p align="center">
Probability( g(x1,x2,x3,w) >= 0 ) <= Delta
<p>

* Risk-bounded safe set:
Set of all locations that does not collide with the uncertain object with high probability,
<p align="center">
{ (x1,x2,x3): Probability( g(x1,x2,x3,w) >= 0 ) <= Delta }
<p>

<p align="center">
<img src="https://github.com/jasour/Risk-Contours/blob/main/Examples/Example_4_3D_Nonconvex/plot.png" width="400" height="400" />
<p align = "center">

