# CBF-RRT<sup>*</sup>

## Algorithm:
### Find $h(x)$
1. Make contours from grid.
2. Set normal vectors to contours border.
3. Solve Laplace equation -> produce $f(x)$ as force to Poisson equation.
4. Solve Poisson equation 
5. Returns

![Safety Function](fig/output_h.svg)

### Safe RRT<sup>*</sup>

## Test Scenario
Scene geometry:

![Scene Geometry](fig/scene_geometry.svg)

Solution of Laplace equation:

![Laplace Equation](fig/laplace_equation.svg)

output is force in Poisson equation for Poisson
Safety Function $h(x)$.

![Vector Field](fig/vector_field_v.svg)

Solution of Poisson Equation:

![Poisson Equation](fig/poisson_equation.svg)

![Vector Field](fig/poisson_safety_function.svg)

Results:
## Without CBF
Results for 50 runs

![Without CBF](fig/rrt_without_cbf.svg)

Single run - produced Tree:

![Without CBF](fig/rrt_without_cbf_tree.svg)

## Without CBF
Results for 50 runs

![With CBF](fig/rrt_with_cbf.svg)

Single run - produced Tree:

![Without CBF](fig/rrt_with_cbf_tree.svg)