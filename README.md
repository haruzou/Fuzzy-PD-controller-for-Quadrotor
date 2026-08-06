# Fuzzy-PD-controller-for-Quadrotors (Matlab r2025a)

In this case, I simulated a fuzzy filter with a 2-input 1-output structure and used two parallel branches for the fuzzy inference system (FIS) of delta Kp and delta Kd.

Regarding the simulation environment, with dx=dy=dz=d_phi=d_theta=d_psi = 0 (error), the results were very good with a very small difference between the desired trajectory and the calculated trajectory (approximately 0.01%).

I simulated this project based on https://doi.org/10.1016/j.rico.2025.100568
# Simulink
<img width="1917" height="1009" alt="image" src="https://github.com/user-attachments/assets/e71dcf3e-4427-41ae-bad3-39e0e28d23ca" />


The trajectory is desired:

xd = 0.5sin(pi*t/20 + pi/2)

yd = 0.5sin(pi*t/2)

zd = 2 + 0.5sin(pi*t/20 + 3pi/2)

psid = 0
<img width="1076" height="679" alt="image" src="https://github.com/user-attachments/assets/cdd0ad3a-9c75-46af-8ee3-9f7af40f20f5" />
# MATLAB mfile
