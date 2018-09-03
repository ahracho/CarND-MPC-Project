# **MPC Project**

The goals / steps of this project are the following:
* Understand how MPC works in Vehicle Control
* Implement MPC in C++ and apply it to the simulator


[//]: # (Image References)

[image1]: ./writeup_images/coordinates.png "coordinates formula"


## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/896/view) individually and describe how I addressed each point in my implementation.  

### Compiling

#### 1. Your code should compile.
I completed the project under Ubuntu bash on Windows 10. I didn't modify CMakeLists.txt and other configuration files, so follow below to compile the code.  

~~~sh
cd build
cmake ..
make
./mpc
~~~

Then, launch Term 2 simulator Project 5 MPC Controller.

### Implementation
#### 1. The Model  
My MPC model follows below.  

##### 1. Change waypoints from map coordinates to the vehicle's coordinate.

> The formula to change the coordinates into vehicle's is as below. Every frame with waypoints, I apply the formula and change them into vehicle's coordinates so that I can visualize the waypoints on the simulator.  

![image1]  

~~~cpp
vector<double> ptsx = j[1]["ptsx"];
vector<double> ptsy = j[1]["ptsy"];

for (unsigned int i = 0; i < ptsx.size(); i++) {
    double x = ptsx[i] - px;
    double y = ptsy[i] - py;
    ptsx[i] = cos(psi)*x + sin(psi)*y;
    ptsy[i] = -sin(psi)*x + cos(psi)*y;
}
~~~  

##### 2. Calculate polynomial coefficients.  

> With waypoints converted into vehicle's coordinates, I fitted them into third-ordered polynomial.  

~~~cpp
// convert std::vector into Eigen::VectorXd
Eigen::VectorXd vcmap_x = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
Eigen::VectorXd vcmap_y = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());

Eigen::VectorXd coeffs = polyfit(vcmap_x, vcmap_y, 3);
~~~  

##### 3. Form the state vector.  

> Because of the latency, I formed the state vector with 100ms-delayed values so that the actuator predictions can be converged in realtime. I set 100ms latency for the prediction.  

~~~cpp
double delay_t = 0.1;

double delay_x = v * delay_t;
double delay_y = 0;
double delay_psi = -v * delta / Lf * delay_t;
double delay_v = v + a * delay_t;
double delay_cte = cte + v * sin(epsi)*delay_t;
double delay_epsi = epsi - v * delta / Lf * delay_t;

Eigen::VectorXd state(6);
state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;
~~~

##### 4. Define(Calculate) cost function.  

> To get accurate prediction, I added several values to the cost including CTE, EPSI, and difference in acceleration/steering angle. The reference velocity is set as 75 mph.  

~~~cpp
for (unsigned int i = 0; i < N; i++) {
    fg[0] += 1000 * CppAD::pow(vars[cte_start + i], 2);
    fg[0] += 1000 * CppAD::pow(vars[epsi_start + i], 2);
    fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
}

for (unsigned int i = 0; i < N - 1; i++) {
    fg[0] += CppAD::pow(vars[delta_start + i], 2);
    fg[0] += CppAD::pow(vars[a_start + i], 2);
}

for (unsigned int i = 0; i < N - 2; i++) {
    fg[0] += 100 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
    fg[0] += 10 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
}
~~~


##### 5. Get new actuator and predicted waypoint values.  

~~~cpp
auto result = mpc.Solve(state, coeffs);

vector<double> mpc_x_vals;
vector<double> mpc_y_vals;

json msgJson;
msgJson["steering_angle"] = result[0];
msgJson["throttle"] = result[1];
          
for (unsigned int i = 0; i < N; i++) {
    mpc_x_vals.push_back(result[2 + i]);
    mpc_y_vals.push_back(result[2 + N + i]);
}
~~~

#### 2. Timestep Length and Elapsed Duration (N & dt)

I set N and dt as 10 and 0.1 so that I predict 1 seconds of the vehicle control. As N gets bigger, the operation gets much larger and takes more time. I though I can cause longer latency.

~~~cpp
size_t N = 10;
double dt = 0.1;
~~~  

#### 3. Polynomial Fitting and MPC Preprocessing

After converting map coordinates of waypoints into vehicle's coordinates, I put them into `polyfit` function to get polynomial coefficients.  

~~~cpp
// convert std::vector into Eigen::VectorXd
Eigen::VectorXd vcmap_x = Eigen::VectorXd::Map(ptsx.data(), ptsx.size());
Eigen::VectorXd vcmap_y = Eigen::VectorXd::Map(ptsy.data(), ptsy.size());

Eigen::VectorXd coeffs = polyfit(vcmap_x, vcmap_y, 3);
~~~  

#### 4. Model Predictive Control with Latency

To adopt latency, I used delayed/predicted state vector to the model. I recalculated state vector after `dt` moment and put them into `Solve` model. Without this process, the final steering angle and acceleration value makes the vehicle move unexpected way.   

~~~cpp
double delay_t = 0.1;

double delay_x = v * delay_t;
double delay_y = 0;
double delay_psi = -v * delta / Lf * delay_t;
double delay_v = v + a * delay_t;
double delay_cte = cte + v * sin(epsi)*delay_t;
double delay_epsi = epsi - v * delta / Lf * delay_t;

Eigen::VectorXd state(6);
state << delay_x, delay_y, delay_psi, delay_v, delay_cte, delay_epsi;
~~~  


### Simulation
#### 1. The vehicle must successfully drive a lap around the track.

Final Lap for the MPC Controller is in [./writeup_images/MPC_Controller.mp4](./writeup_images/MPC_Controller.mp4).  

