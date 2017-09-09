# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
[//]: # (Image References)

[model-predictive-control]: model-predictive-control.png "Model Predictive Control"

## Summary of the Model Predictive Controller

### Model
The MPC controller arrangement in the car takes as input:

* 16 waypoints coordinates in a Global Coordinate System (GCS)
* the car coordinates in GCS
* the feedback controlled variables - yaw and velocity. 

It produces, based on a kinematic model of the car, optimal manipulated variables u - the steering and throttle that minimize the 
cost a quadratic function of cross track error (CTE) as well as velocity deviations from a nominal velocity value over a horizon of N*dt seconds.   
The optimized variables (steering value and throttle) are constrained based on the implementation of the car simulator by lower and upper bounds. 

![model-predictive-control][model-predictive-control]


To fit a 3rd order polynomial line onto the set of received waypoints expressed in the GCS, the coordinates are transformed in the local (car) coordinmate system (LCS). 
This transformation is expressed as: x' = Rx+t where x is the LCS coordinates and x' are the GCS coordinates. t is the distance between the car and the GCS
origin. Given that R rotation matrix R = [cos(psi)  -sin(psi); sin(psi) cos(psi)] is orthonomal we have RR^T=I and solving for 
x = R^T(x'-t). The LCS coordinates are set accordingly in the code. 

The CTE and yaw error were then much more easily to be estimated as the car coordinates are at LCS(0,0). The CTE and yaw error 
are then included as part of the state: (x, y, yaw, velocity, CTE and yaw error). In the cost function, the reference 
state costs, absolute actuation costs and differential actuation costs were included with coefficients that were manually tuned. 
More specifically the car is able to travel through the track with 100ms latency using the following coefficients that correspond 
to quadratic errors:
(cte, yaw error, relative velocity, steering magnitude, throttle magnitude, rate of change of steering, rate of change of throttle) = (1, 1, 1, 1000, 200, 1, 1)


In the optimization problem we selected N=20 and dt=0.1ms. 2sec was an intuitively selected value based on the use case 
(racing track). Intuitively thinking if one plots the uncertainty of the predictive distribution based on simple 
kinematic models we expect to start from small variances and towards the end of the horizon to observe significant 
predictive uncertainty. So it makes sense to limit the N*dt to few seconds and given the sharp turns of he track, 2 seconds were 
selected. dt was then selected based on the quantization error. dt < 100ms would approximate the target trajectory 
better but it will result in large N for the same time horizon of 2secs - this will increase computational complexity. 
 

To deal with latency between the controller and the plant (car), we used the kinematic equations as shown below, to predict the state variables 100ms into the future 
before sending them to MPC. 
 
 
```objectivec
// Fit a 3rd order polynomial to the above x and y coordinates
auto coeffs = polyfit(waypoints_x_lcs, waypoints_y_lcs, 3);


// Predict the state variables at t+100ms (latency)
Eigen::VectorXd state = Eigen::VectorXd::Zero(6);
double latency_ms = 100;
double latency = latency_ms/1000.;

// Calculate the current cte
/* cte = desired_y - actual_y
        = polyeval(coeffs,px)-py
        = polyeval(coeffs,0) because px=py=0 */
double cte = polyeval(coeffs, 0);

// Calculate the current epsi
// epsi =  actual psi-desired psi
// double epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs[3] * pow(px,2));
// = -atan(coeffs[1]), because px=py=0
double epsi = -atan(coeffs[1]);

delta = -delta; // change of sign because turning left is negative sign in simulator but positive yaw for MPC
state[0] = v * cos(psi) * latency; // x: px + v * cos(psi) * latency;
state[1] = 0; // y: py + v * sin(psi) * latency; // y: car coordinates were transformed such that the car moves along the x axis in the model
state[2] = (v / 2.67)  * delta * latency; // psi: psi + (v / 2.67)  * delta * latency;
state[3] = v + throttle * latency;
state[4] = cte + v * sin(epsi) * latency;
state[5] = epsi + state[2];

// Call ipopt solver
auto vars = mpc.Solve(state, coeffs);

```
## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * If challenges to installation are encountered (install script fails).  Please review this thread for tips on installing Ipopt.
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

