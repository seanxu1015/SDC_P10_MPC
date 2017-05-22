# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from [here](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Reflections

1. Model information

The model state is [x, y, psi, v, cte, epsi].

The actuators is [delta, a].

Update equations:

x[t+1] = x[t] + v[t] * cos(psi[t]) * dt

y[t+1] = y[t] + v[t] * sin(psi[t]) * dt

psi[t+1] = psi[t] - v[t] * delta[t] / Lf * dt

v[t+1] = v[t] + a[t] * dt

cte[t+1] = cte[t] + v[t] * sin(epsi[t]) * dt

epsi[t+1] = epsi[t] - v[t] * delta[t] / Lf * dt

There are 7 parts in the cost function:
a) cte; b) epsi; c) v; d) delta; e) a; f) delta[t] - delta[t-1]; g) a[t] - a[t-1]
Each of them has an individual weight which is determined by trial and error.


2. Timestep and Frequency

Timestep:
dt = 0.1 s

Number of timesteps to track in the future: 
N = 7

At the beginning, I arbitrary chose N=10 dt=0.1s, and tune the parameters mentioned above first. After I found an optimized combination, I started to tune N and dt.
Here are the experiments I have done:

dt=0.05 N=7  The car is unstable

dt=0.05 N=10 The car is unstable

dt=0.05 N=20 The car is unstable

dt=0.1  N=5  Failed at any speed

dt=0.1  N=6  MPH=42 ref_v=45 (When exceeding this speed, the car would fail. Same as below) 

dt=0.1  N=7  MPH=80 ref_v=85 

dt=0.1  N=10 MPH=72 ref_v=75 

dt=0.1  N=15 MPH=67 ref_v=70 

dt=0.1  N=20 MPH=67 ref_v=70

dt=0.1  N=25 MPH=68 ref_v=70

dt=0.1  N=30 MPH=58 ref_v=60

dt=0.15 N=7  MPH=67 ref_v=65

dt=0.15 N=10 MPH=72 ref_v=75

dt=0.15 N=15 MPH=69 ref_v=70

Therefore, dt=0.1 and N=7 is the best choice under this condition.

3. Polynomial Fitting and MPC Preprocessing
The waypoints are first converted to car coordiate, then they are fitted with third-order polynomial.
Here are converting equations:

car_x = x * cos(psi) + y * sin(psi)

car_y = y * cos(psi) - x * sin(psi)

where x, y are the world coordinate originated in the car position, and psi is the direction of the car in the world coordinate.

The equations are derived as follows:

![](https://github.com/seanxu1015/SDC_P10_MPC/blob/master/coordinate_converting.png)

define r = sqrt(x^2+y^2)

car_x = r * cos(arccos(x/r) - psi) = r * [cos(arccos(x/r)) * cos(psi) + sin(arccos(x/r)) * sin(psi)]

note that arccos(x/r) = arcsin(y/r)

Thus, car_x = r * [cos(arccos(x/r)) * cos(psi) + sin(arcsin(y/r)) * sin(psi)] = x*cos(psi) + y*sin(psi)

Similarly, 
car_y = r*sin(arccos(x/r) - psi) = r * [sin(arccos(x/r)) * cos(psi) - cos(arccos(x/r)) * sin(psi)]

car_y = y * cos(psi) - x * sin(psi)

considering the 100ms latency, the state is calculated as follows:

dt = 0.1

x = v * dt

y = 0

psi = -v0 * steer_angle / Lf * dt

v = v0 + throttle * dt

cte = cte0 + v0 * sin(epsi0) * dt

epsi = epsi0 - v0 * steer_angle / Lf * dt

where v0, steer_angle and throttle are fetched directly from j[1], and cte0 and epsi0 are calculated after polyfit (cte0=coeffs[0], epsi0=-atan(coeffs[1])). Note that these calculations are in the car coordinate.

4. Model Predictive Control with Latency

The latency issue is described above. 
