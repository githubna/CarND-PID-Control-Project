# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Code Style

Follow [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) by using a code formatter inside Atom editor.

## Project Instructions and Rubric

### Implementation

* CMakeList.txt is modified so the compiler can find the local installed openssl. The macport installs all third-party libs under /opt/local by default.
* An extra argument for PID::Init() function is added to indicate whether to tune PID parameters using twiddle algorithm. By default, it is set to be false.
* The implementation of twiddle algorithms closely follows what is taught in the class. Before to take a new twiddle action, I wait 100 iterations to let the current parameters settle down in the system and then sum the error for the next 100 steps as the twiddling criteria for the next iteration. A small video of the car simulation is included which demonstrates the effectiveness of the selected parameters.

### Rubic points

* The proportional(P) term is proportional to the current error. Using the proportional term alone will result in ripple effects around setpoints.
* The integral(I) term takes accounts past errors and integrates them over time. In our context, it can be applied to balance out a constant bias in CTE. However, for this specific simulation, I noticed even a very small integral term can send car off the track. Therefore, the integral term is set be zero for this project.
* The derivative(D) term is proportional to the rate change of the error. It is sometimes called damping term which can remove the ripple effects caused by the P controller.

### Parameters tuning

I start the tuning by setting p to be {P: 0.0, I: 0.0, D: 0.0} and dp to be {1.0, 1.0, 1.0}. Not surprised, the car is off the track pretty quick. I think it is mainly caused by the integral term. So I decide to manually tune them first to get some rough estimates ({0.15, 0.0, 3.0}). Then I set these rough estimates as the initial guess for the automatic twiddling algorithm. The twiddle process doest not stop until the sum of dp is less than 0.002. It takes around 10 minutes to finish the process. The final parameters is {0.148897, 0, 2.991} with a mean square error of 0.0132319.
