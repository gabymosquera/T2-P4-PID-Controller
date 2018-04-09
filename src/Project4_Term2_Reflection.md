# Self-Driving Car Nanodegree Project 4 - PID Controller

This repository contains all of the code needed to compile and run a PID controller on the Udacity provided Term 2 simulator. The goal for this project is for the PID controller to be created and each of the parameters (Kp, Ki, and Kd) to be tuned to have a car that successfully completes a minimum of one lap around the track. The car shall not leave the drivable portion of the road and/or cause danger if someone was to be in that vehicle and it wasn't just a simulator.

[//]: # (Image References)
[video1]: ./video/PID_Controller_Video_Gaby_Mosquera.mp4

#### Source Code

The src folder contains all of the .cpp and .h files needed to run the project, they are the following:

1. main.cpp
2. PID.cpp
3. PID.h

The starter code provided by Udacity points out several TODOs within main.cpp. Completing these, which can be mostly done by following directions given in the PID Control lesson, is very straight forward. It is also important to complete the PID.cpp file which includes the following functions:

1. Init
2. UpdateError
3. TotalError

#### Running the Code

To run this code please follow the directions in the original link to the project repository below:
[CarND-Controls-PID](https://github.com/udacity/CarND-PID-Control-Project)

## PID & CTE

A PID (Proportional Integral Derivative) controller, as described by Wikipedia, is a control loop feedback mechanism. This control loop feedback is created using the CTE (Cross Track Error) and correcting such based on Proportional, Integral, and Derivative calculations. This correction helps steer a car along the middle of a given path.

### Functions to program a PID

The Init function will initialize the values of the `Kp`, `Kd`, and `Ki` coefficients and the values for the derivative error `d_error`, the proportional error `p_error`, and the integral error `i_error`. The `K` coefficients are vital to the calculation of the total error which happens in the `TotalError` function.

The UpdateError function will go step by step calculating the new controller error given a measurement for the CTE. The steps are the following:
1. Calculate the proportional error, which is just the cte. In this case we just set the variable `p_error = cte` (cte is passed by main.cpp). Refer to line 26 in PID.cpp.
2. Calculate the derivative error, which is just the cte minus the previous cte. In this case we could create a variable to keep track of the previous cte observed, but I decided to follow David's advise from his Q&A video about this project and used `d_error = cte - p_error`. Variable `p_error` will be initialized to 0, but after the first cte is passed to the function the `p_error` becomes the previous cte since `p_error = cte`. As long as we actually calculate `d_error` before `p_error` we can use `p_error` as our "previous_error" tracker. Refer to line 27 in PID.cpp.
3. Calculate the integral error, which is just the sum of ctes. In this case it is a simple `i_error += cte` to keep adding the cte observed to the variable `i_error`. Refer to line 27 in PID.cpp.

Consecutively main.cpp calls the `TotalError` function. This function will now use the errors we just calculated and all 3 `K` coefficients and calculate an overall error using the following formula: `TotalError = Kp * p_error + Ki * i_error + Kd * d_error`. The result is assigned in main.cpp to a `steer_value` variable. This variable is fed into the simulator and is what is able to control the car.

What really makes a PID controller successful is the selection or calculation of the right numbers to be assigned to each coefficient. There are several ways of doing this: manual tuning, twiddle, SGD... But regardless of the method used, the final values aren't just values blindly chosen. They each serve a purpose.

### K Coefficients' Purposes

Starting with the proportional aspect of the PID controller, the Kp coefficient is used to solve the overshooting problem that the PID faces when approching, reaching, and passing the center line of the chosen path and trying to correct back to the center. This causes the car to oscillate back and forth passing the center line each time. So how high or low this value is will determine how strongly the vehicle will turn the wheels to the left or to the right.

Now jumping to the derivative aspect of a PID controller, the Kd coeffient is used to help the controller realize that the cte is actually diminishing over time and prevent that overshooting by steering back before reaching the centerline. Not knowing that the cte is, in fact, diminishing over time will result in overshooting and oscillation and just makes the controller a P controller.

The last coefficient is the integral coefficient. This coeffient is used to determined whether the cte is or is not getting closer to the centerline over a long period of time. It takes care of the bias for the steering of a car. So the higher the number the more bias is assumed for the steering of the particular vehicle.

### Parameter Tunning

I decided to do manual parameter tunning (keeping in mind other methods in case manual didn't get me favorable results). Remembering the lectures and learning more about the topic on how a PID controller works and how the coeffients affect outputs I was able to successfully tune the coefficients manually. 

I tested several different ways. First I ran the code with all of the coefficients equal to zero, this caused the vehicle to overshoot and overcorrect until it lost control and got off road. Then I started testing with different coefficient values (initially I tried values recommended by David in his Q&A video like Kp and Kd = 0.5, and Ki = 0 which improved the car behavior a little bit but still will lose control after a while). It made sense to me that the car was turning to the left and to the right too harshly, therefore I tuned the Kp coefficient to -0.15 and kept the other values as before (Kd = 0.5 and Ki = 0). This provided even more favorable results with less noticeable "jerking" of the vehicle. Now understanding the derivative portion of the control, it also made sense to tune up the value for Kd since this value is what notices that the CTE is actually reducing over time and it makes the car steer before it gets to the centerline and therefore prevents overshooting and reaches that center line much more smoothly. I tuned the Kd parameter to -0.85. For the Ki parameter I did some research and found out in the Udacity forums that the bias for the simulator is assumed to be very, very low, which led me to use a Ki of 0.00001.

Below is a video of the resulting car motion in the Term 2 Simulator using the following K coefficients: Kp = -0.15, Ki = 0.00001, Kd = -0.85.

[Video](./video/PID_Controller_Video_Gaby_Mosquera.mp4)

The car stays within the road lines at all times and completes at least one full lap around the track.
Even though my code has the coefficients hard coded into the main.cpp program (Refer to lines 38, 39, and 40), I used the following code given by David to build the code solution once and be able to run different coefficient values by assigning them through the command line:

```
int main(int argc, char *argv[]){
	//more code here
  	double init_Kp = atof(argv[1]);
  	double init_Ki = atof(argv[2]);
  	double init_Kd = atof(argv[3]);
	//more code here
}
```

And using This in the command line:

```
./pid -.15, 0.00001, -.85
```

This was super helpful and saved me a lot of testing time.