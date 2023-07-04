# HeatSeekingNerfTurret
Nerf Turret that automatically tracks warm targets using MLX90640 Thermal Camera
This project was created for UIUC ECE120 Honors Lab credit. It may not be complete, and was uploaded long after work was stopped.

The general algorithm of the turret was as follows:
The primary control board, a Sparkfun Redboard Artemis, ran the "tracktest" program.
It would read data from the MLX90640 camera and drive the turret's rotation through a stepper motor on an Arduino stepper shield. 
If the board does not detect any targets warmer than a specified threshold, it will aimlessly cycle back and forth. 
Once a target is spotted, the board will simply rotate to place the warmest target that it sees at the center of its view. 
This will repeat for 5 cycles, and the the rotational offset and delay required to center the target is stored every time. 
Once 5 cycles have completed, the board will run a quick regression function using basic linear algebra to compute the target's likely future position. 
The (3.3v) Sparkfun board then sends a signal to a second (5v) Arduino board that runs "Sweep-Variant" to sweep its servo that is connected to the Nerf gun's trigger. 
(The Sparkfun board's 3.3v output was insufficient to signal the servo, and no level shifters were on hand at the time.)

If I were to revisit this project, I would redesign the SparkFun board's algorithm to use non-blocking functions or split the work among two boards.
Since both the stepper's rotation and reading the camera's data would block execution, the turret would move very sporadically;
It could not 'see' when it was telling the stepper to rotate the turret, and it could not rotate when it tried to 'see' using the camera. 
The camera unfortunately was a major limiting factor, as its limited resolution and slow serial speed was a massive bottleneck in the algorithm and the turret's practical accuracy. 

A Logic diagram, Images of the turret, and a close-up of the Transistor-Logic automatic loader are also in this repository.
