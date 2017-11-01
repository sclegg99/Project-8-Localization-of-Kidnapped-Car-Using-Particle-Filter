# Kidnapped Car Project

## Objective
The objective of this project determine the location of a "kidnapped car" using localization with particle filters.  The initial GPS location of the car is provided plus a map with known localizer features.  An accurate location of the car is determined from the motion of the car plus sensor measurements to the localizer features.

## Code Structure
Particle filters are used to localize the car position.  Hence a ParticleFilter class was constructed for the purpose of localizing the car.

ParticleFilter::init initializes N particles with the GPS x and y position and heading theta.  Random error from a normal distribution is introduced to the x and y positions with variation and heading variation which are provided.

ParticleFilter::Predict predicts the new position of each particle given the particle's current location and heading plus the known car velocity and yaw rate.  A bicycle model is used to predict the new particle location.  Since there is some measurement error associated with the velocity and yaw rates, the predicted position and heading has a random error (normally distributed) added to represent the velocity and yaw rate measurement errors.  The predict module takes into account the possibility that the yaw rate may be zero and includes this special case of the generalized bicycle model.

ParticleFilter::Update updates the estimated position as follows:
1. Transform observed landmarks to world coordinates withe respect to particle i.
2. Calculate euclidian distance from sensed landmark to know map landmark.  The associate nearest landmark (min distance) with that particle sensor reading.
3. Calculate new weight for particle given sensed position in world coordinates and the known landmark position.  The new weight is the product of the bivariate distribution (with no covariance) for all the sensed landmarks for the given particle.  It should be noted that a limit to the sensor range thus any particle to landmark distance that exceeded the sensor range was neglected.

The calculation of the landmark associations was performed using a "brut force" algorithm.  That is, the distance from all the sensed landmarks to all the known landmarks was calculated.  This is an O(N*N) operation where N is the number of landmarks.  A more efficient method could have been used such as kd Trees (which is O(N)) but given the sparsity of the landmarks it was decided to opt for the brut force method.

ParticleFilter::resample resampled the particles with replacement based on the particle weight calculated in the update step.

## Evaluation
The program was evaluated using a class suppled simulator.  The simulator provide the program with the car motion and sensor readings.  The simulator calculated the position and heading errors for each time step.  If the final simulator error was below a predefined level and the system time to complete the course the car traveled was also below a predefined level, then the project graded a success.

One hundred (100) particles were used for this project submission.  This was a sufficient number of particles to achieve success as shown in the figure below.

![Successful Completion of the Car Localization](figures/FinalResults.png

Ranges from 10 to 200 particles were also tested and graded as successful.  There was a small improvement in the position and heading errors as the number of particles increased.  However, as the number of particles increased the computational time increased.
