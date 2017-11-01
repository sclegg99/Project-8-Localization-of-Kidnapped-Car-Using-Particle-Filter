/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of
    //   x, y, theta and their uncertainties from GPS) and all weights to 1.
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    
    num_particles = 100; // Number of particles
    
    Particle particle;
    
    double std_x = std[0]; // Define standard deviation of x reading
    double std_y = std[1]; // Define standard deviaiton of y reading
    double std_theta = std[2]; // Define standard deiviaiton of the heading
    
//    default_random_engine gen; // Define random number generator
    random_device rd;  //Will be used to obtain a seed for the random number engine
    mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    normal_distribution<double> dist_x(x, std_x); // Create normal distribution with mean x and std_x
    normal_distribution<double> dist_y(y, std_y); // Create normal distribution with mean y and std_y
    normal_distribution<double> dist_theta(theta, std_theta);   // Create normal distribution
                                                                // with mean theta and std_theta
    
    // Sample from distrubtions x, y and theta to generate the particle locations
    // Let initial particle weights be 1.
    for(int i=0; i<num_particles; i++) {
        particle.id = i;
        particle.x = dist_x(gen);
        particle.y = dist_y(gen);
        particle.theta = dist_theta(gen);
        particle.weight = 1.;
        particles.push_back(particle);
    }
    
    is_initialized = true; // Particle filter is initialized
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    
    double std_x = std_pos[0]; // Define standard deviation of x reading
    double std_y = std_pos[1]; // Define standard deviaiton of y reading
    double std_theta = std_pos[2]; // Define standard deiviaiton of the heading
    
    normal_distribution<double> dist_x(0., std_x); // Create normal distribution with mean 0. and std_x
    normal_distribution<double> dist_y(0., std_y); // Create normal distribution with mean 0. and std_y
    normal_distribution<double> dist_theta(0., std_theta); // Create normal distribution with 0. and std_theta
    
    // Predict new particle location base on current location and heading.
    // Use bicycle model for predicted motion
    // Also add gausian uncertainty to the predicted state.

    if(yaw_rate == 0) {     // yaw_rate is 0
        double vt = velocity*delta_t;
        for(auto& p : particles) {
            p.x += vt*cos(p.theta) + dist_x(gen);
            p.y += vt*sin(p.theta) + dist_y(gen);
            p.theta  = p.theta + dist_theta(gen);
        }
    } else {    // yaw_rate not equal 
        double v2yaw = velocity/yaw_rate;
        double ytdt  = yaw_rate*delta_t;
        for(auto& p : particles) {
            p.x += v2yaw*( sin(p.theta+ytdt) - sin(p.theta)) + dist_x(gen);
            p.y += v2yaw*(-cos(p.theta+ytdt) + cos(p.theta)) + dist_y(gen);
            p.theta  = p.theta + ytdt + dist_theta(gen);
        }
    }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to
    //   implement this method and use it as a helper during the updateWeights phase.
    
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation
    //   3.33
    //   http://planning.cs.uiuc.edu/node99.html
    
    // a few variables to precalculate
    const double bivar_den = 1./(2.*M_PI*std_landmark[0]*std_landmark[1]);
    const double x_den  = 1./(2.*std_landmark[0]*std_landmark[0]);
    const double y_den  = 1./(2.*std_landmark[1]*std_landmark[1]);
    
    // loop through all the particles
    // 1. tranform observed landmarks to world coordinates wrt particle i
    // 2. calcualte euclidian distance from sensed landmark to know map landmark
    // 2. associate nearest landmark (min distance) with particle
    // 3. calculate new weight for particle given sensed distance and landmark associations.
    
    for(auto& p : particles) {
        double xp = p.x;         // particle x location
        double yp = p.y;         // particle y location
        double theta = p.theta;  // particle heading
        double c_theta = cos(theta);
        double s_theta = sin(theta);
        
        // 1. Transform observations from the particle coordiantes into the world coordinates
        vector<LandmarkObs> transformed_observations;         // vector to hold transformed landmark observations
                                                              // wrt particle
        for(auto o : observations) {
            LandmarkObs transformed_observation;
            int id_o   = o.id;    // observed landmark id
            double x_o = o.x;     // observed landmark x location
            double y_o = o.y;     // observed landmark y location
            transformed_observation.id = id_o;
            transformed_observation.x = x_o*c_theta - y_o*s_theta + xp;
            transformed_observation.y = x_o*s_theta + y_o*c_theta + yp;
            transformed_observations.push_back(transformed_observation);
        }
        
        // 2. Calculate euclidian distances and set predicted associations
        //    and predictions of sensed x and y distances from particle to landmark
        vector<int> predicted_landmark;
        vector<double> predicted_sense_x;
        vector<double> predicted_sense_y;
        
        // loop on sensor data transormed world cooridnates wrt particle
        vector<double> distance;    // vector to hold euclidian distance (note: skip sqrt)
        for(auto& xfm_o : transformed_observations) {
            
            distance.clear();
            
            // loop on known map landmarks
            for(auto mp_lm : map_landmarks.landmark_list) {
                double dx = xfm_o.x - mp_lm.x_f;  // difference in x from particle to map
                double dy = xfm_o.y - mp_lm.y_f;  // difference in y from particle to map
                distance.push_back(dx*dx+dy*dy);        // store euclidian distance (squared)
            }
            
            // get index of smallest distance
            int idx = min_element(distance.begin(), distance.end()) - distance.begin();
            
            // if distance is less than sensor_range
            if(sqrt(distance[idx]) < sensor_range) {
                int landmarkID = map_landmarks.landmark_list[idx].id_i;
                predicted_landmark.push_back(landmarkID);  // add map feature id to associations list
                predicted_sense_x.push_back(xfm_o.x);      // add x distance from partical to map feature (world)
                predicted_sense_y.push_back(xfm_o.y);      // add y distance from partical to map feature (world)
            }
        }
        if(predicted_landmark.size() == 0) {
            cout << " No Associations " << endl;
            exit(EXIT_FAILURE);
        }
        SetAssociations(p, predicted_landmark, predicted_sense_x, predicted_sense_y); // set particle associations
        
        // 3. Calculate particle weigth using bivariate normal distribution with covariance = 0.
        //    the total particle weight is the product of the weight for each assocated landmark.
        double particle_weight = 1.0;
        for(int i=0; i<predicted_landmark.size(); i++) {
            int idx = predicted_landmark[i]-1;    // idx is landmark id-1 (landmarks number from one
            float dx = predicted_sense_x[i]-map_landmarks.landmark_list[idx].x_f;  // sensed x - map x
            float dy = predicted_sense_y[i]-map_landmarks.landmark_list[idx].y_f;  // sensed y - map y
            particle_weight *= exp(-(dx*dx*x_den + dy*dy*y_den));           // bivariate distribution
        }
        p.weight = bivar_den*particle_weight;                     // update particle weight
    }
}

void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight.
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    // use resampling wheel algorithm
    vector<Particle> new_particles;                         // vector to hold new samples
    
    int N = particles.size();
    uniform_int_distribution<> ran_i(0, N-1);   // initialize index random variable to uniform dist
    double beta = 0.0;                          // initalize beta variable
    double mw   =-1.0;                          // maximum particle weight (initialize to -1).
    
    // determine maximum particle weight
    for(auto& p : particles) {
        mw = max(p.weight, mw);
    }
    
    // perform wheel algorithm resamling with replacement
    int index = ran_i(gen);
    float mw2 = mw * 2.;
    
    for(int i=0; i<N; i++) {
        beta += ran_i(gen) * mw2;
        while (beta > particles[index].weight) {
            beta -= particles[index].weight;
            index = (index + 1) % N;
        }
        new_particles.push_back(particles[index]); // store new particle sample
    }
    particles = new_particles; // set particles to new_particles
}

Particle ParticleFilter::SetAssociations(Particle &particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    
    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();
    
    particle.associations = associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    
    return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
    vector<int> v = best.associations;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseX(Particle best)
{
    vector<double> v = best.sense_x;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

string ParticleFilter::getSenseY(Particle best)
{
    vector<double> v = best.sense_y;
    stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
