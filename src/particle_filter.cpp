/**
 * particle_filter.cpp
 *
 * Created on: Nov 4th, 2018
 * Author: Gabriel Oh
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <cfloat>
#include <cassert>

#include "particle_filter.h"
#include "helper_functions.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Set the number of particles
  // TODO: Find appropriate value
  num_particles = 5;

  // Normal distributions for x, y, and theta
  default_random_engine generator;
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; ++i) {
    Particle p;
    p.id = i;
    p.x = dist_x(generator);
    p.y = dist_y(generator);
    p.theta = dist_theta(generator);
    p.weight = 1.0;
    particles.push_back(p);
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // Normal distributions for x, y, and theta
  default_random_engine generator;
  normal_distribution<double> dist_x(0.0, std_pos[0]);
  normal_distribution<double> dist_y(0.0, std_pos[1]);
  normal_distribution<double> dist_theta(0.0, std_pos[2]);

  // Predict the position and heading of each particle for delta_t time elapse (prior)
  for (Particle &p: particles) {
    if (yaw_rate == 0.0) {
      p.x = p.x + velocity * cos(p.theta) * delta_t;
      p.y = p.y + velocity * sin(p.theta) * delta_t;
    } else {
      p.x = p.x + (velocity / yaw_rate) * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
      p.y = p.y + (velocity / yaw_rate) * (-cos(p.theta + yaw_rate * delta_t) + cos(p.theta));
      p.theta = p.theta + yaw_rate * delta_t;
    }

    // Add random noises
    p.x += dist_x(generator);
    p.y += dist_y(generator);
    p.theta += dist_theta(generator);

    // Normalize theta to [-PI, +PI)
    p.theta = normalize_angle(p.theta);
  }
}

Map::single_landmark_s ParticleFilter::dataAssociation(LandmarkObs &ob_predicted, const Map &map_landmarks) {
  // Find a map landmark that has the smallest distance with observation prediction
  assert(map_landmarks.landmark_list.size() > 0);
  double min_distance2 = DBL_MAX;
  Map::single_landmark_s lm_assoc;
  for (int i = 0; i < map_landmarks.landmark_list.size(); ++i) {
    Map::single_landmark_s lm = map_landmarks.landmark_list.at(i);
    double d_x = ob_predicted.x - lm.x_f;
    double d_y = ob_predicted.y - lm.y_f;
    double d2 =  d_x * d_x + d_y * d_y;
    if (d2 < min_distance2) {
      min_distance2 = d2;
      lm_assoc = lm;
    }
  }
  return lm_assoc;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

  // Skip the update when either observations or landmarks are empty
  if (observations.size() <= 0 || map_landmarks.landmark_list.size() <= 0)
    return;

  // Update the weights of each particle by associating observations with map landmarks
  for (Particle &p: particles) {
    double weight = 1.0;
    for (LandmarkObs ob: observations) {
      // Transform the observation in particle perspective to map coordinates
      LandmarkObs ob_m;
      ob_m.x = ob.x * cos(p.theta) - ob.y * sin(p.theta) + p.x;
      ob_m.y = ob.x * sin(p.theta) + ob.y * cos(p.theta) + p.y;

      // Find a landmark element that associates with the observation and compute its probability
      Map::single_landmark_s lm_assoc = dataAssociation(ob_m, map_landmarks);
      // NOTE: Since distance of the landmark and observation is important, the parameter order does not matter.
      // The associated landmark as a variable and the predicted observation as a mean because we are evaluating
      // the probability of the landmark association given the observation predictions.
      double assoc_prob = multivariate_normal_distribute({lm_assoc.x_f, lm_assoc.y_f}, {ob_m.x, ob_m.y},
                                                         {std_landmark[0], std_landmark[1]});
      weight *= assoc_prob;
    }
    p.weight = weight;
  }
}

void ParticleFilter::resample() {
  // TODO: Resample particles with replacement with probability proportional to their weight.
  // NOTE: You may find std::discrete_distribution helpful here.
  //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle &particle, const std::vector<int> &associations,
                                         const std::vector<double> &sense_x, const std::vector<double> &sense_y) {
  //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
