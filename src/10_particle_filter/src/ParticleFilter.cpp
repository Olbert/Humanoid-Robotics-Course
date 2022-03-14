#include <particle_filter/ParticleFilter.h>
#include <iostream>
#include <cstdlib>
#include <cmath>

namespace particle_filter {

/**
 * \brief Calculate the probability phi(d, stdev) of a measurement according to a Gaussian distribution.
 * \param[in] d The difference between the measurement and the mean
 * \param[in] stdev The standard deviation of the Gaussian.
 * \return Probability of the measurement.
 */
double ParticleFilter::gaussianProbability(const double& d, const double& stdev) {
	double probability = 0.0;
	/*TODO: Calculate the probability of the measurement for a Gaussian distribution with
	  the given mean and standard deviation */

	probability = exp(-(d*d)/(2*stdev*stdev))/(sqrt(2*M_PI)*stdev);

	return probability;
}

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

/**
 * \brief Draw a sample from a Gaussian distribution.
 * \param[in] mean The mean of the Gaussian.
 * \param[in] stdev The standard deviation of the Gaussian.
 * \return A random sample drawn from the given Gaussian distribution.
 */
double ParticleFilter::sampleFromGaussian(const double& mean, const double& stdev) {
	double result = 0.0;
	//TODO: draw a sample from a 1D Gaussian

	double rand = 0;
	for(int i=0; i<12; i++)
		rand += fRand(-stdev, stdev);

	result = mean + rand/2.;

	return result;
}


/**
 * \brief Initializes the position and weights of the particles.
 * \param[in,out] particles The list of particles.
 *
 * The positions should be distributed uniformly in the interval [0, 10].
 * The weights should be equal and sum up to 1.
 */
void ParticleFilter::initParticles(std::vector<Particle>& particles) {
	//TODO: Distribute the particles randomly between [0, 10] with equal weights

	for(int i=0; i<particles.size(); i++)
	{
		particles[i].x = fRand(0, 10);
		particles[i].weight = 1./particles.size();
	}
}

/**
 * \brief Normalizes the weights of the particle set so that they sum up to 1.
 * \param[in,out] particles The list of particles.
 */
void ParticleFilter::normalizeWeights(std::vector<Particle>& particles) {
	//TODO: normalize the particles' weights so that they sum up to 1.
	double sum = 0.;
	for(int i=0; i<particles.size(); i++)
		sum += particles[i].weight;
	for(int i=0; i<particles.size(); i++)
		particles[i].weight /= sum;
}

/**
 * \brief Displace the particles according to the robot's movements.
 * \param[in,out] particles The list of particles.
 * \param[in] ux The odometry (displacement) of the robot along the x axis.
 * \param[in] stdev The standard deviation of the motion model.
 */
void ParticleFilter::integrateMotion(std::vector<Particle>& particles, const double& ux, const double& stdev) {
	//TODO: Prediction step: Update each sample by drawing the a pose from the motion model.
	for(int i=0; i<particles.size(); i++)
		particles[i].x += sampleFromGaussian(ux, stdev);
}

double min(double a, double b)
{
	return a < b ? a : b;
}

/**
 * \brief Returns the distance between the given x position and the nearest light source.
 * \param[in] x The position on the x axis.
 * \return The distance to the nearest light source.
 */
double ParticleFilter::getDistanceToNearestLight(const double& x) {
	double dist = 10.0;
	//TODO Return the distance from the robot's position x to the nearest light source. (2, 6, 8)

	dist = min(dist, fabs(2.-x));
	dist = min(dist, fabs(6.-x));
	dist = min(dist, fabs(8.-x));

	return dist;
}

/**
 * \brief Updates the particle weights according to the measured distance to the nearest light source.
 * \param[in,out] particles The list of particles.
 * \param[in] measurement The measured distance between the robot and the nearest light source.
 * \param[in] stdev The standard deviation of the observation model.
 */
void ParticleFilter::integrateObservation(std::vector<Particle>& particles, const double measurement,
		const double& stdev) {
	//TODO: Correction step: weight the samples according to the observation model.

	for(int i=0; i<particles.size(); i++)
	{
		particles[i].weight *= gaussianProbability(getDistanceToNearestLight(particles[i].x) - measurement, stdev);
	}

	// Normalize the weights after updating so that they sum up to 1 again:
	normalizeWeights(particles);
}

/**
 * \brief Resamples the particle set by throwing out unlikely particles and duplicating more likely ones.
 * \param[in] particles The old list of particles.
 * \return The new list of particles after resampling.
 */
std::vector<ParticleFilter::Particle> ParticleFilter::resample(const std::vector<Particle>& particles) {
	std::vector<Particle> newParticles;
	/*TODO: Use stochastic universal resampling (also called low variance resampling)
	 * to draw a new set of particles according to the old particles' weights */

	int J = particles.size();
	double r = fRand(0., 1./J);
	double c = particles[0].weight;
	int i = 0;

	for(int j=1; j<J+1; j++)
	{
		double U = r + (j-1)*(1./J);
		while(U > c)
		{
			i++;
			c += particles[i].weight;
		}
		newParticles.push_back(particles[i]);
	}

	return newParticles;
}

}  // namespace particle_filter

