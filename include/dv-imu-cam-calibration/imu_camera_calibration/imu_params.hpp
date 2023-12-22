#pragma once

#include <vector>

struct ImuParameters {
	double updateRate      = 200.0;
	double accNoiseDensity = 1.4e-3;
	double accRandomWalk   = 8.6e-5;
	double gyrNoiseDensity = 8.0e-5;
	double gyrRandomWalk   = 2.2e-6;

	std::tuple<double, double, double> getAccelerometerStatistics() const {
		double accelUncertaintyDiscrete = accNoiseDensity / sqrt(1.0 / updateRate);
		return std::make_tuple(accelUncertaintyDiscrete, accRandomWalk, accNoiseDensity);
	}

	std::tuple<double, double, double> getGyroStatistics() const {
		double gyroUncertaintyDiscrete = gyrNoiseDensity / sqrt(1.0 / updateRate);
		return std::make_tuple(gyroUncertaintyDiscrete, gyrRandomWalk, gyrNoiseDensity);
	}
};
