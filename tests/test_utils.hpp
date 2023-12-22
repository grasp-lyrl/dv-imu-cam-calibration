#pragma once

#include "wrappers//Calibrator.hpp"

namespace fs = boost::filesystem;

int64_t str2int(const std::string &str) {
	char *pEnd;
	return std::strtoll(str.c_str(), &pEnd, 10);
}

template<typename GeometryType, typename DistortionType>
void addImagesToCalibrator(Calibrator<GeometryType, DistortionType> &calibrator,
	const std::vector<fs::path> &imgLeftPaths, const std::vector<fs::path> &imgRightPaths = {}) {
	if (!imgRightPaths.empty()) {
		if (imgLeftPaths.size() != imgRightPaths.size()) {
			throw std::runtime_error("Inconsistent number of images for left and right cameras");
		}
		return;
	}
	for (size_t i = 0; i < imgLeftPaths.size(); i++) {
		CalibratorUtils::StampedImage imageLeft;
		cv::Mat imgLeft = cv::imread(imgLeftPaths[i].string(), cv::IMREAD_GRAYSCALE);
		int64_t tsLeft  = str2int(imgLeftPaths[i].stem().string());
		if (imgLeft.channels() == 3) {
			cv::Mat gray;
			cv::cvtColor(imgLeft, gray, cv::COLOR_BGR2GRAY);
			imageLeft = CalibratorUtils::StampedImage(gray, tsLeft);
		}
		else {
			imageLeft = CalibratorUtils::StampedImage(imgLeft, tsLeft);
		}
		if (!imgRightPaths.empty()) {
			CalibratorUtils::StampedImage imageRight;
			cv::Mat imgRight = cv::imread(imgRightPaths[i].string(), cv::IMREAD_GRAYSCALE);
			int64_t tsRight  = str2int(imgRightPaths[i].stem().string());
			if (imgRight.channels() == 3) {
				cv::Mat gray;
				cv::cvtColor(imgRight, gray, cv::COLOR_BGR2GRAY);
				imageRight = CalibratorUtils::StampedImage(gray, tsLeft);
			}
			else {
				imageRight = CalibratorUtils::StampedImage(imgRight, tsLeft);
			}
			calibrator.addImages({imageLeft, imageRight});
		}
		else {
			calibrator.addImages({imageLeft});
		}
	}
}

template<typename GeometryType, typename DistortionType>
void addImuToCalibrator(Calibrator<GeometryType, DistortionType> &calibrator, const std::vector<fs::path> &imuPaths) {
	for (const auto &path : imuPaths) {
		std::ifstream infile(path.string());
		std::string line;

		std::getline(infile, line);
		int64_t ts = str2int(line);

		std::getline(infile, line);
		double gX = std::stod(line) * M_PI / 180.0;

		std::getline(infile, line);
		double gY = std::stod(line) * M_PI / 180.0;

		std::getline(infile, line);
		double gZ = std::stod(line) * M_PI / 180.0;

		std::getline(infile, line);
		double aX = std::stod(line) * 9.81;

		std::getline(infile, line);
		double aY = std::stod(line) * 9.81;

		std::getline(infile, line);
		double aZ = std::stod(line) * 9.81;

		Eigen::Vector3d gyro(gX, gY, gZ);
		Eigen::Vector3d acc(aX, aY, aZ);
		calibrator.addImu(ts, gyro, acc);
	}
}
