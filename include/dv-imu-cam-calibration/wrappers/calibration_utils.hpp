#pragma once

#include "../imu_camera_calibration/imu_params.hpp"

#include <dv-processing/camera/calibrations/camera_calibration.hpp>

#include <Eigen/Eigen>
#include <ostream>
#include <string>

struct PatternInfo {
	std::string mName;
	cv::Size mShape;
	float mTagSize;
	float mTagSpacing;

	PatternInfo(std::string name, cv::Size shape, float tagSize, float tagSpacing) :
		mName(name),
		mShape(shape),
		mTagSize(tagSize),
		mTagSpacing(tagSpacing) {
	}
};

// todo(giovanni): replace with dvp  (dv::camera::DistortionModel)
namespace ImuCamModelTypes {
const std::string RADTAN      = "Pinhole-RadialTangential";
const std::string EQUIDISTANT = "Pinhole-Equidistant";
} // namespace ImuCamModelTypes

namespace QualityStrings {
const std::string EXCELLENT = "excellent";
const std::string GOOD      = "good";
const std::string POOR      = "poor";
const std::string BAD       = "bad";
} // namespace QualityStrings

// todo(giovanni): remove namespace and move to repro_error_utils
namespace CameraCalibrationUtils {
struct ErrorInfo {
	// Mean of the reprojection error per x/y coordinate
	Eigen::Vector2d mean;

	// Standard deviation of the reprojection error per x/y coordinate
	Eigen::Vector2d std;

	// Mean of the reprojection error norms per observation of the calibration grid
	double errorNormMean;

	// Standard deviation of the reprojection error norms per observation of the calibration grid
	double errorNormStd;

	ErrorInfo(const Eigen::Vector2d &_mean = Eigen::Vector2d(-1, -1),
		const Eigen::Vector2d &_std = Eigen::Vector2d(-1, -1), const double _errorNormMean = -1,
		const double _errorNormStd = -1) :
		mean(_mean),
		std(_std),
		errorNormMean(_errorNormMean),
		errorNormStd(_errorNormStd) {
	}
};

struct CalibrationResult {
	const std::vector<double> projection;
	const std::vector<double> distortion;
	const ErrorInfo err_info;
	Eigen::Matrix4d baseline;

	CalibrationResult(const std::vector<double> &_projection, const std::vector<double> &_distortion,
		const ErrorInfo &_err_info, const Eigen::Matrix4d &_baseline) :
		projection(_projection),
		distortion(_distortion),
		err_info(_err_info),
		baseline(_baseline) {
	}
};

static void printResult(const CameraCalibrationUtils::CalibrationResult &result, std::ostream &ss) {
	ss << "Intrinsic calibration results:" << std::endl;
	ss << "\n  projection: \n\t\t";
	for (const auto val : result.projection) {
		ss << val << " ";
	}
	ss << std::endl;
	ss << "\n  distortion: \n\t\t";
	for (const auto val : result.distortion) {
		ss << val << " ";
	}
	ss << std::endl;

	ss << "\n  reprojection error: \n\t\t[" << result.err_info.mean.x() << ", " << result.err_info.mean.y() << "] +- ["
	   << result.err_info.std.x() << ", " << result.err_info.std.y() << "]" << std::endl;

	ss << "\n  baseline: \n" << result.baseline.block<1, 4>(0, 0) << std::endl;
	ss << result.baseline.block<1, 4>(1, 0) << std::endl;
	ss << result.baseline.block<1, 4>(2, 0) << std::endl;
	ss << result.baseline.block<1, 4>(3, 0) << std::endl;
}

} // namespace CameraCalibrationUtils

std::string getCalibrationQuality(float std) {
	std::string quality;
	if (std < 0.1) {
		quality = QualityStrings::EXCELLENT;
	}
	else if (std < 0.4) {
		quality = QualityStrings::GOOD;
	}
	else if (std < 0.5) {
		quality = QualityStrings::POOR;
	}
	else {
		quality = QualityStrings::BAD;
	}

	return quality;
}

void drawQuality(cv::Mat &image, const std::string &quality) {
	cv::Scalar color;
	if (quality == QualityStrings::EXCELLENT) {
		color = cv::Scalar(0, 255, 0);
	}
	else if (quality == QualityStrings::GOOD) {
		color = cv::Scalar(145, 255, 0);
	}
	else if (quality == QualityStrings::POOR) {
		color = cv::Scalar(145, 0, 255);
	}
	else {
		color = cv::Scalar(0, 0, 255);
	}
	cv::putText(image, fmt::format("Quality: {0}", quality), cv::Point(20, image.rows - 20), cv::FONT_HERSHEY_DUPLEX,
		1.0, color, 2);
}

dv::camera::calibrations::CameraCalibration::Metadata getCameraCalibrationMetadata(
	const CameraCalibrationUtils::CalibrationResult &intrinsicResult, const PatternInfo &pattern,
	std::optional<std::string> comment = std::nullopt, int64_t timestamp = 0) {
	auto error         = intrinsicResult.err_info.errorNormMean;
	const auto quality = getCalibrationQuality(error);

	if (!comment.has_value()) {
		comment = fmt::format(
			"Reprojected average RMS reprojection error: {}, deviation: {}. Reprojection error mean: [{}, {}], "
			"reprojection error std: [{}, {}]",
			intrinsicResult.err_info.errorNormMean, intrinsicResult.err_info.errorNormStd,
			intrinsicResult.err_info.mean.x(), intrinsicResult.err_info.mean.y(), intrinsicResult.err_info.std.x(),
			intrinsicResult.err_info.std.y());
	}

	return dv::camera::calibrations::CameraCalibration::Metadata(pattern.mShape, pattern.mShape, pattern.mName,
		pattern.mTagSize, pattern.mTagSpacing, error, std::to_string(timestamp), quality, comment.value(),
		std::nullopt);
}

template<typename DistortionType>
dv::camera::calibrations::CameraCalibration getIntrinsicCalibrationData(
	const CameraCalibrationUtils::CalibrationResult &res, PatternInfo &patternInfo, const std::string &position,
	const std::string &inputName, const std::optional<std::string> &comment = std::nullopt,
	cv::Size resolution = cv::Size(640, 480), bool isMaster = true) {
	const Eigen::Matrix<float, 4, 4, Eigen::RowMajor> floatTransform = res.baseline.cast<float>().eval();

	dv::camera::DistortionModel distortionModel;
	if constexpr (std::is_same<DistortionType, aslam::cameras::EquidistantDistortion>()) {
		distortionModel = dv::camera::DistortionModel::EQUIDISTANT;
	}
	if constexpr (std::is_same<DistortionType, aslam::cameras::RadialTangentialDistortion>()) {
		distortionModel = dv::camera::DistortionModel::RADIAL_TANGENTIAL;
	}
	else {
		throw std::runtime_error("Unexpected distortion model type provided. Currently, only Radial tangential and "
								 "Equidistant distortion is supported");
	}

	dv::camera::calibrations::CameraCalibration cal(inputName, position, isMaster, resolution,
		cv::Point2f(static_cast<float>(res.projection.at(2)), static_cast<float>(res.projection.at(3))),
		cv::Point2f(static_cast<float>(res.projection.at(0)), static_cast<float>(res.projection.at(1))),
		std::vector<float>(res.distortion.begin(), res.distortion.end()), distortionModel,
		dv::kinematics::Transformationf{0, floatTransform}, getCameraCalibrationMetadata(res, patternInfo, comment));

	return cal;
}

namespace CalibratorUtils {
double toSec(const int64_t time) {
	return static_cast<double>(time) / 1e6;
}

/**
 * Hold image and corresponding timestamp.
 */
struct StampedImage {
	cv::Mat image;
	int64_t timestamp;

	StampedImage() {};

	StampedImage(cv::Mat img, const int64_t ts) : image(std::move(img)), timestamp(ts) {};

	/**
	 * Clone the underlying image.
	 *
	 * @return
	 */
	StampedImage clone() const {
		StampedImage clone;
		clone.image     = image.clone();
		clone.timestamp = timestamp;
		return clone;
	}
};

enum PatternType {
	CHESSBOARD,
	ASYMMETRIC_CIRCLES_GRID,
	APRIL_GRID
};

enum State {
	INITIALIZED,
	COLLECTING,
	COLLECTED,
	CALIBRATING,
	CALIBRATED
};

struct Options {
	// todo(giovanni): use PatternInfo from utils.hpp
	// Calibration pattern
	size_t rows           = 11;
	size_t cols           = 4;
	double spacingMeters  = 0.05;
	double patternSpacing = 0.3;
	PatternType pattern   = PatternType::ASYMMETRIC_CIRCLES_GRID;

	// Optimization problem
	size_t maxIter       = 20;
	bool timeCalibration = true;

	// IMU
	ImuParameters imuParameters;

	// Camera
	struct CameraInits {
		std::vector<double> intrinsics;
		std::vector<double> distCoeffs;
		cv::Size imageSize;
	};

	std::vector<CameraInits> cameraInitialSettings;
};

StampedImage previewImageWithText(
	const std::string &text, const int64_t timestamp = 0LL, const cv::Size &size = cv::Size(640, 480)) {
	cv::Mat img = cv::Mat::zeros(size, CV_8UC3);

	cv::putText(img, text, cv::Point(size.width / 8, size.height / 2), cv::FONT_HERSHEY_DUPLEX, 1.0,
		cv::Scalar(255, 255, 255), 2);

	return {img, timestamp};
}

} // namespace CalibratorUtils

struct CameraCalibrationInfo {
	int numImagesTotal;
	int numImagesUsed;
	int numCornerOutliers;
};
