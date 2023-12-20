#pragma once

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
		distortionModel = dv::camera::DistortionModel::Equidistant;
	}
	if constexpr (std::is_same<DistortionType, aslam::cameras::RadialTangentialDistortion>()) {
		distortionModel = dv::camera::DistortionModel::RadTan;
	}
	else {
		throw std::runtime_error("Unexpected distortion model type provided. Currently, only Radial tangential and "
								 "Equidistant distortion is supported");
	}

	dv::camera::calibrations::CameraCalibration cal(inputName, position, isMaster, resolution,
		cv::Point2f(static_cast<float>(res.projection.at(2)), static_cast<float>(res.projection.at(3))),
		cv::Point2f(static_cast<float>(res.projection.at(0)), static_cast<float>(res.projection.at(1))),
		std::vector<float>(res.distortion.begin(), res.distortion.end()), distortionModel,
		std::vector<float>(floatTransform.data(), floatTransform.data() + 16),
		getCameraCalibrationMetadata(res, patternInfo, comment));

	return cal;
}

/**
 * Compute the reprojection error (in x and y coordinates) between observed 2D keypoints and the expected 3D landmarks
 * for a camera calibration grid. Note that the reprojection error is computed as:
 * error = observed2DKeypoint - reprojected3DLandmark
 * and is therefore signed.
 *
 * @tparam CameraGeometryType Type of camera geometry used for projecting 3D landmarks to 2D keypoints
 * @param observedKeypoints Vector of observed 2D keypoints, where each entry in the vector corresponds to the set
 * observed 2D keypoints for each observation of the calibration grid.
 * @param worldLandmarks Expected 3D landmarks for the calibration grid.
 * @param cameraGeometry Pointer to a CameraGeometry instance describing how 3D landmarks are projected to 2D keypoints.
 * @return Vector of vectors of reprojection errors, containing the reprojection error (in x and y coordinates) for each
 * observed 2D keypoint for each observation of the calibration grid. The first index, therefore, corresponds to each
 * observation of the calibration grid, while the second index corresponds to each observed 2D keypoint in the
 * calibration grid.
 */
template<typename CameraGeometryType>
std::vector<std::vector<Eigen::Vector2d>> computeReprojectionErrors(
	const std::vector<aslam::cameras::GridCalibrationTargetObservation> &observedKeypoints,
	const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> &worldLandmarks,
	const boost::shared_ptr<CameraGeometryType> &cameraGeometry) {
	// Construct vector of vectors of reprojection errors
	std::vector<std::vector<Eigen::Vector2d>> reprojectionErrors;

	// Iterate over all observations of the calibration grid. obs contains all observed 2D keypoints for a given
	// observation of the calibration grid
	for (const auto &obs : observedKeypoints) {
		// Compute the pose transformation from camera to calibration grid (using PnP)
		sm::kinematics::Transformation T_t_c;
		cameraGeometry->estimateTransformation(obs, T_t_c);

		// Compute the inverse transformation from calibration grid to camera
		const auto T_cam_w = T_t_c.inverse();

		// Use the computed transformation to reproject each 3D landmark to the camera frame
		std::vector<Eigen::Vector2d> reprojectionErrorPerGrid;
		for (size_t i = 0; i < worldLandmarks->size(); ++i) {
			// Extract the observed 2D keypoint from the set of observations
			Eigen::Vector2d detectedImagePoint;
			const bool success = obs.imagePoint(i, detectedImagePoint);
			// If no observation exists for the given 3D landmark, skip
			if (!success) {
				continue;
			}

			// Compute the 3D landmark in camera frame
			const Eigen::Vector3d worldPtCamFrame = (T_cam_w * worldLandmarks->point(i));
			const Eigen::Vector4d worldPtHomog    = sm::kinematics::toHomogeneous(worldPtCamFrame);

			// Reproject the 3D landmark to 2D keypoint
			Eigen::Vector2d reprojectedPt;
			cameraGeometry->homogeneousToKeypoint(worldPtHomog, reprojectedPt);

			// Compute the reprojection error (actual - expected)
			const Eigen::Vector2d reprojectionError = detectedImagePoint - reprojectedPt;
			reprojectionErrorPerGrid.push_back(reprojectionError);
		}

		if (!reprojectionErrorPerGrid.empty()) {
			reprojectionErrors.push_back(reprojectionErrorPerGrid);
		}
	}

	return reprojectionErrors;
}

/**
 * Given a vector of vectors of reprojection errors, compute the average reprojection error norm per calibration grid
 * observation.
 * @param reprojectionErrors Vector of vectors of reprojection errors, containing the reprojection error (in x and y
 * coordinates) for each observed 2D keypoint for each observation of the calibration grid.
 * @return Average reprojection error norm (L2 norm) for each observation of the calibration grid.
 */
std::vector<double> computeReprojectionErrorNormsPerGrid(
	const std::vector<std::vector<Eigen::Vector2d>> &reprojectionErrors) {
	std::vector<double> reprojectionErrorNorms;
	for (const auto &reprojectionErrorPerGrid : reprojectionErrors) {
		double averageErrorNormPerGrid = 0;
		for (const auto &reprojectionError : reprojectionErrorPerGrid) {
			averageErrorNormPerGrid += reprojectionError.norm(); // L2 Norm
		}
		averageErrorNormPerGrid /= static_cast<double>(reprojectionErrorPerGrid.size());
		reprojectionErrorNorms.push_back(averageErrorNormPerGrid);
	}
	return reprojectionErrorNorms;
}

/**
 * Compute the mean and standard deviation from a vector of values.
 * @param vals vector of values.
 * @return Computed mean and standard deviation of the given values.
 */
std::tuple<double, double> meanStd(const std::vector<double> &vals) {
	double sum = 0.0;
	for (const auto &val : vals) {
		sum += val;
	}
	double mean = sum / static_cast<double>(vals.size());

	double stdSum = 0.0;
	for (const auto &val : vals) {
		double diff = val - mean;
		stdSum      += diff * diff;
	}
	double std = sqrt(stdSum / (static_cast<double>(vals.size() - 1)));

	return std::make_tuple(mean, std);
}

/**
 * Compute statistics from a vector of vectors of reprojection errors, such as the average reprojection error per x/y
 * coordinate and the standard deviation of the reprojection errors per x/y coordinate.
 * @param all_rerrs Vector of vectors of reprojection errors, containing the reprojection error (in x and y
 * coordinates) for each observed 2D keypoint for each observation of the calibration grid.
 * @return Mean and standard deviation of the reprojection error per x/y coordinates.
 */
std::tuple<Eigen::Vector2d, Eigen::Vector2d> getReprojectionErrorStatistics(
	const std::vector<std::vector<Eigen::Vector2d>> &all_rerrs) {
	std::vector<double> xVals, yVals;
	for (const auto &view_rerrs : all_rerrs) {
		if (view_rerrs.empty()) {
			continue;
		}

		for (const auto &rerr : view_rerrs) {
			xVals.push_back(rerr.x());
			yVals.push_back(rerr.y());
		}
	}

	const auto [xMean, xStd] = meanStd(xVals);
	const auto [yMean, yStd] = meanStd(yVals);

	return std::make_tuple(Eigen::Vector2d(xMean, yMean), Eigen::Vector2d(xStd, yStd));
}
