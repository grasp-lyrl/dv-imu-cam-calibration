#pragma once

// todo(giovanni): move inside camera_calibration
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
std::vector<std::vector<std::optional<Eigen::Vector2d>>> computeReprojectionErrors(
	const std::vector<aslam::cameras::GridCalibrationTargetObservation> &observedKeypoints,
	const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> &worldLandmarks,
	const boost::shared_ptr<CameraGeometryType> &cameraGeometry) {
	// Construct vector of vectors of reprojection errors
	std::vector<std::vector<std::optional<Eigen::Vector2d>>> reprojectionErrors;

	// Iterate over all observations of the calibration grid. obs contains all observed 2D keypoints for a given
	// observation of the calibration grid
	for (const auto &obs : observedKeypoints) {
		// Compute the pose transformation from camera to calibration grid (using PnP)
		sm::kinematics::Transformation T_t_c;
		cameraGeometry->estimateTransformation(obs, T_t_c);

		// Compute the inverse transformation from calibration grid to camera
		const auto T_cam_w = T_t_c.inverse();

		// Use the computed transformation to reproject each 3D landmark to the camera frame
		std::vector<std::optional<Eigen::Vector2d>> reprojectionErrorPerGrid;
		for (size_t i = 0; i < worldLandmarks->size(); ++i) {
			// Extract the observed 2D keypoint from the set of observations
			Eigen::Vector2d detectedImagePoint;
			const bool success = obs.imagePoint(i, detectedImagePoint);
			// If no observation exists for the given 3D landmark, add nullopt to reprojection error
			if (!success) {
				reprojectionErrorPerGrid.push_back(std::nullopt);
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
	const std::vector<std::vector<std::optional<Eigen::Vector2d>>> &reprojectionErrors) {
	std::vector<double> reprojectionErrorNorms;
	for (const auto &reprojectionErrorPerGrid : reprojectionErrors) {
		double averageErrorNormPerGrid = 0;
		size_t numObservedCorners      = 0;
		for (const auto &reprojectionError : reprojectionErrorPerGrid) {
			if (reprojectionError.has_value()) {
				averageErrorNormPerGrid += reprojectionError->norm(); // L2 Norm
				++numObservedCorners;
			}
		}
		if (numObservedCorners == 0) {
			throw std::runtime_error("Trying to compute reprojection error for target with no detected keypoints");
		}

		averageErrorNormPerGrid /= static_cast<double>(numObservedCorners);
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
		double diff  = val - mean;
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
	const std::vector<std::vector<std::optional<Eigen::Vector2d>>> &all_rerrs) {
	std::vector<double> xVals, yVals;
	for (const auto &view_rerrs : all_rerrs) {
		if (view_rerrs.empty()) {
			continue;
		}

		for (const auto &rerr : view_rerrs) {
			if (!rerr.has_value()) {
				continue;
			}
			xVals.push_back(rerr->x());
			yVals.push_back(rerr->y());
		}
	}

	const auto [xMean, xStd] = meanStd(xVals);
	const auto [yMean, yStd] = meanStd(yVals);

	return std::make_tuple(Eigen::Vector2d(xMean, yMean), Eigen::Vector2d(xStd, yStd));
}
