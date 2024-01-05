#pragma once

#include <aslam/calibration/core/IncrementalEstimator.h>

#include "../wrappers/calibration_utils.hpp"
#include "kalibr_calibration_target_optimization_problem.hpp"
#include "kalibr_optimization_diverged.hpp"

/**
 * Kalibr corresponding class in CameraCalibrator.py
 * @tparam CameraGeometryType
 * @tparam DistortionType
 */
template<typename CameraGeometryType, typename DistortionType>
class CameraCalibration {
private:
	std::vector<boost::shared_ptr<CameraGeometry<CameraGeometryType, DistortionType>>> cameras;
	bool estimateLandmarks;
	bool useBlakeZissermanMest;
	boost::shared_ptr<aslam::calibration::IncrementalEstimator> estimator = nullptr;
	boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target   = nullptr;
	std::vector<boost::shared_ptr<CalibrationTargetOptimizationProblem<CameraGeometryType, DistortionType>>> views;
	std::vector<boost::shared_ptr<sm::kinematics::Transformation>> baselines;
	std::vector<std::pair<boost::shared_ptr<aslam::backend::RotationQuaternion>,
		boost::shared_ptr<aslam::backend::EuclideanPoint>>>
		dv_baselines;

public:
	CameraCalibration(const std::vector<boost::shared_ptr<CameraGeometry<CameraGeometryType, DistortionType>>> &cams,
		const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> &_target,
		const std::vector<boost::shared_ptr<sm::kinematics::Transformation>> &_baselines, bool _estimateLandmarks,
		bool _useBlakeZissermanMest) :
		cameras(cams),
		target(_target),
		baselines(_baselines),
		estimateLandmarks(_estimateLandmarks),
		useBlakeZissermanMest(_useBlakeZissermanMest) {
		static constexpr bool verbose = false;

		// create the incremental estimator and set options
		estimator = boost::make_shared<aslam::calibration::IncrementalEstimator>(CALIBRATION_GROUP_ID);
		estimator->getOptions().infoGainDelta = 0.2;
		estimator->getOptions().checkValidity = true;
		estimator->getOptions().verbose       = verbose;

		estimator->getLinearSolverOptions().columnScaling = true;
		estimator->getLinearSolverOptions().verbose       = verbose;
		estimator->getLinearSolverOptions().epsSVD        = 1e-6;

		estimator->getOptimizerOptions().maxIterations = 50;
		estimator->getOptimizerOptions().verbose       = verbose;

		inititializeBaselines();
	}

	void inititializeBaselines() {
		for (const auto &baseline : baselines) {
			auto dv_T_q = boost::make_shared<aslam::backend::RotationQuaternion>(baseline->q());
			auto dv_T_t = boost::make_shared<aslam::backend::EuclideanPoint>(baseline->t());
			dv_baselines.emplace_back(dv_T_q, dv_T_t);
		}
	}

	bool addTargetView(
		const std::map<size_t, aslam::cameras::GridCalibrationTargetObservation> &observations, bool force = false) {
		// Find observation with most points visible and use that target-camera transform estimation
		// as T_tc_guess
		if (observations.empty()) {
			std::cout << "observations is empty" << std::endl;
		}
		auto obsWithMostPoints
			= std::max_element(observations.begin(), observations.end(), [](const auto &a, const auto &b) {
				  std::vector<unsigned int> aIdx, bIdx;
				  a.second.getCornersIdx(aIdx);
				  b.second.getCornersIdx(bIdx);
				  return aIdx.size() > bIdx.size();
			  });
		auto T_tc_guess = obsWithMostPoints->second.T_t_c();

		if (cameras.empty()) {
			std::cout << "Cameras empty..." << std::endl;
		}
		if (target == nullptr) {
			std::cout << "Target empty..." << std::endl;
		}
		if (dv_baselines.empty()) {
			std::cout << "dv_baselines empty..." << std::endl;
		}
		//        if(T_tc_guess)
		if (observations.empty()) {
			std::cout << "observations empty..." << std::endl;
		}

		auto batch_problem
			= boost::make_shared<CalibrationTargetOptimizationProblem<CameraGeometryType, DistortionType>>(
				cameras, target, dv_baselines, T_tc_guess, observations, estimateLandmarks, useBlakeZissermanMest);

		if (estimator == nullptr) {
			std::cout << "estimator is null" << std::endl;
		}
		if (batch_problem == nullptr) {
			std::cout << "batch_problem is null" << std::endl;
		}

		auto estimator_return_value = estimator->addBatch(batch_problem, force);

		// TODO: add condition if optimization diverges
		//		if (estimator_return_value.numIterations >= estimator->getOptimizerOptions().maxIterations) {
		//			throw OptimizationDiverged("");
		//		}

		bool success = estimator_return_value.batchAccepted;
		if (success) {
			views.push_back(batch_problem);
		}

		return success;
	}

	size_t getNumBatches() {
		return estimator->getNumBatches();
	}

	size_t getNumCorners(const size_t batch_id, const size_t cameraId) {
		const auto obs = views.at(batch_id)->rig_observations.at(cameraId);

		std::vector<unsigned int> cornerIndices;
		obs.getCornersIdx(cornerIndices);
		return cornerIndices.size();
	}

	void removeBatch(const size_t batch_id) {
		estimator->removeBatch(batch_id);
		views.erase(views.begin() + batch_id);
	}

	bool replaceBatch(const size_t batch_id,
		boost::shared_ptr<CalibrationTargetOptimizationProblem<CameraGeometryType, DistortionType>> new_batch) {
		estimator->removeBatch(views.at(batch_id));
		views.at(batch_id) = new_batch;

		const auto rval = estimator->addBatch(new_batch, false);

		// queue the batch for removal if the corrected batch was rejected
		if (!rval.batchAccepted) {
			views.erase(views.begin() + batch_id);
		}

		return rval.batchAccepted;
	}

	std::vector<aslam::cameras::GridCalibrationTargetObservation> getObservations(const size_t cameraId) {
		std::vector<aslam::cameras::GridCalibrationTargetObservation> obsList;
		for (const auto &view : views) {
			const auto &obs = view->rig_observations[cameraId];
			obsList.push_back(obs);
		}
		return obsList;
	}

	std::vector<std::vector<std::optional<Eigen::Vector2d>>> getReprojectionErrors(const size_t cameraId) {
		std::vector<std::vector<std::optional<Eigen::Vector2d>>> reprojectionErrorAllViews;
		for (auto &view : views) {
			std::vector<std::optional<Eigen::Vector2d>> reprojectionErrorPerGrid;
			for (const auto &rerr : view->rerrs[cameraId]) {
				// Note: corners not observed are populated with nullptr in CalibrationOptimizationProblem
				if (!rerr) {
					// Observation does not exist
					reprojectionErrorPerGrid.push_back(std::nullopt);
					continue;
				}

				// add if the corners were observed
				Eigen::Vector2d corner, reprojection;
				corner       = rerr->getMeasurement();
				reprojection = rerr->getPredictedMeasurement();

				const Eigen::Vector2d reprojectionError = corner - reprojection;
				reprojectionErrorPerGrid.push_back(reprojectionError);
			}

			reprojectionErrorAllViews.push_back(reprojectionErrorPerGrid);
		}
		return reprojectionErrorAllViews;
	}

	size_t nOfViews() {
		return views.size();
	}

	boost::shared_ptr<CalibrationTargetOptimizationProblem<CameraGeometryType, DistortionType>> removeCornersFromBatch(
		const size_t batch_id, const size_t cameraId, const std::vector<size_t> &cornerIdList,
		const bool useBlakeZissermanMest) {
		//        std::cout << "Views size: " << views.size() << ", batch id : " << batch_id << std::endl;

		auto &batch = views.at(batch_id);

		// disable the corners
		bool hasCornerRemoved = false;
		try {
			for (const size_t cornerId : cornerIdList) {
				batch->rig_observations.at(cameraId).removeImagePoint(cornerId);
				hasCornerRemoved = true;
			}
		}
		catch (std::exception &ex) {
			std::cout << ex.what() << std::endl;
		}
		assert(hasCornerRemoved);

		// rebuild problem
		auto new_problem = boost::make_shared<CalibrationTargetOptimizationProblem<CameraGeometryType, DistortionType>>(
			batch->cameras, batch->target, batch->baselines, batch->T_tc_guess, batch->rig_observations,
			estimateLandmarks, useBlakeZissermanMest);
		return new_problem;
	}

	CameraCalibrationUtils::CalibrationResult getResult(const size_t cameraId) {
		const auto projectionMat = cameras[cameraId]->getDv()->projectionDesignVariable()->getParameters();
		assert(projectionMat.rows() == 4);
		assert(projectionMat.cols() == 1);
		std::vector<double> projection;
		projection.push_back(projectionMat(0, 0));
		projection.push_back(projectionMat(1, 0));
		projection.push_back(projectionMat(2, 0));
		projection.push_back(projectionMat(3, 0));

		const auto distortionMat = cameras[cameraId]->getDv()->distortionDesignVariable()->getParameters();
		if constexpr (std::is_same<CameraGeometryType, aslam::cameras::EquidistantDistortedPinholeCameraGeometry>()
					  && std::is_same<DistortionType, aslam::cameras::EquidistantDistortion>()) {
			if (distortionMat.rows() != 4) {
				throw std::runtime_error("Four params are expected for Equidistant model.");
			}
		}
		if constexpr (std::is_same<CameraGeometryType, aslam::cameras::DistortedPinholeCameraGeometry>()
					  && std::is_same<DistortionType, aslam::cameras::RadialTangentialDistortion>()) {
			if (distortionMat.rows() != 4) {
				throw std::runtime_error("Four params are expected for RadialTangential model.");
			}
		}
		if constexpr (std::is_same<CameraGeometryType, aslam::cameras::FovDistortedPinholeCameraGeometry>()
					  && std::is_same<DistortionType, aslam::cameras::FovDistortion>()) {
			if (distortionMat.rows() != 1) {
				std::cout << "One param only is expected for Fov model." << std::endl;
				throw std::runtime_error("One param only is expected for Fov model.");
			}
		}
		assert(distortionMat.cols() == 1);
		std::vector<double> distortion;
		if constexpr (std::is_same<CameraGeometryType, aslam::cameras::FovDistortedPinholeCameraGeometry>()
					  && std::is_same<DistortionType, aslam::cameras::FovDistortion>()) {
			distortion.push_back(distortionMat(0, 0));
		}
		else {
			distortion.push_back(distortionMat(0, 0));
			distortion.push_back(distortionMat(1, 0));
			distortion.push_back(distortionMat(2, 0));
			distortion.push_back(distortionMat(3, 0));
		}

		// reproj error statistics
		CameraCalibrationUtils::ErrorInfo err_info;

		const auto reprojectionErrors = getReprojectionErrors(cameraId);
		if (!reprojectionErrors.empty()) {
			const auto [me, std]                     = getReprojectionErrorStatistics(reprojectionErrors);
			const auto reprojectionErrorNorms        = computeReprojectionErrorNormsPerGrid(reprojectionErrors);
			const auto [errorNormMean, errorNormStd] = meanStd(reprojectionErrorNorms);
			err_info = CameraCalibrationUtils::ErrorInfo(me, std, errorNormMean, errorNormStd);
		}

		Eigen::Matrix4d baseline   = Eigen::Matrix4d::Identity();
		baseline.block<3, 3>(0, 0) = dv_baselines[cameraId].first->toRotationMatrix();
		baseline.block<3, 1>(0, 3) = dv_baselines[cameraId].second->toEuclidean();

		return {projection, distortion, err_info, baseline};
	}
};
