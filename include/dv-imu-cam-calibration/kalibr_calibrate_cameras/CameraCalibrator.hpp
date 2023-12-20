#pragma once

#include "kalibr_imu_camera_calibration/iccCamera.hpp"
#include "kalibr_imu_camera_calibration/iccImu.hpp"

#include "utilities/utils.hpp"

#include <aslam/backend/CameraDesignVariable.hpp>
#include <aslam/backend/HomogeneousPoint.hpp>
#include <aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/ReprojectionError.hpp>
#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/cameras.hpp>

#include <ostream>

boost::shared_ptr<aslam::backend::TransformationBasic> addPoseDesignVariable(
	boost::shared_ptr<aslam::backend::OptimizationProblem> &problem, const sm::kinematics::Transformation &T0) {
	auto q_Dv = boost::make_shared<aslam::backend::RotationQuaternion>(T0.q());
	q_Dv->setActive(true);
	problem->addDesignVariable(q_Dv);

	auto t_Dv = boost::make_shared<aslam::backend::EuclideanPoint>(T0.t());
	t_Dv->setActive(true);
	problem->addDesignVariable(t_Dv);

	return boost::make_shared<aslam::backend::TransformationBasic>(q_Dv->toExpression(), t_Dv->toExpression());
}

template<typename CameraGeometryType, typename DistortionType>
class CameraGeometry {
private:
	boost::shared_ptr<IccCamera<CameraGeometryType, DistortionType>> iccCamera     = nullptr;
	boost::shared_ptr<aslam::backend::CameraDesignVariable<CameraGeometryType>> dv = nullptr;
	bool isGeometryInitialized;

public:
	CameraGeometry(boost::shared_ptr<IccCamera<CameraGeometryType, DistortionType>> camera) {
		iccCamera = camera;

		auto geometry = camera->getCameraGeometry();

		auto casted = boost::dynamic_pointer_cast<CameraGeometryType>(geometry);
		if (casted == nullptr) {
			throw std::runtime_error("Something went wrong with casting of the pointer of camera geometry");
		}

		// create the design variables
		dv = boost::make_shared<aslam::backend::CameraDesignVariable<CameraGeometryType>>(casted);

		// design variable: projection and distortion active, shutter not
		setDvActiveStatus(true, true, false);
		isGeometryInitialized = false;
	}

	bool initGeometryFromObservations(
		boost::shared_ptr<std::map<int64_t, aslam::cameras::GridCalibrationTargetObservation>> observationsMap,
		boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target) {
		// Obtain focal length guess
		auto geometry = iccCamera->getCameraGeometry();
		std::vector<aslam::cameras::GridCalibrationTargetObservation> observationsVector;
		observationsVector.reserve(observationsMap->size());
		for (const auto &[_, obs] : *observationsMap) {
			observationsVector.emplace_back(obs);
		}
		auto success = geometry->initializeIntrinsics(observationsVector);
		if (!success) {
			std::cout << "Initialization of focal length failed" << std::endl;
		}

		// Optimize for intrinsics and distortion
		success = calibrateIntrinsics(observationsVector, target);
		if (!success) {
			std::cout << "Intrinsic calib failed." << std::endl;
			// nothing to print
		}

		isGeometryInitialized = success;
		return success;
	}

	bool getIsGeometryInitialized() {
		return isGeometryInitialized;
	}

	boost::shared_ptr<CameraGeometryType> getCameraGeometry() {
		return iccCamera->getCameraGeometry();
	}

	boost::shared_ptr<aslam::backend::CameraDesignVariable<CameraGeometryType>> getDv() {
		return dv;
	}

	void setDvActiveStatus(bool projectionActive, bool distortionActive, bool shutterActive) {
		dv->projectionDesignVariable()->setActive(projectionActive);
		dv->distortionDesignVariable()->setActive(distortionActive);
		dv->shutterDesignVariable()->setActive(shutterActive);
	}

protected:
	// This function Python equivalent can be found in:
	// thirdparty/kalibr/aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraIntializers.py
	// It was moved to avoid circular dependencies
	bool calibrateIntrinsics(
		const std::vector<aslam::cameras::GridCalibrationTargetObservation> &obslist, // 2D keypoint location
		boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target,          // 3D landmark location
		bool distortionActive = true, bool intrinsicsActive = true) {
		// verbose output
		Eigen::MatrixXd params;
		iccCamera->getCameraGeometry()->getParameters(params, true, true, true);
		std::cout << "calibrateIntrinsics: intrinsics guess: " << params.block<4, 1>(0, 0).transpose() << std::endl;
		std::cout << "calibrateIntrinsics: distortion guess: " << params.block(4, 0, params.rows() - 4, 1).transpose()
				  << std::endl;

		// ############################################
		// ## solve the bundle adjustment
		// ############################################
		auto problem = boost::make_shared<aslam::backend::OptimizationProblem>();

		// add camera dvs
		setDvActiveStatus(intrinsicsActive, distortionActive, false);
		problem->addDesignVariable(dv->distortionDesignVariable());
		problem->addDesignVariable(dv->projectionDesignVariable());
		problem->addDesignVariable(dv->shutterDesignVariable());

		// corner uncertainty
		const double cornerUncertainty = 1.0;
		const auto R                   = Eigen::Matrix2d::Identity() * cornerUncertainty * cornerUncertainty;
		const auto invR                = R.inverse();

		// # target pose dv for all target views (=T_camL_w)
		typedef aslam::backend::ReprojectionError<CameraGeometryType> ReprojectionError;
		std::cout << "calibrateIntrinsics: adding camera error terms for " << obslist.size() << " calibration targets"
				  << std::endl;
		std::vector<boost::shared_ptr<aslam::backend::TransformationBasic>> target_pose_dvs;
		target_pose_dvs.reserve(obslist.size());
		size_t numErrorTerms = 0;
		for (const auto &obs : obslist) {
			sm::kinematics::Transformation T_t_c;
			iccCamera->getCameraGeometry()->estimateTransformation(obs, T_t_c);
			auto target_pose_dv = addPoseDesignVariable(problem, T_t_c);
			target_pose_dvs.push_back(target_pose_dv);

			const auto T_cam_w = target_pose_dv->toExpression().inverse();

			// add error terms
			for (size_t i = 0; i < target->size(); ++i) {
				auto p_target = aslam::backend::HomogeneousExpression(sm::kinematics::toHomogeneous(target->point(i)));
				Eigen::Vector2d y;
				if (obs.imagePoint(i, y)) {
					auto rerr = boost::make_shared<ReprojectionError>(y, invR, T_cam_w * p_target, *dv);
					problem->addErrorTerm(rerr);
					++numErrorTerms;
				}
			}
		}
		std::cout << "calibrateIntrinsics: added " << numErrorTerms << " RE camera error terms" << std::endl;

		// ############################################
		// ## solve
		// ############################################
		aslam::backend::Optimizer2Options options;
		options.verbose           = true;
		options.nThreads          = 4;
		options.convergenceDeltaX = 1e-3;
		options.convergenceDeltaJ = 1;
		options.maxIterations     = 200;
		options.trustRegionPolicy = boost::make_shared<aslam::backend::LevenbergMarquardtTrustRegionPolicy>(10);

		aslam::backend::Optimizer2 optimizer(options);
		optimizer.setProblem(problem);

		// verbose output
		auto printReprErrors = [&](const std::string &prefix) {
			const std::vector<std::vector<Eigen::Vector2d>> reprojectionErrors
				= computeReprojectionErrors<CameraGeometryType>(obslist, target, iccCamera->getCameraGeometry());
			const std::vector<double> reprojectionErrorNorms = computeReprojectionErrorNormsPerGrid(reprojectionErrors);
			const auto [mean, std]                           = meanStd(reprojectionErrorNorms);
			std::cout << prefix << " RE mean: " << mean << " std: " << std << std::endl;
		};
		printReprErrors("calibrateIntrinsics: Before Optimization: ");

		// run intrinsic calibration
		bool success = false;
		try {
			auto retval = optimizer.optimize();
			if (retval.linearSolverFailure) {
				std::cout << "calibrateIntrinsics: Optimization failed!" << std::endl;
			}
			success = not retval.linearSolverFailure;
		}
		catch (...) {
			std::cout << "calibrateIntrinsics: Optimization failed!" << std::endl;
		}

		printReprErrors("calibrateIntrinsics: After Optimization: ");

		iccCamera->getCameraGeometry()->getParameters(params, true, true, true);
		std::cout << "calibrateIntrinsics: optimized intrinsics guess: " << params.block<4, 1>(0, 0).transpose()
				  << std::endl;
		std::cout << "calibrateIntrinsics: optimized distortion guess: "
				  << params.block(4, 0, params.rows() - 4, 1).transpose() << std::endl;

		return success;
	}
};

template<typename CameraGeometryType, typename DistortionType>
class CalibrationTargetOptimizationProblem : public aslam::calibration::OptimizationProblem {
public:
	// arguments
	std::vector<boost::shared_ptr<CameraGeometry<CameraGeometryType, DistortionType>>> cameras;
	boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> target;
	sm::kinematics::Transformation T_tc_guess;
	std::map<size_t, aslam::cameras::GridCalibrationTargetObservation> rig_observations;

	// others
	std::vector<boost::shared_ptr<aslam::backend::RotationQuaternion>> dv_T_target_camera_q;
	std::vector<boost::shared_ptr<aslam::backend::EuclideanPoint>> dv_T_target_camera_t;
	std::vector<boost::shared_ptr<aslam::backend::TransformationBasic>> dv_T_target_camera;
	std::vector<std::pair<boost::shared_ptr<aslam::backend::RotationQuaternion>,
		boost::shared_ptr<aslam::backend::EuclideanPoint>>>
		baselines;
	std::vector<boost::shared_ptr<aslam::backend::TransformationBasic>> baselinesT;
	std::vector<aslam::backend::HomogeneousExpression> P_t_ex;
	std::vector<boost::shared_ptr<aslam::backend::HomogeneousPoint>> P_t_dv;
	std::vector<std::vector<boost::shared_ptr<aslam::backend::ReprojectionError<CameraGeometryType>>>> rerrs;

	// constructor is merged with factory method from Python "fromTargetViewObservations"
	CalibrationTargetOptimizationProblem(
		const std::vector<boost::shared_ptr<CameraGeometry<CameraGeometryType, DistortionType>>> &_cameras,
		const boost::shared_ptr<aslam::cameras::GridCalibrationTargetBase> &_target,
		const std::vector<std::pair<boost::shared_ptr<aslam::backend::RotationQuaternion>,
			boost::shared_ptr<aslam::backend::EuclideanPoint>>> &_baselines,
		const sm::kinematics::Transformation &_T_tc_guess,
		const std::map<size_t, aslam::cameras::GridCalibrationTargetObservation> &_rig_observations,
		bool estimateLandmarks, bool useBlakeZissermanMest) :
		cameras(_cameras),
		target(_target),
		T_tc_guess(_T_tc_guess),
		rig_observations(_rig_observations),
		baselines(_baselines) {
		// 1. Create a design variable for this pose
		const auto T_target_camera = T_tc_guess;

		for (size_t i = 0; i < cameras.size(); i++) {
			auto dv_T_q = boost::make_shared<aslam::backend::RotationQuaternion>(T_target_camera.q());
			dv_T_q->setActive(true);
			this->addDesignVariable(dv_T_q, TRANSFORMATION_GROUP_ID);
			auto dv_T_t = boost::make_shared<aslam::backend::EuclideanPoint>(T_target_camera.t());
			dv_T_t->setActive(true);
			this->addDesignVariable(dv_T_t, TRANSFORMATION_GROUP_ID);

			dv_T_target_camera.push_back(boost::make_shared<aslam::backend::TransformationBasic>(
				dv_T_q->toExpression(), dv_T_t->toExpression()));

			dv_T_target_camera_q.push_back(dv_T_q);
			dv_T_target_camera_t.push_back(dv_T_t);
		}

		// 2. Add all baselines DVs
		for (const auto &baseline : baselines) {
			baseline.first->setActive(true);
			this->addDesignVariable(baseline.first, CALIBRATION_GROUP_ID);
			baseline.second->setActive(true);
			this->addDesignVariable(baseline.second, CALIBRATION_GROUP_ID);

			baselinesT.push_back(boost::make_shared<aslam::backend::TransformationBasic>(
				baseline.first->toExpression(), baseline.second->toExpression()));
		}

		// 3. Add landmark DVs
		P_t_ex.reserve(target->size());
		P_t_dv.reserve(target->size());
		for (size_t i = 0; i < target->size(); ++i) {
			auto p_t_dv
				= boost::make_shared<aslam::backend::HomogeneousPoint>(sm::kinematics::toHomogeneous(target->point(i)));
			p_t_dv->setActive(estimateLandmarks);
			auto p_t_ex = p_t_dv->toExpression();
			P_t_ex.push_back(p_t_ex);
			P_t_dv.push_back(p_t_dv);

			this->addDesignVariable(p_t_dv, LANDMARK_GROUP_ID);
		}

		// 4. add camera DVs
		for (const auto &camera : cameras) {
			if (!camera->getIsGeometryInitialized()) {
				std::cout << "CAMERA GEOMETRY ERROR..." << std::endl;
				throw std::runtime_error(
					"The camera geometry is not initialized. Please initialize with initGeometry() or "
					"initGeometryFromDataset()");
			}
			camera->setDvActiveStatus(true, true, false);
			this->addDesignVariable(camera->getDv()->distortionDesignVariable(), CALIBRATION_GROUP_ID);
			this->addDesignVariable(camera->getDv()->projectionDesignVariable(), CALIBRATION_GROUP_ID);
			this->addDesignVariable(camera->getDv()->shutterDesignVariable(), CALIBRATION_GROUP_ID);
		}

		// 5. add all observations for this view
		size_t rerr_cnt = 0;
		for (const auto &camera : cameras) {
			rerrs.emplace_back().reserve(P_t_ex.size());
		}

		double cornerUncertainty = 1.0;
		const auto R             = Eigen::Matrix2d::Identity() * cornerUncertainty * cornerUncertainty;
		const auto invR          = R.inverse();

		// add reprojection errors
		// build baseline chain (target->cam0->baselines->camN)
		for (auto &[camId, rig_observation] : rig_observations) {
			const auto T_cam0_target = dv_T_target_camera[camId]->toExpression().inverse();
			auto T_camN_calib        = T_cam0_target;

			for (size_t cId = 0; cId <= camId; cId++) {
				T_camN_calib = baselinesT[cId]->toExpression() * T_camN_calib;
			}

			Eigen::Vector2d y;
			for (size_t i = 0; i < P_t_ex.size(); ++i) {
				const auto &p_target = P_t_ex[i];
				if (rig_observation.imagePoint(i, y)) {
					++rerr_cnt;
					// create an error term.
					auto rerr = boost::make_shared<aslam::backend::ReprojectionError<CameraGeometryType>>(
						y, invR, T_camN_calib * p_target, *cameras[camId]->getDv());

					if (useBlakeZissermanMest) {
						throw std::runtime_error("useBlakeZissermanMest not implemented");
					}
					this->addErrorTerm(rerr);
					rerrs[camId].push_back(rerr);
				}
				else {
					rerrs[camId].emplace_back(nullptr);
				}
			}
		}
	}
};

class OptimizationDiverged : public std::exception {
private:
	std::string message_;

public:
	OptimizationDiverged(const std::string &message) : message_(message) {
	}

	const char *what() const noexcept override {
		return message_.c_str();
	}
};

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

		bool success = estimator_return_value.batchAccepted;
		if (success) {
			views.push_back(batch_problem);
		}

		return success;
	}

	size_t getNumBatches() {
		return estimator->getNumBatches();
	}

	void replaceBatch(const size_t batch_id,
		boost::shared_ptr<CalibrationTargetOptimizationProblem<CameraGeometryType, DistortionType>> new_batch) {
		estimator->removeBatch(views.at(batch_id));
		views.at(batch_id) = new_batch;

		const auto rval = estimator->addBatch(new_batch, false);

		// queue the batch for removal if the corrected batch was rejected
		if (!rval.batchAccepted) {
			views.erase(views.begin() + batch_id);
		}
	}

	std::vector<aslam::cameras::GridCalibrationTargetObservation> getObservations(const size_t cameraId) {
		std::vector<aslam::cameras::GridCalibrationTargetObservation> obsList;
		for (const auto &view : views) {
			const auto &obs = view->rig_observations[cameraId];
			obsList.push_back(obs);
		}
		return obsList;
	}

	std::vector<std::vector<Eigen::Vector2d>> getReprojectionErrors(const size_t cameraId) {
		std::vector<std::vector<Eigen::Vector2d>> reprojectionErrorAllViews;
		for (auto &view : views) {
			std::vector<Eigen::MatrixXd> view_corners, view_reprojections;
			std::vector<Eigen::Vector2d> reprojectionErrorPerGrid;
			for (const auto &rerr : view->rerrs[cameraId]) {
				if (!rerr) {
					std::cerr << "Warning: encountered view in CameraCalibration with no data." << std::endl;
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
			;
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
