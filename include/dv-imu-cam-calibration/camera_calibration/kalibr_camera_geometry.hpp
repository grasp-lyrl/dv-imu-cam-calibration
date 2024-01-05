#pragma once

#include "imu_camera_calibration/kalibr_iccCamera.hpp"
#include "wrappers/repro_error_utils.hpp"

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

/**
 * Kalibr corresponding class in CameraCalibrator.py
 * @tparam CameraGeometryType
 * @tparam DistortionType
 */
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
			const std::vector<std::vector<std::optional<Eigen::Vector2d>>> reprojectionErrors
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
