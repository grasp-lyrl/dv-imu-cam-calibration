#pragma once

#include <aslam/backend/HomogeneousPoint.hpp>

#include "kalibr_camera_geometry.hpp"

/**
 * Kalibr corresponding class in CameraCalibrator.py
 * @tparam CameraGeometryType
 * @tparam DistortionType
 */
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
