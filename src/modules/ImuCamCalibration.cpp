#include <aslam/cameras.hpp>

#include "wrappers/Calibrator.hpp"
#include "wrappers/CalibratorBase.hpp"

#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/io/mono_camera_writer.hpp>

#include <dv-sdk/module.hpp>
#include <filesystem>
#include <fmt/chrono.h>
#include <fstream>
#include <regex>
#include <string>
#include <thread>

namespace pt = boost::property_tree;
namespace fs = std::filesystem;

std::string getTimeString() {
	return fmt::format("{:%Y-%m-%dT%H-%M-%SZ}",
		std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()));
}

class ImuCamCalibration : public dv::ModuleBase {
protected:
	// Calibrator options
	CalibratorUtils::Options mOptions;
	std::string mCalibrationModel = ImuCamModelTypes::RADTAN;

	std::thread mThreadCalibrate;
	std::string mQuality;
	std::unique_ptr<CalibratorBase> mCalibrator        = nullptr;
	std::unique_ptr<dv::io::MonoCameraWriter> mDataLog = nullptr;
	using DVMessage                                    = std::variant<dv::TimedKeyPointPacket, dv::Frame, dv::IMU>;
	using DVStreamAndMessage                           = std::pair<std::string, DVMessage>;
	std::multimap<int64_t, DVStreamAndMessage> mDataLogBuffer;
	std::string mTimestampString;
	std::vector<int64_t> mTimes;
	int64_t mStartTime      = -1;
	int64_t mWarmUpDuration = 100'000;

#if WITH_IMU_CALIBRATION
	// Cached IMU biases / scales parsed from the config strings. Refreshed when
	// any string changes between IMU packets.
	std::string mAccelBiasStringCache;
	std::string mGyroBiasStringCache;
	std::string mAccelScaleStringCache;
	Eigen::Vector3d mAccelBiasCache = Eigen::Vector3d::Zero();
	Eigen::Vector3d mGyroBiasCache  = Eigen::Vector3d::Zero();
	Eigen::Vector3d mAccelInvScaleCache = Eigen::Vector3d::Ones();
#endif

	boost::circular_buffer<dv::Frame> mLeftFrames  = boost::circular_buffer<dv::Frame>(5);
	boost::circular_buffer<dv::Frame> mRightFrames = boost::circular_buffer<dv::Frame>(5);
	dv::io::MonoCameraWriter::Config mWriterConfig;

	enum CollectionState {
		BEFORE_COLLECTING,
		DURING_COLLECTING,
		AFTER_COLLECTING,
		CALIBRATING,
		CALIBRATED
	};

	CollectionState collectionState = BEFORE_COLLECTING;

#if WITH_IMU_CALIBRATION
	static std::optional<double> estimateFrequency(const std::vector<int64_t> &timestamps) {
		std::vector<double> freq;
		if (timestamps.size() < 2) {
			return std::nullopt;
		}

		for (auto iter = std::next(timestamps.begin()); iter < timestamps.end(); iter++) {
			// We are interested in frequencies in 1Hz to 1000Hz, so converting to milliseconds, this should give a good
			// enough approximation
			int64_t t1 = *std::prev(iter);
			int64_t t2 = *iter;
			freq.push_back(1e+6 / static_cast<double>(t2 - t1));
		}

		return std::accumulate(freq.begin(), freq.end(), 0.) / static_cast<double>(freq.size());
	}

	std::optional<double> imuUpdateRate = std::nullopt;

#endif

	void writeDataLogBuffer(const int64_t leaveLastMicroseconds = 0) {
		if (mDataLogBuffer.empty() || mDataLog == nullptr) {
			// Nothing to write
			return;
		}

		// Write the data up to last - minus a bit microseconds
		const auto lastTs     = mDataLogBuffer.rbegin()->first;
		int64_t lastWrittenTs = -1;
		for (const auto &[ts, streamAndData] : mDataLogBuffer) {
			if (ts > lastTs - leaveLastMicroseconds) {
				break;
			}
			lastWrittenTs              = ts;
			const auto &[stream, data] = streamAndData;
			if (const auto *pval = std::get_if<dv::TimedKeyPointPacket>(&data)) {
				mDataLog->writePacket(*pval, stream);
			}
			else if (const auto *pval = std::get_if<dv::Frame>(&data)) {
				mDataLog->writePacket(*pval, stream);
			}
			else if (const auto *pval = std::get_if<dv::IMU>(&data)) {
				mDataLog->writeImu(*pval, stream);
			}
			else {
				throw std::runtime_error("Unknown packet type");
			}
		}

		// Erase the data that has been written
		if (lastWrittenTs != -1) {
			const auto it = mDataLogBuffer.find(lastWrittenTs);
			mDataLogBuffer.erase(mDataLogBuffer.begin(), it);
		}
	}

public:
	static void initInputs(dv::InputDefinitionList &in) {
		in.addFrameInput("left");
		in.addFrameInput("right", true);
#if WITH_IMU_CALIBRATION
		in.addIMUInput("imu", true);
#endif
	}

	static void initOutputs(dv::OutputDefinitionList &out) {
		out.addFrameOutput("left");
		out.addFrameOutput("right");
	}

	static const char *initDescription() {
		return ("Calibrate the IMU with respect to the event camera.");
	}

	static void initConfigOptions(dv::RuntimeConfig &config) {
		// Camera calibration file output
		config.add("outputCalibrationDirectory",
			dv::ConfigOption::directoryOption(
				"Path to a directory to save the calibration settings in", dv::portable_get_user_home_directory()));

		// Calibration pattern
		config.add(
			"numPatternColumns", dv::ConfigOption::intOption("Number of columns in the calibration pattern", 6, 1, 50));
		config.add(
			"numPatternRows", dv::ConfigOption::intOption("Number of rows in the calibration pattern", 6, 1, 50));
		config.add("markerSize",
			dv::ConfigOption::floatOption("Size of a calibration pattern element in meters", 0.05, 0.0, 1.0));
		config.add("markerSpacing",
			dv::ConfigOption::floatOption("Ratio of space between tags to tagSize (AprilGrid only)", 0.3, 0.0, 1.0));
		config.add("minDetectedCorners",
			dv::ConfigOption::intOption("Minimum detected corners required to accept a frame", 1, 1, 5000));
		config.add("patternType", dv::ConfigOption::listOption("Type of calibration pattern to use", "aprilGrid",
									  {"aprilGrid", "asymmetricCirclesGrid", "chessboard"}, false));
		config.add("priorIntrinsicsXml",
			dv::ConfigOption::fileOpenOption(
				"Optional OpenCV-XML calibration file to use as a prior for camera intrinsics and distortion",
				"xml"));

		// All the supported models are pinhole projection camera model. What change is the distortion.
		config.add(
			"calibrationModel", dv::ConfigOption::listOption("Calibration model to use", ImuCamModelTypes::RADTAN,
									{ImuCamModelTypes::RADTAN, ImuCamModelTypes::EQUIDISTANT}, false));

		// Module control buttons
		config.add("startCollecting",
			dv::ConfigOption::buttonOption("Start collecting calibration images", "StartCollecting"));
		config.add(
			"stopCollecting", dv::ConfigOption::buttonOption("Stop collecting calibration images", "StopCollecting"));
		config.add("discard", dv::ConfigOption::buttonOption("Discard the collected images", "Discard"));
		config.add("calibrate", dv::ConfigOption::buttonOption("Start calibration algorithm", "Calibrate"));
		config.add("calibrationFinished", dv::ConfigOption::boolOption("Calibration finished", false, true));

		// Optimization options
		config.add("maxIter",
			dv::ConfigOption::intOption("Maximum number of iteration of calibration optimization problem", 50, 1, 100));
#if WITH_IMU_CALIBRATION
		config.add("timeCalibration",
			dv::ConfigOption::boolOption("If true, time offset between the sensors will be calibrated", true));
		config.add("timeOffsetPadding",
			dv::ConfigOption::floatOption(
				"Padding in seconds used for camera/IMU spline time bounds during time calibration", 0.5, 0.0, 5.0));
#endif

		// IMU noise parameters
		config.add("recordData", dv::ConfigOption::boolOption("Record collected data in the output directory", true));

#if WITH_IMU_CALIBRATION
		// Per-device IMU bias priors as comma-separated x,y,z triples.
		// Subtracted from every IMU sample before it reaches kalibr, so the bias
		// spline starts near zero rather than having to absorb a large constant
		// factory offset. Defaults are accelerometer_bias_refined_m_s2 and
		// gyroscope_bias_rad_s from the static-orientation bias calibration tool;
		// override per-device by editing the strings in the GUI. Set to empty to
		// disable debiasing.
		config.add("accelBias",
			dv::ConfigOption::stringOption("Accelerometer bias x,y,z [m/s^2] (subtracted from every IMU sample)",
				"0.822333771293767, -0.4204056391087951, 0.13869418012856144"));
		config.add("gyroBias",
			dv::ConfigOption::stringOption("Gyroscope bias x,y,z [rad/s] (subtracted from every IMU sample)",
				"-0.008375375021517997, -0.0031113306658109284, -0.0039612173365261875"));
		// Per-axis accelerometer scale factor: measured = scale * true, so
		// corrected = (raw - bias) / scale, applied independently per axis.
		// Defaults are `per_axis[x|y|z].scale_factor` from the static-orientation
		// bias calibration tool. The z axis was not scale-calibrated (no z_up pair),
		// so its scale defaults to 1.0 — the refined bias still applies. Set any
		// component to 1.0 to disable scale correction on that axis.
		config.add("accelScale",
			dv::ConfigOption::stringOption(
				"Accelerometer scale factors x,y,z (corrected = (raw - bias) / scale, per-axis)",
				"0.9963230510072503, 0.9925088890752373, 1.0"));
#endif

		config.setPriorityOptions({"outputCalibrationDirectory", "calibrationModel", "boardHeight", "boardWidth",
			"markerSize", "patternType", "priorIntrinsicsXml", "accelBias", "accelScale", "gyroBias",
			"startCollecting", "stopCollecting", "discard", "calibrate"});
	}

	void handleCollectionState() {
		if (mCalibrationModel != config.getString("calibrationModel")) {
			mCalibrationModel = config.getString("calibrationModel");
			initializeCalibrator();
			collectionState = BEFORE_COLLECTING;
			log.info(fmt::format("Calibration model has changed to : {0}", mCalibrationModel));
		}
		if (mCalibrator == nullptr) {
			initializeCalibrator();
			collectionState = BEFORE_COLLECTING;
		}

		// Handle user input
		switch (collectionState) {
			case BEFORE_COLLECTING: {
				if (config.getBool("startCollecting")) {
					// Re-initialize so the latest GUI values for pattern type / rows /
					// columns / marker size / spacing / priorIntrinsicsXml etc. are
					// picked up. Without this the calibrator stays bound to whichever
					// values were live at module-load time.
					initializeCalibrator();
					collectionState = DURING_COLLECTING;
					mCalibrator->startCollecting();
					log.info("Started collecting images");
				}
				break;
			}
			case DURING_COLLECTING: {
				if (config.getBool("stopCollecting")) {
					collectionState = AFTER_COLLECTING;
					mCalibrator->stopCollecting();
					log.info("Stopped collecting images");
				}
				break;
			}
			case AFTER_COLLECTING: {
				if (config.getBool("calibrate")) {
					mDataLog = nullptr;

					collectionState  = CALIBRATING;
					mThreadCalibrate = std::thread([&]() {
#if WITH_IMU_CALIBRATION
						calibrate(inputs.isConnected("imu"));
#else
						calibrate(false);
#endif
					});
				}

				if (config.getBool("discard")) {
					collectionState = BEFORE_COLLECTING;
					if (mThreadCalibrate.joinable()) {
						mThreadCalibrate.join();
					}
					log.info("Discarded all collected data");
					mCalibrator->reset();
					initializeCalibrator();
				}
				break;
			}
			case CALIBRATING: {
				break;
			}
			case CALIBRATED: {
				if (config.getBool("discard")) {
					collectionState = BEFORE_COLLECTING;
					if (mThreadCalibrate.joinable()) {
						mThreadCalibrate.join();
					}
					log.info("Discarded all collected data");

					initializeCalibrator();
				}
				break;
			}
			default:
				throw std::runtime_error("Invalid collection state");
		}

		// Enable/Disable buttons based on current state
		switch (collectionState) {
			case BEFORE_COLLECTING: {
				config.setBool("startCollecting", false);
				config.setBool("stopCollecting", true);
				config.setBool("discard", true);
				config.setBool("calibrate", true);
				break;
			}
			case DURING_COLLECTING: {
				config.setBool("startCollecting", true);
				config.setBool("stopCollecting", false);
				config.setBool("discard", true);
				config.setBool("calibrate", true);
				break;
			}
			case AFTER_COLLECTING: {
				config.setBool("startCollecting", true);
				config.setBool("stopCollecting", true);
				config.setBool("discard", false);
				config.setBool("calibrate", false);
				break;
			}
			case CALIBRATING: {
				config.setBool("startCollecting", true);
				config.setBool("stopCollecting", true);
				config.setBool("discard", true);
				config.setBool("calibrate", true);
				break;
			}
			case CALIBRATED: {
				config.setBool("startCollecting", true);
				config.setBool("stopCollecting", true);
				config.setBool("discard", false);
				config.setBool("calibrate", true);
				break;
			}
			default:
				throw std::runtime_error("Invalid collection state");
		}
	}

	void setupCalibrator() {
		if (mCalibrator != nullptr) {
			mCalibrator->reset();
		}
		// All the supported models are pinhole projection camera model. What change is the distortion.
		if (config.getString("calibrationModel") == ImuCamModelTypes::EQUIDISTANT) {
			mCalibrator = std::make_unique<Calibrator<aslam::cameras::EquidistantDistortedPinholeCameraGeometry,
				aslam::cameras::EquidistantDistortion>>(mOptions);
		}
		else {
			mCalibrator = std::make_unique<
				Calibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>>(
				mOptions);
		}
	}

	void configUpdate() override {
		handleCollectionState();
	}

	// Load camera intrinsics + distortion from an OpenCV-XML calibration file written
	// by the standalone camera calibrator. The XML wraps the data in a per-device node
	// (e.g. <DVXplorerM_DXUS0034>) with `camera_matrix` and `distortion_coefficients`
	// children; we pick the first such node so the device-name suffix doesn't matter.
	// Distortion is truncated to 4 coefficients (k1, k2, p1, p2) to match kalibr's
	// RadialTangentialDistortion model.
	void loadIntrinsicsPriorFromXml(const std::string &path, CalibratorUtils::Options::CameraInits &cam) {
		try {
			cv::FileStorage fs(path, cv::FileStorage::READ);
			if (!fs.isOpened()) {
				log.warning << "priorIntrinsicsXml: could not open " << path << dv::logEnd;
				return;
			}

			cv::FileNode camNode;
			for (auto it = fs.root().begin(); it != fs.root().end(); ++it) {
				const cv::FileNode child = *it;
				if (child.isMap() && !child["camera_matrix"].empty()) {
					camNode = child;
					break;
				}
			}
			if (camNode.empty()) {
				log.warning << "priorIntrinsicsXml: no camera_matrix found in " << path << dv::logEnd;
				return;
			}

			cv::Mat K, D;
			camNode["camera_matrix"] >> K;
			camNode["distortion_coefficients"] >> D;
			if (K.rows != 3 || K.cols != 3) {
				log.warning << "priorIntrinsicsXml: camera_matrix is not 3x3 in " << path << dv::logEnd;
				return;
			}

			cam.intrinsics = {K.at<double>(0, 0), K.at<double>(1, 1), K.at<double>(0, 2), K.at<double>(1, 2)};

			cam.distCoeffs.clear();
			const int nCoeffs = std::min<int>(4, static_cast<int>(D.total()));
			for (int i = 0; i < nCoeffs; ++i) {
				cam.distCoeffs.push_back(D.at<double>(i));
			}
			while (cam.distCoeffs.size() < 4) {
				cam.distCoeffs.push_back(0.0);
			}

			log.info << "priorIntrinsicsXml: loaded intrinsics fx=" << cam.intrinsics[0]
					 << " fy=" << cam.intrinsics[1] << " cx=" << cam.intrinsics[2]
					 << " cy=" << cam.intrinsics[3] << " from " << path << dv::logEnd;
		}
		catch (const std::exception &ex) {
			log.warning << "priorIntrinsicsXml: failed to parse " << path << ": " << ex.what() << dv::logEnd;
		}
	}

#if WITH_IMU_CALIBRATION
	// Parse a "x, y, z" triple. Returns Vector3d::Zero() and logs a warning on
	// any failure (empty or malformed string), so a typo just disables debiasing
	// for that axis-set rather than aborting the calibration.
	Eigen::Vector3d parseBiasTriple(const std::string &raw, const std::string &label) {
		Eigen::Vector3d out = Eigen::Vector3d::Zero();
		if (raw.empty()) {
			return out;
		}
		std::stringstream ss(raw);
		std::string token;
		size_t idx = 0;
		while (std::getline(ss, token, ',') && idx < 3) {
			try {
				out(static_cast<int>(idx)) = std::stod(token);
				++idx;
			}
			catch (const std::exception &) {
				log.warning << label << ": could not parse component '" << token << "' in '" << raw
							<< "' — disabling debiasing for this axis-set." << dv::logEnd;
				return Eigen::Vector3d::Zero();
			}
		}
		if (idx != 3) {
			log.warning << label << ": expected 3 comma-separated values, got " << idx << " in '" << raw
						<< "' — disabling debiasing for this axis-set." << dv::logEnd;
			return Eigen::Vector3d::Zero();
		}
		return out;
	}
#endif

	void initializeCalibrator() {
		const auto frameInput = inputs.getFrameInput("left");
		mTimestampString      = getTimeString();
		mOptions              = CalibratorUtils::Options();

		mOptions.pattern        = getPatternType();
		mOptions.cols           = static_cast<size_t>(config.getInt("numPatternColumns"));
		mOptions.rows           = static_cast<size_t>(config.getInt("numPatternRows"));
		mOptions.spacingMeters  = static_cast<double>(config.getFloat("markerSize"));
		mOptions.patternSpacing = static_cast<double>(config.getFloat("markerSpacing"));
		mOptions.minDetectedCorners = static_cast<size_t>(config.getInt("minDetectedCorners"));
		mOptions.cameraInitialSettings.emplace_back().imageSize = frameInput.size();

		// Add config for the second camera if it is connected
		const auto rightInput = inputs.getFrameInput("right");
		if (rightInput.isConnected()) {
			mOptions.cameraInitialSettings.emplace_back().imageSize = rightInput.size();
		}

		// If a prior calibration XML is provided, load camera intrinsics + distortion
		// and use them as a starting guess instead of the (width, width, w/2, h/2)
		// defaults. Stereo: applied to the left camera only.
		const auto priorXml = config.getString("priorIntrinsicsXml");
		if (!priorXml.empty()) {
			loadIntrinsicsPriorFromXml(priorXml, mOptions.cameraInitialSettings.front());
		}

#if WITH_IMU_CALIBRATION
		// Seed the IMU extrinsic prior with a 180-degree rotation about y
		// (diag(-1, +1, -1), zero translation). This matches the typical IMU-vs-camera
		// mounting on this rig and gives findOrientationPriorCameraToImu a much
		// better starting point than identity.
		Eigen::Matrix4d T_cam_imu_prior  = Eigen::Matrix4d::Identity();
		T_cam_imu_prior(0, 0)            = -1.0;
		T_cam_imu_prior(2, 2)            = -1.0;
		for (auto &cam : mOptions.cameraInitialSettings) {
			cam.T_cam_imu_initial = T_cam_imu_prior;
		}
#endif

		mOptions.maxIter = static_cast<size_t>(config.getInt("maxIter"));

#if WITH_IMU_CALIBRATION
		mOptions.timeCalibration = config.getBool("timeCalibration");
		mOptions.timeOffsetPadding = static_cast<double>(config.getFloat("timeOffsetPadding"));

		if (imuUpdateRate.has_value()) {
			mOptions.imuParameters.updateRate = *imuUpdateRate;
		}
#endif
		setupCalibrator();

		// TODO: wrap a class around the MonoCameraWriter and the StereoCameraWriter
		if (config.getBool("recordData")) {
			// Reset the writer config to a fresh instance so re-entry of
			// initializeCalibrator() (e.g. on each Start Collecting press) does not
			// re-add streams to the same Config — dv-processing rejects duplicates
			// with "Writer already contains a stream with the given name".
			mWriterConfig = dv::io::MonoCameraWriter::Config(getCameraID("left"));
			mWriterConfig.addFrameStream(frameInput.size(), "frames");
#if WITH_IMU_CALIBRATION
			if (inputs.getIMUInput("imu").isConnected()) {
				mWriterConfig.addImuStream("imu");
			}
#endif
			mWriterConfig.addFrameStream(frameInput.size(), "left_frames", frameInput.getOriginDescription());
			mWriterConfig.addStream<dv::TimedKeyPointPacket>("left_markers");
			if (rightInput.isConnected()) {
				mWriterConfig.addFrameStream(rightInput.size(), "right_frames", rightInput.getOriginDescription());
				mWriterConfig.addStream<dv::TimedKeyPointPacket>("right_markers");
			}
		}
	}

	ImuCamCalibration() : mWriterConfig(getCameraID("left")) {
		// Input output
		const auto frameInput  = inputs.getFrameInput("left");
		const auto inputSize   = frameInput.size();
		const auto description = frameInput.getOriginDescription();
		outputs.getFrameOutput("left").setup(inputSize.width, inputSize.height, description);

		if (inputs.isConnected("right")) {
			outputs.getFrameOutput("right").setup(inputs.getFrameInput("right"));
		}
		else {
			// Setup using left camera info, but it will not output anything
			outputs.getFrameOutput("right").setup(inputs.getFrameInput("left"));
		}

#if WITH_IMU_CALIBRATION
		// If imu input is connected, do not initialize and wait until imu frequency is estimated
		if (!inputs.getIMUInput("imu").isConnected()) {
			initializeCalibrator();
		}
#endif

		// Unclick all the buttons and update the state
		config.setBool("startCollecting", false);
		config.setBool("stopCollecting", false);
		config.setBool("discard", false);
		config.setBool("calibrate", false);
		handleCollectionState();
	}

	~ImuCamCalibration() override {
		if (mThreadCalibrate.joinable()) {
			mThreadCalibrate.join();
		}
		mCalibrator->reset();
		writeDataLogBuffer();
	}

	void logObservationData() {
		auto detection = mCalibrator->getLatestObservations();
		if (!detection.first.empty() && !detection.second.empty()) {
			dv::runtime_assert(
				[&detection] {
					return detection.first.size() == detection.second.size();
				},
				[] {
					return "Observation count does not match image count!";
				});
		}
		else {
			return;
		}
		// TODO: Log both streams
		size_t index = 0;
		for (size_t index = 0; index < detection.first.size(); index++) {
			int64_t timestamp = detection.first[index].timestamp;
			dv::TimedKeyPointPacket markers;
			std::vector<uint32_t> ids;
			size_t count = detection.second[index]->getCornersIdx(ids);

			markers.elements.reserve(count);
			for (uint32_t id : ids) {
				Eigen::Vector2d cornerEigen;
				detection.second[index]->imagePoint(id, cornerEigen);
				markers.elements.emplace_back(
					dv::Point2f(static_cast<float>(cornerEigen(0)), static_cast<float>(cornerEigen(1))), 1.f, -1.f, 1.f,
					0, id, timestamp);
			}
			if (index == 0) {
				const auto ts = detection.first[index].timestamp;
				mDataLogBuffer.emplace(
					std::make_pair(ts, std::make_pair("left_frames", dv::Frame(ts, detection.first[index].image))));
				mDataLogBuffer.emplace(std::make_pair(ts, std::make_pair("left_markers", markers)));
			}
			else {
				const auto ts = detection.first[index].timestamp;
				mDataLogBuffer.emplace(
					std::make_pair(ts, std::make_pair("right_frames", dv::Frame(ts, detection.first[index].image))));
				mDataLogBuffer.emplace(std::make_pair(ts, std::make_pair("right_markers", markers)));
			}
		}

		// Write the log data from the buffer only if it is more than 1 second old
		writeDataLogBuffer(1000000);
	}

#if WITH_IMU_CALIBRATION
	std::optional<size_t> estimateImuFrequency(const dv::IMUPacket &packet) {
		for (const auto &measurement : packet.elements) {
			mTimes.push_back(measurement.timestamp);
		}
		if (mTimes.size() > 10) {
			auto result = estimateFrequency(mTimes);
			mTimes.clear();
			return result;
		}
		else {
			return std::nullopt;
		}
	}
#endif

	const dv::Frame &closestRightFrame(const int64_t timestamp) {
		auto iter
			= std::min_element(mRightFrames.begin(), mRightFrames.end(), [timestamp](const auto &a, const auto &b) {
				  return std::abs(a.timestamp - timestamp) < std::abs(b.timestamp - timestamp);
			  });
		return *iter;
	}

	void run() override {
		if (mCalibrator == nullptr) {
			handleCollectionState();
		}

#if WITH_IMU_CALIBRATION
		// Process IMU input
		if (inputs.isConnected("imu")) {
			auto imuInput = inputs.getIMUInput("imu");
			if (auto imuData = imuInput.data()) {
				if (!mCalibrator) {
					if (mStartTime < 0) {
						mStartTime = imuData.front().timestamp;
					}

					// Skip some data during warm up, the timestamps are not well aligned during startup
					if (imuData.front().timestamp - mStartTime < mWarmUpDuration) {
						return;
					}

					if (auto frequency = estimateImuFrequency(*imuData.getBasePointer()); frequency.has_value()) {
						imuUpdateRate = static_cast<double>(*frequency);
					}
					// wait for frequency estimation
					return;
				}

				// Refresh bias / scale caches when the user edits the strings. No-op on
				// the hot path when nothing changed.
				const auto accelBiasStr  = config.getString("accelBias");
				const auto gyroBiasStr   = config.getString("gyroBias");
				const auto accelScaleStr = config.getString("accelScale");
				if (accelBiasStr != mAccelBiasStringCache) {
					mAccelBiasStringCache = accelBiasStr;
					mAccelBiasCache       = parseBiasTriple(accelBiasStr, "accelBias");
					log.info << "Accelerometer bias updated to [" << mAccelBiasCache.x() << ", " << mAccelBiasCache.y()
							 << ", " << mAccelBiasCache.z() << "] m/s^2" << dv::logEnd;
				}
				if (gyroBiasStr != mGyroBiasStringCache) {
					mGyroBiasStringCache = gyroBiasStr;
					mGyroBiasCache       = parseBiasTriple(gyroBiasStr, "gyroBias");
					log.info << "Gyroscope bias updated to [" << mGyroBiasCache.x() << ", " << mGyroBiasCache.y()
							 << ", " << mGyroBiasCache.z() << "] rad/s" << dv::logEnd;
				}
				if (accelScaleStr != mAccelScaleStringCache) {
					mAccelScaleStringCache       = accelScaleStr;
					const Eigen::Vector3d scale  = parseBiasTriple(accelScaleStr, "accelScale");
					// Each axis: invert if positive, else fall back to identity (1.0)
					// so a zero/empty/garbage entry just disables scale correction on
					// that axis instead of producing inf.
					for (int i = 0; i < 3; ++i) {
						mAccelInvScaleCache(i) = (scale(i) > 1e-6) ? (1.0 / scale(i)) : 1.0;
					}
					log.info << "Accelerometer scale updated to [" << scale.x() << ", " << scale.y() << ", " << scale.z()
							 << "]" << dv::logEnd;
				}

				for (const auto &singleImu : imuData) {
					const Eigen::Vector3d gyro = singleImu.getAngularVelocities().cast<double>() - mGyroBiasCache;
					// Per-axis: corrected_i = (raw_i - bias_i) * (1 / scale_i).
					// cwiseProduct does the per-axis multiply.
					const Eigen::Vector3d acc = (singleImu.getAccelerations().cast<double>() - mAccelBiasCache)
													.cwiseProduct(mAccelInvScaleCache);
					mCalibrator->addImu(singleImu.timestamp, gyro, acc);
				}
				if (collectionState == DURING_COLLECTING) {
					for (const auto &singleImu : imuData) {
						const auto ts = singleImu.timestamp;
						mDataLogBuffer.emplace(std::make_pair(ts, std::make_pair("imu", singleImu)));
					}
				}
			}
		}
#endif

		const auto &rightInput = inputs.getFrameInput("right");
		const bool stereo      = rightInput.isConnected();

		if (stereo) {
			if (auto frame = rightInput.data()) {
				mRightFrames.push_back(*frame.getBasePointer());
			}
		}

		// Process frame input
		auto frameInput = inputs.getFrameInput("left");
		if (auto frame = frameInput.data()) {
			mLeftFrames.push_back(*frame.getBasePointer());
		}

		if (stereo && !mRightFrames.full()) {
			return;
		}

		if (mLeftFrames.full()) {
			std::vector<CalibratorUtils::StampedImage> images;
			dv::Frame &left = mLeftFrames.front();
			if (left.image.channels() == 3) {
				cv::Mat gray;
				cv::cvtColor(left.image, gray, cv::COLOR_BGR2GRAY);
				images.emplace_back(gray, left.timestamp);
			}
			else {
				images.emplace_back(left.image, left.timestamp);
			}

			if (stereo) {
				const dv::Frame &right = closestRightFrame(left.timestamp);
				// Pass in left timestamp so it can be easily pair-matched
				if (right.image.channels() == 3) {
					cv::Mat gray;
					cv::cvtColor(right.image, gray, cv::COLOR_BGR2GRAY);
					images.emplace_back(gray, right.timestamp);
				}
				else {
					images.emplace_back(right.image, left.timestamp);
				}
			}
			mCalibrator->addImages(images);

			// Output preview image
			auto previews = mCalibrator->getPreviewImages();
			if (!previews.empty()) {
				if (collectionState == BEFORE_COLLECTING) {
#if WITH_IMU_CALIBRATION
					if (!inputs.isConnected("imu")) {
						cv::putText(previews[0].image, "No IMU data", cv::Point(20, previews[0].image.rows - 20),
							cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0, 165, 255), 2);
					}
#endif
					cv::putText(previews[0].image, config.getString("calibrationModel"), cv::Point(20, 80),
						cv::FONT_HERSHEY_DUPLEX, .5, cv::Scalar(255, 0, 0), 2);
				}
				else if (collectionState == CALIBRATED) {
					drawQuality(previews[0].image, mQuality);
				}
				outputs.getFrameOutput("left") << previews[0].timestamp << previews[0].image << dv::commit;
				if (previews.size() == 2) {
					outputs.getFrameOutput("right") << previews[1].timestamp << previews[1].image << dv::commit;
				}
			}
			if (mDataLog) {
				logObservationData();
			}
			mLeftFrames.pop_front();
		}
	}

protected:
	fs::path getCalibrationSaveDirectory() {
		const auto frameInput = inputs.getFrameInput("left");
		const auto cameraID   = frameInput.getOriginDescription();
		fs::path outputDir{config.getString("outputCalibrationDirectory")};

		if (outputDir.empty()) {
			outputDir = dv::portable_get_user_home_directory();
		}

		const std::string dirName = "dv_calibration_" + mTimestampString;
		auto outputDirWithTs      = outputDir / dirName;

		if (!fs::is_directory(outputDirWithTs)) {
			fs::create_directories(outputDirWithTs);
		}

		return outputDirWithTs;
	}

	CalibratorUtils::PatternType getPatternType() {
		const auto pattern = config.getString("patternType");
		if (pattern == "aprilGrid") {
			return CalibratorUtils::PatternType::APRIL_GRID;
		}
		else if (pattern == "asymmetricCirclesGrid") {
			return CalibratorUtils::PatternType::ASYMMETRIC_CIRCLES_GRID;
		}
		else if (pattern == "chessboard") {
			return CalibratorUtils::PatternType::CHESSBOARD;
		}
		else {
			throw std::runtime_error("Unknown calibration pattern: " + pattern);
		}
	}

	int getPatternColumns() {
		return config.getInt("numPatternColumns");
	}

	int getPatternRows() {
		return config.getInt("numPatternRows");
	}

	int getInternalPatternColumns() {
		int cols = getPatternColumns();
		if (getPatternType() == CalibratorUtils::PatternType::CHESSBOARD) {
			// In case of chessboard we actually detect the inner corners
			cols -= 1;
		}
		return cols;
	}

	int getInternalPatternRows() {
		int rows = getPatternRows();
		if (getPatternType() == CalibratorUtils::PatternType::CHESSBOARD) {
			// In case of chessboard we actually detect the inner corners
			rows -= 1;
		}
		return rows;
	}

	std::string getCameraID(const std::string &inputName) {
		const auto frameInput        = inputs.getFrameInput(inputName);
		const auto originDescription = frameInput.getOriginDescription();
		static const std::regex filenameCleanupRegex{"[^a-zA-Z-_\\d]"};
		auto cameraID = std::regex_replace(originDescription, filenameCleanupRegex, "_");
		if ((cameraID[0] == '-') || (std::isdigit(cameraID[0]) != 0)) {
			cameraID = "_" + cameraID;
		}
		return cameraID;
	}

	bool isInputMaster(const std::string &inputName) const {
		for (const auto &child :
			inputs.getFrameInput(inputName).infoNode().getParent().getParent().getParent().getChildren()) {
			if (child.getName() == "sourceInfo") {
				return child.getAttribute<dv::Config::AttributeType::BOOL>("deviceIsMaster").value;
			}
		}
		return false;
	}

	cv::Size getInputResolution(const std::string &inputName) {
		const auto frameInput = inputs.getFrameInput(inputName);
		return {frameInput.sizeX(), frameInput.sizeY()};
	}

#if WITH_IMU_CALIBRATION
	std::tuple<float, float, cv::Point3f, cv::Point3f, float, float, float, float, float, float>
		getIMUCharacteristics() {
		float omega_max, acc_max, omega_offset_var, acc_offset_var, omega_noise_density, acc_noise_density,
			omega_noise_random_walk, acc_noise_random_walk;

		cv::Point3f omega_offset_avg, acc_offset_avg;

		const auto cameraId = getCameraID("left");
		if (cameraId.substr(0, 9) == "DVXplorer") {
			omega_max               = 34.89f;
			acc_max                 = 156.96f;
			omega_offset_avg        = {0, 0, 0};
			acc_offset_avg          = {0, 0, 0};
			omega_offset_var        = 0.03f;
			acc_offset_var          = 0.1f;
			omega_noise_density     = 8.0e-5f;
			acc_noise_density       = 1.0e-3f;
			omega_noise_random_walk = 4.0e-6f;
			acc_noise_random_walk   = 4.0e-5f;
		}
		else if (cameraId.substr(0, 5) == "DAVIS") {
			omega_max               = 7.8f;
			acc_max                 = 176.0f;
			omega_offset_avg        = {0, 0, 0};
			acc_offset_avg          = {0, 0, 0};
			omega_offset_var        = 0.03f;
			acc_offset_var          = 0.1f;
			omega_noise_density     = 0.00018f;
			acc_noise_density       = 0.002f;
			omega_noise_random_walk = 0.001f;
			acc_noise_random_walk   = 4.0e-5f;
		}
		else {
			log.warning << "Could not determine the camera type. IMU noise characteristics are unknown." << dv::logEnd;
		}

		return std::make_tuple(omega_max, acc_max, omega_offset_avg, acc_offset_avg, omega_offset_var, acc_offset_var,
			omega_noise_density, acc_noise_density, omega_noise_random_walk, acc_noise_random_walk);
	}

	dv::camera::calibrations::IMUCalibration getIMUCalibrationData(
		const IccCalibratorUtils::CalibrationResult &result) {
		const auto &[om, am, ooavg, aoavg, oovar, aovar, onden, anden, onrw, anrw] = getIMUCharacteristics();

		dv::kinematics::Transformationf transData{0, result.T_cam_imu.cast<float>()};

		std::stringstream ssCom;
		ssCom << "Time offset usage: t_correct = t_imu - offset"
			  << " Mean reprojection error: " << result.error_info.meanReprojectionError
			  << " Mean acc error: " << result.error_info.meanAccelerometerError
			  << " Mean gyroscope error: " << result.error_info.meanGyroscopeError;

		auto cal = dv::camera::calibrations::IMUCalibration(getCameraID("left"), om, am, ooavg, aoavg, oovar, aovar,
			onden, anden, onrw, anrw, static_cast<int64_t>(result.t_cam_imu * 1e+6), transData,
			dv::camera::calibrations::IMUCalibration::Metadata{mTimestampString, ssCom.str()});

		return cal;
	}
#endif

	void saveIntrinsicCalibration(const std::vector<CameraCalibrationUtils::CalibrationResult> &intrinsicResult) {
		const auto saveDir = getCalibrationSaveDirectory();

		const auto filePath = saveDir / "calibration.json";

		dv::camera::CalibrationSet calib;

		// todo(giovanni): make method out of this block -> reused in "saveCalibration"
		auto calibrationInfo = mCalibrator->getCalibrationInfo();
		PatternInfo patternInfo(config.getString("patternType"), cv::Size(getPatternRows(), getPatternColumns()),
			config.getFloat("markerSize"), config.getFloat("markerSpacing"));
		std::ostringstream optimizationInfo;
		optimizationInfo << "kalibr: " << calibrationInfo[0].numImagesUsed << " out of "
						 << calibrationInfo[0].numImagesTotal << " images used";

		addCalibration(calib, intrinsicResult[0], patternInfo, "left", "left", optimizationInfo.str());

		log.info << "Calibration quality left camera: " << calib.getCameraCalibration("left")->metadata->quality
				 << dv::logEnd;
		if (intrinsicResult.size() > 1) {
			optimizationInfo.clear();
			optimizationInfo << "kalibr: " << calibrationInfo[1].numImagesUsed << " out of "
							 << calibrationInfo[1].numImagesTotal << " images used";
			addCalibration(calib, intrinsicResult[1], patternInfo, "right", "right", optimizationInfo.str());

			log.info << "Calibration quality right camera: " << calib.getCameraCalibration("right")->metadata->quality
					 << dv::logEnd;
		}

		calib.writeToFile(filePath.string());

		log.info << "Saved intrinsic calibration to a file: " << filePath << dv::logEnd;
	}

	// Bullet-proof dump of the calibration result to plain-text files. Avoids any
	// dv-processing schema/serialization code so it can't be crashed by a bad cast,
	// schema validation, etc. Two files: raw_result.txt (human-readable) and
	// raw_result.json (loose JSON, no schema). Run before saveCalibration so the
	// numbers survive even if saveCalibration segfaults.
	void saveRawResult(const std::vector<CameraCalibrationUtils::CalibrationResult> &intrinsicResult,
		const IccCalibratorUtils::CalibrationResult &result) {
		try {
			const auto saveDir = getCalibrationSaveDirectory();

			std::ofstream txt(saveDir / "raw_result.txt");
			txt << "converged: " << (result.converged ? "true" : "false") << "\n";
			txt << "T_cam_imu (4x4 row-major):\n" << result.T_cam_imu << "\n";
			txt << "t_cam_imu_seconds: " << result.t_cam_imu << "\n";
			txt << "mean_reprojection_error_px: " << result.error_info.meanReprojectionError << "\n";
			txt << "mean_gyroscope_error_rad_s: " << result.error_info.meanGyroscopeError << "\n";
			txt << "mean_accelerometer_error_m_s2: " << result.error_info.meanAccelerometerError << "\n\n";
			for (size_t i = 0; i < intrinsicResult.size(); ++i) {
				const auto &r = intrinsicResult[i];
				txt << "camera[" << i << "].fx: " << r.projection.at(0) << "\n";
				txt << "camera[" << i << "].fy: " << r.projection.at(1) << "\n";
				txt << "camera[" << i << "].cx: " << r.projection.at(2) << "\n";
				txt << "camera[" << i << "].cy: " << r.projection.at(3) << "\n";
				for (size_t k = 0; k < r.distortion.size(); ++k) {
					txt << "camera[" << i << "].dist[" << k << "]: " << r.distortion[k] << "\n";
				}
				txt << "camera[" << i << "].baseline (4x4 row-major):\n" << r.baseline << "\n\n";
			}
			txt.flush();
			txt.close();

			std::ofstream js(saveDir / "raw_result.json");
			js << "{\n";
			js << "  \"converged\": " << (result.converged ? "true" : "false") << ",\n";
			js << "  \"t_cam_imu_seconds\": " << result.t_cam_imu << ",\n";
			js << "  \"T_cam_imu\": [";
			for (int row = 0; row < 4; ++row) {
				js << "[";
				for (int col = 0; col < 4; ++col) {
					js << result.T_cam_imu(row, col);
					if (col < 3) {
						js << ", ";
					}
				}
				js << "]";
				if (row < 3) {
					js << ", ";
				}
			}
			js << "],\n";
			js << "  \"errors\": {\n";
			js << "    \"reprojection_px\": " << result.error_info.meanReprojectionError << ",\n";
			js << "    \"gyro_rad_s\": " << result.error_info.meanGyroscopeError << ",\n";
			js << "    \"accel_m_s2\": " << result.error_info.meanAccelerometerError << "\n";
			js << "  },\n";
			js << "  \"cameras\": [\n";
			for (size_t i = 0; i < intrinsicResult.size(); ++i) {
				const auto &r = intrinsicResult[i];
				js << "    {\n";
				js << "      \"fx\": " << r.projection.at(0) << ", \"fy\": " << r.projection.at(1)
				   << ", \"cx\": " << r.projection.at(2) << ", \"cy\": " << r.projection.at(3) << ",\n";
				js << "      \"dist\": [";
				for (size_t k = 0; k < r.distortion.size(); ++k) {
					js << r.distortion[k];
					if (k + 1 < r.distortion.size()) {
						js << ", ";
					}
				}
				js << "]\n";
				js << "    }" << (i + 1 < intrinsicResult.size() ? "," : "") << "\n";
			}
			js << "  ]\n";
			js << "}\n";
			js.flush();
			js.close();

			log.info << "Wrote raw calibration dump to " << saveDir << "/raw_result.{txt,json}" << dv::logEnd;
		}
		catch (const std::exception &ex) {
			log.error << "saveRawResult failed: " << ex.what() << dv::logEnd;
		}
	}

	void saveCalibration(const std::vector<CameraCalibrationUtils::CalibrationResult> &intrinsicResult,
		const IccCalibratorUtils::CalibrationResult &result) {
		log.info << "Saving calibration..." << dv::logEnd;
		const auto saveDir = getCalibrationSaveDirectory();

		const auto filePath = saveDir / "calibration.json";

		dv::camera::CalibrationSet calib;
		auto calibrationInfo = mCalibrator->getCalibrationInfo();
		PatternInfo patternInfo(config.getString("patternType"), cv::Size(getPatternRows(), getPatternColumns()),
			config.getFloat("markerSize"), config.getFloat("markerSpacing"));
		std::ostringstream optimizationInfo;
		optimizationInfo << "kalibr: " << calibrationInfo[0].numImagesUsed << " out of "
						 << calibrationInfo[0].numImagesTotal << " images used";
		addCalibration(calib, intrinsicResult[0], patternInfo, "left", "left", optimizationInfo.str());

		log.info << "Calibration quality left camera: " << calib.getCameraCalibration("left")->metadata->quality
				 << dv::logEnd;
		if (intrinsicResult.size() > 1) {
			optimizationInfo.clear();
			optimizationInfo << "kalibr: " << calibrationInfo[1].numImagesUsed << " out of "
							 << calibrationInfo[1].numImagesTotal << " images used";
			addCalibration(calib, intrinsicResult[1], patternInfo, "right", "right", optimizationInfo.str());

			log.info << "Calibration quality right camera: " << calib.getCameraCalibration("right")->metadata->quality
					 << dv::logEnd;
		}

#if WITH_IMU_CALIBRATION
		calib.addImuCalibration(getIMUCalibrationData(result));
#endif

		calib.writeToFile(filePath.string());

		log.info << "Saved calibration to a file: " << filePath << dv::logEnd;
	}

	void calibrate(const bool calibrateImu) {
		if (config.getBool("recordData")) {
			mDataLog = std::make_unique<dv::io::MonoCameraWriter>(
				fs::path(getCalibrationSaveDirectory()) / "data.aedat4", mWriterConfig);
		}

		std::ofstream outLog(getCalibrationSaveDirectory() / "log.txt");
		outLog << "Calibrating begins..." << std::endl;

		mCalibrator->print(outLog);

		const auto &rdbuf = std::cout.rdbuf();
		std::cout.rdbuf(outLog.rdbuf());

		log.info("Calibrating the intrinsics of the camera...");
		auto intrinsicsResult = mCalibrator->calibrateCameraIntrinsics();
		if (!intrinsicsResult.has_value()) {
			throw dv::exceptions::RuntimeError(
				"Failed to calibrate intrinsics! Please check that the pattern was well detected on the images");
		}

#if WITH_IMU_CALIBRATION
		if (calibrateImu) {
			outLog << "Building the problem..." << std::endl;
			mCalibrator->buildProblem();

			mCalibrator->getDvInfoBeforeOptimization(outLog);

			outLog << "Optimizing..." << std::endl;
			std::optional<IccCalibratorUtils::CalibrationResult> resultOpt;
			try {
				resultOpt = mCalibrator->calibrate();
				mCalibrator->getDvInfoAfterOptimization(outLog);
			}
			catch (const std::exception &ex) {
				outLog << "Optimization threw: " << ex.what() << std::endl;
				log.error << "Optimization threw: " << ex.what() << dv::logEnd;
			}

			if (resultOpt.has_value()) {
				outLog << "RESULT" << std::endl;
				IccCalibratorUtils::printResult(*resultOpt, outLog);
				outLog.flush();

				// Fail-safe: dump all numerical values to plain-text files BEFORE
				// invoking the dv-processing schema-formatted save. If saveCalibration
				// crashes (segfault inside dv-processing), the user still has the
				// numbers on disk to recover by hand.
				saveRawResult(intrinsicsResult.value(), *resultOpt);

				try {
					saveCalibration(intrinsicsResult.value(), *resultOpt);
					config.setBool("calibrationFinished", true);
					collectionState = CALIBRATED;
					if (!resultOpt->converged) {
						log.warning
							<< "Optimization did not fully converge (hit maxIter). "
							   "Calibration was still saved; consider re-recording with more motion."
							<< dv::logEnd;
					}
				}
				catch (const std::exception &saveEx) {
					outLog << "Save failed: " << saveEx.what() << std::endl;
					log.error << "Save failed: " << saveEx.what() << dv::logEnd;
					// Even if the schema-formatted save failed, the raw dump is on disk.
					config.setBool("calibrationFinished", true);
					collectionState = CALIBRATED;
				}
			}
			else {
				log.error << "Optimization aborted; nothing to save. Re-record with more motion." << dv::logEnd;
				initializeCalibrator();
				collectionState = BEFORE_COLLECTING;
			}
		}
		else
#endif
		{
			saveIntrinsicCalibration(intrinsicsResult.value());
			config.setBool("calibrationFinished", true);
			collectionState = CALIBRATED;
		}
		std::cout.rdbuf(rdbuf);
	}

	void addCalibration(dv::camera::CalibrationSet &calib, const CameraCalibrationUtils::CalibrationResult &res,
		PatternInfo patternInfo, const std::string &position, const std::string &inputName,
		const std::string &comment = "", cv::Size resolution = cv::Size(640, 480), bool isMaster = true) {
		if (mCalibrationModel == ImuCamModelTypes::EQUIDISTANT) {
			calib.addCameraCalibration(getIntrinsicCalibrationData<aslam::cameras::EquidistantDistortion>(
				res, patternInfo, position, inputName, comment));
		}
		else if (mCalibrationModel == ImuCamModelTypes::RADTAN) {
			calib.addCameraCalibration(getIntrinsicCalibrationData<aslam::cameras::RadialTangentialDistortion>(
				res, patternInfo, position, inputName, comment));
		}
		else {
			throw std::invalid_argument("Unexpected model name encountered when running module");
		}
	}
};

registerModuleClass(ImuCamCalibration)
