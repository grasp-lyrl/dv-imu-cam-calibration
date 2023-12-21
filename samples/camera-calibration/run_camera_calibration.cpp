#include "utilities/Calibrator.hpp"

#include <aslam/cameras.hpp>

#include "CLI/CLI.hpp"

#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/core/multi_stream_slicer.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include <dv-processing/io/mono_camera_writer.hpp>

#include <boost/property_tree/json_parser.hpp>

#include <cstdlib>
#include <fstream>
#include <string>

using namespace boost::property_tree;

/**
 * @param framesPath path to file containing frames with patterns to be detected for calibration
 * @param outputFilepath path to file where to save the calibration
 * @param patternPath path to json file containing calibration pattern information (name, shape, dimensions)
 * @param width number of pixels along the width of sensor
 * @param height number of pixels along the height of sensor
 */
void runCameraCalibration(const std::string &framesPath, const std::string &outputFilepath,
	const std::string &patternPath, const int32_t width, const int32_t height);

int main(int ac, char **av) {
	std::string framesPath;
	std::string patternPath;
	std::string outputFilepath;
	int32_t width;
	int32_t height;

	// Use CLI11 library to handle argument parsing
	CLI::App app{""};
	app.add_option("-f,--framesPath", framesPath,
		   "Path to file containing frames with patterns to be detected for calibration.")
		->required();
	app.add_option("-o,--outputFile", outputFilepath, "Path to file where to save the calibration.")->required();
	app.add_option("-p,--pattern", patternPath,
		   "Path to json file containing calibration pattern information (name, shape, dimensions)")
		->required();
	app.add_option("--width", width, "Number of pixels along the width of sensor.")->default_val(640);
	app.add_option("--height", height, "Number of pixels along the height of sensor.")->default_val(480);

	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	runCameraCalibration(framesPath, outputFilepath, patternPath, width, height);

	return EXIT_SUCCESS;
}

void runCameraCalibration(const std::string &framesPath, const std::string &outputFilepath,
	const std::string &patternPath, const int32_t width, const int32_t height) {
	ptree patternTree;
	read_json(patternPath, patternTree);

	const auto patternName    = patternTree.get<std::string>("name");
	const auto numPatternRows = patternTree.get<int32_t>("rows");
	const auto numPatternCols = patternTree.get<int32_t>("cols");
	const auto markerSize     = patternTree.get<float>("size");
	const auto markerSpacing  = patternTree.get<float>("spacing");

	CalibratorUtils::PatternType patternType;
	if (patternName == "CHESSBOARD") {
		patternType = CalibratorUtils::PatternType::CHESSBOARD;
	}
	else if (patternName == "ASYMMETRIC_CIRCLES_GRID") {
		patternType = CalibratorUtils::PatternType::ASYMMETRIC_CIRCLES_GRID;
	}
	else if (patternName == "APRIL_GRID") {
		patternType = CalibratorUtils::PatternType::APRIL_GRID;
	}
	else {
		throw std::invalid_argument(
			"Pattern type name not supported, should be one of CHESSBOARD, ASYMMETRIC_CIRCLES_GRID, APRIL_GRID");
	}

	PatternInfo patternInfo(patternName, cv::Size(numPatternRows, numPatternCols), markerSize, markerSpacing);
	dv::io::MonoCameraRecording reader(framesPath);
	dv::FrameStreamSlicer slicer;

	CalibratorUtils::Options options;

	boost::circular_buffer<dv::Frame> mLeftFrames(5);
	std::optional<boost::circular_buffer<dv::Frame>> mRightFrames;

	options = CalibratorUtils::Options();
	options.cameraInitialSettings.emplace_back();
	options.cameraInitialSettings[0].imageSize = {width, height};
	options.pattern                            = patternType;
	options.cols                               = numPatternCols;
	options.rows                               = numPatternRows;
	options.spacingMeters                      = markerSize;
	options.patternSpacing                     = markerSpacing;
	options.maxIter                            = static_cast<size_t>(50);

	auto calibrator
		= Calibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>(
			options);

	slicer.doEveryNumberOfElements(1, [&](const auto &frames) {
		// Retrieve frames, although we get one frame per slice, it is stored in the configured container

		// Process frame input
		mLeftFrames.push_back(frames.at(0));

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

			calibrator.addImages(images);

			mLeftFrames.pop_front();
		}
	});

	calibrator.startCollecting();
	while (reader.isRunning()) {
		const auto frame = reader.getNextFrame();
		if (frame.has_value()) {
			slicer.accept(*frame);
		}
	}
	calibrator.stopCollecting();

	std::cout << "Calibrating the intrinsics of the camera..." << std::endl;
	auto start = std::chrono::system_clock::now();

	auto intrinsicsResult = calibrator.calibrateCameraIntrinsics();
	auto end              = std::chrono::system_clock::now();

	std::chrono::duration<double> elapsed_seconds = end - start;
	std::time_t end_time                          = std::chrono::system_clock::to_time_t(end);
	std::cout << "Finished. Calibration took " << elapsed_seconds.count() << "s" << std::endl;

	if (!intrinsicsResult.has_value()) {
		throw dv::exceptions::RuntimeError(
			"Failed to calibrate intrinsics! Please check that the pattern was well detected on the images");
	}

	auto calibrationInfo = calibrator.getCalibrationInfo()[0];
	auto result          = intrinsicsResult->at(0);

	// SAVE CALIBRATION
	dv::camera::CalibrationSet calib;
	std::ostringstream optimizationInfo;
	optimizationInfo << "kalibr: " << calibrationInfo.numImagesUsed << " out of " << calibrationInfo.numImagesTotal
					 << " images used";
	calib.addCameraCalibration(getIntrinsicCalibrationData<aslam::cameras::RadialTangentialDistortion>(
		result, patternInfo, "left", "left", optimizationInfo.str()));

	calib.writeToFile(outputFilepath);
}
