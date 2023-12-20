#include "utilities/Calibrator.hpp"

#include <aslam/cameras.hpp>

#include "CLI/CLI.hpp"

#include <dv-processing/camera/calibration_set.hpp>
#include <dv-processing/core/multi_stream_slicer.hpp>
#include <dv-processing/io/mono_camera_recording.hpp>
#include <dv-processing/io/mono_camera_writer.hpp>

#include <cstdlib>
#include <fstream>
#include <regex>
#include <string>
#include <thread>

int main(int ac, char **av) {
	std::string filepath;
	int numPatternRows;
	int numPatternCols;
	float markerSize;
	float markerSpacing;
	std::string outputFilepath;
	cv::Size resolution(640, 480);

	// Use CLI11 library to handle argument parsing
	CLI::App app{""};
	app.add_option("-f,--filepath", filepath, "");
	app.add_option("-r,--row", numPatternRows, "Number of points to detect along one row")->required();
	app.add_option("-c,--col", numPatternCols, "Number of points to detect along one column")->required();
	app.add_option("--size", markerSize, "Size of a single marker on calibration pattern [m]")->required();
	app.add_option("--spacing", markerSpacing, "Space proportion wrt marker size")->required();
	app.add_option("-o,--outputFile", outputFilepath, "Absolute filepath where to save the calibration file");

	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	dv::runtime_assert(!outputFilepath.empty(), "Empty path to save calibration file.");

	PatternInfo patternInfo("apriltag", cv::Size(numPatternRows, numPatternCols), markerSize, markerSpacing);
	dv::io::MonoCameraRecording reader(filepath);
	dv::FrameStreamSlicer slicer;

	CalibratorUtils::Options options;

	boost::circular_buffer<dv::Frame> mLeftFrames(5);
	std::optional<boost::circular_buffer<dv::Frame>> mRightFrames;

	options = CalibratorUtils::Options();
	options.cameraInitialSettings.emplace_back();
	options.cameraInitialSettings[0].imageSize = resolution;
	options.pattern                            = CalibratorUtils::PatternType::APRIL_GRID;
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
	calib.addCameraCalibration(
		getIntrinsicCalibrationData(result, patternInfo, "left", "left", optimizationInfo.str()));

	calib.writeToFile(outputFilepath);

	return EXIT_SUCCESS;
}
