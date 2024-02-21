#include "CLI/CLI.hpp"
#include "run_camera_calibration.hpp"

int main(int ac, char **av) {
	std::string deviceRecordingPath;
	std::string outputFilepath;
	int32_t width;
	int32_t height;

	// Use CLI11 library to handle argument parsing
	CLI::App app{"Runs camera calibration from all different patterns."};
	app.add_option("-f,--deviceRecordingPath", deviceRecordingPath,
		   "full path to folder (ex: dvxplorer) containing multiple recordings using different patterns (ex: "
		   "april-tag-0.5m, asymmetric-grid-0.5m, checkerboard-0.5m)")
		->required();
	app.add_option("-o,--outputFolder", outputFilepath,
		   "path to folder that will contain different calibration folders, each one will contain the different "
		   "calibrations obtained from a single pattern")
		->required();
	app.add_option("--width", width, "number of pixels along the width of sensor")->default_val(640);
	app.add_option("--height", height, "number of pixels along the height of sensor")->default_val(480);

	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	runCameraCalibrationForAllPatterns(deviceRecordingPath, outputFilepath, width, height);

	return EXIT_SUCCESS;
}
