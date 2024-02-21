#include "CLI/CLI.hpp"
#include "run_camera_calibration.hpp"

int main(int ac, char **av) {
	std::string devicesFolderPath;
	std::string outputFilepath;

	// Use CLI11 library to handle argument parsing
	CLI::App app{"Runs camera calibration for all different devices."};
	app.add_option("-f,--devicesFolderPath", devicesFolderPath,
		   "Full path to folder (ex: recordings) containing multiple device folders (ex: davis, dvx-micro, dvxplorer)")
		->required();
	app.add_option("-o,--outputFolder", outputFilepath,
		   "Path to folder that will contain different calibration folders, each one will contain the different "
		   "calibrations obtained for a single device")
		->required();

	try {
		app.parse(ac, av);
	}
	catch (const CLI::ParseError &e) {
		return app.exit(e);
	}

	runCameraCalibrationForAllDevices(devicesFolderPath, outputFilepath);

	return EXIT_SUCCESS;
}
