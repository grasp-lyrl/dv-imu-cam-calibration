#include "CLI/CLI.hpp"
#include "run_camera_calibration.hpp"

int main(int ac, char **av) {
	std::string framesPath;
	std::string patternPath;
	std::string outputFilepath;
	int32_t width;
	int32_t height;

	// Use CLI11 library to handle argument parsing
	CLI::App app{"Runs camera calibration from all different recordings from the same pattern given "
				 "provided sensor and pattern info."};
	app.add_option("-f,--framesPath", framesPath,
		   "Full path to folder (ex: april-tag-0.5m) containing multiple recordings (ex: rec1, rec2, rec3) using the "
		   "same pattern.")
		->required();
	app.add_option("-o,--outputFolder", outputFilepath,
		   "Path to folder that will contain all different calibration files, each one will contain the calibration "
		   "obtained from a single set of frames.")
		->required();
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

	runCameraCalibrationForAllRecordings(framesPath, outputFilepath, patternPath, width, height);

	return EXIT_SUCCESS;
}
