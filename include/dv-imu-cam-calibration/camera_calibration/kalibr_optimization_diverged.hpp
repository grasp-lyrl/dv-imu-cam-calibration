#include <stdexcept>

/**
 * Kalibr corresponding class in CameraCalibrator.py
 */
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
