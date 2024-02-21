import pathlib
import json
import subprocess
from collections.abc import Callable


def get_json_path_from_aedat4_path(aedat4_path: str) -> str:
    """
    Manipulate aedat4 path to get corresponding json path
    :param aedat4_path: path to aeda4 file containing frames
    :return: path to json file containing corresponding calibration data
    """
    json_path = aedat4_path.replace("/reconstructed/", "/").replace("recordings",
                                                                    "intrinsic-results").replace(".aedat4", ".json")
    return json_path


def read_error_from_file(aedat4_path: str) -> float | None:
    """
    Read calibration error from calibration file
    :param aedat4_path: path to aedat4 frames file
    :return: calibration error as float
    """
    json_path = get_json_path_from_aedat4_path(aedat4_path)
    if not pathlib.Path(json_path).is_file():
        return None
    with open(json_path, "r") as f:
        calibration_data = json.load(f)
    error = calibration_data["cameras"]["C0"]["metadata"]["calibrationError"]
    return error


def read_detection_count_from_file(aedat4_path: str) -> int | None:
    """
    Read number of detections obtained before calibration from calibration file
    :param aedat4_path: path to aedat4 frames file
    :return: detection count
    """
    json_path = get_json_path_from_aedat4_path(aedat4_path)
    if not pathlib.Path(json_path).is_file():
        return None
    with open(json_path, "r") as f:
        calibration_data = json.load(f)
    comment = calibration_data["cameras"]["C0"]["metadata"]["comment"]
    detection_count = int(comment.split(" ")[-3])
    return detection_count


def read_used_count_from_file(aedat4_path: str) -> int | None:
    """
    Read number of used images for calibration from calibration file
    :param aedat4_path: path to aedat4 frames file
    :return: used count
    """
    json_path = get_json_path_from_aedat4_path(aedat4_path)
    if not pathlib.Path(json_path).is_file():
        return None
    with open(json_path, "r") as f:
        calibration_data = json.load(f)
    comment = calibration_data["cameras"]["C0"]["metadata"]["comment"]
    used_count = int(comment.split(" ")[1])
    return used_count


def read_frame_count_from_file(aedat4_path: str) -> int:
    """
    Read number of frames from frames file
    :param aedat4_path: path to aedat4 frames file
    :return: number of frames contained in aedat4 file
    """
    # Run dv-filestat on frames file (from docker)
    frames_file_docker = aedat4_path.replace("/home/calibration/calibration/data/", "/data/")
    command = f"docker run -v /home/calibration/calibration/data/:/data/ run_calibration dv-filestat {frames_file_docker}"
    outputs = subprocess.check_output(command.split(" ")).decode("utf-8").split("\n")
    # Convert dv-filestat output to single frame count integer
    frame_count = [int(row.split(" ")[-1]) for row in outputs if "DataTable elements" in row][-1]
    return frame_count


def read_store_all_info(source_path: pathlib.Path, get_info: Callable[[str], any]) -> dict:
    """
    Read some info from all files in folder and store them accordingly in a dictionary
    :param source_path: path to source folder where to find all files
    :param get_info: function that will retrieve the desired info for each file
    :return: the dictionary containing all calibration info stored as:
    device -> pattern -> distance -> recording -> method -> info
    """
    all_info_dict = {}
    # Loop over device folders (dvxplorer, davis, dvx-micro)
    for device_folder in sorted(source_path.iterdir()):
        if not device_folder.is_dir():
            continue
        device_name = device_folder.name
        all_info_dict[device_name] = {}
        # Loop over pattern-distance folders (april-tag-0.5m, april-tag-1m, april-tag-3m, april-tag-5m, asymmetric-grid-0.5m, checkerboard-0.5m)
        for pattern_folder in sorted(device_folder.iterdir()):
            if not pattern_folder.is_dir():
                continue
            # Get pattern name and distance by splitting folder name (ex: april-tag-0.5m -> april-tag and 0.5m)
            pattern_split = pattern_folder.name.split("-")
            pattern_name = "-".join(pattern_split[:-1])
            if pattern_name not in all_info_dict[device_name]:
                all_info_dict[device_name][pattern_name] = {}
            distance = pattern_split[-1]
            all_info_dict[device_name][pattern_name][distance] = {}
            # Loop over recordings folders (rec1, rec2, rec3)
            for rec_folder in sorted(pattern_folder.iterdir()):
                if not rec_folder.is_dir():
                    continue
                rec_name = rec_folder.name
                all_info_dict[device_name][pattern_name][distance][rec_name] = {}
                reconstructed_folder = rec_folder / "reconstructed"
                # Loop over all methods json files
                for file_path in sorted(reconstructed_folder.iterdir()):
                    method_name = str(file_path.name).split("_")[0]
                    error = get_info(str(file_path))
                    # Store info as a string at the corresponding logical position
                    all_info_dict[device_name][pattern_name][distance][rec_name][method_name] = str(error)

    return all_info_dict


def print_info_as_csv(info_dict: dict):
    """
    Print info dictionary as a double entry table
    :param info_dict: the dictionary containing the info
    """
    # All method names to ensure values are printed even if not present, and sorted for consistency
    method_names = [
        "E2VID", "E2VID+", "ET-Net", "FireNet", "FireNet+", "HyperE2VID", "SPADE-E2VID", "SSL-E2VID", "accumulator"
    ]

    for device_name in info_dict.keys():
        device_dict = info_dict[device_name]
        print()
        print(device_name)
        for pattern_name in device_dict.keys():
            pattern_dict = device_dict[pattern_name]
            print()
            print(pattern_name)
            distances_row = ["distance"]
            recs_row = ["recording"]
            method_rows = {}
            for distance in pattern_dict.keys():
                distance_dict = pattern_dict[distance]
                for rec_name in distance_dict.keys():
                    distances_row.append(distance)
                    recs_row.append(rec_name)
                    rec_dict = distance_dict[rec_name]
                    for method_name in method_names:
                        if method_name not in method_rows:
                            method_rows[method_name] = [method_name]
                        if method_name not in rec_dict.keys():
                            info = "None"
                        else:
                            info = rec_dict[method_name]
                        method_rows[method_name].append(info)
            print(",".join(distances_row))
            print(",".join(recs_row))
            for method_row in method_rows.values():
                print(",".join(method_row))


def main():
    source = pathlib.Path("/home/calibration/calibration/data/recordings")

    print("Reprojection errors:")
    all_errors_dict = read_store_all_info(source, read_error_from_file)
    print_info_as_csv(all_errors_dict)
    print()
    print("===========================================================================================================")

    print("Detection counts:")
    all_detection_count_dict = read_store_all_info(source, read_detection_count_from_file)
    print_info_as_csv(all_detection_count_dict)
    print()
    print("===========================================================================================================")

    print("Used counts:")
    all_used_count_dict = read_store_all_info(source, read_used_count_from_file)
    print_info_as_csv(all_used_count_dict)
    print()
    print("===========================================================================================================")

    print("Frame Counts:")
    all_frame_count_dict = read_store_all_info(source, read_frame_count_from_file)
    print_info_as_csv(all_frame_count_dict)


if __name__ == '__main__':
    main()
