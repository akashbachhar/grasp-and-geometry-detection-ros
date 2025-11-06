#include <string>
#include <fstream>
#include <Eigen/Dense>

#include <gpd/grasp_detector.h>
#include <gpd/candidate/hand.h>

namespace gpd {
namespace apps {
namespace detect_grasps {

bool checkFileExists(const std::string &file_name) {
std::ifstream file;
file.open(file_name.c_str());
if (!file) {
std::cout << "File " + file_name + " could not be found!\n";
return false;
}
file.close();
return true;
}

int DoMain(int argc, char *argv[]) {
if (argc < 3) {
std::cout << "Error: Not enough input arguments!\n\n";
std::cout << "Usage: detect_grasps CONFIG_FILE PCD_FILE [NORMALS_FILE]\n\n";
return (-1);
}

std::string config_filename = argv[1];
std::string pcd_filename = argv[2];

if (!checkFileExists(config_filename) || !checkFileExists(pcd_filename)) {
return (-1);
}

util::ConfigFile config_file(config_filename);
config_file.ExtractKeys();

std::vector<double> camera_position =
config_file.getValueOfKeyAsStdVectorDouble("camera_position", "0.0 0.0 0.0");
Eigen::Matrix3Xd view_points(3, 1);
view_points << camera_position[0], camera_position[1], camera_position[2];

util::Cloud cloud(pcd_filename, view_points);
if (cloud.getCloudOriginal()->size() == 0) {
std::cout << "Error: Input point cloud is empty or does not exist!\n";
return (-1);
}

if (argc > 3) {
std::string normals_filename = argv[3];
cloud.setNormalsFromFile(normals_filename);
std::cout << "Loaded surface normals from file: " << normals_filename << "\n";
}

GraspDetector detector(config_filename);
detector.preprocessPointCloud(cloud);

bool centered_at_origin =
config_file.getValueOfKey<bool>("centered_at_origin", false);
if (centered_at_origin) {
printf("Reversing normal directions ...\n");
cloud.setNormals(cloud.getNormals() * (-1.0));
}

auto grasps = detector.detectGrasps(cloud);
std::cout << "Detected " << grasps.size() << " grasps." << std::endl;

// Save to CSV
std::string save_path = std::string(std::getenv("HOME")) + "/Documents/grasp_poses.csv";
std::ofstream csv_file(save_path);

// Header
csv_file << "id,score,width,pos_x,pos_y,pos_z,axis_x,axis_y,axis_z,approach_x,approach_y,approach_z,binormal_x,binormal_y,binormal_z\n";

for (size_t i = 0; i < grasps.size(); ++i) {
const auto &grasp = grasps[i];

Eigen::Vector3d position = grasp->getPosition();
Eigen::Vector3d axis = grasp->getAxis(); // closing direction
Eigen::Vector3d approach = grasp->getApproach(); // approach direction
Eigen::Vector3d binormal = axis.cross(approach); // binormal = side direction

csv_file << i << ","
<< grasp->getScore() << ","
<< grasp->getGraspWidth() << ","
<< position[0] << "," << position[1] << "," << position[2] << ","
<< axis[0] << "," << axis[1] << "," << axis[2] << ","
<< approach[0] << "," << approach[1] << "," << approach[2] << ","
<< binormal[0] << "," << binormal[1] << "," << binormal[2] << "\n";
}

csv_file.close();
std::cout << "Grasp poses saved to: " << save_path << std::endl;

return 0;
}

} // namespace detect_grasps
} // namespace apps
} // namespace gpd

int main(int argc, char *argv[]) {
return gpd::apps::detect_grasps::DoMain(argc, argv);
}