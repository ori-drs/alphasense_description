#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>

void yamlMatrixToEigen(const YAML::Node& matrix_node, Eigen::Matrix4d& T_out){
  int rows = 4;
  int cols = 4;


  for(int i = 0; i < rows; i++){
    std::vector<double> row = matrix_node[i].as<std::vector<double>>();
    for(int j = 0; j < cols; j++){
      T_out(i,j) = row[j];
    }
  }
  std::cout << "Converted matrix \n " << T_out << std::endl;
}




void poseToYaml(YAML::Emitter& emitter,
                std::string name,
                const Eigen::Vector3d& xyz,
                const Eigen::Vector3d& rpy,
                std::string comment = "")
{
  emitter << YAML::Value << YAML::BeginMap << YAML::Key << name;
  std::vector<std::string> coordinates = {"x", "y", "z"};
  std::vector<std::string> euler_names = {"roll", "pitch", "yaw"};

  emitter << YAML::Value << YAML::BeginMap;
  for(size_t i = 0; i < 3; i++){

    emitter << YAML::Key << coordinates[i];
    emitter << YAML::Value << xyz(i);
    if(comment != ""){
      emitter << YAML::Comment(comment);
    }
  }

  for(size_t i = 0; i < 3; i++){
    emitter << YAML::Key << euler_names[i];
    emitter << YAML::Value << rpy(i);
    if(comment != ""){
      emitter << YAML::Comment(comment);
    }
  }
  emitter << YAML::EndMap;
  emitter << YAML::EndMap;
  emitter << YAML::Newline;
  emitter << YAML::Newline;
}

using namespace boost::filesystem;

int main(int argc, char** argv){

  if(argc < 3){
    std::cerr << "Wrong number of arguments. Usage: kalibr_urdf_calibration_converter imu_camera_calib_in.yaml urdf_calib_out.yaml" << std::endl;
    std::cerr << "Wrong number of arguments. Alternative Usage: kalibr_urdf_calibration_converter /absolute/path/to/multiple/kalibr/configs/ urdf_calib_out.yaml" << std::endl;
    return -1;
  }

  std::string extrinsics_path_str = argv[1];
  path extrinsics_path = path(extrinsics_path_str);

  std::string file_out = argv[2];
  if(!is_regular_file(extrinsics_path) && !is_directory(extrinsics_path)){
    std::cerr << "Can't open input file or directory " << extrinsics_path_str << " . Quitting ..." << std::endl;
    return -1;
  }

  std::set<std::string> yaml_filenames_sorted;
  if(is_directory(extrinsics_path)){
    for (const auto & entry : directory_iterator(extrinsics_path)){
      //std::cout << "Found file " << entry.path().string() << std::endl;
      // the valid file should have yaml extension and should contain the string "camchain-imucam"
      // in the filename. Note the files will be
      if(entry.path().extension() == ".yaml" && entry.path().string().find("camchain-imucam") != std::string::npos){
        std::cout << "Found file " << entry.path().string() << " to be processed." << std::endl;
        yaml_filenames_sorted.insert(entry.path().string());
      }
    }
  } else {
    // if we are here, we have a single file to process
    yaml_filenames_sorted.insert(extrinsics_path_str);
  }

  YAML::Emitter output;
  output.SetIndent(2);
  output.SetDoublePrecision(12);

  output.SetBoolFormat(YAML::TrueFalseBool);

  output << YAML::Comment("File generated from " + extrinsics_path_str);
  output << YAML::Newline;
/*  output << YAML::BeginMap;
  output << YAML::Key << "parent_to_imu";

  poseToYaml(output, Eigen::Vector3d(0.005303,
                                     0.037340,
                                     0.063319),
             Eigen::Quaterniond(1,0,0,0),
             "Data is obtained from CAD");




  output << YAML::Key << "base_to_mesh";
  poseToYaml(output, Eigen::Vector3d(-0.364, 0.0, 0.0),
             Eigen::Quaterniond(1,0,0,0),
             "Data is obtained from CAD");
*/
  for(auto& item : yaml_filenames_sorted){
    std::cout << "Processing file " << item << std::endl;

    YAML::Node extrinsics_node;
    try {
      extrinsics_node = YAML::LoadFile(item);
    } catch (...) {
      throw std::invalid_argument("Error reading config file: " + extrinsics_path_str);
    }


    for(size_t cam = 0; cam < extrinsics_node.size(); cam++){
      std::string camera_name = "cam" + std::to_string(cam);
      YAML::Node camera_extrinsics_node = extrinsics_node[camera_name];
      std::string camera_name_from_topic = camera_extrinsics_node["rostopic"].as<std::string>();

      // find where does "cam" appear in the topic name
      size_t cam_name_from_topic_start = camera_name_from_topic.find("cam");
      if(cam_name_from_topic_start != std::string::npos){
        size_t cam_name_from_topic_end = camera_name_from_topic.find("/", cam_name_from_topic_start);
        camera_name_from_topic = camera_name_from_topic.substr(cam_name_from_topic_start, cam_name_from_topic_end-cam_name_from_topic_start);

        if(camera_name != camera_name_from_topic){
          std::cerr << "WARNING: the camera name indicated in the yaml differs from the one indicated in the topic" << std::endl;
          std::cerr << camera_name << " != " << camera_name_from_topic << std::endl;
          std::cerr << "SETTING camera name to: " << camera_name_from_topic << std::endl;
          camera_name = camera_name_from_topic;
        }
      }

      YAML::Node camera_extrinsics_node_T = camera_extrinsics_node["T_cam_imu"];
      Eigen::Matrix4d T_cam_imu_eigen;
      yamlMatrixToEigen(camera_extrinsics_node_T, T_cam_imu_eigen);

      Eigen::Matrix4d T_imu_cam_eigen = T_cam_imu_eigen.inverse();

      //Eigen::Isometry3d transform_eigen = Eigen::Isometry3d::Identity();
      Eigen::Matrix3d rotation_eigen = T_imu_cam_eigen.block<3,3>(0,0);
      Eigen::Vector3d I_r_IC = T_imu_cam_eigen.block<3,1>(0,3);

      Eigen::Vector3d rpy = rotation_eigen.eulerAngles(2, 1, 0).reverse();

      Eigen::Matrix3d rotation_eigen_orthogonalized = rotation_eigen *
          (rotation_eigen.transpose() * rotation_eigen).sqrt().inverse();


      if(!rotation_eigen.isApprox(rotation_eigen_orthogonalized)){
        std::cerr << "WARNING: the rotation matrix provided by calibration is not orthogonal!" << std::endl;
        std::cerr << "Using the orthogonalized version of the matrix for now, but you should re-run the calibration!" << std::endl;
        rotation_eigen = rotation_eigen_orthogonalized;
      } else {
        std::cerr << "Orthogonality check passed!" << std::endl;
      }

      poseToYaml(output, "imu_to_" + camera_name, I_r_IC, rpy);

    }
  }
  output << YAML::EndMap;


  std::ofstream fout(file_out);
  fout << output.c_str();
  fout.close();
  return 0;
}
