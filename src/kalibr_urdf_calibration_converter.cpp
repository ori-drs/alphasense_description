#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>


void poseToYaml(YAML::Emitter& emitter,
                std::string name,
                const Eigen::Vector3d& pose,
                const Eigen::Quaterniond& quat,
                std::string comment = "")
{
  emitter << YAML::Value << YAML::BeginMap << YAML::Key << name;
  std::vector<std::string> coordinates = {"x", "y", "z"};
  std::vector<std::string> euler_names = {"roll", "pitch", "yaw"};

  emitter << YAML::Value << YAML::BeginMap;
  for(size_t i = 0; i < 3; i++){

    emitter << YAML::Key << coordinates[i];
    emitter << YAML::Value << pose(i);
    if(comment != ""){
      emitter << YAML::Comment(comment);
    }
  }

  // quat to euler, ZYX convention (not used)
  //Eigen::Vector3d rpy = quat.toRotationMatrix().eulerAngles(2, 1, 0).reverse();

  // quat to Euler, XYZ convention
  Eigen::Vector3d rpy = quat.toRotationMatrix().eulerAngles(0, 1, 2);

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

      int rows = 4;
      int cols = 4;

      Eigen::Matrix4d T_cam_imu_eigen;


      for(int i = 0; i < rows; i++){
        std::vector<double> row = camera_extrinsics_node_T[i].as<std::vector<double>>();
        for(int j = 0; j < cols; j++){
          T_cam_imu_eigen(i,j) = row[j];
        }
      }


      //Eigen::Isometry3d transform_eigen = Eigen::Isometry3d::Identity();
      Eigen::Matrix3d rotation_eigen = T_cam_imu_eigen.block<3,3>(0,0);
      Eigen::Vector3d translation_eigen = T_cam_imu_eigen.block<3,1>(0,3);
      Eigen::Quaterniond rotation_q(rotation_eigen);
      // normalize the quaternion to ensure it is represents a valid rotation
      rotation_q = rotation_q.normalized();

      Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();

      iso.translate(translation_eigen);
      iso.rotate(rotation_q);

      iso = iso.inverse(); // iso = T_IC;

      Eigen::Vector3d I_r_IC = iso.translation();
      Eigen::Quaterniond q_IC = Eigen::Quaterniond(iso.rotation());


      poseToYaml(output, "imu_to_" + camera_name, I_r_IC, q_IC);

    }
  }
  output << YAML::EndMap;


  std::ofstream fout(file_out);
  fout << output.c_str();
  fout.close();
  return 0;
}
