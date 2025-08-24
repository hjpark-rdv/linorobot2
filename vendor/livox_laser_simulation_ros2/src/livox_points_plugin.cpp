#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/logging.hpp>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <chrono>

#include "ros2_livox/livox_points_plugin.h"
#include "ros2_livox/csv_reader.hpp"
#include "ros2_livox/livox_ode_multiray_shape.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

namespace gazebo
{
    GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

    LivoxPointsPlugin::LivoxPointsPlugin() {}
    LivoxPointsPlugin::~LivoxPointsPlugin() {}

    void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos)
    {
        avia_infos.reserve(datas.size());
        double deg_2_rad = M_PI / 180.0;
        for (auto &data : datas)
        {
            if (data.size() == 3)
            {
                avia_infos.emplace_back();
                avia_infos.back().time = data[0];
                avia_infos.back().azimuth = data[1] * deg_2_rad;
                avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2; // 표준 오른손 좌표계 각도로 변환
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("convertDataToRotateInfo"), "data size is not 3!");
            }
        }
    }

    void LivoxPointsPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr sdf)
    {
        node_ = gazebo_ros::Node::Get(sdf);
        
        std::vector<std::vector<double>> datas;
        std::string file_name = sdf->Get<std::string>("csv_file_name");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "load csv file name: %s", file_name.c_str());
        if (!CsvReader::ReadCsvFile(file_name, datas))
        {   
            RCLCPP_ERROR(rclcpp::get_logger("LivoxPointsPlugin"), "cannot get csv file! %s will return !", file_name.c_str());
            return;
        }

        sdfPtr = sdf;
        raySensor = _parent;

        auto curr_scan_topic = sdf->Get<std::string>("topic");
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "ros topic name: %s", curr_scan_topic.c_str());

        // ROS 2 Publisher 생성
        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(curr_scan_topic + "_PointCloud2", 10);
        custom_pub = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(curr_scan_topic, 10);

        // Livox 스캔 패턴 정보 로드
        aviaInfos.clear();
        convertDataToRotateInfo(datas, aviaInfos);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "scan info size: %ld", aviaInfos.size());
        maxPointSize = aviaInfos.size();

        RayPlugin::Load(_parent, sdfPtr);
        parentEntity = this->world->EntityByName(_parent->ParentName());

        auto physics = world->Physics();
        laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
        laserCollision->SetName("ray_sensor_collision");
        laserCollision->SetRelativePose(_parent->Pose());
        laserCollision->SetInitialRelativePose(_parent->Pose());

        rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
        laserCollision->SetShape(rayShape);

        samplesStep = sdfPtr->Get<int>("samples");
        downSample = sdfPtr->Get<int>("downsample");
        if (downSample < 1) downSample = 1;

        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "sample: %ld", samplesStep);
        RCLCPP_INFO(rclcpp::get_logger("LivoxPointsPlugin"), "downsample: %ld", downSample);

        auto rangeElem = sdfPtr->GetElement("ray")->GetElement("range");
        minDist = rangeElem->Get<double>("min");
        maxDist = rangeElem->Get<double>("max");
        
        // =========================================================================
        //  OPTIMIZATION 1: 모든 레이의 상대 방향을 미리 계산하여 저장
        // =========================================================================
        precomputed_rays_.reserve(maxPointSize);
        for (const auto& rotate_info : aviaInfos)
        {
            ignition::math::Quaterniond ray_quat;
            ray_quat.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            precomputed_rays_.push_back(ray_quat * ignition::math::Vector3d(1.0, 0.0, 0.0));
        }

        // AddRay를 통해 물리 엔진에 레이 객체들을 미리 생성
        int num_rays_to_add = samplesStep / downSample;
        rayShape->RayShapes().reserve(num_rays_to_add);
        for (int i = 0; i < num_rays_to_add; ++i)
        {
            rayShape->AddRay(ignition::math::Vector3d(0,0,0), ignition::math::Vector3d(0,0,0));
        }
    }

    void LivoxPointsPlugin::OnNewLaserScans()
    {
        if (!rayShape) return;

        // pair<ray_index, world_frame_ray_direction>
        std::vector<std::pair<int, ignition::math::Vector3d>> points_pair;
        InitializeRays(points_pair, rayShape);
        
        rayShape->Update(); // 물리 엔진에서 충돌 계산 수행

        // CustomMsg 메시지 준비
        livox_ros_driver2::msg::CustomMsg pp_livox;
        pp_livox.header.stamp = node_->get_clock()->now();
        pp_livox.header.frame_id = raySensor->Name();
        pp_livox.timebase = pp_livox.header.stamp.sec * 1000000000ull + pp_livox.header.stamp.nanosec;

        // PointCloud2 변환을 위한 중간 PointCloud 메시지 준비
        sensor_msgs::msg::PointCloud cloud;
        cloud.header = pp_livox.header;
        
        // =========================================================================
        //  OPTIMIZATION 2: 시간 계산을 위한 상수 미리 계산
        // =========================================================================
        double update_rate = raySensor->UpdateRate();
        double scan_duration_ns = (update_rate > 0) ? (1.0 / update_rate * 1e9) : 0.0;
        int total_points = points_pair.size();

        if (total_points == 0) return;

        for (size_t i = 0; i < total_points; ++i)
        {
            const auto& pair = points_pair[i];
            int ray_index = pair.first;
            
            double range = rayShape->GetRange(ray_index);
            double intensity = rayShape->GetRetro(ray_index);

            if (range >= RangeMax()) range = std::numeric_limits<double>::infinity();
            else if (range <= RangeMin()) range = -std::numeric_limits<double>::infinity();

            // 월드 좌표계 기준 레이 방향을 사용하여 센서 좌표계 기준 포인트 위치 계산
            const auto& world_ray_direction = pair.second;
            auto sensor_pose = raySensor->Pose();
            auto point_in_sensor_frame = sensor_pose.Rot().RotateVectorReverse(range * world_ray_direction);

            // CustomMsg 포인트 채우기
            livox_ros_driver2::msg::CustomPoint p;
            p.x = point_in_sensor_frame.X();
            p.y = point_in_sensor_frame.Y();
            p.z = point_in_sensor_frame.Z();
            p.reflectivity = intensity;

            // =========================================================================
            //  OPTIMIZATION 3: 수학적 계산으로 타임스탬프 오프셋 결정
            // =========================================================================
            p.offset_time = static_cast<uint32_t>((static_cast<double>(i) / total_points) * scan_duration_ns);
            pp_livox.points.push_back(p);

            // PointCloud 포인트 채우기
            // BUG FIX 4: 중복된 emplace_back 호출 수정
            geometry_msgs::msg::Point32 p_cloud;
            p_cloud.x = p.x;
            p_cloud.y = p.y;
            p_cloud.z = p.z;
            cloud.points.push_back(p_cloud);
        }

        // 메시지 발행
        pp_livox.point_num = pp_livox.points.size();
        custom_pub->publish(pp_livox);

        sensor_msgs::msg::PointCloud2 cloud2;
        sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2);
        cloud2.header = cloud.header; // 헤더 정보 다시 한번 확인
        cloud2_pub->publish(cloud2);
    }

    void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, ignition::math::Vector3d>> &points_pair,
                                           boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape)
    {
        auto &rays = ray_shape->RayShapes();
        auto offset = laserCollision->RelativePose(); // 센서의 현재 자세
        int64_t end_index = currStartIndex + samplesStep;
        long unsigned int ray_index = 0;
        
        points_pair.reserve(rays.size());

        for (int k = currStartIndex; k < end_index; k += downSample)
        {
            if (ray_index >= rays.size()) break;

            auto index = k % maxPointSize;
            
            // 미리 계산된 상대 방향 벡터에 현재 센서의 회전값을 적용하여 월드 좌표계 방향 계산
            auto world_ray_direction = offset.Rot() * precomputed_rays_[index];

            auto start_point = (minDist * world_ray_direction) + offset.Pos();
            auto end_point = (maxDist * world_ray_direction) + offset.Pos();
            
            rays[ray_index]->SetPoints(start_point, end_point);
            points_pair.emplace_back(ray_index, world_ray_direction);
            
            ray_index++;
        }
        currStartIndex = end_index;
    }

    void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan)
    {
        // This function seems unused for PointCloud generation, but kept for compatibility.
        // Store the latest laser scans into laserMsg
        msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
        scan->set_angle_min(AngleMin().Radian());
        scan->set_angle_max(AngleMax().Radian());
        scan->set_angle_step(AngleResolution());
        scan->set_count(RangeCount());
        scan->set_vertical_angle_min(VerticalAngleMin().Radian());
        scan->set_vertical_angle_max(VerticalAngleMax().Radian());
        scan->set_vertical_angle_step(VerticalAngleResolution());
        scan->set_vertical_count(VerticalRangeCount());
        scan->set_range_min(RangeMin());
        scan->set_range_max(RangeMax());
        scan->clear_ranges();
        scan->clear_intensities();
    }

    void LivoxPointsPlugin::SendRosTf(const ignition::math::Pose3d &pose, const std::string &father_frame, const std::string &child_frame)
    {
        // TF broadcasting logic if needed.
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMin() const { if (rayShape) return rayShape->MinAngle(); else return -1; }
    ignition::math::Angle LivoxPointsPlugin::AngleMax() const { if (rayShape) return ignition::math::Angle(rayShape->MaxAngle().Radian()); else return -1; }
    double LivoxPointsPlugin::RangeMin() const { if (rayShape) return rayShape->GetMinRange(); else return -1; }
    double LivoxPointsPlugin::RangeMax() const { if (rayShape) return rayShape->GetMaxRange(); else return -1; }
    double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }
    double LivoxPointsPlugin::RangeResolution() const { if (rayShape) return rayShape->GetResRange(); else return -1; }
    int LivoxPointsPlugin::RayCount() const { if (rayShape) return rayShape->GetSampleCount(); else return -1; }
    int LivoxPointsPlugin::RangeCount() const { if (rayShape) return rayShape->GetSampleCount() * rayShape->GetScanResolution(); else return -1; }
    int LivoxPointsPlugin::VerticalRayCount() const { if (rayShape) return rayShape->GetVerticalSampleCount(); else return -1; }
    int LivoxPointsPlugin::VerticalRangeCount() const { if (rayShape) return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution(); else return -1; }
    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const { if (rayShape) return ignition::math::Angle(rayShape->VerticalMinAngle().Radian()); else return -1; }
    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const { if (rayShape) return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian()); else return -1; }
    double LivoxPointsPlugin::VerticalAngleResolution() const { return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1); }

    // Deprecated getters
    double LivoxPointsPlugin::GetAngleResolution() const { return this->AngleResolution(); }
    double LivoxPointsPlugin::GetRangeMin() const { return this->RangeMin(); }
    double LivoxPointsPlugin::GetRangeMax() const { return this->RangeMax(); }
    double LivoxPointsPlugin::GetRangeResolution() const { return this->RangeResolution(); }
    int LivoxPointsPlugin::GetRayCount() const { return this->RayCount(); }
    int LivoxPointsPlugin::GetRangeCount() const { return this->RangeCount(); }
    int LivoxPointsPlugin::GetVerticalRayCount() const { return this->VerticalRayCount(); }
    int LivoxPointsPlugin::GetVerticalRangeCount() const { return this->VerticalRangeCount(); }
    double LivoxPointsPlugin::GetVerticalAngleResolution() const { return this->VerticalAngleResolution(); }

} // namespace gazebo