#include <cstdio>
#include <mapconversion/HeightMap.h>
#include <mapconversion/SlopeMap.h>
#include <nav_msgs/Odometry.h>
#include <ufo/map/occupancy_map.h>
#include <ufomap_msgs/UFOMapStamped.h>
// UFO와 ROS 간의 변환을 위해 필요한 헤더 파일들
#include <ufomap_msgs/conversions.h>
#include <ufomap_ros/conversions.h>

#include "MapConverter.hh"
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

class MapToMap {
private:
  // ROS 노드 핸들
  ros::NodeHandle nh; // ROS의 노드 핸들 객체 생성

  // 구독자들
  ros::Subscriber subUfoMap; // UFO 맵 구독자
  ros::Subscriber subOdom;   // 오도메트리(위치 추정) 구독자

  // 퍼블리셔들
  ros::Publisher pubMapUGV, pubMapUAV, pubMapFloor, pubMapCeiling, pubHeightMap,
      pubMapSlopeVis, pubMapSlope; // 다양한 맵들을 퍼블리시할 퍼블리셔들

  // 메시지 타입들
  nav_msgs::OccupancyGrid mapMsg; // ROS OccupancyGrid 메시지 객체
  mapconversion::HeightMap heightMsg; // 높이 맵 메시지 객체
  mapconversion::SlopeMap slopeMsg; // 경사 맵 메시지 객체

  MapConverter *MC; // MapConverter 객체 포인터 (맵 변환을 담당)
  ufo::map::OccupancyMap *ufoMap; // UFO 맵 객체 포인터

  // 맵 관련 설정 값들
  double resolution; // 맵의 해상도
  double slopeMax; // 최대 경사값
  int slopeEstimationSize; // 경사 추정 크기
  double minimumZ; // 최소 높이 값
  int minimumOccupancy; // 최소 점유 비율
  string mapFrame; // 맵의 프레임 ID
  double mapZpos; // 맵의 Z 위치

public:
  MapToMap() {
    ros::NodeHandle nh_priv("~"); // private ROS 노드 핸들 생성
    slopeMax = nh_priv.param("max_slope_ugv", INFINITY); // 최대 경사값 초기화
    slopeEstimationSize = nh_priv.param("slope_estimation_size", 1); // 경사 추정 크기 초기화
    slopeEstimationSize = max(slopeEstimationSize, 1); // 경사 추정 크기 최소값을 1로 설정
    minimumZ = nh_priv.param("minimum_z", 1.0); // 최소 높이값 설정
    minimumOccupancy = nh_priv.param("minimum_occupancy", 10); // 최소 점유 비율 설정
    mapFrame = nh_priv.param("map_frame", string("map")); // 맵 프레임 이름 설정
    mapZpos = nh_priv.param("map_position_z", 0.0); // 맵의 Z 위치 설정

    subUfoMap = nh.subscribe("/ufomap", 1, &MapToMap::mapCallback, this); // UFO 맵을 구독하는 구독자 생성

    // 퍼블리셔들 설정
    pubMapUGV = nh.advertise<nav_msgs::OccupancyGrid>("/mapUGV", 5); // UGV 맵 퍼블리셔
    pubMapUAV = nh.advertise<nav_msgs::OccupancyGrid>("/mapUAV", 5); // UAV 맵 퍼블리셔
    pubMapFloor = nh.advertise<nav_msgs::OccupancyGrid>("/visualization_floor_map", 5); // 바닥 맵 시각화 퍼블리셔
    pubMapCeiling = nh.advertise<nav_msgs::OccupancyGrid>("/visualization_ceiling_map", 5); // 천장 맵 시각화 퍼블리셔
    pubHeightMap = nh.advertise<mapconversion::HeightMap>("/heightMap", 5); // 높이 맵 퍼블리셔
    pubMapSlopeVis = nh.advertise<nav_msgs::OccupancyGrid>("/visualization_slope_map", 5); // 경사 맵 시각화 퍼블리셔
    pubMapSlope = nh.advertise<mapconversion::SlopeMap>("/slopeMap", 5); // 경사 맵 퍼블리셔
    ufoMap = NULL; // UFO 맵 객체 초기화
  }

  ~MapToMap() {} // 소멸자 (현재는 아무 작업도 하지 않음)

  void mapCallback(ufomap_msgs::UFOMapStamped::ConstPtr const &msg) {
    // UFO 맵의 타입이 지원되는지 확인
    if (msg->map.info.id != "occupancy_map") { // 만약 "occupancy_map"이 아니라면
      ROS_FATAL("This ROS node does not support colored UFOMaps. Please turn "
                "off the color in UFOMaps settings."); // 경고 메시지 출력
      ros::shutdown(); // ROS 종료
      return;
    }

    // 맵 해상도 설정
    if (ufoMap == NULL) {
      resolution = msg->map.info.resolution; // 해상도 설정
      MC = new MapConverter(resolution, slopeEstimationSize, minimumZ,
                            minimumOccupancy); // MapConverter 객체 생성
      ufoMap = new ufo::map::OccupancyMap(resolution); // UFO 맵 객체 생성
    }

    // ROS 메시지를 UFO 맵으로 변환
    ufomap_msgs::msgToUfo(msg->map, *ufoMap);

    if (msg->map.info.bounding_volume.aabbs.size() <= 0) // 경계 박스가 없으면 리턴
      return;

    // 새로운 AABB(축에 맞춘 경계 박스) 생성
    ufo::geometry::AABB newAABB;
    newAABB = ufomap_msgs::msgToUfo(msg->map.info.bounding_volume.aabbs[0]);

    // 전체 맵의 높이 계산
    ufo::math::Vector3 max_point(newAABB.getMax()); // 최대 점 계산
    ufo::math::Vector3 min_point(newAABB.getMin()); // 최소 점 계산
    min_point.z() = (ufoMap->getKnownBBX().getMin().z()); // Z 최소값 설정
    max_point.z() = (ufoMap->getKnownBBX().getMax().z()); // Z 최대값 설정

    // X, Y 크기는 새로운 맵에서 가져오고, Z는 전체 맵에서 가져와 AABB 생성
    ufo::geometry::AABB aabb(min_point, max_point);

    update2Dmap(aabb); // 2D 맵 업데이트
    pub(); // 퍼블리시 함수 호출
  }

  // AABB 영역 내의 2D 맵을 업데이트하는 함수
  void update2Dmap(ufo::geometry::AABB aabb) {
    ufo::math::Vector3 max_point(aabb.getMax()); // AABB의 최대 점을 가져옴
    ufo::math::Vector3 min_point(aabb.getMin()); // AABB의 최소 점을 가져옴
    vector<double> minMax(6); // 최소/최대 값을 저장할 벡터
    minMax[0] = min_point.x(); // 최소 X 값
    minMax[1] = max_point.x(); // 최대 X 값
    minMax[2] = min_point.y(); // 최소 Y 값
    minMax[3] = max_point.y(); // 최대 Y 값
    minMax[4] = min_point.z(); // 최소 Z 값
    minMax[5] = max_point.z(); // 최대 Z 값
    vector<voxel> voxelList; // 저장할 voxel 리스트

    // AABB 영역 내에서 모든 자유 공간 및 점유된 voxel을 가져오기
    for (auto it = ufoMap->beginLeaves(aabb, true, true, false, false, 0),
              it_end = ufoMap->endLeaves(); // AABB 내의 리프 노드를 순회
         it != it_end; ++it) {
      voxel v;
      v.position.x = it.getX(); // voxel의 X 좌표
      v.position.y = it.getY(); // voxel의 Y 좌표
      v.position.z = it.getZ(); // voxel의 Z 좌표
      v.halfSize = it.getHalfSize(); // voxel의 반지름 크기
      v.occupied = it.isOccupied(); // voxel이 점유되었는지 여부
      voxelList.push_back(v); // voxel 리스트에 추가
    }
    MC->updateMap(voxelList, minMax); // MapConverter를 사용해 맵 업데이트
  }


  void pub() {
    mapMsg.header.stamp = ros::Time::now(); // 맵 메시지의 헤더 시간 갱신
    mapMsg.header.frame_id = mapFrame; // 맵 메시지의 프레임 ID 설정
    heightMsg.header = mapMsg.header; // 높이 맵 메시지의 헤더를 맵 메시지의 헤더와 동일하게 설정
    mapMsg.info.width = MC->map.sizeX(); // 맵의 너비 설정
    mapMsg.info.height = MC->map.sizeY(); // 맵의 높이 설정
    mapMsg.info.resolution = MC->map.getResulution(); // 맵의 해상도 설정
    mapMsg.info.origin.position.x = MC->map.offsetX(); // 맵 원점의 X 좌표 설정
    mapMsg.info.origin.position.y = MC->map.offsetY(); // 맵 원점의 Y 좌표 설정
    mapMsg.info.origin.position.z = mapZpos; // 맵 원점의 Z 좌표 설정
    mapMsg.info.origin.orientation.x = 0; // 맵 원점의 방향 X 설정
    mapMsg.info.origin.orientation.y = 0; // 맵 원점의 방향 Y 설정
    mapMsg.info.origin.orientation.z = 0; // 맵 원점의 방향 Z 설정
    mapMsg.info.origin.orientation.w = 1; // 맵 원점의 방향 W 설정 (단위 quaternion)
    heightMsg.info = mapMsg.info; // 높이 맵 정보 설정
    slopeMsg.info = mapMsg.info; // 경사 맵 정보 설정
    mapMsg.data.resize(mapMsg.info.width * mapMsg.info.height); // 맵 데이터 배열 크기 설정
    heightMsg.top.resize(mapMsg.info.width * mapMsg.info.height); // 높이 맵의 상단 배열 크기 설정
    heightMsg.bottom.resize(mapMsg.info.width * mapMsg.info.height); // 높이 맵의 하단 배열 크기 설정
    slopeMsg.slope.resize(mapMsg.info.width * mapMsg.info.height); // 경사 맵 데이터 배열 크기 설정

    // UGV 맵 퍼블리시
    if (pubMapUGV.getNumSubscribers() != 0) { // UGV 맵 퍼블리셔에 구독자가 있는 경우
      if (isinf(slopeMax)) { // 최대 경사값이 설정되지 않았으면 경고 메시지 출력
        ROS_WARN(
            "The max slope UGV is not set; obstacles will not be included in "
            "the UGV map. \nAdd \'_max_slope_ugv:=x\' to rosrun command, or "
            "\'<param name=\"max_slope_ugv\" value=\"x\"/>\' in launch file.");
      }

      mapMsg.header.stamp = ros::Time::now(); // 맵 메시지의 헤더 시간 갱신
      for (int y = 0; y < MC->map.sizeY(); y++) { // Y 좌표를 순회
        for (int x = 0; x < MC->map.sizeX(); x++) { // X 좌표를 순회
          int index = x + y * MC->map.sizeX(); // 1차원 배열 인덱스 계산

          mapMsg.data[index] = MC->map.get(x, y, slopeMax); // UGV 맵 데이터를 계산하여 배열에 설정
        }
      }
      pubMapUGV.publish(mapMsg); // UGV 맵 퍼블리시
    }
    // UAV 맵 퍼블리시
    if (pubMapUAV.getNumSubscribers() != 0) { // UAV 맵 퍼블리셔에 구독자가 있을 경우
      for (int y = 0; y < MC->map.sizeY(); y++) { // Y 좌표를 순회
        for (int x = 0; x < MC->map.sizeX(); x++) { // X 좌표를 순회
          int index = x + y * MC->map.sizeX(); // 1차원 배열 인덱스 계산

          mapMsg.data[index] = MC->map.get(x, y); // UAV 맵 데이터 설정
        }
      }
      pubMapUAV.publish(mapMsg); // UAV 맵 퍼블리시
    }

    // RViz에서 바닥 높이 맵 시각화 퍼블리시
    if (pubMapFloor.getNumSubscribers() != 0) { // 바닥 맵 퍼블리셔에 구독자가 있을 경우
      if (MC->map.sizeY() == 0 || MC->map.sizeX() == 0) // 맵 크기가 0이면 리턴
        return;
      double vMax = NAN, vMin = NAN; // 최대값 및 최소값 초기화
      // 정규화를 위한 최대값 및 최소값 찾기
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          double v = MC->map.getHeight(x, y); // 바닥 높이 값 가져오기
          if (isnan(v)) // 유효하지 않은 값은 건너뜀
            continue;
          if (isnan(vMax)) // 최대값 초기화
            vMax = v;
          else
            vMax = max(vMax, v); // 최대값 갱신
          if (isnan(vMin)) // 최소값 초기화
            vMin = v;
          else
            vMin = min(vMin, v); // 최소값 갱신
        }
      }

      // RViz 시각화를 위한 바닥 높이 맵 데이터 설정
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX(); // 1차원 배열 인덱스 계산
          double value = MC->map.getHeight(x, y); // 바닥 높이 값 가져오기
          if (!isnan(value)) { // 값이 유효하면
            value = (value - vMin) / (vMax - vMin) * 199; // 정규화
            if (value > 99)
              value -= 199; // RViz에서 색상 범위 설정
          }
          mapMsg.data[index] = value; // 바닥 맵 데이터 설정
        }
      }

      pubMapFloor.publish(mapMsg); // 바닥 맵 퍼블리시
    }

    // RViz에서 천장 높이 맵 시각화 퍼블리시
    if (pubMapCeiling.getNumSubscribers() != 0) { // 천장 맵 퍼블리셔에 구독자가 있을 경우
      if (MC->map.sizeY() == 0 || MC->map.sizeX() == 0) // 맵 크기가 0이면 리턴
        return;
      double vMax = NAN, vMin = NAN; // 최대값 및 최소값 초기화
      // 정규화를 위한 최대값 및 최소값 찾기
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          double v = MC->map.getHeightTop(x, y); // 천장 높이 값 가져오기
          if (isnan(v)) // 유효하지 않은 값은 건너뜀
            continue;
          if (isnan(vMax)) // 최대값 초기화
            vMax = v;
          else
            vMax = max(vMax, v); // 최대값 갱신
          if (isnan(vMin)) // 최소값 초기화
            vMin = v;
          else
            vMin = min(vMin, v); // 최소값 갱신
        }
      }

      // RViz 시각화를 위한 천장 높이 맵 데이터 설정
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX(); // 1차원 배열 인덱스 계산
          double value = MC->map.getHeightTop(x, y); // 천장 높이 값 가져오기
          if (!isnan(value)) { // 값이 유효하면
            value = (value - vMin) / (vMax - vMin) * 199; // 정규화
            if (value > 99)
              value -= 199; // RViz에서 색상 범위 설정
          }
          mapMsg.data[index] = value; // 천장 맵 데이터 설정
        }
      }

      pubMapCeiling.publish(mapMsg); // 천장 맵 퍼블리시
    }
    // 높이 맵 퍼블리시
    if (pubHeightMap.getNumSubscribers() != 0) { // 구독자가 있을 경우
      for (int y = 0; y < MC->map.sizeY(); y++) { // Y 좌표 순회
        for (int x = 0; x < MC->map.sizeX(); x++) { // X 좌표 순회
          int index = x + y * MC->map.sizeX(); // 1차원 배열 인덱스 계산

          heightMsg.bottom[index] = MC->map.getHeight(x, y); // 하단 높이 값 설정
          heightMsg.top[index] = MC->map.getHeightTop(x, y); // 상단 높이 값 설정
        }
      }
      pubHeightMap.publish(heightMsg); // 높이 맵 퍼블리시
    }

    // RViz에서 경사 맵 시각화 퍼블리시
    if (pubMapSlopeVis.getNumSubscribers() != 0) { // 구독자가 있을 경우
      double vMax = slopeMax * 2, vMin = 0; // 최대값과 최소값 초기화

      // 경사 맵 계산
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX(); // 1차원 배열 인덱스 계산
          double value = min(MC->map.getSlope(x, y), slopeMax * 2); // 경사 값 계산
          if (!isnan(value)) { // 유효한 값일 경우
            value = (value - vMin) / (vMax - vMin) * 198 + 1; // 정규화
            if (value > 99) // RViz에서 색상 범위 맞추기
              value -= 199; 
          }
          mapMsg.data[index] = value; // 경사 맵 데이터 설정
        }
      }
      pubMapSlopeVis.publish(mapMsg); // 경사 맵 시각화 퍼블리시
    }

    // 경사 맵 퍼블리시
    if (pubMapSlope.getNumSubscribers() != 0) { // 구독자가 있을 경우
      for (int y = 0; y < MC->map.sizeY(); y++) { // Y 좌표 순회
        for (int x = 0; x < MC->map.sizeX(); x++) { // X 좌표 순회
          int index = x + y * MC->map.sizeX(); // 1차원 배열 인덱스 계산

          slopeMsg.slope[index] = MC->map.getSlope(x, y); // 경사 값 계산하여 설정
        }
      }
      pubMapSlope.publish(slopeMsg); // 경사 맵 퍼블리시
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_saver"); // ROS 노드 초기화

  MapToMap mtm; // MapToMap 객체 생성

  ros::spin(); // ROS 이벤트 루프 실행

  return 0; // 종료
}




