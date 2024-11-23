#include "MapConverter.hh" // MapConverter 클래스에 대한 헤더 파일 포함
#include "octomap/octomap_types.h" // Octomap 라이브러리의 데이터 타입 정의 포함
#include "octomap_msgs/msg/octomap.hpp" // Octomap 메시지 타입 포함
#include <cstddef> // 표준 라이브러리에서 크기 관련 타입 정의 포함
#include <cstdio> // 표준 입출력 함수 제공을 위한 헤더 파일 포함
#include <mapconversion_msgs/msg/height_map.hpp> // HeightMap 메시지 타입 포함
#include <mapconversion_msgs/msg/slope_map.hpp> // SlopeMap 메시지 타입 포함
#include <memory> // 스마트 포인터와 같은 메모리 관리 기능 제공
#include <nav_msgs/msg/occupancy_grid.hpp> // ROS의 Occupancy Grid 메시지 타입 포함
#include <nav_msgs/msg/odometry.hpp> // ROS의 Odometry 메시지 타입 포함
#include <octomap/AbstractOcTree.h> // Octomap의 추상적인 OcTree 클래스 정의 포함
#include <octomap/OcTree.h> // Octomap의 OcTree 구현 포함
#include <octomap/octomap.h> // Octomap 라이브러리의 주요 기능 포함
#include <octomap_msgs/conversions.h> // Octomap 메시지와 데이터 간의 변환 함수 포함
#include <octomap_msgs/msg/octomap.h> // Octomap 메시지 타입 포함 (중복 포함 주의)
#include <rclcpp/executors.hpp> // ROS 2 실행기 관련 클래스와 함수 포함
#include <rclcpp/logging.hpp> // ROS 2의 로깅 기능 포함
#include <rclcpp/rclcpp.hpp> // ROS 2의 주요 API 포함
#include <rclcpp/utilities.hpp> // ROS 2의 유틸리티 함수 제공
#include <vector> // 벡터 자료구조를 사용하기 위한 헤더 파일 포함

using namespace std; // std 네임스페이스의 모든 요소를 전역적으로 사용

class MapToMap : public rclcpp::Node { // MapToMap 클래스 정의, rclcpp::Node를 상속받음
private:
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subOctMap; // Octomap 메시지를 구독하는 서브스크립션

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubMapUGV, // UGV 용 OccupancyGrid 메시지 퍼블리셔
      pubMapUAV, // UAV 용 OccupancyGrid 메시지 퍼블리셔
      pubMapFloor, // 바닥 맵 OccupancyGrid 메시지 퍼블리셔
      pubMapCeiling, // 천장 맵 OccupancyGrid 메시지 퍼블리셔
      pubMapSlopeVis; // Slope 시각화를 위한 OccupancyGrid 메시지 퍼블리셔

  rclcpp::Publisher<mapconversion_msgs::msg::HeightMap>::SharedPtr pubHeightMap; // HeightMap 메시지 퍼블리셔
  rclcpp::Publisher<mapconversion_msgs::msg::SlopeMap>::SharedPtr pubMapSlope; // SlopeMap 메시지 퍼블리셔

  nav_msgs::msg::OccupancyGrid mapMsg; // OccupancyGrid 메시지 객체
  mapconversion_msgs::msg::HeightMap heightMsg; // HeightMap 메시지 객체
  mapconversion_msgs::msg::SlopeMap slopeMsg; // SlopeMap 메시지 객체

  MapConverter *MC; // MapConverter 객체 포인터
  octomap::OcTree *OcMap; // Octomap의 OcTree 객체 포인터

  double resolution; // 맵의 해상도
  double slopeMax; // 최대 경사값
  int slopeEstimationSize; // 경사 추정 크기
  double minimumZ; // 최소 Z 값
  int minimumOccupancy; // 최소 점유 값
  string mapFrame; // 맵 프레임 이름
  double mapZpos; // 맵 Z 위치

public:
  MapToMap() : Node("map_conversion") { // MapToMap 생성자 정의, 노드 이름은 "map_conversion"
    slopeMax = this->declare_parameter("max_slope_ugv", INFINITY); // UGV의 최대 경사값을 파라미터로 선언, 기본값은 무한대(INFINITY)
    slopeEstimationSize = this->declare_parameter("slope_estimation_size", 1); // 경사 추정 크기를 파라미터로 선언, 기본값은 1
    slopeEstimationSize = max(slopeEstimationSize, 1); // 경사 추정 크기가 1보다 작으면 1로 설정
    minimumZ = this->declare_parameter("minimum_z", 1.0); // 최소 Z 값을 파라미터로 선언, 기본값은 1.0
    minimumOccupancy = this->declare_parameter("minimum_occupancy", 10); // 최소 점유값을 파라미터로 선언, 기본값은 10
    mapFrame = this->declare_parameter("map_frame", string("map")); // 맵 프레임 이름을 파라미터로 선언, 기본값은 "map"
    mapZpos = this->declare_parameter("map_position_z", 0.0); // 맵 Z 위치를 파라미터로 선언, 기본값은 0.0

    subOctMap = this->create_subscription<octomap_msgs::msg::Octomap>( // Octomap 메시지를 구독할 서브스크립션 생성
        "/octomap", 1, // 구독 토픽 이름은 "/octomap", 큐 사이즈는 1
        std::bind(&MapToMap::mapCallback, this, std::placeholders::_1)); // mapCallback 함수와 연결

    pubMapUGV = // UGV 맵 OccupancyGrid 메시지를 퍼블리시할 퍼블리셔 생성
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapUGV", 5); // 퍼블리시할 토픽 이름은 "/mapUGV", 큐 사이즈는 5
    pubMapUAV = // UAV 맵 OccupancyGrid 메시지를 퍼블리시할 퍼블리셔 생성
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapUAV", 5); // 퍼블리시할 토픽 이름은 "/mapUAV", 큐 사이즈는 5
    pubMapFloor = this->create_publisher<nav_msgs::msg::OccupancyGrid>( // 바닥 맵 OccupancyGrid 메시지 퍼블리셔 생성
        "/visualization_floor_map", 5); // 퍼블리시할 토픽 이름은 "/visualization_floor_map", 큐 사이즈는 5
    pubMapCeiling = this->create_publisher<nav_msgs::msg::OccupancyGrid>( // 천장 맵 OccupancyGrid 메시지 퍼블리셔 생성
        "/visualization_ceiling_map", 5); // 퍼블리시할 토픽 이름은 "/visualization_ceiling_map", 큐 사이즈는 5
    pubHeightMap = this->create_publisher<mapconversion_msgs::msg::HeightMap>( // HeightMap 메시지를 퍼블리시할 퍼블리셔 생성
        "/heightMap", 5); // 퍼블리시할 토픽 이름은 "/heightMap", 큐 사이즈는 5
    pubMapSlopeVis = this->create_publisher<nav_msgs::msg::OccupancyGrid>( // Slope 시각화 맵 OccupancyGrid 메시지 퍼블리셔 생성
        "/visualization_slope_map", 5); // 퍼블리시할 토픽 이름은 "/visualization_slope_map", 큐 사이즈는 5
    pubMapSlope = this->create_publisher<mapconversion_msgs::msg::SlopeMap>( // SlopeMap 메시지를 퍼블리시할 퍼블리셔 생성
        "/slopeMap", 5); // 퍼블리시할 토픽 이름은 "/slopeMap", 큐 사이즈는 5

    OcMap = NULL; // OcTree 객체 포인터 초기화 (NULL로 설정)
    MC = NULL; // MapConverter 객체 포인터 초기화 (NULL로 설정)
  }

  ~MapToMap() {} // 소멸자 정의, 현재는 특별한 작업 없이 빈 상태

void mapCallback(octomap_msgs::msg::Octomap msg) { // Octomap 메시지를 처리하는 콜백 함수
    // Set resulution
    if (MC == NULL) { // MapConverter 객체가 초기화되지 않았다면
      resolution = msg.resolution; // 메시지에서 해상도 값을 가져옴
      MC = new MapConverter(resolution, slopeEstimationSize, minimumZ, minimumOccupancy); // MapConverter 객체 생성
    }
    // Convert ROS message to Octomap
    octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(msg); // ROS 메시지를 Octomap 데이터로 변환
    octomap::OcTree *newOcMap = dynamic_cast<octomap::OcTree *>(tree); // 변환된 데이터를 OcTree로 캐스팅
    if (!(newOcMap)) // 캐스팅 실패 시
      return; // 함수 종료

    double min_x, min_y, min_z; // 최소 좌표 변수 선언
    double max_x, max_y, max_z; // 최대 좌표 변수 선언

    newOcMap->getMetricMin(min_x, min_y, min_z); // 새 Octomap의 최소 좌표 가져오기
    newOcMap->getMetricMax(max_x, max_y, max_z); // 새 Octomap의 최대 좌표 가져오기
    vector<double> minMax(6); // 최소/최대 좌표를 저장할 벡터 생성
    if (OcMap == NULL) { // 기존 Octomap이 초기화되지 않았다면
      minMax[0] = min_x; // 최소 X 좌표 저장
      minMax[1] = max_x; // 최대 X 좌표 저장
      minMax[2] = min_y; // 최소 Y 좌표 저장
      minMax[3] = max_y; // 최대 Y 좌표 저장
      OcMap = newOcMap; // 새로운 Octomap을 기존 Octomap으로 설정
    } else { // 기존 Octomap이 존재한다면
      computeBoundingBox(minMax, newOcMap, OcMap); // 바운딩 박스 계산
      delete OcMap; // 기존 Octomap 삭제
      OcMap = newOcMap; // 새로운 Octomap으로 업데이트
    }
    minMax[4] = min_z; // 최소 Z 좌표 저장
    minMax[5] = max_z; // 최대 Z 좌표 저장

    update2Dmap(minMax); // 2D 맵 업데이트
    pub(); // 퍼블리시 호출
}

void computeBoundingBox(vector<double> &minMax, octomap::OcTree *tree1, octomap::OcTree *tree2) { 
    // 두 Octomap 트리 간 바운딩 박스를 계산하는 함수
    double min_x = INFINITY; // 최소 X 좌표 초기화
    double min_y = INFINITY; // 최소 Y 좌표 초기화
    double min_z = INFINITY; // 최소 Z 좌표 초기화
    double max_x = -INFINITY; // 최대 X 좌표 초기화
    double max_y = -INFINITY; // 최대 Y 좌표 초기화
    double max_z = -INFINITY; // 최대 Z 좌표 초기화

    // Iterate over all leaf nodes in tree1
    for (octomap::OcTree::leaf_iterator it = tree1->begin_leafs(), // tree1의 리프 노드 시작점
                                        end = tree1->end_leafs(); // tree1의 리프 노드 끝점
         it != end; ++it) { // 리프 노드를 순회
      octomap::OcTreeKey key = it.getKey(); // 현재 노드의 키 가져오기
      octomap::OcTreeNode *node2 = tree2->search(key); // tree2에서 동일한 키를 가진 노드 검색
      if (node2 != nullptr) { // tree2에서 해당 노드가 존재하면
        bool occupied1 = tree1->isNodeOccupied(*it); // tree1에서 현재 노드가 점유되었는지 확인
        bool occupied2 = tree2->isNodeOccupied(node2); // tree2에서 해당 노드가 점유되었는지 확인
        if (occupied1 != occupied2) { // 두 노드의 점유 상태가 다르다면
          double x = it.getX(); // 현재 노드의 X 좌표
          double y = it.getY(); // 현재 노드의 Y 좌표
          double z = it.getZ(); // 현재 노드의 Z 좌표
          updateBoundingBox(x, y, z, min_x, min_y, min_z, max_x, max_y, max_z); // 바운딩 박스 업데이트
        }
      } else { // tree2에서 해당 노드를 찾을 수 없다면
        double x = it.getX(); // 현재 노드의 X 좌표
        double y = it.getY(); // 현재 노드의 Y 좌표
        double z = it.getZ(); // 현재 노드의 Z 좌표
        updateBoundingBox(x, y, z, min_x, min_y, min_z, max_x, max_y, max_z); // 바운딩 박스 업데이트
      }
    }

    minMax[0] = min_x; // 계산된 최소 X 좌표 저장
    minMax[1] = max_x; // 계산된 최대 X 좌표 저장
    minMax[2] = min_y; // 계산된 최소 Y 좌표 저장
    minMax[3] = max_y; // 계산된 최대 Y 좌표 저장
}



  void updateBoundingBox(double x, double y, double z, double &min_x,
                         double &min_y, double &min_z, double &max_x,
                         double &max_y, double &max_z) {
    // 입력된 x 값이 현재 최소값보다 작으면 최소 x 값을 갱신
    if (x < min_x)
      min_x = x;
    // 입력된 y 값이 현재 최소값보다 작으면 최소 y 값을 갱신
    if (y < min_y)
      min_y = y;
    // 입력된 z 값이 현재 최소값보다 작으면 최소 z 값을 갱신
    if (z < min_z)
      min_z = z;
    // 입력된 x 값이 현재 최대값보다 크면 최대 x 값을 갱신
    if (x > max_x)
      max_x = x;
    // 입력된 y 값이 현재 최대값보다 크면 최대 y 값을 갱신
    if (y > max_y)
      max_y = y;
    // 입력된 z 값이 현재 최대값보다 크면 최대 z 값을 갱신
    if (z > max_z)
      max_z = z;
  }

  // 2D 맵을 축소된 AABB(Axis-Aligned Bounding Box)의 영역 내에서 갱신
  void update2Dmap(vector<double> minMax) {
    // minMax 벡터에 무한대 값이 있는 경우 함수를 종료
    for (double mM : minMax)
      if (isinf(mM))
        return;

    vector<voxel> voxelList; // 바운딩 박스 내의 점 데이터를 저장할 리스트
    // 최소 점과 최대 점을 octomap 포인트로 변환
    octomap::point3d minPoint(minMax[0], minMax[2], minMax[4]);
    octomap::point3d maxPoint(minMax[1], minMax[3], minMax[5]);
    // 바운딩 박스 안의 리프 노드들을 순회
    for (auto it = OcMap->begin_leafs_bbx(minPoint, maxPoint),
              it_end = OcMap->end_leafs_bbx();
         it != it_end; ++it) {
      voxel v; // 각 노드에 대한 정보를 담을 voxel 객체 생성
      v.position.x = it.getX(); // 노드의 x 좌표 설정
      v.position.y = it.getY(); // 노드의 y 좌표 설정
      v.position.z = it.getZ(); // 노드의 z 좌표 설정
      v.halfSize = it.getSize() / 2; // 노드의 절반 크기 설정
      v.occupied = OcMap->isNodeOccupied(*it); // 노드가 점유된 상태인지 확인
      voxelList.push_back(v); // voxel 리스트에 추가
    }
    // voxel 데이터를 기반으로 2D 맵을 업데이트
    MC->updateMap(voxelList, minMax);
  }
  void pub() {
    mapMsg.header.stamp = this->now();  // 현재 시간으로 맵 메시지의 헤더에 타임스탬프 설정
    mapMsg.header.frame_id = mapFrame;  // 맵 메시지의 프레임 ID를 mapFrame 변수로 설정
    heightMsg.header = mapMsg.header;  // 높이 메시지의 헤더를 맵 메시지의 헤더와 동일하게 설정
    mapMsg.info.width = MC->map.sizeX();  // 맵 메시지의 폭 정보를 MC 객체의 X 크기로 설정
    mapMsg.info.height = MC->map.sizeY();  // 맵 메시지의 높이 정보를 MC 객체의 Y 크기로 설정
    mapMsg.info.resolution = MC->map.getResulution();  // 맵 메시지의 해상도를 MC 객체의 해상도로 설정
    mapMsg.info.origin.position.x = MC->map.offsetX();  // 맵 메시지의 원점 X 위치를 MC 객체의 X 오프셋으로 설정
    mapMsg.info.origin.position.y = MC->map.offsetY();  // 맵 메시지의 원점 Y 위치를 MC 객체의 Y 오프셋으로 설정
    mapMsg.info.origin.position.z = mapZpos;  // 맵 메시지의 원점 Z 위치를 mapZpos 변수로 설정
    mapMsg.info.origin.orientation.x = 0;  // 맵 메시지 원점의 쿼터니언 X 방향을 0으로 설정
    mapMsg.info.origin.orientation.y = 0;  // 맵 메시지 원점의 쿼터니언 Y 방향을 0으로 설정
    mapMsg.info.origin.orientation.z = 0;  // 맵 메시지 원점의 쿼터니언 Z 방향을 0으로 설정
    mapMsg.info.origin.orientation.w = 1;  // 맵 메시지 원점의 쿼터니언 W 방향을 1로 설정 (회전 없음)
    heightMsg.info = mapMsg.info;  // 높이 메시지의 정보를 맵 메시지의 정보와 동일하게 설정
    slopeMsg.info = mapMsg.info;  // 경사 메시지의 정보를 맵 메시지의 정보와 동일하게 설정
    mapMsg.data.resize(mapMsg.info.width * mapMsg.info.height);  // 맵 메시지의 데이터 크기를 폭과 높이의 곱으로 설정
    heightMsg.top.resize(mapMsg.info.width * mapMsg.info.height);  // 높이 메시지의 상단 데이터 크기를 폭과 높이의 곱으로 설정
    heightMsg.bottom.resize(mapMsg.info.width * mapMsg.info.height);  // 높이 메시지의 하단 데이터 크기를 폭과 높이의 곱으로 설정
    slopeMsg.slope.resize(mapMsg.info.width * mapMsg.info.height);  // 경사 메시지의 데이터 크기를 폭과 높이의 곱으로 설정

    // UGV(무인 지상 차량)를 위한 맵 퍼블리싱
    if (pubMapUGV->get_subscription_count() != 0) {  // UGV 맵 퍼블리셔에 구독자가 있는지 확인
      if (isinf(slopeMax)) {  // slopeMax 값이 무한대인지 확인
        RCLCPP_WARN(
            this->get_logger(),
            "The max slope UGV is not set; obstacles will not be included in "
            "the UGV map. \nAdd \'_max_slope_ugv:=x\' to rosrun command, or "
            "\'<param name=\"max_slope_ugv\" value=\"x\"/>\' in launch file.");  // slopeMax가 설정되지 않았음을 경고
      }

      mapMsg.header.stamp = this->now();  // 맵 메시지 헤더의 타임스탬프를 현재 시간으로 갱신
      for (int y = 0; y < MC->map.sizeY(); y++) {  // Y 좌표에 대해 루프 실행
        for (int x = 0; x < MC->map.sizeX(); x++) {  // X 좌표에 대해 루프 실행
          int index = x + y * MC->map.sizeX();  // 맵 데이터를 1D 배열로 변환하기 위한 인덱스 계산
          mapMsg.data[index] = MC->map.get(x, y, slopeMax);  // 해당 위치의 맵 데이터를 slopeMax 기준으로 가져옴
        }
      }
      pubMapUGV->publish(mapMsg);  // UGV 맵 메시지를 퍼블리시
    }

    // UAV(무인 항공기)를 위한 맵 퍼블리싱
    if (pubMapUAV->get_subscription_count() != 0) {  // UAV 맵 퍼블리셔에 구독자가 있는지 확인
      for (int y = 0; y < MC->map.sizeY(); y++) {  // Y 좌표를 루프
        for (int x = 0; x < MC->map.sizeX(); x++) {  // X 좌표를 루프
          int index = x + y * MC->map.sizeX();  // 1D 인덱스 계산
          mapMsg.data[index] = MC->map.get(x, y);  // 맵 데이터 가져오기
        }
      }
      pubMapUAV->publish(mapMsg);  // UAV 맵 메시지를 퍼블리시
    }

    // 바닥 높이 맵을 RViz 시각화로 퍼블리싱
    if (pubMapFloor->get_subscription_count() != 0) {  // 바닥 맵 퍼블리셔에 구독자가 있는지 확인
      if (MC->map.sizeY() == 0 || MC->map.sizeX() == 0)  // 맵의 크기가 0인지 확인
        return;
      double vMax = NAN, vMin = NAN;  // 정규화를 위한 최대값과 최소값 초기화
      // 최대값과 최소값 계산
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          double v = MC->map.getHeight(x, y);  // 맵의 높이값 가져오기
          if (isnan(v))  // 값이 유효하지 않으면 건너뜀
            continue;
          if (isnan(vMax))  // 최대값 초기 설정
            vMax = v;
          else
            vMax = max(vMax, v);  // 최대값 갱신
          if (isnan(vMin))  // 최소값 초기 설정
            vMin = v;
          else
            vMin = min(vMin, v);  // 최소값 갱신
        }
      }

      for (int y = 0; y < MC->map.sizeY(); y++) {  // Y 좌표를 루프
        for (int x = 0; x < MC->map.sizeX(); x++) {  // X 좌표를 루프
          int index = x + y * MC->map.sizeX();  // 1D 인덱스 계산
          double value = MC->map.getHeight(x, y);  // 맵의 높이값 가져오기
          if (!isnan(value)) {  // 값이 유효하다면 정규화 수행
            value = (value - vMin) / (vMax - vMin) * 199;  // 0~199 범위로 정규화
            if (value > 99)  // 색상 범위를 조정
              value -= 199;
          }
          mapMsg.data[index] = value;  // 정규화된 값을 맵 데이터에 저장
        }
      }

      pubMapFloor->publish(mapMsg);  // 바닥 맵 메시지를 퍼블리시
    }

    // 천장 높이 맵을 RViz 시각화로 퍼블리싱
    if (pubMapCeiling->get_subscription_count() != 0) {  // 천장 맵 퍼블리셔에 구독자가 있는지 확인
      if (MC->map.sizeY() == 0 || MC->map.sizeX() == 0)  // 맵의 크기가 0인지 확인
        return;
      double vMax = NAN, vMin = NAN;  // 정규화를 위한 최대값과 최소값 초기화
      // 최대값과 최소값 계산
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          double v = MC->map.getHeightTop(x, y);  // 천장 높이값 가져오기
          if (isnan(v))  // 값이 유효하지 않으면 건너뜀
            continue;
          if (isnan(vMax))  // 최대값 초기 설정
            vMax = v;
          else
            vMax = max(vMax, v);  // 최대값 갱신
          if (isnan(vMin))  // 최소값 초기 설정
            vMin = v;
          else
            vMin = min(vMin, v);  // 최소값 갱신
        }
      }

      for (int y = 0; y < MC->map.sizeY(); y++) {  // Y 좌표를 루프
        for (int x = 0; x < MC->map.sizeX(); x++) {  // X 좌표를 루프
          int index = x + y * MC->map.sizeX();  // 1D 인덱스 계산
          double value = MC->map.getHeightTop(x, y);  // 천장 높이값 가져오기
          if (!isnan(value)) {  // 값이 유효하다면 정규화 수행
            value = (value - vMin) / (vMax - vMin) * 199;  // 0~199 범위로 정규화
            if (value > 99)  // 색상 범위를 조정
              value -= 199;
          }
          mapMsg.data[index] = value;  // 정규화된 값을 맵 데이터에 저장
        }
      }

      pubMapCeiling->publish(mapMsg);  // 천장 맵 메시지를 퍼블리시
    }

    // 높이 맵을 퍼블리싱
    if (pubHeightMap->get_subscription_count() != 0) {  // 높이 맵 퍼블리셔에 구독자가 있는지 확인
      for (int y = 0; y < MC->map.sizeY(); y++) {  // Y 좌표를 루프
        for (int x = 0; x < MC->map.sizeX(); x++) {  // X 좌표를 루프
          int index = x + y * MC->map.sizeX();  // 1D 인덱스 계산

          heightMsg.bottom[index] = MC->map.getHeight(x, y);  // 바닥 높이 설정
          heightMsg.top[index] = MC->map.getHeightTop(x, y);  // 천장 높이 설정
        }
      }
      pubHeightMap->publish(heightMsg);  // 높이 맵 메시지를 퍼블리시
    }

    // 경사 맵을 RViz 시각화로 퍼블리싱
    if (pubMapSlopeVis->get_subscription_count() != 0) {  // 경사 맵 퍼블리셔에 구독자가 있는지 확인
      double vMax = slopeMax * 2, vMin = 0;  // 경사값 범위 설정

      for (int y = 0; y < MC->map.sizeY(); y++) {  // Y 좌표를 루프
        for (int x = 0; x < MC->map.sizeX(); x++) {  // X 좌표를 루프
          int index = x + y * MC->map.sizeX();  // 1D 인덱스 계산
          double value = min(MC->map.getSlope(x, y), slopeMax * 2);  // 경사값 가져오기
          if (!isnan(value)) {  // 값이 유효하다면 정규화 수행
            value = (value - vMin) / (vMax - vMin) * 198 + 1;  // 1~199 범위로 정규화
            if (value > 99)  // 색상 범위를 조정
              value -= 199;
          }
          mapMsg.data[index] = value;  // 정규화된 값을 맵 데이터에 저장
        }
      }
      pubMapSlopeVis->publish(mapMsg);  // 경사 맵 메시지를 퍼블리시
    }

    // 경사 맵을 퍼블리싱
    if (pubMapSlope->get_subscription_count() != 0) {  // 경사 맵 퍼블리셔에 구독자가 있는지 확인
      for (int y = 0; y < MC->map.sizeY(); y++) {  // Y 좌표를 루프
        for (int x = 0; x < MC->map.sizeX(); x++) {  // X 좌표를 루프
          int index = x + y * MC->map.sizeX();  // 1D 인덱스 계산

          slopeMsg.slope[index] = MC->map.getSlope(x, y);  // 경사값 설정
        }
      }
      pubMapSlope->publish(slopeMsg);  // 경사 맵 메시지를 퍼블리시
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);  // ROS2 초기화

  auto Node = make_shared<MapToMap>();  // MapToMap 노드 생성
  rclcpp::spin(Node);  // 노드 실행
  rclcpp::shutdown();  // ROS2 종료

  return 0;  // 프로그램 종료
}

  


