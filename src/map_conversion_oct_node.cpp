#include "MapConverter.hh"  // 맵 변환 관련 헤더 파일
#include "octomap/octomap_types.h"  // OctoMap 관련 데이터 타입
#include "ros/console.h"  // ROS 콘솔 출력
#include <cstddef>  // 크기 관련 헤더
#include <cstdio>  // C 표준 입출력
#include <mapconversion/HeightMap.h>  // HeightMap 메시지
#include <mapconversion/SlopeMap.h>  // SlopeMap 메시지
#include <nav_msgs/OccupancyGrid.h>  // OccupancyGrid 메시지
#include <nav_msgs/Odometry.h>  // Odometry 메시지
#include <octomap/OcTree.h>  // OctoMap 트리 구조
#include <octomap/octomap.h>  // OctoMap 라이브러리
#include <octomap_msgs/Octomap.h>  // OctoMap 메시지
#include <octomap_msgs/conversions.h>  // OctoMap 메시지 변환 함수
#include <vector>  // STL 벡터

using namespace std;  // 표준 네임스페이스 사용

class MapToMap {
private:
  // ROS NodeHandle
  ros::NodeHandle nh;

  // ROS 구독자
  ros::Subscriber subOctMap;  // OctoMap 데이터 구독
  ros::Subscriber subOdom;  // Odometry 데이터 구독

  // ROS 퍼블리셔
  ros::Publisher pubMapUGV;  // UGV 맵 퍼블리셔
  ros::Publisher pubMapUAV;  // UAV 맵 퍼블리셔
  ros::Publisher pubMapFloor;  // 바닥 맵 퍼블리셔
  ros::Publisher pubMapCeiling;  // 천장 맵 퍼블리셔
  ros::Publisher pubHeightMap;  // 높이 맵 퍼블리셔
  ros::Publisher pubMapSlopeVis;  // 경사 맵 시각화 퍼블리셔
  ros::Publisher pubMapSlope;  // 경사 맵 퍼블리셔

  // 메시지 객체
  nav_msgs::OccupancyGrid mapMsg;  // 점유 그리드 메시지
  mapconversion::HeightMap heightMsg;  // 높이 맵 메시지
  mapconversion::SlopeMap slopeMsg;  // 경사 맵 메시지

  MapConverter *MC;  // MapConverter 클래스의 포인터
  octomap::OcTree *OcMap;  // OctoMap 트리 객체의 포인터

  // 파라미터
  double resolution;  // 맵 해상도
  double slopeMax;  // 최대 경사 값
  int slopeEstimationSize;  // 경사 추정 크기
  double minimumZ;  // 최소 Z 값
  int minimumOccupancy;  // 최소 점유 수준
  string mapFrame;  // 맵의 프레임 ID
  double mapZpos;  // 맵 Z 위치


public:
  // 생성자
  MapToMap() {
    ros::NodeHandle nh_priv("~");  // 프라이빗 NodeHandle 생성

    // 매개변수 초기화
    slopeMax = nh_priv.param("max_slope_ugv", INFINITY);  // 최대 경사값 설정
    slopeEstimationSize = nh_priv.param("slope_estimation_size", 1);  // 경사 추정 크기 설정
    slopeEstimationSize = max(slopeEstimationSize, 1);  // 최소값 보장
    minimumZ = nh_priv.param("minimum_z", 1.0);  // 최소 Z 값 설정
    minimumOccupancy = nh_priv.param("minimum_occupancy", 10);  // 최소 점유 수준 설정
    mapFrame = nh_priv.param("map_frame", string("map"));  // 맵 프레임 ID 설정
    mapZpos = nh_priv.param("map_position_z", 0.0);  // 맵 Z 위치 설정

    // OctoMap 데이터를 구독
    subOctMap = nh.subscribe("octomap", 1, &MapToMap::mapCallback, this);

    // 퍼블리셔 초기화
    pubMapUGV = nh.advertise<nav_msgs::OccupancyGrid>("/mapUGV", 5);  // UGV 맵 퍼블리셔
    pubMapUAV = nh.advertise<nav_msgs::OccupancyGrid>("/mapUAV", 5);  // UAV 맵 퍼블리셔
    pubMapFloor =
        nh.advertise<nav_msgs::OccupancyGrid>("/visualization_floor_map", 5);  // 바닥 맵 퍼블리셔
    pubMapCeiling =
        nh.advertise<nav_msgs::OccupancyGrid>("/visualization_ceiling_map", 5);  // 천장 맵 퍼블리셔
    pubHeightMap = nh.advertise<mapconversion::HeightMap>("/heightMap", 5);  // 높이 맵 퍼블리셔
    pubMapSlopeVis =
        nh.advertise<nav_msgs::OccupancyGrid>("/visualization_slope_map", 5);  // 경사 맵 시각화 퍼블리셔
    pubMapSlope = nh.advertise<mapconversion::SlopeMap>("/slopeMap", 5);  // 경사 맵 퍼블리셔

    // OctoMap 및 MapConverter 초기화
    OcMap = NULL;  // OctoMap 객체 초기화
    MC = NULL;  // MapConverter 객체 초기화
  }

  // 소멸자
  ~MapToMap() {}


  // OctoMap 메시지 콜백 함수
  void mapCallback(const octomap_msgs::Octomap::ConstPtr &msg) {
    // 해상도 설정
    if (MC == NULL) {  // MapConverter가 아직 생성되지 않은 경우
      resolution = msg->resolution;  // 메시지에서 해상도 추출
      MC = new MapConverter(resolution, slopeEstimationSize, minimumZ,
                            minimumOccupancy);  // MapConverter 생성
    }

    // ROS 메시지를 OctoMap으로 변환
    octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);  // 메시지를 AbstractOcTree로 변환
    octomap::OcTree *newOcMap = dynamic_cast<octomap::OcTree *>(tree);  // OcTree로 캐스팅
    if (!(newOcMap))  // 변환 실패 시 종료
      return;

    // OctoMap의 최소 및 최대 좌표 가져오기
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    newOcMap->getMetricMin(min_x, min_y, min_z);  // 최소 좌표
    newOcMap->getMetricMax(max_x, max_y, max_z);  // 최대 좌표

    vector<double> minMax(6);  // 최소 및 최대값 저장 벡터
    if (OcMap == NULL) {  // 이전 맵이 없으면
      minMax[0] = min_x;  // X 최소값
      minMax[1] = max_x;  // X 최대값
      minMax[2] = min_y;  // Y 최소값
      minMax[3] = max_y;  // Y 최대값
      OcMap = newOcMap;  // 새로운 맵 설정
    } else {
      computeBoundingBox(minMax, newOcMap, OcMap);  // 경계 박스 계산
      delete OcMap;  // 이전 맵 삭제
      OcMap = newOcMap;  // 새로운 맵 설정
    }
    minMax[4] = min_z;  // Z 최소값
    minMax[5] = max_z;  // Z 최대값

    update2Dmap(minMax);  // 2D 맵 업데이트
    pub();  // 데이터 퍼블리시
  }

  // 두 OctoMap의 경계 박스를 계산
  void computeBoundingBox(vector<double> &minMax, octomap::OcTree *tree1,
                          octomap::OcTree *tree2) {
    // 최소/최대 좌표 초기화
    double min_x = INFINITY;
    double min_y = INFINITY;
    double min_z = INFINITY;
    double max_x = -INFINITY;
    double max_y = -INFINITY;
    double max_z = -INFINITY;

    // tree1의 모든 리프 노드를 순회
    for (octomap::OcTree::leaf_iterator it = tree1->begin_leafs(),
                                        end = tree1->end_leafs();
         it != end; ++it) {
      octomap::OcTreeKey key = it.getKey();  // 현재 노드의 키 가져오기
      octomap::OcTreeNode *node2 = tree2->search(key);  // tree2에서 해당 키로 노드 검색
      if (node2 != nullptr) {  // tree2에서 노드를 찾은 경우
        bool occupied1 = tree1->isNodeOccupied(*it);  // tree1에서의 점유 상태
        bool occupied2 = tree2->isNodeOccupied(node2);  // tree2에서의 점유 상태
        if (occupied1 != occupied2) {  // 점유 상태가 달라진 경우
          double x = it.getX();
          double y = it.getY();
          double z = it.getZ();
          updateBoundingBox(x, y, z, min_x, min_y, min_z, max_x, max_y, max_z);  // 경계 박스 갱신
        }
      } else {  // tree2에서 노드를 찾지 못한 경우 (노드가 제거된 경우)
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        updateBoundingBox(x, y, z, min_x, min_y, min_z, max_x, max_y, max_z);  // 경계 박스 갱신
      }
    }

    // 계산된 최소/최대값을 minMax 벡터에 저장
    minMax[0] = min_x;  // X 최소값
    minMax[1] = max_x;  // X 최대값
    minMax[2] = min_y;  // Y 최소값
    minMax[3] = max_y;  // Y 최대값
  }

  // 경계 박스 값을 업데이트
  void updateBoundingBox(double x, double y, double z, double &min_x,
                         double &min_y, double &min_z, double &max_x,
                         double &max_y, double &max_z) {
    if (x < min_x)  // 현재 X 값이 최소값보다 작으면 갱신
      min_x = x;
    if (y < min_y)  // 현재 Y 값이 최소값보다 작으면 갱신
      min_y = y;
    if (z < min_z)  // 현재 Z 값이 최소값보다 작으면 갱신
      min_z = z;
    if (x > max_x)  // 현재 X 값이 최대값보다 크면 갱신
      max_x = x;
    if (y > max_y)  // 현재 Y 값이 최대값보다 크면 갱신
      max_y = y;
    if (z > max_z)  // 현재 Z 값이 최대값보다 크면 갱신
      max_z = z;
  }

  // 2D 맵을 AABB(Axis-Aligned Bounding Box) 영역으로 업데이트
  void update2Dmap(vector<double> minMax) {
    for (double mM : minMax)  // minMax에 무한대 값이 포함되어 있으면 종료
      if (isinf(mM))
        return;

    vector<voxel> voxelList;  // 점유 공간 및 자유 공간 복셀 리스트
    // AABB 영역을 정의하는 최소 및 최대 좌표
    octomap::point3d minPoint(minMax[0], minMax[2], minMax[4]);  // AABB 최소점
    octomap::point3d maxPoint(minMax[1], minMax[3], minMax[5]);  // AABB 최대점

    // AABB 내의 리프 노드들을 순회
    for (auto it = OcMap->begin_leafs_bbx(minPoint, maxPoint),
              it_end = OcMap->end_leafs_bbx();
         it != it_end; ++it) {
      voxel v;  // 새로운 복셀 객체
      v.position.x = it.getX();  // X 좌표
      v.position.y = it.getY();  // Y 좌표
      v.position.z = it.getZ();  // Z 좌표
      v.halfSize = it.getSize() / 2;  // 복셀의 반쪽 크기
      v.occupied = OcMap->isNodeOccupied(*it);  // 노드 점유 상태
      voxelList.push_back(v);  // 복셀 리스트에 추가
    }

    // MapConverter를 사용하여 맵 업데이트
    MC->updateMap(voxelList, minMax);
  }

  void pub() {
    mapMsg.header.stamp = ros::Time::now(); // 맵 메시지의 헤더에 현재 시간 설정
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
    mapMsg.data.resize(mapMsg.info.width * mapMsg.info.height); // 맵 데이터 배열의 크기를 너비 x 높이로 설정
    heightMsg.top.resize(mapMsg.info.width * mapMsg.info.height); // 높이 맵의 상단 데이터 배열 크기 설정
    heightMsg.bottom.resize(mapMsg.info.width * mapMsg.info.height); // 높이 맵의 하단 데이터 배열 크기 설정
    slopeMsg.slope.resize(mapMsg.info.width * mapMsg.info.height); // 경사 맵 데이터 배열 크기 설정

    // UGV를 위한 맵 퍼블리시
    if (pubMapUGV.getNumSubscribers() != 0) { // UGV 맵 퍼블리셔에 구독자가 있는 경우
      if (isinf(slopeMax)) { // 최대 경사값이 설정되지 않았으면 경고 메시지 출력
        ROS_WARN(
            "The max slope UGV is not set; obstacles will not be included in "
            "the UGV map. \nAdd \'_max_slope_ugv:=x\' to rosrun command, or "
            "\'<param name=\"max_slope_ugv\" value=\"x\"/>\' in launch file.");
      }

      mapMsg.header.stamp = ros::Time::now(); // 맵 메시지의 헤더에 현재 시간 갱신
      for (int y = 0; y < MC->map.sizeY(); y++) { // Y 좌표를 순회
        for (int x = 0; x < MC->map.sizeX(); x++) { // X 좌표를 순회
          int index = x + y * MC->map.sizeX(); // 1차원 배열 인덱스 계산

          mapMsg.data[index] = MC->map.get(x, y, slopeMax); // UGV 맵 데이터 계산 및 설정
        }
      }
      pubMapUGV.publish(mapMsg); // UGV 맵 퍼블리시
    }


    // UAV 맵 퍼블리시
    if (pubMapUAV.getNumSubscribers() != 0) { // UAV 맵 퍼블리셔에 구독자가 있는 경우
      for (int y = 0; y < MC->map.sizeY(); y++) { // Y 좌표를 순회
        for (int x = 0; x < MC->map.sizeX(); x++) { // X 좌표를 순회
          int index = x + y * MC->map.sizeX(); // 1차원 배열 인덱스 계산

          mapMsg.data[index] = MC->map.get(x, y); // UAV 맵 데이터 계산 및 설정
        }
      }
      pubMapUAV.publish(mapMsg); // UAV 맵 퍼블리시
    }

    // RViz에서 바닥 높이 맵 시각화 퍼블리시
    if (pubMapFloor.getNumSubscribers() != 0) { // 바닥 맵 퍼블리셔에 구독자가 있는 경우
      if (MC->map.sizeY() == 0 || MC->map.sizeX() == 0) // 맵의 크기가 0이면 종료
        return;
      double vMax = NAN, vMin = NAN; // 최대값 및 최소값 초기화
      // 정규화를 위한 최대값 및 최소값 찾기
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          double v = MC->map.getHeight(x, y); // 바닥 높이 값 가져오기
          if (isnan(v)) // 값이 유효하지 않으면 건너뜀
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
          mapMsg.data[index] = value; // 바닥 높이 맵 데이터 설정
        }
      }

      pubMapFloor.publish(mapMsg); // 바닥 높이 맵 퍼블리시
    }

       // RViz에서 천장 높이 맵 시각화 퍼블리시
    if (pubMapCeiling.getNumSubscribers() != 0) { // 천장 맵 퍼블리셔에 구독자가 있는 경우
      if (MC->map.sizeY() == 0 || MC->map.sizeX() == 0) // 맵 크기가 0이면 종료
        return;
      double vMax = NAN, vMin = NAN; // 최대값 및 최소값 초기화
      // 정규화를 위한 최대값 및 최소값 찾기
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          double v = MC->map.getHeightTop(x, y); // 천장 높이 값 가져오기
          if (isnan(v)) // 값이 유효하지 않으면 건너뜀
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
          mapMsg.data[index] = value; // 천장 높이 맵 데이터 설정
        }
      }

      pubMapCeiling.publish(mapMsg); // 천장 높이 맵 퍼블리시
    }

    // 높이 맵 퍼블리시
    if (pubHeightMap.getNumSubscribers() != 0) { // 높이 맵 퍼블리셔에 구독자가 있는 경우
      for (int y = 0; y < MC->map.sizeY(); y++) { // Y 좌표를 순회
        for (int x = 0; x < MC->map.sizeX(); x++) { // X 좌표를 순회
          int index = x + y * MC->map.sizeX(); // 1차원 배열 인덱스 계산

          heightMsg.bottom[index] = MC->map.getHeight(x, y); // 바닥 높이 설정
          heightMsg.top[index] = MC->map.getHeightTop(x, y); // 천장 높이 설정
        }
      }
      pubHeightMap.publish(heightMsg); // 높이 맵 퍼블리시
    }

    // RViz에서 경사 맵 시각화 퍼블리시
    if (pubMapSlopeVis.getNumSubscribers() != 0) { // 경사 맵 시각화 퍼블리셔에 구독자가 있는 경우
      double vMax = slopeMax * 2, vMin = 0; // 경사 맵의 최대값 및 최소값 설정

      for (int y = 0; y < MC->map.sizeY(); y++) { // Y 좌표를 순회
        for (int x = 0; x < MC->map.sizeX(); x++) { // X 좌표를 순회
          int index = x + y * MC->map.sizeX(); // 1차원 배열 인덱스 계산
          double value = min(MC->map.getSlope(x, y), slopeMax * 2); // 경사 값 계산
          if (!isnan(value)) { // 값이 유효하면
            value = (value - vMin) / (vMax - vMin) * 198 + 1; // 정규화
            if (value > 99)
              value -= 199; // RViz에서 색상 범위 설정
          }
          mapMsg.data[index] = value; // 경사 맵 데이터 설정
        }
      }
      pubMapSlopeVis.publish(mapMsg); // 경사 맵 시각화 퍼블리시
    }

    // 경사 맵 퍼블리시
    if (pubMapSlope.getNumSubscribers() != 0) { // 경사 맵 퍼블리셔에 구독자가 있는 경우
      for (int y = 0; y < MC->map.sizeY(); y++) { // Y 좌표를 순회
        for (int x = 0; x < MC->map.sizeX(); x++) { // X 좌표를 순회
          int index = x + y * MC->map.sizeX(); // 1차원 배열 인덱스 계산

          slopeMsg.slope[index] = MC->map.getSlope(x, y); // 경사 값 설정
        }
      }
      pubMapSlope.publish(slopeMsg); // 경사 맵 퍼블리시
    }
  }
 };

int main(int argc, char **argv) {
  ros::init(argc, argv, "map_saver"); // ROS 노드를 "map_saver"라는 이름으로 초기화

  MapToMap mtm; // MapToMap 객체 생성

  ros::spin(); // ROS 콜백 함수가 호출되도록 대기

  return 0; // 프로그램 종료
}
