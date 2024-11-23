#include "datatypes.hh"  // 사용자 정의 데이터 타입 헤더 포함

#include <geometry_msgs/Pose.h>  // ROS Pose 메시지 포함
#include <mapconversion/HeightMap.h>  // HeightMap 메시지 포함
#include <nav_msgs/Path.h>  // ROS Path 메시지 포함

using namespace std;  // 표준 네임스페이스 사용

class PathConverter {
private:
  // ROS NodeHandle
  ros::NodeHandle nh;

  // ROS 구독자
  ros::Subscriber subPath, subHeight;

  // ROS 퍼블리셔
  ros::Publisher pubPath;

  // 맵 데이터
  mapconversion::HeightMap hMap;  // HeightMap 데이터
  nav_msgs::Path cPath;  // 변환된 경로 데이터
  double currentResulution = 0;  // 현재 맵의 해상도

  // 충돌 영역 데이터
  bool collsionShape;  // 충돌 영역 모양 사용 여부
  double collisionRadius;  // 충돌 영역 반경

  // 충돌 포인트 구조체 정의
  struct collisionPoint {
    int x, y;  // 충돌 포인트의 X, Y 좌표
    double z;  // 충돌 포인트의 Z 좌표
  };

  vector<collisionPoint> collisionArea;  // 충돌 영역을 저장하는 벡터

  // 경로 매개변수
  int pathSmothingLength;  // 경로 부드럽게 만들기 위한 길이
  double pathOffset;  // 경로 오프셋

public:
  // 생성자
  PathConverter() {
    ros::NodeHandle nh_priv("~");  // 프라이빗 NodeHandle

    // 매개변수 초기화
    collsionShape = nh_priv.param("use_collision_sphere", false);  // 충돌 구 사용 여부
    collisionRadius = nh_priv.param("collision_radius", 1.0);  // 충돌 반경
    pathOffset = nh_priv.param("path_offset", 0.0);  // 경로 오프셋
    pathSmothingLength = nh_priv.param("path_smothing_length", 5);  // 경로 부드러움 길이

    // HeightMap 구독자 초기화
    subHeight =
        nh.subscribe("heightMap", 1, &PathConverter::heightCallback, this);

    // 입력 경로 구독자 초기화
    subPath = nh.subscribe("pathIn", 10, &PathConverter::pathCallback, this);

    // 출력 경로 퍼블리셔 초기화
    pubPath = nh.advertise<nav_msgs::Path>("pathOut", 10);
  }

  // 소멸자
  ~PathConverter() {}

  // HeightMap 콜백 함수
  void heightCallback(mapconversion::HeightMap newHeightMap) {
    hMap = newHeightMap;  // 새 HeightMap 데이터 저장
    updateCollisionArea(newHeightMap.info.resolution);  // 충돌 영역 업데이트
    updatePath();  // 경로 업데이트
  }

  // 충돌 영역을 업데이트
  void updateCollisionArea(double newResulution) {
    if (newResulution == currentResulution)  // 해상도가 변경되지 않았다면 종료
      return;
    currentResulution = newResulution;  // 새로운 해상도를 현재 해상도로 설정
    if (currentResulution <= 0.0)  // 해상도가 유효하지 않으면 종료
      return;

    if (collsionShape) {  // 충돌 구를 사용하는 경우
      generateCollisionSphere();  // 충돌 구 생성
    } else {
      collisionArea.clear();  // 기존 충돌 영역을 초기화
      collisionArea.push_back({0, 0, 0.0});  // 로봇 위치에 단일 점으로 충돌 영역 설정
    }
  }

  // XY 평면을 기준으로 반구 형태의 충돌 구 생성
  void generateCollisionSphere() {
    int radiusSize = int(ceil(collisionRadius / currentResulution));  // 충돌 반경을 셀 단위로 계산
    collisionArea.clear();  // 기존 충돌 영역 초기화
    for (int x = -radiusSize; x <= radiusSize; x++) {  // X 방향으로 범위 순회
      for (int y = -radiusSize; y <= radiusSize; y++) {  // Y 방향으로 범위 순회
        if (sqrt(x * x + y * y) > radiusSize)  // 반경 밖의 점은 무시
          continue;
        double rX = x * currentResulution;  // X 좌표를 실제 위치로 변환
        double rY = y * currentResulution;  // Y 좌표를 실제 위치로 변환
        double rZ = sqrt(collisionRadius * collisionRadius - rX * rX - rY * rY);  // Z 좌표 계산
        if (isnan(rZ))  // 계산된 Z 좌표가 유효하지 않으면 무시
          continue;
        collisionPoint p = {x, y, rZ};  // 충돌 점 생성
        collisionArea.push_back(p);  // 충돌 영역에 추가
      }
    }
  }

  // 입력 경로 콜백
  void pathCallback(nav_msgs::Path inPath) {
    cPath = inPath;  // 입력 경로 저장
    updatePath();  // 경로 업데이트
  }

  // 경로 업데이트
  void updatePath() {
    if (collisionArea.size() == 0)  // 충돌 영역이 비어 있으면 종료
      return;
    nav_msgs::Path outPath = cPath;  // 입력 경로를 출력 경로로 복사

    setPathHeight3D(&outPath);  // 3D 높이 설정

    if (collsionShape)  // 충돌 구가 설정된 경우
      solveCollisionUAV(&outPath);  // UAV 충돌 해결

    pubPath.publish(outPath);  // 변환된 경로 퍼블리시
  }

  // 경로의 바닥 높이를 기반으로 3D 경로 설정
  void setPathHeight3D(nav_msgs::Path *path) {
    vector<double> heightList;  // 경로 높이 리스트
    for (int i = 0; i < pathSmothingLength && i < path->poses.size(); i++) {  // 경로의 초기 부분을 처리
      auto p = path->poses[i];  // 경로 점 가져오기
      point_int point = worldToMap(p.pose);  // 월드 좌표를 맵 좌표로 변환
      double height = getHeight(point.x, point.y);  // 해당 위치의 높이 가져오기
      if (isnan(height))  // 높이가 유효하지 않으면 건너뜀
        continue;
      heightList.insert(heightList.begin(), height);  // 높이 리스트에 추가
    }

    for (int i = 0; i < path->poses.size(); i++) {  // 전체 경로를 처리
      if (i + pathSmothingLength < path->poses.size()) {  // 부드러운 길이 범위를 초과하지 않으면
        auto p = path->poses[i + pathSmothingLength];  // 경로 점 가져오기
        point_int point = worldToMap(p.pose);  // 월드 좌표를 맵 좌표로 변환
        double height = getHeight(point.x, point.y);  // 해당 위치의 높이 가져오기
        if (isnan(height))  // 높이가 유효하지 않으면 건너뜀
          continue;
        heightList.insert(heightList.begin(), height);  // 높이 리스트에 추가
        if (heightList.size() > pathSmothingLength * 2)  // 높이 리스트 크기 제한
          heightList.pop_back();
      }

      double pathHeight = -INFINITY;  // 초기 경로 높이 값 설정
      for (double h : heightList)  // 높이 리스트에서 최대값 계산
        pathHeight = max(pathHeight, h);

      if (isinf(pathHeight))  // 경로 높이가 유효하지 않으면 건너뜀
        continue;

      path->poses[i].pose.position.z = pathHeight + pathOffset;  // 경로 높이에 오프셋 추가
    }
  }
  // UAV 경로를 충돌을 피하도록 상하 이동
  void solveCollisionUAV(nav_msgs::Path *path) {
    for (int i = 0; i < path->poses.size(); i++) {  // 경로의 모든 점에 대해 반복
      auto *pose = &path->poses[i];  // 현재 경로 점 가져오기
      point_int point = worldToMap(pose->pose);  // 경로 점의 월드 좌표를 맵 좌표로 변환

      // 바닥과의 충돌 체크 및 해결
      double maxHeight = pose->pose.position.z;  // 초기 최대 높이는 현재 Z 값
      for (auto ap : collisionArea) {  // 충돌 영역의 각 점 확인
        double h = getHeight(point.x + ap.x, point.y + ap.y);  // 해당 위치의 바닥 높이 가져오기
        if (isnan(h))  // 높이가 유효하지 않으면 건너뜀
          continue;
        h += ap.z;  // 충돌 영역 Z 값 추가
        if (maxHeight < h)  // 최대 높이를 갱신
          maxHeight = h;
      }

      // 천장과의 충돌 체크 및 해결
      double minHeight = INFINITY;  // 초기 최소 높이는 무한대
      for (auto ap : collisionArea) {  // 충돌 영역의 각 점 확인
        double h = getHeightTop(point.x + ap.x, point.y + ap.y);  // 해당 위치의 천장 높이 가져오기
        if (isnan(h))  // 높이가 유효하지 않으면 건너뜀
          continue;
        h -= ap.z;  // 충돌 영역 Z 값 차감
        if (minHeight > h)  // 최소 높이를 갱신
          minHeight = h;
      }

      pose->pose.position.z = min(maxHeight, minHeight);  // 경로 점의 Z 값을 충돌이 없는 높이로 설정
    }
  }

  // 월드 좌표를 맵 좌표로 변환
  point_int worldToMap(geometry_msgs::Pose position) {
    double rez = hMap.info.resolution;  // 맵 해상도
    return {int((position.position.x - hMap.info.origin.position.x) / rez),  // X 좌표 변환
            int((position.position.y - hMap.info.origin.position.y) / rez)};  // Y 좌표 변환
  }

  // 특정 X, Y 위치의 바닥 높이 반환
  double getHeight(int x, int y) {
    int index = x + y * hMap.info.width;  // 1D 인덱스로 변환
    if (index < 0 || index >= hMap.bottom.size())  // 유효한 인덱스인지 확인
      return NAN;
    double value = hMap.bottom[index];  // 바닥 높이 값 가져오기
    return value;
  }

  // 특정 X, Y 위치의 천장 높이 반환
  double getHeightTop(int x, int y) {
    int index = x + y * hMap.info.width;  // 1D 인덱스로 변환
    if (index < 0 || index >= hMap.top.size())  // 유효한 인덱스인지 확인
      return NAN;
    double value = hMap.top[index];  // 천장 높이 값 가져오기
    return value;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_converter");  // ROS 노드 초기화

  PathConverter PC;  // PathConverter 객체 생성
  ros::spin();  // ROS 콜백 함수 실행

  return 0;  // 프로그램 종료
}

 
