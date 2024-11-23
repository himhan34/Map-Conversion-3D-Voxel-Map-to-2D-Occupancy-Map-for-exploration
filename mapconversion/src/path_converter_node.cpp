#include "datatypes.hh" 
#include <geometry_msgs/msg/pose.hpp> // Pose 메시지 포함
#include <mapconversion_msgs/msg/height_map.hpp> // HeightMap 메시지 포함
#include <nav_msgs/msg/path.hpp> // Path 메시지 포함
#include <rclcpp/rclcpp.hpp> // ROS2 노드 인터페이스 포함

using namespace std; // std 네임스페이스 사용

// PathConverter 클래스 정의, rclcpp::Node를 상속받음
class PathConverter : public rclcpp::Node {
private:
  // HeightMap 구독자
  rclcpp::Subscription<mapconversion_msgs::msg::HeightMap>::SharedPtr subHeight;
  // Path 구독자
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath;
  // Path 퍼블리셔
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

  // 맵 데이터
  mapconversion_msgs::msg::HeightMap hMap; // HeightMap 데이터 저장
  nav_msgs::msg::Path cPath; // 경로 데이터 저장
  double currentResulution = 0; // 현재 해상도

  // 충돌 영역 관련 변수
  bool collsionShape; // 충돌 영역 형태 사용 여부
  double collisionRadius; // 충돌 반경

  // 충돌 영역 좌표 구조체 정의
  struct collisionPoint {
    int x, y; // 충돌 지점의 x, y 좌표
    double z; // 충돌 지점의 높이 값
  };

  vector<collisionPoint> collisionArea; // 충돌 영역 좌표 저장 벡터

  // 경로 관련 매개변수
  int pathSmothingLength; // 경로 스무딩 길이
  double pathOffset; // 경로 오프셋

public:
  // 생성자 정의
  PathConverter() : Node("path_converter") {
    // 매개변수 선언 및 초기화
    collsionShape = this->declare_parameter("use_collision_sphere", false); // 충돌 영역 형태 사용 여부
    collisionRadius = this->declare_parameter("collision_radius", 1.0); // 충돌 반경 초기값
    pathOffset = this->declare_parameter("path_offset", 0.0); // 경로 오프셋 초기값
    pathSmothingLength = this->declare_parameter("path_smothing_length", 5); // 경로 스무딩 길이 초기값

    // HeightMap 구독자 생성
    subHeight = this->create_subscription<mapconversion_msgs::msg::HeightMap>(
        "heightMap", 1,
        std::bind(&PathConverter::heightCallback, this, std::placeholders::_1));

    // Path 구독자 생성
    subPath = this->create_subscription<nav_msgs::msg::Path>(
        "pathIn", 10,
        std::bind(&PathConverter::pathCallback, this, std::placeholders::_1));

    // Path 퍼블리셔 생성
    pubPath = this->create_publisher<nav_msgs::msg::Path>("pathOut", 10);
  }

  // 소멸자 정의
  ~PathConverter() {}

// 높이 맵 데이터를 받는 콜백 함수
void heightCallback(mapconversion_msgs::msg::HeightMap newHeightMap) {
    hMap = newHeightMap; // 새로운 높이 맵 데이터를 저장
    updateCollisionArea(newHeightMap.info.resolution); // 충돌 영역을 업데이트
    updatePath(); // 경로를 업데이트
}

// 충돌 영역을 업데이트하는 함수
void updateCollisionArea(double newResulution) {
    if (newResulution == currentResulution) // 해상도가 변경되지 않았으면 실행하지 않음
        return;
    currentResulution = newResulution; // 현재 해상도를 업데이트
    if (currentResulution <= 0.0) // 해상도가 유효하지 않으면 실행하지 않음
        return;

    if (collsionShape) { // 충돌 형태가 설정되어 있는 경우
        generateCollisionSphere(); // 충돌 구체를 생성
    } else {
        collisionArea.clear(); // 충돌 영역 초기화
        collisionArea.push_back({0, 0, 0.0}); // 로봇 위치에 점을 설정
    }
}

// 반구 형태의 충돌 영역을 생성하는 함수
void generateCollisionSphere() {
    int radiusSize = int(ceil(collisionRadius / currentResulution)); // 반경 크기 계산
    collisionArea.clear(); // 기존 충돌 영역 초기화
    for (int x = -radiusSize; x <= radiusSize; x++) { // x축 탐색
        for (int y = -radiusSize; y <= radiusSize; y++) { // y축 탐색
            if (sqrt(x * x + y * y) > radiusSize) // 반지름 외부는 제외
                continue;
            double rX = x * currentResulution; // x 좌표 변환
            double rY = y * currentResulution; // y 좌표 변환
            double rZ = sqrt(collisionRadius * collisionRadius - rX * rX - rY * rY); // z 좌표 계산
            if (isnan(rZ)) // 계산 오류 시 제외
                continue;
            collisionPoint p = {x, y, rZ}; // 충돌 점 생성
            collisionArea.push_back(p); // 충돌 영역에 추가
        }
    }
}

// 경로 데이터를 받는 콜백 함수
void pathCallback(nav_msgs::msg::Path inPath) {
    cPath = inPath; // 경로 데이터 저장
    updatePath(); // 경로 업데이트
}

// 경로를 업데이트하는 함수
void updatePath() {
    if (collisionArea.size() == 0) // 충돌 영역이 비어 있으면 실행하지 않음
        return;
    nav_msgs::msg::Path outPath = cPath; // 경로 데이터 복사

    setPathHeight3D(&outPath); // 경로 높이 설정

    if (collsionShape) // 충돌 형태가 있는 경우
        solveCollisionUAV(&outPath); // UAV 충돌 해결

    pubPath->publish(outPath); // 결과 경로를 퍼블리시
}

// 경로의 높이를 설정하는 함수
void setPathHeight3D(nav_msgs::msg::Path *path) {
    vector<double> heightList; // 경로 높이 리스트
    for (int i = 0; i < pathSmothingLength && i < path->poses.size(); i++) { // 초기 높이 계산
        auto p = path->poses[i];
        point_int point = worldToMap(p.pose); // 월드 좌표를 맵 좌표로 변환
        double height = getHeight(point.x, point.y); // 높이 값 가져오기
        if (isnan(height)) // 높이가 유효하지 않으면 제외
            continue;
        heightList.insert(heightList.begin(), height); // 리스트에 삽입
    }
    for (int i = 0; i < path->poses.size(); i++) { // 경로 높이 업데이트
        if (i + pathSmothingLength < path->poses.size()) {
            auto p = path->poses[i + pathSmothingLength];
            point_int point = worldToMap(p.pose);
            double height = getHeight(point.x, point.y);
            if (isnan(height))
                continue;
            heightList.insert(heightList.begin(), height); // 새 높이 추가
            if (heightList.size() > pathSmothingLength * 2)
                heightList.pop_back(); // 오래된 값 제거
        }
        double pathHeight = -INFINITY; // 최대 높이 초기화
        for (double h : heightList)
            pathHeight = max(pathHeight, h); // 최대 높이 계산
        if (isinf(pathHeight))
            continue;
        path->poses[i].pose.position.z = pathHeight + pathOffset; // 경로 높이 설정
    }
}

// UAV 경로에서 충돌을 해결하는 함수
void solveCollisionUAV(nav_msgs::msg::Path *path) {
    for (int i = 0; i < path->poses.size(); i++) {
        auto *pose = &path->poses[i];
        point_int point = worldToMap(pose->pose); // 월드 좌표를 맵 좌표로 변환

        double maxHeight = pose->pose.position.z; // 최대 높이 초기화
        for (auto ap : collisionArea) { // 바닥 충돌 확인
            double h = getHeight(point.x + ap.x, point.y + ap.y);
            if (isnan(h))
                continue;
            h += ap.z;
            if (maxHeight < h)
                maxHeight = h; // 최대 높이 갱신
        }

        double minHeight = INFINITY; // 최소 높이 초기화
        for (auto ap : collisionArea) { // 천장 충돌 확인
            double h = getHeightTop(point.x + ap.x, point.y + ap.y);
            if (isnan(h))
                continue;
            h -= ap.z;
            if (minHeight > h)
                minHeight = h; // 최소 높이 갱신
        }
        pose->pose.position.z = min(maxHeight, minHeight); // 최종 높이 설정
    }
}

// 월드 좌표를 맵 좌표로 변환하는 함수
point_int worldToMap(geometry_msgs::msg::Pose position) {
    double rez = hMap.info.resolution; // 해상도 가져오기
    return {int((position.position.x - hMap.info.origin.position.x) / rez),
            int((position.position.y - hMap.info.origin.position.y) / rez)};
}

// 특정 좌표의 높이를 가져오는 함수
double getHeight(int x, int y) {
    int index = x + y * hMap.info.width; // 1차원 인덱스 계산
    if (index < 0 || index >= hMap.bottom.size()) // 범위를 벗어나면 NaN 반환
        return NAN;
    double value = hMap.bottom[index]; // 높이 값 가져오기
    return value;
}

// 특정 좌표의 천장 높이를 가져오는 함수
double getHeightTop(int x, int y) {
    int index = x + y * hMap.info.width; // 1차원 인덱스 계산
    if (index < 0 || index >= hMap.top.size()) // 범위를 벗어나면 NaN 반환
        return NAN;
    double value = hMap.top[index]; // 높이 값 가져오기
    return value;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // ROS2 초기화

    auto node = std::make_shared<PathConverter>(); // PathConverter 노드 생성
    rclcpp::spin(node); // 노드 실행
    rclcpp::shutdown(); // ROS2 종료

    return 0; // 프로그램 종료
}
