#pragma once  // 헤더 파일 중복 포함 방지

#include "ros/ros.h"  // ROS 관련 기능 포함
#include <Eigen/Dense>  // Eigen 라이브러리 포함 (선형대수 연산)
#include <vector>  // STL 벡터 포함

using namespace std;  // 표준 네임스페이스 사용
using namespace Eigen;  // Eigen 네임스페이스 사용

// 2D 정수 좌표를 나타내는 구조체
struct point_int {
  int x, y;  // x, y 좌표
};

// 8방향을 나타내는 상수 배열
point_int DIRECTIONS[] = {
    {1, 0},   // 동쪽
    {1, 1},   // 북동쪽
    {0, 1},   // 북쪽
    {-1, 1},  // 북서쪽
    {-1, 0},  // 서쪽
    {-1, -1}, // 남서쪽
    {0, -1},  // 남쪽
    {1, -1}   // 남동쪽
};

// 2D 실수 좌표를 나타내는 구조체
struct point2D {
  double x, y;  // x, y 좌표
};

// 3D 실수 좌표를 나타내는 구조체
struct point3D {
  double x, y, z;  // x, y, z 좌표
};

// 복셀(voxel) 데이터를 나타내는 구조체
struct voxel {
  point3D position;  // 복셀의 중심 좌표
  float halfSize;  // 복셀의 반쪽 크기
  bool occupied;  // 복셀의 점유 상태 (점유 여부)
};

// 높이 범위를 나타내는 구조체
struct heightRange {
  double top, bottom;  // 범위의 상단과 하단 높이
};
