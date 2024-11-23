#pragma once
// 헤더 파일의 중복 포함을 방지하기 위한 매크로

#include <Eigen/Dense>
// 선형 대수 계산을 위한 Eigen 라이브러리를 포함

#include <vector>
// 표준 벡터 자료구조를 사용하기 위해 포함

using namespace std;
// 표준 라이브러리(std)를 전역에서 사용하도록 설정

using namespace Eigen;
// Eigen 네임스페이스를 전역에서 사용하도록 설정

struct point_int {
  int x, y;
  // 정수형 2D 좌표를 저장하는 구조체 (x와 y 값을 저장)
};

struct point2D {
  double x, y;
  // 실수형 2D 좌표를 저장하는 구조체 (x와 y 값을 저장)
};

struct point3D {
  double x, y, z;
  // 실수형 3D 좌표를 저장하는 구조체 (x, y, z 값을 저장)
};

struct voxel {
  point3D position;
  // 보폭(voxel)의 중심 좌표 (3D 위치 정보)

  float halfSize;
  // 보폭의 한 변의 절반 길이 (보폭의 크기 정보)

  bool occupied;
  // 해당 보폭이 점유 상태인지 여부를 나타내는 플래그
};

struct heightRange {
  double top, bottom;
  // 높이의 범위를 나타내는 구조체 (상단과 하단 값을 저장)
};
