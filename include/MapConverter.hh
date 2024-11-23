#pragma once  // 헤더 파일의 중복 포함 방지

#include "HeightRangeMap.hh"  // HeightRangeMap 관련 헤더 포함
#include "Map2D.hh"  // Map2D 관련 헤더 포함
#include "datatypes.hh"  // 사용자 정의 데이터 타입 헤더 포함

using namespace std;  // 표준 네임스페이스 사용

class MapConverter {  // 맵 변환기를 정의하는 클래스
public:
  Map2D map;  // 2D 맵 객체
  // 생성자: 해상도, 경사 추정 크기, 최소 Z 값, 최소 점유 수준 초기화
  MapConverter(double resolution, int slopeEstimationSize, double minimumZ,
               int minimumOccupancy);
  ~MapConverter();  // 소멸자

  // 주어진 복셀 정보로 글로벌 맵을 업데이트
  void updateMap(vector<voxel> vMap, vector<double> minMax);

  // 주어진 복셀 리스트에 대해 맵 프레임 기준의 x, y 최소/최대값을 계산
  // 반환값: <minX, maxX, minY, maxY, minZ, maxZ> 형태의 벡터
  vector<double> minMaxVoxel(vector<voxel> list);

private:
  int slopeEstimationSize;  // 경사 추정을 위한 영역 크기
  double resolution;  // 맵의 해상도
  int minOcc;  // 최소 점유 수준
  double minZ;  // 맵의 최소 Z 값
};
