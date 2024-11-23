#pragma once // 헤더 파일의 중복 포함을 방지하기 위해 사용

#include "Map2D.hh" // Map2D 클래스의 정의를 포함하기 위해 Map2D.hh 파일을 포함
#include "datatypes.hh" // voxel 및 관련 데이터 타입 정의를 포함하기 위해 datatypes.hh 파일을 포함

using namespace std; // 표준 라이브러리의 네임스페이스(std)를 전역에서 사용하도록 설정

// MapConverter 클래스 정의
class MapConverter {
public:
  Map2D map; // 2D 맵 데이터를 저장하는 Map2D 클래스 객체
  MapConverter(double resolution, int slopeEstimationSize, double minimumZ,
               int minimumOccupancy); // 클래스의 생성자로, 매개변수를 통해 초기화
  ~MapConverter(); // 클래스 소멸자, 리소스 해제를 위해 사용

  // 주어진 voxel 리스트를 기반으로 지정된 영역 내에서 전역 맵을 업데이트하는 함수
  void updateMap(vector<voxel> vMap, vector<double> minMax);

  // 주어진 voxel 리스트를 맵 프레임 기준으로 변환하여 x, y, z 축의 최소/최대 값을 반환하는 함수
  // <minX, maxX, minY, maxY, minZ, maxZ> 형태의 벡터를 반환
  vector<double> minMaxVoxel(vector<voxel> list);

private:
  int slopeEstimationSize; // 경사 추정을 위한 윈도우 크기
  double resolution; // 맵의 해상도
  int minOcc; // 점유를 최소로 간주하는 점유 값
  double minZ; // Z 축의 최소값 기준
};
