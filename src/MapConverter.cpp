#include "MapConverter.hh"  // MapConverter 클래스 헤더 포함

// MapConverter 클래스 생성자
MapConverter::MapConverter(double resolution, int slopeEstimationSize,
                           double minimumZ, int minimumOccupancy) {
  MapConverter::resolution = resolution;  // 맵 해상도 초기화
  MapConverter::slopeEstimationSize = slopeEstimationSize;  // 경사 추정 크기 초기화
  MapConverter::minOcc = minimumOccupancy;  // 최소 점유 수준 초기화
  MapConverter::minZ = minimumZ;  // 최소 Z 값 초기화
  Map2D m(resolution);  // 주어진 해상도로 Map2D 객체 생성
  MapConverter::map = m;  // Map2D 객체를 맵으로 설정
}

// MapConverter 클래스 소멸자
MapConverter::~MapConverter() {}

void MapConverter::updateMap(vector<voxel> vMap, vector<double> minMax) {
  if (vMap.size() == 0)  // 복셀 벡터가 비어 있으면 함수 종료
    return;

  for (double mM : minMax)  // minMax에 무한대 값이 있으면 함수 종료
    if (isinf(mM))
      return; // 목표 해상도에 충분한 복셀 크기가 확보되지 않은 경우

  // 그리드에서 셀의 개수 계산
  int xSize = (minMax[1] - minMax[0]) / MapConverter::map.getResulution();  // X 방향 셀 개수
  int ySize = (minMax[3] - minMax[2]) / MapConverter::map.getResulution();  // Y 방향 셀 개수

  // 로컬 높이 맵 생성
  HeightRangeMap hMap(xSize, ySize);

  for (voxel v : vMap) {  // 주어진 복셀 벡터를 순회
    // 복셀 크기가 맵 셀보다 클 수 있으므로 복셀에 포함된 모든 셀을 순회
    int sizeIndex = v.halfSize * 2 / MapConverter::map.getResulution();
    for (int x = 0; x < sizeIndex; x++) {
      for (int y = 0; y < sizeIndex; y++) {
        int posX = (v.position.x - v.halfSize + resolution * x - minMax[0]) /
                   resolution;  // 셀 단위의 X 좌표 계산
        int posY = (v.position.y - v.halfSize + resolution * y - minMax[2]) /
                   resolution;  // 셀 단위의 Y 좌표 계산

        if (posX < 0 || posX >= xSize)  // 셀이 로컬 맵 그리드 밖에 있으면 무시
          continue;
        if (posY < 0 || posY >= ySize)
          continue;

        // 높이 범위를 약간 확장하여 더 나은 중복 탐지를 지원
        heightRange hr = {v.position.z + v.halfSize + resolution * 0.001,
                          v.position.z - v.halfSize - resolution * 0.001};
        hMap.addRange(posX, posY, hr, v.occupied,
                      v.position.x - v.halfSize + resolution * x,
                      v.position.y - v.halfSize + resolution * y);
      }
    }
  }

  // 로봇 안전 마진보다 작은 자유 공간 제거
  for (int x = 1; x < xSize - 1; x++) {
    for (int y = 1; y < ySize - 1; y++) {
      hMap.removeFreeRanges(x, y, minZ);  // 지정된 최소 높이 이하의 자유 공간 제거
      int mapValue = 0;

      if (hMap.free[x][y].size() == 0) {  // 자유 공간이 없으면 좌표 계산
        hMap.posMap[x][y].x = x * resolution + minMax[0];
        hMap.posMap[x][y].y = y * resolution + minMax[2];
        mapValue = -1;
      }

      if (mapValue == -1 &&
          map.get(hMap.posMap[x][y].x, hMap.posMap[x][y].y) == -1)
        continue;  // 맵 값이 이미 설정된 경우 건너뜀

      // 자유 공간 및 높이 맵 설정
      map.set(hMap.posMap[x][y].x, hMap.posMap[x][y].y, mapValue);
      if (mapValue == -1)
        continue;

      map.setHeight(hMap.posMap[x][y].x, hMap.posMap[x][y].y,
                    hMap.free[x][y].back().bottom);  // 바닥 높이 설정
      map.setHeightTop(hMap.posMap[x][y].x, hMap.posMap[x][y].y,
                       hMap.free[x][y][0].top);  // 천장 높이 설정
    }
  }

  // 인접한 점유된 셀이 있는지 확인
  for (int x = 2; x < xSize - 2; x++) {
    for (int y = 2; y < ySize - 2; y++) {
      if (hMap.free[x][y].size() == 0)  // 자유 공간이 없는 셀은 건너뜀
        continue;

      for (auto d : DIRECTIONS) {  // 모든 방향을 확인
        if (hMap.free[x + d.x][y + d.y].size() != 0)  // 인접 셀이 자유 공간이면 건너뜀
          continue;

        // 점유 상태를 맵에 설정
        map.set(hMap.posMap[x + d.x][y + d.y].x, hMap.posMap[x][y].y,
                hMap.getOccupation(x + d.x, y + d.y, -1, minOcc));
      }
    }
  }

  // 경사값 업데이트
  map.updateSlope(slopeEstimationSize, minMax[0], minMax[1], minMax[2], minMax[3]);
}

vector<double> MapConverter::minMaxVoxel(vector<voxel> list) {
  vector<double> minMax(6);  // 최소/최대값을 저장할 벡터
  minMax[0] = INFINITY;  // X 최소값 초기화
  minMax[1] = -INFINITY;  // X 최대값 초기화
  minMax[2] = INFINITY;  // Y 최소값 초기화
  minMax[3] = -INFINITY;  // Y 최대값 초기화
  minMax[4] = INFINITY;  // Z 최소값 초기화
  minMax[5] = -INFINITY;  // Z 최대값 초기화

  for (voxel v : list) {  // 주어진 복셀 리스트를 순회
    // 새로운 영역의 신뢰성 있는 크기 추정을 위해 Octree에서 가장 작은 복셀만 사용
    if (v.halfSize > resolution * .9)  // 복셀의 반 크기가 해상도의 0.9배보다 크면 무시
      continue;

    // 각 좌표에서 최소/최대값 계산
    minMax[0] = min(minMax[0], v.position.x);  // X 최소값 갱신
    minMax[1] = max(minMax[1], v.position.x);  // X 최대값 갱신
    minMax[2] = min(minMax[2], v.position.y);  // Y 최소값 갱신
    minMax[3] = max(minMax[3], v.position.y);  // Y 최대값 갱신
    minMax[4] = min(minMax[4], v.position.z);  // Z 최소값 갱신
    minMax[5] = max(minMax[5], v.position.z);  // Z 최대값 갱신
  }

  return minMax;  // 최소/최대값 벡터 반환
}
