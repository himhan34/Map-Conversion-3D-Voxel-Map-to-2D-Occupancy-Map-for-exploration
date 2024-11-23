#include "MapConverter.hh"
#include "HeightRangeMap.hh"

// MapConverter 생성자: 지도 변환 객체를 초기화
MapConverter::MapConverter(double resolution, int slopeEstimationSize,
                           double minimumZ, int minimumOccupancy) {
  // 주어진 해상도를 저장 (지도의 단위 셀 크기를 결정)
  MapConverter::resolution = resolution;
  // 경사 추정 크기를 저장 (경사 계산 시 주변 셀 크기를 설정)
  MapConverter::slopeEstimationSize = slopeEstimationSize;
  // 최소 점유치(Occupancy)를 저장 (점유로 간주할 최소한의 값)
  MapConverter::minOcc = minimumOccupancy;
  // 최소 Z값을 저장 (로봇이 이동 가능한 최소 높이)
  MapConverter::minZ = minimumZ;
  // 해상도를 기반으로 2D 지도 객체 생성
  Map2D m(resolution);
  // MapConverter 클래스의 지도(map) 필드에 초기화된 지도 객체를 할당
  MapConverter::map = m;
}

// MapConverter 소멸자: 특별한 동작 없이 객체를 소멸
MapConverter::~MapConverter() {}

// 지도를 업데이트하는 함수
// vMap: voxel 벡터 (3D 공간 데이터를 담은 단위 요소)
// minMax: X, Y, Z의 최소/최대값으로 이루어진 범위 데이터
void MapConverter::updateMap(vector<voxel> vMap, vector<double> minMax) {
  // voxel 벡터가 비어 있으면 함수 종료
  if (vMap.size() == 0)
    return;
  
  // minMax 범위 내에 무한 값이 포함되어 있다면 유효하지 않은 입력으로 간주하고 종료
  for (double mM : minMax)
    if (isinf(mM))
      return;

  // 그리드의 셀 크기를 계산 (X와 Y 축에 대해)
  int xSize = (minMax[1] - minMax[0]) / MapConverter::map.getResulution();
  int ySize = (minMax[3] - minMax[2]) / MapConverter::map.getResulution();

  // HeightRangeMap 객체 생성 (로컬 맵)
  // 이 객체는 특정 위치에서의 높이 범위를 저장
  HeightRangeMap hMap(xSize, ySize);

  // voxel 데이터를 기반으로 로컬 맵 업데이트
  for (voxel v : vMap) {
    // voxel의 크기를 현재 해상도 단위로 변환
    int sizeIndex = v.halfSize * 2 / MapConverter::map.getResulution();
    for (int x = 0; x < sizeIndex; x++) {
      for (int y = 0; y < sizeIndex; y++) {
        // 현재 voxel이 차지하는 셀의 X 좌표 계산
        int posX = (v.position.x - v.halfSize + resolution * x - minMax[0]) /
                   resolution;
        // 현재 voxel이 차지하는 셀의 Y 좌표 계산
        int posY = (v.position.y - v.halfSize + resolution * y - minMax[2]) /
                   resolution;

        // 셀이 로컬 맵 범위를 벗어나면 건너뜀
        if (posX < 0 || posX >= xSize)
          continue;
        if (posY < 0 || posY >= ySize)
          continue;

        // 높이 범위를 약간 확장 (0.001 해상도 단위)
        // 중첩 감지를 더 정확하게 하기 위함
        heightRange hr = {v.position.z + v.halfSize + resolution * 0.001,
                          v.position.z - v.halfSize - resolution * 0.001};

        // 로컬 맵의 해당 셀에 높이 범위를 추가
        hMap.addRange(posX, posY, hr, v.occupied,
                      v.position.x - v.halfSize + resolution * x,
                      v.position.y - v.halfSize + resolution * y);
      }
    }
  }

  // 로봇 안전 기준보다 작은 빈 공간을 제거
  for (int x = 1; x < xSize - 1; x++) {
    for (int y = 1; y < ySize - 1; y++) {
      // 빈 공간 데이터에서 로봇 최소 높이 기준 이하의 범위를 제거
      hMap.removeFreeRanges(x, y, minZ);
      int mapValue = 0; // 기본값으로 빈 공간(-1이 아닌 상태) 설정

      // 해당 위치에 빈 공간이 없으면 위치와 값을 업데이트
      if (hMap.free[x][y].size() == 0) {
        hMap.posMap[x][y].x = x * resolution + minMax[0];
        hMap.posMap[x][y].y = y * resolution + minMax[2];
        mapValue = -1; // 빈 공간으로 설정
      }

      // 기존 지도에서 이미 빈 공간(-1)으로 설정된 경우 생략
      if (mapValue == -1 &&
          map.get(hMap.posMap[x][y].x, hMap.posMap[x][y].y) == -1)
        continue;

      // 빈 공간 값을 지도에 설정
      map.set(hMap.posMap[x][y].x, hMap.posMap[x][y].y, mapValue);

      // 빈 공간이 아닌 경우 높이 데이터를 지도에 추가
      if (mapValue != -1) {
        map.setHeight(hMap.posMap[x][y].x, hMap.posMap[x][y].y,
                      hMap.free[x][y].back().bottom); // 하단 높이
        map.setHeightTop(hMap.posMap[x][y].x, hMap.posMap[x][y].y,
                         hMap.free[x][y][0].top); // 상단 높이
      }
    }
  }

  // 인접 셀에서 점유 상태를 검사하여 빈 공간 보정
  for (int x = 2; x < xSize - 2; x++) {
    for (int y = 2; y < ySize - 2; y++) {
      // 현재 셀이 빈 공간이 아닌 경우 건너뜀
      if (hMap.free[x][y].size() == 0)
        continue;

      // 모든 방향(DIRECTIONS)으로 이웃 셀 검사
      for (auto d : DIRECTIONS) {
        // 이웃 셀이 빈 공간이 아니면 건너뜀
        if (hMap.free[x + d.x][y + d.y].size() != 0)
          continue;

        // 점유 상태를 지도에 설정
        map.set(hMap.posMap[x + d.x][y + d.y].x, hMap.posMap[x][y].y,
                hMap.getOccupation(x + d.x, y + d.y, -1, minOcc));
      }
    }
  }

  // 경사 정보를 지도에 업데이트
  map.updateSlope(slopeEstimationSize, minMax[0], minMax[1], minMax[2],
                  minMax[3]);
}


// voxel 리스트에서 최소 및 최대 좌표를 계산하는 함수
vector<double> MapConverter::minMaxVoxel(vector<voxel> list) {
  // X, Y, Z의 최소/최대값을 저장할 벡터를 생성하고 초기화
  vector<double> minMax(6);
  minMax[0] = INFINITY;    // X의 최소값 초기화
  minMax[1] = -INFINITY;   // X의 최대값 초기화
  minMax[2] = INFINITY;    // Y의 최소값 초기화
  minMax[3] = -INFINITY;   // Y의 최대값 초기화
  minMax[4] = INFINITY;    // Z의 최소값 초기화
  minMax[5] = -INFINITY;   // Z의 최대값 초기화

  // 주어진 voxel 리스트를 반복
  for (voxel v : list) {
    // 새로운 영역의 신뢰할 수 있는 크기를 추정하기 위해
    // 옥트리에서 가장 작은 voxel만 사용
    if (v.halfSize > resolution * .9)
      continue; // halfSize가 해상도의 0.9배보다 크면 무시

    // X의 최소값 갱신
    minMax[0] = min(minMax[0], v.position.x);
    // X의 최대값 갱신
    minMax[1] = max(minMax[1], v.position.x);
    // Y의 최소값 갱신
    minMax[2] = min(minMax[2], v.position.y);
    // Y의 최대값 갱신
    minMax[3] = max(minMax[3], v.position.y);
    // Z의 최소값 갱신
    minMax[4] = min(minMax[4], v.position.z);
    // Z의 최대값 갱신
    minMax[5] = max(minMax[5], v.position.z);
  }

  // 계산된 최소/최대값 배열 반환
  return minMax;
}


