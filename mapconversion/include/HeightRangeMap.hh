#pragma once
#include "datatypes.hh"

// 8방향 이동을 나타내는 좌표 배열 정의
point_int DIRECTIONS[] = {{1, 0},  {1, 1},   {0, 1},  {-1, 1},
                          {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};

// HeightRangeMap 클래스 정의
class HeightRangeMap {
public:
  // 자유 영역을 저장하는 3D 벡터
  vector<vector<vector<heightRange>>> free;
  // 점유 영역을 저장하는 3D 벡터
  vector<vector<vector<heightRange>>> occupied;
  // 좌표 맵을 저장하는 2D 벡터
  vector<vector<point2D>> posMap;

  // 생성자: 너비와 높이를 초기화하고 크기를 설정
  HeightRangeMap(int width = 0, int height = 0) { setSize(width, height); }

  // 크기를 설정하고 height map 초기화
  void setSize(int width, int height) {
    free.resize(width); // free 벡터 크기 조정
    occupied.resize(width); // occupied 벡터 크기 조정
    posMap.resize(width); // posMap 벡터 크기 조정
    for (int i = 0; i < width; i++) {
      free[i].resize(height); // 각 x에 대해 y 크기 조정
      occupied[i].resize(height); // 각 x에 대해 y 크기 조정
      posMap[i].resize(height); // 각 x에 대해 y 크기 조정
    }
  }

  // 점유 및 자유 영역 범위를 추가, 중복되는 경우 병합
  void addRange(int x, int y, heightRange hr, bool isOccupied, double posX,
                double posY) {
    point2D p = {posX, posY}; // posMap에 저장할 좌표 생성
    posMap[x][y] = p; // 좌표 맵에 값 저장
    if (isOccupied) // 점유 여부에 따라 처리
      inserthghtRange(hr, &occupied[x][y]); // 점유 영역에 범위 추가
    else
      inserthghtRange(hr, &free[x][y]); // 자유 영역에 범위 추가
  }

  // 지정된 위치에서 minSize보다 작은 범위를 제거
  void removeFreeRanges(int x, int y, double minSize) {
    for (int i = 0; i < free[x][y].size(); i++) { // 각 범위에 대해 반복
      if (free[x][y][i].top - free[x][y][i].bottom > minSize) // 크기 확인
        continue;
      free[x][y].erase(free[x][y].begin() + i); // 범위 제거
      i--; // 인덱스 감소
    }
  }

  // 셀의 점유 수준을 계산하여 0-100 사이 값 반환
  int getOccupation(int x, int y, int index, int minOcc) {
    int occupation = -1; // 초기 점유 상태
    for (auto d : DIRECTIONS) { // 8방향으로 확인
      if (x + d.x < 0 || x + d.x >= free.size()) // 경계 확인
        continue;
      if (y + d.y < 0 || y + d.y >= free.back().size()) // 경계 확인
        continue;
      occupation =
          max(occupation, getOccupation(x, y, index, x + d.x, y + d.y, minOcc)); // 최대 점유율 계산
    }
    return occupation;
  }

  // 특정 셀의 점유 수준을 비교하여 계산
  int getOccupation(int x1, int y1, int index, int x2, int y2, int minOcc) {
    if (index < 0) // 음수 인덱스 처리
      index = free[x2][y2].size() + index;
    if (index < 0 || index >= free[x2][y2].size()) // 인덱스 유효성 확인
      return -1;
    heightRange freeHr = free[x2][y2][index]; // 자유 범위 가져오기
    vector<heightRange> occupiedHr = occupied[x1][y1]; // 점유 범위 가져오기

    return getOccupation(freeHr, occupiedHr, minOcc); // 점유율 계산
  }

  // 자유 범위와 점유 범위를 비교하여 점유율 계산
  int getOccupation(heightRange freeHr, vector<heightRange> occupiedHr,
                    int minOcc) {
    double overlapDistans = 0; // 겹치는 거리 초기화
    for (auto oHr : occupiedHr) { // 점유 범위 반복
      heightRange compare = {min(freeHr.top, oHr.top),
                             max(freeHr.bottom, oHr.bottom)}; // 범위 비교
      if (compare.top - compare.bottom < 0) // 겹치지 않으면 건너뜀
        continue;
      overlapDistans += compare.top - compare.bottom; // 겹치는 길이 계산
    }
    int occupation =
        int(overlapDistans / (freeHr.top - freeHr.bottom) * 100) * 2; // 점유율 계산
    if (occupation < minOcc) // 최소 점유율 확인
      return -1;
    return min(occupation, 100); // 최대 100 반환
  }

private:
  // heightRange를 리스트에 추가, 중첩된 경우 병합
  void inserthghtRange(heightRange hr, vector<heightRange> *list) {
    int overlapIndex = -1; // 중첩 인덱스 초기화
    for (int i = 0; i < list->size(); i++) { // 리스트에서 범위 반복
      if (hr.top < list->at(i).bottom) // 상단 비교
        continue;
      if (hr.bottom > list->at(i).top) // 하단 비교
        continue;
      overlapIndex = i; // 중첩 인덱스 설정
      break;
    }
    if (overlapIndex != -1) { // 중첩된 경우 병합 처리
      heightRange hrNew;
      heightRange hrOld = list->at(overlapIndex);
      list->erase(list->begin() + overlapIndex); // 기존 범위 제거
      hrNew.top = max(hrOld.top, hr.top); // 새로운 상단 설정
      hrNew.bottom = min(hrOld.bottom, hr.bottom); // 새로운 하단 설정
      inserthghtRange(hrNew, list); // 병합된 범위 다시 추가
      return;
    }
    for (int i = 0; i < list->size(); i++) { // 올바른 위치에 삽입
      if (hr.bottom <= list->at(i).top)
        continue;
      list->insert(list->begin() + i, hr); // 범위 삽입
      return;
    }
    list->push_back(hr); // 마지막에 추가
  }
};
