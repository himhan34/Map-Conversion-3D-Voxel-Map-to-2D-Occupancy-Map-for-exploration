#pragma once  // 헤더 파일 중복 포함 방지를 위한 전처리기 지시문
#include "datatypes.hh"  // 사용자 정의 데이터 타입을 포함한 헤더 파일 포함

class HeightRangeMap {  // 높이 범위 맵을 정의하는 클래스
public:
  vector<vector<vector<heightRange>>> free;  // 자유 공간의 높이 범위를 저장하는 3D 벡터
  vector<vector<vector<heightRange>>> occupied;  // 점유된 공간의 높이 범위를 저장하는 3D 벡터
  vector<vector<point2D>> posMap;  // 각 셀의 위치 정보를 저장하는 2D 벡터

  // 생성자: 기본 크기를 설정할 수 있는 생성자
  HeightRangeMap(int width = 0, int height = 0) { setSize(width, height); }

  // 크기를 업데이트하고 높이 맵 초기화
  void setSize(int width, int height) {
    free.resize(width);  // 자유 공간 벡터의 폭을 설정
    occupied.resize(width);  // 점유 공간 벡터의 폭을 설정
    posMap.resize(width);  // 위치 맵의 폭을 설정
    for (int i = 0; i < width; i++) {  // 폭에 해당하는 각 열을 순회
      free[i].resize(height);  // 자유 공간 벡터의 높이를 설정
      occupied[i].resize(height);  // 점유 공간 벡터의 높이를 설정
      posMap[i].resize(height);  // 위치 맵의 높이를 설정
    }
  }

  // 점유 및 자유 범위를 추가하며, 범위가 기존 범위와 겹치면 병합
  void addRange(int x, int y, heightRange hr, bool isOccupied, double posX,
                double posY) {
    point2D p = {posX, posY};  // 셀의 위치를 설정
    posMap[x][y] = p;  // 해당 셀에 위치 정보 저장
    if (isOccupied)  // 점유된 공간인 경우
      inserthghtRange(hr, &occupied[x][y]);  // 점유 공간 범위에 높이 범위 추가
    else  // 자유 공간인 경우
      inserthghtRange(hr, &free[x][y]);  // 자유 공간 범위에 높이 범위 추가
  }


  // x, y 위치에서 minSize보다 작은 모든 자유 공간 범위를 제거
  void removeFreeRanges(int x, int y, double minSize) {
    for (int i = 0; i < free[x][y].size(); i++) {  // 자유 공간 범위를 순회
      if (free[x][y][i].top - free[x][y][i].bottom > minSize)  // 범위가 minSize보다 크면 건너뜀
        continue;
      free[x][y].erase(free[x][y].begin() + i);  // 범위를 제거
      i--;  // 삭제 후 인덱스를 조정
    }
  }

  // 셀의 점유 수준을 반환 (0~100 범위), 자유 높이 범위와 비교
  int getOccupation(int x, int y, int index, int minOcc) {
    int occupation = -1;  // 초기 점유 수준 설정
    for (auto d : DIRECTIONS) {  // 모든 방향을 순회
      if (x + d.x < 0 || x + d.x >= free.size())  // x 범위 확인
        continue;
      if (y + d.y < 0 || y + d.y >= free.back().size())  // y 범위 확인
        continue;
      occupation =  // 최대 점유 수준을 계산
          max(occupation, getOccupation(x, y, index, x + d.x, y + d.y, minOcc));
    }
    return occupation;  // 계산된 점유 수준 반환
  }

  // x1, y1 위치의 셀 점유 수준을 x2, y2 위치의 자유 높이 범위와 비교하여 반환
  int getOccupation(int x1, int y1, int index, int x2, int y2, int minOcc) {
    if (index < 0)  // 인덱스가 음수이면 뒤에서부터 계산
      index = free[x2][y2].size() + index;
    if (index < 0 || index >= free[x2][y2].size())  // 유효하지 않은 인덱스이면 -1 반환
      return -1;
    heightRange freeHr = free[x2][y2][index];  // 자유 높이 범위를 가져옴
    vector<heightRange> occupiedHr = occupied[x1][y1];  // 점유 높이 범위를 가져옴

    return getOccupation(freeHr, occupiedHr, minOcc);  // 점유 수준 계산
  }

  // 자유 높이 범위와 점유 높이 범위를 비교하여 점유 수준 반환 (0~100 범위)
  int getOccupation(heightRange freeHr, vector<heightRange> occupiedHr,
                    int minOcc) {
    double overlapDistans = 0;  // 겹치는 거리 초기화
    for (auto oHr : occupiedHr) {  // 점유 범위를 순회
      heightRange compare = {min(freeHr.top, oHr.top),  // 겹치는 범위 계산
                             max(freeHr.bottom, oHr.bottom)};
      if (compare.top - compare.bottom < 0)  // 겹치는 부분이 없으면 건너뜀
        continue;
      overlapDistans += compare.top - compare.bottom;  // 겹치는 거리 누적
    }
    int occupation =
        int(overlapDistans / (freeHr.top - freeHr.bottom) * 100) * 2;  // 점유 수준 계산
    if (occupation < minOcc)  // 점유 수준이 최소 기준보다 낮으면 -1 반환
      return -1;
    return min(occupation, 100);  // 최대 100으로 제한하여 점유 수준 반환
  }

private:
  // 리스트에 heightRange를 삽입, 리스트 내의 heightRange가 겹치는 경우 병합
  void inserthghtRange(heightRange hr, vector<heightRange> *list) {
    int overlapIndex = -1;  // 겹치는 인덱스를 저장할 변수 초기화
    for (int i = 0; i < list->size(); i++) {  // 리스트 내의 모든 heightRange를 순회
      if (hr.top < list->at(i).bottom)  // 현재 heightRange가 비교 대상의 아래에 있으면 건너뜀
        continue;
      if (hr.bottom > list->at(i).top)  // 현재 heightRange가 비교 대상의 위에 있으면 건너뜀
        continue;
      overlapIndex = i;  // 겹치는 범위가 발견되면 인덱스를 저장
      break;  // 첫 번째 겹치는 범위만 확인하고 루프 종료
    }
    if (overlapIndex != -1) {  // 겹치는 범위가 있을 경우
      heightRange hrNew;  // 병합된 새로운 heightRange 생성
      heightRange hrOld = list->at(overlapIndex);  // 기존의 겹치는 heightRange 가져오기
      list->erase(list->begin() + overlapIndex);  // 기존 heightRange를 리스트에서 제거
      hrNew.top = max(hrOld.top, hr.top);  // 병합된 범위의 상단 값 설정
      hrNew.bottom = min(hrOld.bottom, hr.bottom);  // 병합된 범위의 하단 값 설정
      inserthghtRange(hrNew, list);  // 병합된 범위를 리스트에 다시 삽입
      return;  // 함수 종료
    }
    for (int i = 0; i < list->size(); i++) {  // 리스트 내의 범위를 순회
      if (hr.bottom <= list->at(i).top)  // 삽입 위치를 확인
        continue;
      list->insert(list->begin() + i, hr);  // 적절한 위치에 새로운 heightRange 삽입
      return;  // 함수 종료
    }
    list->push_back(hr);  // 적절한 위치가 없으면 리스트 끝에 추가
  }
};
