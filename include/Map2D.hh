#pragma once  // 헤더 파일의 중복 포함을 방지
#include "datatypes.hh"  // 데이터 타입 정의를 포함한 헤더 파일
const int SUBMAP_SIZE = 128;  // 서브맵의 크기를 128로 설정

class Map2D {  // 2D 맵을 정의하는 클래스
public:
  // 생성자: 맵 해상도, X 오프셋, Y 오프셋 초기화
  Map2D(double resolution = 0, double offsetX = 0, double offsetY = 0) {
    Map2D::resolution = resolution;  // 맵의 해상도 설정
    setOffset(offsetX, offsetY);  // 맵의 오프셋 설정
    mapSizeX = 0;  // 맵의 X 크기 초기화
    mapSizeY = 0;  // 맵의 Y 크기 초기화
    subMapOffsetX = 0;  // 서브맵의 X 오프셋 초기화
    subMapOffsetY = 0;  // 서브맵의 Y 오프셋 초기화
    growSubMaps(1, 1);  // 서브맵 크기를 초기화

    // 3x3 영역의 좌표 행렬 A를 초기화
    MatrixXd A(9, 3);
    vector<point3D> points;  // 3D 좌표를 저장하는 벡터
    for (int dx = -1; dx <= 1; dx++) {  // X 좌표를 -1에서 1까지 순회
      for (int dy = -1; dy <= 1; dy++) {  // Y 좌표를 -1에서 1까지 순회
        point3D p = {double(dx), double(dy), 0};  // 좌표 생성
        points.push_back(p);  // 좌표를 벡터에 추가
      }
    }
    for (int i = 0; i < 9; ++i) {  // 행렬 A를 채움
      A(i, 0) = points[i].x;  // X 좌표 값 설정
      A(i, 1) = points[i].y;  // Y 좌표 값 설정
      A(i, 2) = 1;  // 상수값 설정
    }

    // 행렬 사전 계산
    MatrixXd AtA = A.transpose() * A;  // A^T * A 계산
    MatrixXd AtA_inv = AtA.inverse();  // A^T * A의 역행렬 계산
    MatrixXd At = A.transpose();  // A의 전치행렬 계산
    preCalcA = AtA_inv * At;  // 사전 계산된 행렬 저장
  }

  // 모든 점유 데이터를 가져옴
  vector<int> get() {
    vector<int> newData;  // 데이터를 저장할 벡터
    for (int y = 0; y < sizeY(); y++) {  // Y 방향으로 루프 실행
      for (int x = 0; x < sizeX(); x++) {  // X 방향으로 루프 실행
        newData.push_back(get(x, y));  // 각 셀의 점유 데이터를 벡터에 추가
      }
    }
    return newData;  // 점유 데이터 반환
  }

  // 경사 임계값을 기준으로 필터링된 모든 점유 데이터를 가져옴
  vector<int> get(double slope) {
    vector<int> newData;  // 데이터를 저장할 벡터
    for (int y = 0; y < sizeY(); y++) {  // Y 방향으로 루프 실행
      for (int x = 0; x < sizeX(); x++) {  // X 방향으로 루프 실행
        double newSlope = getSlope(x, y);  // 셀의 경사값 가져오기
        if (!isnan(newSlope) && newSlope > slope) {  // 경사값이 유효하고 임계값을 초과하면
          newData.push_back(100);  // 점유값을 100으로 설정
        } else {
          newData.push_back(get(x, y));  // 기존 점유값을 추가
        }
      }
    }
    return newData;  // 필터링된 점유 데이터 반환
  }

  // 특정 x, y 위치의 점유 데이터를 가져옴
  int get(double x, double y) {
    int posX = (x - mapOffsetX) / resolution;  // X 좌표를 셀 단위로 변환
    int posY = (y - mapOffsetY) / resolution;  // Y 좌표를 셀 단위로 변환
    return get(posX, posY);  // 인덱스를 기반으로 점유 데이터 반환
  }

  // 특정 x, y 인덱스의 점유 데이터를 가져옴
  int get(int x, int y) {
    if (x < 0 || sizeX() <= x)  // X 범위를 벗어난 경우
      return -1;  // 유효하지 않은 값 반환
    if (y < 0 || sizeY() <= y)  // Y 범위를 벗어난 경우
      return -1;  // 유효하지 않은 값 반환
    x += subMapOffsetX;  // X 좌표에 서브맵 오프셋 추가
    y += subMapOffsetY;  // Y 좌표에 서브맵 오프셋 추가
    int subIndexX = x / SUBMAP_SIZE;  // X 방향 서브맵 인덱스 계산
    int subIndexY = y / SUBMAP_SIZE;  // Y 방향 서브맵 인덱스 계산
    if (subMap[subIndexX][subIndexY] == nullptr)  // 서브맵이 비어 있는 경우
      return -1;  // 유효하지 않은 값 반환
    int indexX = x % SUBMAP_SIZE;  // 서브맵 내 X 인덱스 계산
    int indexY = y % SUBMAP_SIZE;  // 서브맵 내 Y 인덱스 계산
    return subMap[subIndexX][subIndexY][indexX][indexY];  // 점유 데이터 반환
  }

  // 경사 임계값을 기준으로 특정 x, y 인덱스의 점유 데이터를 가져옴
  int get(int x, int y, double maxSlope) {
    if (getSlope(x, y) > maxSlope)  // 경사값이 임계값을 초과하면
      return 100;  // 점유값을 100으로 반환
    return get(x, y);  // 기존 점유값 반환
  }





  // 모든 바닥 데이터를 가져옴
  vector<double> getHeight() {
    vector<double> newData;  // 바닥 데이터를 저장할 벡터
    for (int y = 0; y < sizeY(); y++) {  // Y 방향으로 루프 실행
      for (int x = 0; x < sizeX(); x++) {  // X 방향으로 루프 실행
        newData.push_back(getHeight(x, y));  // 각 셀의 바닥 데이터를 벡터에 추가
      }
    }
    return newData;  // 바닥 데이터 반환
  }

  // 특정 x, y 위치의 바닥 데이터를 가져옴
  double getHeight(int x, int y) {
    if (x < 0 || sizeX() <= x)  // X 좌표가 유효 범위를 벗어나면
      return NAN;  // 유효하지 않은 값 반환
    if (y < 0 || sizeY() <= y)  // Y 좌표가 유효 범위를 벗어나면
      return NAN;  // 유효하지 않은 값 반환
    x += subMapOffsetX;  // X 좌표에 서브맵 오프셋 추가
    y += subMapOffsetY;  // Y 좌표에 서브맵 오프셋 추가
    int subIndexX = x / SUBMAP_SIZE;  // X 방향 서브맵 인덱스 계산
    int subIndexY = y / SUBMAP_SIZE;  // Y 방향 서브맵 인덱스 계산
    if (subMapHeight[subIndexX][subIndexY] == nullptr)  // 서브맵이 비어 있으면
      return NAN;  // 유효하지 않은 값 반환
    int indexX = x % SUBMAP_SIZE;  // 서브맵 내 X 인덱스 계산
    int indexY = y % SUBMAP_SIZE;  // 서브맵 내 Y 인덱스 계산
    return subMapHeight[subIndexX][subIndexY][indexX][indexY];  // 바닥 데이터 반환
  }

  // 모든 천장 데이터를 가져옴
  vector<double> getHeightTop() {
    vector<double> newData;  // 천장 데이터를 저장할 벡터
    for (int y = 0; y < sizeY(); y++) {  // Y 방향으로 루프 실행
      for (int x = 0; x < sizeX(); x++) {  // X 방향으로 루프 실행
        newData.push_back(getHeightTop(x, y));  // 각 셀의 천장 데이터를 벡터에 추가
      }
    }
    return newData;  // 천장 데이터 반환
  }

  // 특정 x, y 위치의 천장 데이터를 가져옴
  double getHeightTop(int x, int y) {
    if (x < 0 || sizeX() <= x)  // X 좌표가 유효 범위를 벗어나면
      return NAN;  // 유효하지 않은 값 반환
    if (y < 0 || sizeY() <= y)  // Y 좌표가 유효 범위를 벗어나면
      return NAN;  // 유효하지 않은 값 반환
    x += subMapOffsetX;  // X 좌표에 서브맵 오프셋 추가
    y += subMapOffsetY;  // Y 좌표에 서브맵 오프셋 추가
    int subIndexX = x / SUBMAP_SIZE;  // X 방향 서브맵 인덱스 계산
    int subIndexY = y / SUBMAP_SIZE;  // Y 방향 서브맵 인덱스 계산
    if (subMapHeightTop[subIndexX][subIndexY] == nullptr)  // 서브맵이 비어 있으면
      return NAN;  // 유효하지 않은 값 반환
    int indexX = x % SUBMAP_SIZE;  // 서브맵 내 X 인덱스 계산
    int indexY = y % SUBMAP_SIZE;  // 서브맵 내 Y 인덱스 계산
    return subMapHeightTop[subIndexX][subIndexY][indexX][indexY];  // 천장 데이터 반환
  }

  // 모든 경사 데이터를 가져옴
  vector<double> getSlope() {
    vector<double> newData;  // 경사 데이터를 저장할 벡터
    for (int y = 0; y < sizeY(); y++) {  // Y 방향으로 루프 실행
      for (int x = 0; x < sizeX(); x++) {  // X 방향으로 루프 실행
        newData.push_back(getSlope(x, y));  // 각 셀의 경사 데이터를 벡터에 추가
      }
    }
    return newData;  // 경사 데이터 반환
  }

  // 특정 x, y 위치의 경사 데이터를 가져옴
  double getSlope(int x, int y) {
    if (x < 0 || sizeX() <= x)  // X 좌표가 유효 범위를 벗어나면
      return NAN;  // 유효하지 않은 값 반환
    if (y < 0 || sizeY() <= y)  // Y 좌표가 유효 범위를 벗어나면
      return NAN;  // 유효하지 않은 값 반환
    x += subMapOffsetX;  // X 좌표에 서브맵 오프셋 추가
    y += subMapOffsetY;  // Y 좌표에 서브맵 오프셋 추가
    int subIndexX = x / SUBMAP_SIZE;  // X 방향 서브맵 인덱스 계산
    int subIndexY = y / SUBMAP_SIZE;  // Y 방향 서브맵 인덱스 계산
    if (subMapSlope[subIndexX][subIndexY] == nullptr)  // 서브맵이 비어 있으면
      return NAN;  // 유효하지 않은 값 반환
    int indexX = x % SUBMAP_SIZE;  // 서브맵 내 X 인덱스 계산
    int indexY = y % SUBMAP_SIZE;  // 서브맵 내 Y 인덱스 계산
    return subMapSlope[subIndexX][subIndexY][indexX][indexY];  // 경사 데이터 반환
  }

  // 맵의 너비를 반환
  int sizeX() { return mapSizeX; }

  // 맵의 높이를 반환
  int sizeY() { return mapSizeY; }

  // 원점으로부터 X 오프셋을 반환
  double offsetX() { return mapOffsetX; }

  // 원점으로부터 Y 오프셋을 반환
  double offsetY() { return mapOffsetY; }

  // 맵의 해상도를 반환
  double getResulution() { return resolution; }

  // 특정 x, y 위치에서 점유 값을 설정
  void set(double x, double y, int value) {
    int posX = (x - mapOffsetX) / resolution;  // X 좌표를 셀 단위로 변환
    int posY = (y - mapOffsetY) / resolution;  // Y 좌표를 셀 단위로 변환
    set(posX, posY, value);  // 인덱스를 기반으로 점유 값 설정
  }

  // 특정 x, y 인덱스에서 점유 값을 설정
  void set(int x, int y, int value) {
    int subIndexX, subIndexY, indexX, indexY;
    getIndexSubMap(x, y, subIndexX, subIndexY, indexX, indexY);  // 서브맵과 셀 인덱스 계산
    if (subMap[subIndexX][subIndexY] == nullptr)  // 서브맵이 비어 있는 경우 초기화
      subMap[subIndexX][subIndexY] = generateSubMapInt();
    subMap[subIndexX][subIndexY][indexX][indexY] = value;  // 점유 값 설정
  }

  // 특정 x, y 위치에서 바닥 높이를 설정
  void setHeight(double x, double y, double value) {
    int posX = (x - mapOffsetX) / resolution;  // X 좌표를 셀 단위로 변환
    int posY = (y - mapOffsetY) / resolution;  // Y 좌표를 셀 단위로 변환
    setHeight(posX, posY, value);  // 인덱스를 기반으로 바닥 높이 설정
  }

  // 특정 x, y 인덱스에서 바닥 높이를 설정
  void setHeight(int x, int y, double value) {
    int subIndexX, subIndexY, indexX, indexY;
    getIndexSubMap(x, y, subIndexX, subIndexY, indexX, indexY);  // 서브맵과 셀 인덱스 계산
    if (subMapHeight[subIndexX][subIndexY] == nullptr)  // 서브맵이 비어 있는 경우 초기화
      subMapHeight[subIndexX][subIndexY] = generateSubMapDouble();
    subMapHeight[subIndexX][subIndexY][indexX][indexY] = value;  // 바닥 높이 설정
  }

  // 특정 x, y 위치에서 천장 높이를 설정
  void setHeightTop(double x, double y, double value) {
    int posX = (x - mapOffsetX) / resolution;  // X 좌표를 셀 단위로 변환
    int posY = (y - mapOffsetY) / resolution;  // Y 좌표를 셀 단위로 변환
    setHeightTop(posX, posY, value);  // 인덱스를 기반으로 천장 높이 설정
  }

  // 특정 x, y 인덱스에서 천장 높이를 설정
  void setHeightTop(int x, int y, double value) {
    int subIndexX, subIndexY, indexX, indexY;
    getIndexSubMap(x, y, subIndexX, subIndexY, indexX, indexY);  // 서브맵과 셀 인덱스 계산
    if (subMapHeightTop[subIndexX][subIndexY] == nullptr)  // 서브맵이 비어 있는 경우 초기화
      subMapHeightTop[subIndexX][subIndexY] = generateSubMapDouble();
    subMapHeightTop[subIndexX][subIndexY][indexX][indexY] = value;  // 천장 높이 설정
  }

  // 특정 x, y 위치에서 경사 값을 설정
  void setSlope(double x, double y, double value) {
    int posX = (x - mapOffsetX) / resolution;  // X 좌표를 셀 단위로 변환
    int posY = (y - mapOffsetY) / resolution;  // Y 좌표를 셀 단위로 변환
    setSlope(posX, posY, value);  // 인덱스를 기반으로 경사 값 설정
  }

  // 특정 x, y 인덱스에서 경사 값을 설정
  void setSlope(int x, int y, double value) {
    int subIndexX, subIndexY, indexX, indexY;
    getIndexSubMap(x, y, subIndexX, subIndexY, indexX, indexY);  // 서브맵과 셀 인덱스 계산
    if (subMapSlope[subIndexX][subIndexY] == nullptr)  // 서브맵이 비어 있는 경우 초기화
      subMapSlope[subIndexX][subIndexY] = generateSubMapDouble();
    subMapSlope[subIndexX][subIndexY][indexX][indexY] = value;  // 경사 값 설정
  }

  // 맵의 X, Y 오프셋을 설정
  void setOffset(double offsetX, double offsetY) {
    mapOffsetX = offsetX;  // X 오프셋 설정
    mapOffsetY = offsetY;  // Y 오프셋 설정
  }
  // 맵의 지정된 영역에서 경사를 업데이트
  void updateSlope(int areaSize, double minX, double maxX, double minY,
                   double maxY) {
    int posMinX = (minX - mapOffsetX) / resolution;  // 최소 X 좌표를 셀 단위로 변환
    int posMaxX = (maxX - mapOffsetX) / resolution;  // 최대 X 좌표를 셀 단위로 변환
    int posMinY = (minY - mapOffsetY) / resolution;  // 최소 Y 좌표를 셀 단위로 변환
    int posMaxY = (maxY - mapOffsetY) / resolution;  // 최대 Y 좌표를 셀 단위로 변환
    updateSlope(areaSize, posMinX, posMaxX, posMinY, posMaxY);  // 셀 단위로 업데이트 수행
  }

  // 맵의 지정된 셀 범위에서 경사를 업데이트
  void updateSlope(int areaSize, int minX, int maxX, int minY, int maxY) {
    for (int x = minX + 1; x < maxX - 1; x++) {  // X 좌표를 루프
      for (int y = minY + 1; y < maxY - 1; y++) {  // Y 좌표를 루프
        if (isnan(getHeight(x, y)))  // 해당 셀이 비어 있다면 건너뜀
          continue;
        vector<point3D> points;  // 경사 계산에 사용할 포인트 리스트 생성
        for (int dx = -areaSize; dx <= areaSize; dx++) {  // X 방향으로 주변 영역 확인
          for (int dy = -areaSize; dy <= areaSize; dy++) {  // Y 방향으로 주변 영역 확인
            if (isnan(getHeight(x + dx, y + dy)))  // 비어 있는 셀은 건너뜀
              continue;
            point3D p = {double(dx), double(dy), getHeight(x + dx, y + dy)};  // 3D 점 생성
            points.push_back(p);  // 점을 리스트에 추가
          }
        }
        if (points.size() < 3)  // 최소한 3개의 점이 없으면 건너뜀 (MS 문제 해결을 위해 필요)
          continue;
        point2D p = getSlopeOfPoints(points);  // 포인트 리스트를 기반으로 경사 계산
        setSlope(x, y, sqrt(p.x * p.x + p.y * p.y));  // 경사값을 설정 (크기 계산)
      }
    }
  }
private:
  vector<vector<int **>> subMap;  // 점유 상태를 저장하는 서브맵
  vector<vector<double **>> subMapHeight;  // 바닥 높이를 저장하는 서브맵
  vector<vector<double **>> subMapHeightTop;  // 천장 높이를 저장하는 서브맵
  vector<vector<double **>> subMapSlope;  // 경사값을 저장하는 서브맵
  int subMapOffsetX;  // 서브맵의 X 방향 오프셋
  int subMapOffsetY;  // 서브맵의 Y 방향 오프셋
  int mapSizeX, mapSizeY;  // 맵의 X와 Y 크기
  double mapOffsetX, mapOffsetY;  // 맵의 X와 Y 오프셋
  double resolution;  // 맵의 해상도
  MatrixXd preCalcA;  // 사전 계산된 행렬

  // 점들을 평면에 맞추고 평면의 경사를 계산하여 최대 경사를 반환
  point2D getSlopeOfPoints(vector<point3D> points) {
    int n = points.size();  // 점의 개수
    double a, b, c;  // 평면 방정식의 계수
    // 행렬 A와 벡터 B 생성
    MatrixXd A(n, 3);  // n x 3 행렬
    VectorXd B(n);  // n 크기의 벡터
    VectorXd x(3);  // 결과 벡터

    for (int i = 0; i < n; ++i) {
      B(i) = points[i].z;  // 벡터 B에 각 점의 z 좌표 설정
    }

    if (true || n != 9) {  // 점의 개수가 9가 아니거나 기본 경로로 계산
      for (int i = 0; i < n; ++i) {
        A(i, 0) = points[i].x;  // 행렬 A의 첫 번째 열에 x 좌표 설정
        A(i, 1) = points[i].y;  // 행렬 A의 두 번째 열에 y 좌표 설정
        A(i, 2) = 1;  // 행렬 A의 세 번째 열에 상수값 설정
      }
      x = (A.transpose() * A).inverse() * A.transpose() * B;  // 선형 시스템 해결
    } else {
      x = preCalcA * B;  // 사전 계산된 행렬을 이용해 해결
    }

    // 평면 방정식 계수 추출
    a = x(0);  // x에 대한 계수
    b = x(1);  // y에 대한 계수
    c = x(2);  // 상수항
    return {a, b};  // x와 y 방향의 경사 반환
  }

  // 맵과 서브맵의 인덱스를 계산, x, y 인덱스가 맵 바깥에 있으면 맵을 확장
  void getIndexSubMap(int x, int y, int &subIndexX, int &subIndexY, int &indexX,
                      int &indexY) {
    // x, y가 맵 크기 내에 있는 경우
    if (sizeX() > x && x >= 0 && sizeY() > y && y >= 0) {
      x += subMapOffsetX;  // X 좌표에 서브맵 오프셋 추가
      y += subMapOffsetY;  // Y 좌표에 서브맵 오프셋 추가
      subIndexX = x / SUBMAP_SIZE;  // X 방향 서브맵 인덱스 계산
      subIndexY = y / SUBMAP_SIZE;  // Y 방향 서브맵 인덱스 계산
      indexX = x % SUBMAP_SIZE;  // 서브맵 내 X 인덱스 계산
      indexY = y % SUBMAP_SIZE;  // 서브맵 내 Y 인덱스 계산
      return;  // 함수 종료
    }

    // x가 맵 크기를 초과하면 맵 크기 확장
    if (sizeX() <= x) {
      mapSizeX = x + 1;  // 맵의 X 크기를 x+1로 설정
    } else if (x < 0) {  // x가 음수이면
      mapSizeX -= x;  // 맵 크기를 감소
      mapOffsetX += x * resolution;  // 맵 X 오프셋 조정
      subMapOffsetX += x;  // 서브맵 X 오프셋 조정
      x = 0;  // x를 0으로 설정
    }

    // y가 맵 크기를 초과하면 맵 크기 확장
    if (sizeY() <= y) {
      mapSizeY = y + 1;  // 맵의 Y 크기를 y+1로 설정
    } else if (y < 0) {  // y가 음수이면
      mapSizeY -= y;  // 맵 크기를 감소
      mapOffsetY += y * resolution;  // 맵 Y 오프셋 조정
      subMapOffsetY += y;  // 서브맵 Y 오프셋 조정
      y = 0;  // y를 0으로 설정
    }

    x += subMapOffsetX;  // X 좌표에 서브맵 오프셋 추가
    y += subMapOffsetY;  // Y 좌표에 서브맵 오프셋 추가
    subIndexX = x / SUBMAP_SIZE - signbit(x);  // X 방향 서브맵 인덱스 계산
    subIndexY = y / SUBMAP_SIZE - signbit(y);  // Y 방향 서브맵 인덱스 계산

    int growX = 0, growY = 0;  // 맵 확장 크기 초기화
    if (subMap.size() <= subIndexX) {  // 서브맵 X 크기가 부족하면
      growX = subIndexX - subMap.size() + 1;  // 확장할 X 크기 계산
    } else if (subIndexX < 0) {  // X 서브맵 인덱스가 음수이면
      growX = subIndexX;  // 음수 크기만큼 확장
    }
    if (subMap[0].size() <= subIndexY) {  // 서브맵 Y 크기가 부족하면
      growY = subIndexY - subMap[0].size() + 1;  // 확장할 Y 크기 계산
    } else if (subIndexY < 0) {  // Y 서브맵 인덱스가 음수이면
      growY = subIndexY;  // 음수 크기만큼 확장
    }

    if (growX != 0 || growY != 0) {  // 확장할 크기가 있다면
      growSubMaps(growX, growY);  // 맵 확장 수행
      if (subIndexX < 0) {  // X 서브맵 인덱스가 음수인 경우
        x = subMapOffsetX - 1;  // X 좌표를 서브맵 오프셋 -1로 설정
        subIndexX = x / SUBMAP_SIZE;  // X 서브맵 인덱스 재계산
      }
      if (subIndexY < 0) {  // Y 서브맵 인덱스가 음수인 경우
        y = subMapOffsetY - 1;  // Y 좌표를 서브맵 오프셋 -1로 설정
        subIndexY = y / SUBMAP_SIZE;  // Y 서브맵 인덱스 재계산
      }
    }

    indexX = x % SUBMAP_SIZE;  // 서브맵 내 X 인덱스 계산
    indexY = y % SUBMAP_SIZE;  // 서브맵 내 Y 인덱스 계산
  }


  // 서브맵을 확장
  void growSubMaps(int growX, int growY) {
    // 새로운 서브맵 크기 계산
    int newSizeX, newSizeY;
    if (subMap.size() == 0) {  // 기존 서브맵이 없는 경우
      newSizeX = abs(growX);  // 확장 크기를 절대값으로 설정
      newSizeY = abs(growY);
    } else {  // 기존 서브맵이 있는 경우
      if (growX != 0)
        growX = growX / abs(growX) * max(int(subMap.size()), abs(growX));  // 확장 방향과 크기 설정
      if (growY != 0)
        growY = growY / abs(growY) * max(int(subMap[0].size()), abs(growY));
      newSizeX = subMap.size() + abs(growX);  // 새로운 서브맵 X 크기 계산
      newSizeY = subMap[0].size() + abs(growY);  // 새로운 서브맵 Y 크기 계산
    }

    // 새로운 서브맵 생성
    vector<vector<int **>> newSubMap(newSizeX);  // 새로운 점유 상태 서브맵 생성
    vector<vector<double **>> newSubMapHeight(newSizeX);  // 새로운 바닥 높이 서브맵 생성
    vector<vector<double **>> newSubMapHeightTop(newSizeX);  // 새로운 천장 높이 서브맵 생성
    vector<vector<double **>> newSubMapSlope(newSizeX);  // 새로운 경사 서브맵 생성
    for (int i = 0; i < newSizeX; i++) {  // X 방향으로 서브맵 초기화
      newSubMap[i] = vector<int **>(newSizeY, nullptr);  // 점유 상태 서브맵 초기화
      newSubMapHeight[i] = vector<double **>(newSizeY, nullptr);  // 바닥 높이 서브맵 초기화
      newSubMapHeightTop[i] = vector<double **>(newSizeY, nullptr);  // 천장 높이 서브맵 초기화
      newSubMapSlope[i] = vector<double **>(newSizeY, nullptr);  // 경사 서브맵 초기화
    }

    // 기존 서브맵 데이터를 새로운 서브맵에 복사
    int offsetIndexX = growX < 0 ? -growX : 0;  // X 방향 오프셋 계산
    int offsetIndexY = growY < 0 ? -growY : 0;  // Y 방향 오프셋 계산
    for (int x = 0; x < subMap.size(); x++) {  // 기존 서브맵 데이터 복사
      for (int y = 0; y < subMap[0].size(); y++) {
        newSubMap[x + offsetIndexX][y + offsetIndexY] = subMap[x][y];  // 점유 상태 복사
        newSubMapHeight[x + offsetIndexX][y + offsetIndexY] = subMapHeight[x][y];  // 바닥 높이 복사
        newSubMapHeightTop[x + offsetIndexX][y + offsetIndexY] = subMapHeightTop[x][y];  // 천장 높이 복사
        newSubMapSlope[x + offsetIndexX][y + offsetIndexY] = subMapSlope[x][y];  // 경사 데이터 복사
      }
    }

    // 오프셋 업데이트
    subMapOffsetX += offsetIndexX * SUBMAP_SIZE;  // X 방향 서브맵 오프셋 갱신
    subMapOffsetY += offsetIndexY * SUBMAP_SIZE;  // Y 방향 서브맵 오프셋 갱신
    // 새로운 서브맵으로 교체
    subMap = newSubMap;
    subMapHeight = newSubMapHeight;
    subMapHeightTop = newSubMapHeightTop;
    subMapSlope = newSubMapSlope;
  }

  // 점유 상태를 위한 서브맵 생성
  int **generateSubMapInt() {
    int **map = new int *[SUBMAP_SIZE];  // SUBMAP_SIZE 크기의 배열 생성
    for (int x = 0; x < SUBMAP_SIZE; x++) {  // X 방향 초기화
      map[x] = new int[SUBMAP_SIZE];  // Y 방향 배열 생성
      for (int y = 0; y < SUBMAP_SIZE; y++) {  // Y 방향 초기화
        map[x][y] = -1;  // 점유 상태 초기값 설정 (-1)
      }
    }
    return map;  // 생성된 서브맵 반환
  }

  // 바닥, 천장 높이 및 경사를 위한 서브맵 생성
  double **generateSubMapDouble() {
    double **map = new double *[SUBMAP_SIZE];  // SUBMAP_SIZE 크기의 배열 생성
    for (int x = 0; x < SUBMAP_SIZE; x++) {  // X 방향 초기화
      map[x] = new double[SUBMAP_SIZE];  // Y 방향 배열 생성
      for (int y = 0; y < SUBMAP_SIZE; y++) {  // Y 방향 초기화
        map[x][y] = NAN;  // 초기값 설정 (NAN)
      }
    }
    return map;  // 생성된 서브맵 반환
  }
};
