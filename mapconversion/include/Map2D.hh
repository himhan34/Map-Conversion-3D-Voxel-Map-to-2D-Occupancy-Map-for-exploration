#pragma once
#include "datatypes.hh"

// 서브맵의 크기를 정의하는 상수
const int SUBMAP_SIZE = 128;

// 2D 맵 클래스를 정의
class Map2D {
public:
  // 생성자: 해상도 및 오프셋 초기화
  Map2D(double resolution = 0, double offsetX = 0, double offsetY = 0) {
    Map2D::resolution = resolution; // 맵의 해상도 설정
    setOffset(offsetX, offsetY); // 오프셋 값 설정
    mapSizeX = 0; // 맵의 X 방향 크기 초기화
    mapSizeY = 0; // 맵의 Y 방향 크기 초기화
    subMapOffsetX = 0; // 서브맵 X 방향 오프셋 초기화
    subMapOffsetY = 0; // 서브맵 Y 방향 오프셋 초기화
    growSubMaps(1, 1); // 서브맵 크기를 1x1로 초기 확장

    // 선형 회귀 행렬 계산을 위한 초기화
    MatrixXd A(9, 3);
    vector<point3D> points; // 3D 포인트 데이터 저장 벡터
    for (int dx = -1; dx <= 1; dx++) { // X 방향으로 -1에서 1까지 반복
      for (int dy = -1; dy <= 1; dy++) { // Y 방향으로 -1에서 1까지 반복
        point3D p = {double(dx), double(dy), 0}; // 포인트 생성
        points.push_back(p); // 포인트 벡터에 추가
      }
    }
    for (int i = 0; i < 9; ++i) { // 행렬 A를 포인트 값으로 초기화
      A(i, 0) = points[i].x;
      A(i, 1) = points[i].y;
      A(i, 2) = 1;
    }

    // 선형 회귀 계산에 필요한 행렬 초기화
    MatrixXd AtA = A.transpose() * A; // A의 전치 행렬과 곱셈
    MatrixXd AtA_inv = AtA.inverse(); // 역행렬 계산
    MatrixXd At = A.transpose(); // A의 전치 행렬
    preCalcA = AtA_inv * At; // 사전 계산된 행렬 저장
  }

  // 모든 점유 데이터를 가져오는 함수
  vector<int> get() {
    vector<int> newData; // 새 데이터를 저장할 벡터
    for (int y = 0; y < sizeY(); y++) { // Y 방향 크기만큼 반복
      for (int x = 0; x < sizeX(); x++) { // X 방향 크기만큼 반복
        newData.push_back(get(x, y)); // 각 위치의 점유 데이터를 추가
      }
    }
    return newData; // 점유 데이터를 반환
  }

  // 특정 기울기 기준으로 필터링된 점유 데이터를 가져오는 함수
  vector<int> get(double slope) {
    vector<int> newData; // 새 데이터를 저장할 벡터
    for (int y = 0; y < sizeY(); y++) { // Y 방향 크기만큼 반복
      for (int x = 0; x < sizeX(); x++) { // X 방향 크기만큼 반복
        double newSlope = getSlope(x, y); // 해당 위치의 기울기를 계산
        if (!isnan(newSlope) && newSlope > slope) { // 기울기가 유효하고 기준값 초과 시
          newData.push_back(100); // 값 100을 추가
        } else {
          newData.push_back(get(x, y)); // 기존 점유 데이터를 추가
        }
      }
    }
    return newData; // 필터링된 데이터를 반환
  }

  // X, Y 좌표에서 점유 데이터를 가져오는 함수
  int get(double x, double y) {
    int posX = (x - mapOffsetX) / resolution; // 좌표를 인덱스로 변환
    int posY = (y - mapOffsetY) / resolution; // 좌표를 인덱스로 변환
    return get(posX, posY); // 인덱스 기반 데이터를 반환
  }

  // X, Y 인덱스에서 점유 데이터를 가져오는 함수
  int get(int x, int y) {
    if (x < 0 || sizeX() <= x) // X 인덱스가 유효하지 않으면
      return -1; // -1 반환
    if (y < 0 || sizeY() <= y) // Y 인덱스가 유효하지 않으면
      return -1; // -1 반환
    x += subMapOffsetX; // 서브맵 오프셋을 추가
    y += subMapOffsetY; // 서브맵 오프셋을 추가
    int subIndexX = x / SUBMAP_SIZE; // 서브맵의 X 인덱스 계산
    int subIndexY = y / SUBMAP_SIZE; // 서브맵의 Y 인덱스 계산
    if (subMap[subIndexX][subIndexY] == nullptr) // 서브맵이 비어 있으면
      return -1; // -1 반환
    int indexX = x % SUBMAP_SIZE; // 서브맵 내부의 X 인덱스 계산
    int indexY = y % SUBMAP_SIZE; // 서브맵 내부의 Y 인덱스 계산
    return subMap[subIndexX][subIndexY][indexX][indexY]; // 점유 데이터 반환
  }

  // 특정 기울기 기준으로 필터링된 점유 데이터를 가져오는 함수
  int get(int x, int y, double maxSlope) {
    if (getSlope(x, y) > maxSlope) // 기울기가 기준값 초과 시
      return 100; // 값 100 반환
    return get(x, y); // 기존 점유 데이터 반환
  }

  // 모든 바닥 데이터를 가져오는 함수
  vector<double> getHeight() {
    vector<double> newData; // 새 데이터를 저장할 벡터
    for (int y = 0; y < sizeY(); y++) { // Y 방향 크기만큼 반복
      for (int x = 0; x < sizeX(); x++) { // X 방향 크기만큼 반복
        newData.push_back(getHeight(x, y)); // 각 위치의 바닥 데이터를 추가
      }
    }
    return newData; // 바닥 데이터를 반환
  }

  // 특정 위치의 바닥 데이터를 가져오는 함수
  double getHeight(int x, int y) {
    if (x < 0 || sizeX() <= x) // X 인덱스가 유효하지 않으면
      return NAN; // NAN 반환
    if (y < 0 || sizeY() <= y) // Y 인덱스가 유효하지 않으면
      return NAN; // NAN 반환
    x += subMapOffsetX; // 서브맵 오프셋 추가
    y += subMapOffsetY; // 서브맵 오프셋 추가
    int subIndexX = x / SUBMAP_SIZE; // 서브맵의 X 인덱스 계산
    int subIndexY = y / SUBMAP_SIZE; // 서브맵의 Y 인덱스 계산
    if (subMapHeight[subIndexX][subIndexY] == nullptr) // 서브맵이 비어 있으면
      return NAN; // NAN 반환
    int indexX = x % SUBMAP_SIZE; // 서브맵 내부의 X 인덱스 계산
    int indexY = y % SUBMAP_SIZE; // 서브맵 내부의 Y 인덱스 계산
    return subMapHeight[subIndexX][subIndexY][indexX][indexY]; // 바닥 데이터 반환
  }

  // 모든 천장 데이터를 가져오는 함수
  vector<double> getHeightTop() {
    vector<double> newData; // 새 데이터를 저장할 벡터
    for (int y = 0; y < sizeY(); y++) { // Y 방향 크기만큼 반복
      for (int x = 0; x < sizeX(); x++) { // X 방향 크기만큼 반복
        newData.push_back(getHeightTop(x, y)); // 각 위치의 천장 데이터를 추가
      }
    }
    return newData; // 천장 데이터를 반환
  }

  // 특정 위치의 천장 데이터를 가져오는 함수
  double getHeightTop(int x, int y) {
    if (x < 0 || sizeX() <= x) // X 인덱스가 유효하지 않으면
      return NAN; // NAN 반환
    if (y < 0 || sizeY() <= y) // Y 인덱스가 유효하지 않으면
      return NAN; // NAN 반환
    x += subMapOffsetX; // 서브맵 오프셋 추가
    y += subMapOffsetY; // 서브맵 오프셋 추가
    int subIndexX = x / SUBMAP_SIZE; // 서브맵의 X 인덱스 계산
    int subIndexY = y / SUBMAP_SIZE; // 서브맵의 Y 인덱스 계산
    if (subMapHeightTop[subIndexX][subIndexY] == nullptr) // 서브맵이 비어 있으면
      return NAN; // NAN 반환
    int indexX = x % SUBMAP_SIZE; // 서브맵 내부의 X 인덱스 계산
    int indexY = y % SUBMAP_SIZE; // 서브맵 내부의 Y 인덱스 계산
    return subMapHeightTop[subIndexX][subIndexY][indexX][indexY]; // 천장 데이터 반환
  }

  // 모든 기울기 데이터를 가져오는 함수
  vector<double> getSlope() {
    vector<double> newData; // 새 데이터를 저장할 벡터
    for (int y = 0; y < sizeY(); y++) { // Y 방향 크기만큼 반복
      for (int x = 0; x < sizeX(); x++) { // X 방향 크기만큼 반복
        newData.push_back(getSlope(x, y)); // 각 위치의 기울기 데이터를 추가
      }
    }
    return newData; // 기울기 데이터를 반환
  }

  // 특정 위치의 기울기 데이터를 가져오는 함수
  double getSlope(int x, int y) {
    if (x < 0 || sizeX() <= x) // X 인덱스가 범위를 벗어난 경우
      return NAN; // 유효하지 않은 값을 반환
    if (y < 0 || sizeY() <= y) // Y 인덱스가 범위를 벗어난 경우
      return NAN; // 유효하지 않은 값을 반환
    x += subMapOffsetX; // 서브맵 오프셋을 더함
    y += subMapOffsetY; // 서브맵 오프셋을 더함
    int subIndexX = x / SUBMAP_SIZE; // 서브맵의 X 인덱스 계산
    int subIndexY = y / SUBMAP_SIZE; // 서브맵의 Y 인덱스 계산
    if (subMapSlope[subIndexX][subIndexY] == nullptr) // 서브맵이 비어 있으면
      return NAN; // 유효하지 않은 값을 반환
    int indexX = x % SUBMAP_SIZE; // 서브맵 내부 X 인덱스 계산
    int indexY = y % SUBMAP_SIZE; // 서브맵 내부 Y 인덱스 계산
    return subMapSlope[subIndexX][subIndexY][indexX][indexY]; // 기울기 값 반환
  }

  // 맵의 너비를 가져오는 함수
  int sizeX() { return mapSizeX; }

  // 맵의 높이를 가져오는 함수
  int sizeY() { return mapSizeY; }

  // 프레임 원점으로부터의 X 방향 오프셋을 가져오는 함수
  double offsetX() { return mapOffsetX; }

  // 프레임 원점으로부터의 Y 방향 오프셋을 가져오는 함수
  double offsetY() { return mapOffsetY; }

  // 맵의 해상도를 가져오는 함수
  double getResulution() { return resolution; }

  // 특정 좌표에 점유 데이터를 설정하는 함수
  void set(double x, double y, int value) {
    int posX = (x - mapOffsetX) / resolution; // X 좌표를 인덱스로 변환
    int posY = (y - mapOffsetY) / resolution; // Y 좌표를 인덱스로 변환
    set(posX, posY, value); // 인덱스를 사용하여 값 설정
  }

  // 특정 인덱스에 점유 데이터를 설정하는 함수
  void set(int x, int y, int value) {
    int subIndexX, subIndexY, indexX, indexY;
    getIndexSubMap(x, y, subIndexX, subIndexY, indexX, indexY); // 서브맵 인덱스 계산
    if (subMap[subIndexX][subIndexY] == nullptr) // 서브맵이 비어 있다면
      subMap[subIndexX][subIndexY] = generateSubMapInt(); // 서브맵 생성
    subMap[subIndexX][subIndexY][indexX][indexY] = value; // 값 설정
  }

  // 특정 좌표에 바닥 높이 데이터를 설정하는 함수
  void setHeight(double x, double y, double value) {
    int posX = (x - mapOffsetX) / resolution; // X 좌표를 인덱스로 변환
    int posY = (y - mapOffsetY) / resolution; // Y 좌표를 인덱스로 변환
    setHeight(posX, posY, value); // 인덱스를 사용하여 값 설정
  }

  // 특정 인덱스에 바닥 높이 데이터를 설정하는 함수
  void setHeight(int x, int y, double value) {
    int subIndexX, subIndexY, indexX, indexY;
    getIndexSubMap(x, y, subIndexX, subIndexY, indexX, indexY); // 서브맵 인덱스 계산
    if (subMapHeight[subIndexX][subIndexY] == nullptr) // 서브맵이 비어 있다면
      subMapHeight[subIndexX][subIndexY] = generateSubMapDouble(); // 서브맵 생성
    subMapHeight[subIndexX][subIndexY][indexX][indexY] = value; // 값 설정
  }

  // 특정 좌표에 천장 높이 데이터를 설정하는 함수
  void setHeightTop(double x, double y, double value) {
    int posX = (x - mapOffsetX) / resolution; // X 좌표를 인덱스로 변환
    int posY = (y - mapOffsetY) / resolution; // Y 좌표를 인덱스로 변환
    setHeightTop(posX, posY, value); // 인덱스를 사용하여 값 설정
  }

  // 특정 인덱스에 천장 높이 데이터를 설정하는 함수
  void setHeightTop(int x, int y, double value) {
    int subIndexX, subIndexY, indexX, indexY;
    getIndexSubMap(x, y, subIndexX, subIndexY, indexX, indexY); // 서브맵 인덱스 계산
    if (subMapHeightTop[subIndexX][subIndexY] == nullptr) // 서브맵이 비어 있다면
      subMapHeightTop[subIndexX][subIndexY] = generateSubMapDouble(); // 서브맵 생성
    subMapHeightTop[subIndexX][subIndexY][indexX][indexY] = value; // 값 설정
  }

  // 특정 좌표에 기울기 데이터를 설정하는 함수
  void setSlope(double x, double y, double value) {
    int posX = (x - mapOffsetX) / resolution; // X 좌표를 인덱스로 변환
    int posY = (y - mapOffsetY) / resolution; // Y 좌표를 인덱스로 변환
    setSlope(posX, posY, value); // 인덱스를 사용하여 값 설정
  }

  // 특정 인덱스에 기울기 데이터를 설정하는 함수
  void setSlope(int x, int y, double value) {
    int subIndexX, subIndexY, indexX, indexY;
    getIndexSubMap(x, y, subIndexX, subIndexY, indexX, indexY); // 서브맵 인덱스 계산
    if (subMapSlope[subIndexX][subIndexY] == nullptr) // 서브맵이 비어 있다면
      subMapSlope[subIndexX][subIndexY] = generateSubMapDouble(); // 서브맵 생성
    subMapSlope[subIndexX][subIndexY][indexX][indexY] = value; // 값 설정
  }

  // 맵의 오프셋 값을 설정하는 함수
  void setOffset(double offsetX, double offsetY) {
    mapOffsetX = offsetX; // X 방향 오프셋 설정
    mapOffsetY = offsetY; // Y 방향 오프셋 설정
  }

  // 특정 영역의 기울기를 업데이트하는 함수
  void updateSlope(int areaSize, double minX, double maxX, double minY, double maxY) {
    int posMinX = (minX - mapOffsetX) / resolution; // 최소 X 좌표를 인덱스로 변환
    int posMaxX = (maxX - mapOffsetX) / resolution; // 최대 X 좌표를 인덱스로 변환
    int posMinY = (minY - mapOffsetY) / resolution; // 최소 Y 좌표를 인덱스로 변환
    int posMaxY = (maxY - mapOffsetY) / resolution; // 최대 Y 좌표를 인덱스로 변환
    updateSlope(areaSize, posMinX, posMaxX, posMinY, posMaxY); // 인덱스를 사용하여 기울기 업데이트
  }

  // 특정 영역의 기울기를 인덱스를 기준으로 업데이트하는 함수
  void updateSlope(int areaSize, int minX, int maxX, int minY, int maxY) {
    for (int x = minX + 1; x < maxX - 1; x++) { // X 방향 범위에서 반복
      for (int y = minY + 1; y < maxY - 1; y++) { // Y 방향 범위에서 반복
        if (isnan(getHeight(x, y))) // 바닥 높이가 유효하지 않다면
          continue; // 기울기 계산을 건너뜀
        vector<point3D> points; // 주변 점들을 저장할 벡터
        for (int dx = -areaSize; dx <= areaSize; dx++) { // X 방향 이웃 점 탐색
          for (int dy = -areaSize; dy <= areaSize; dy++) { // Y 방향 이웃 점 탐색
            if (isnan(getHeight(x + dx, y + dy))) // 이웃 점의 높이가 유효하지 않다면
              continue; // 건너뜀
            point3D p = {double(dx), double(dy), getHeight(x + dx, y + dy)}; // 3D 점 생성
            points.push_back(p); // 점을 벡터에 추가
          }
        }
        if (points.size() < 3) // 최소 3개의 점이 필요하지 않다면
          continue; // 기울기 계산 건너뜀
        point2D p = getSlopeOfPoints(points); // 점들로부터 기울기를 계산
        setSlope(x, y, sqrt(p.x * p.x + p.y * p.y)); // 기울기의 크기를 설정
      }
    }
  }

private:
// 2차원 벡터로 하위 맵의 데이터를 저장하는 변수
vector<vector<int **>> subMap; 
// 하위 맵의 높이 데이터를 저장하는 2차원 벡터
vector<vector<double **>> subMapHeight; 
// 하위 맵의 상단 높이 데이터를 저장하는 2차원 벡터
vector<vector<double **>> subMapHeightTop; 
// 하위 맵의 기울기 데이터를 저장하는 2차원 벡터
vector<vector<double **>> subMapSlope; 
// 하위 맵의 X축 오프셋 값
int subMapOffsetX; 
// 하위 맵의 Y축 오프셋 값
int subMapOffsetY; 
// 맵의 X축 크기
int mapSizeX, mapSizeY; 
// 맵의 X축과 Y축 오프셋
double mapOffsetX, mapOffsetY; 
// 맵의 해상도 값
double resolution; 
// 사전에 계산된 행렬 A를 저장하는 변수
MatrixXd preCalcA; 

// 주어진 점들의 기울기를 계산하여 반환하는 함수
point2D getSlopeOfPoints(vector<point3D> points) {
    // 점의 개수 저장
    int n = points.size(); 
    double a, b, c; 
    // 행렬 A와 벡터 B를 생성
    MatrixXd A(n, 3); 
    VectorXd B(n); 
    VectorXd x(3); 
    // 각 점의 z값을 벡터 B에 저장
    for (int i = 0; i < n; ++i) {
        B(i) = points[i].z; 
    }

    // 점의 개수가 9가 아니거나 다른 조건이 있을 경우
    if (true || n != 9) {
        for (int i = 0; i < n; ++i) {
            // A 행렬에 각 점의 x, y 값을 저장
            A(i, 0) = points[i].x; 
            A(i, 1) = points[i].y; 
            A(i, 2) = 1; 
        }
        // 최소 제곱법으로 계수를 계산
        x = (A.transpose() * A).inverse() * A.transpose() * B; 
    } else {
        // 사전 계산된 A 행렬을 사용해 계산
        x = preCalcA * B; 
    }
    // 평면 방정식의 계수를 저장
    a = x(0); 
    b = x(1); 
    c = x(2); 
    // 기울기 값 반환
    return {a, b}; 
}

// 주어진 x, y 좌표의 맵과 하위 맵 인덱스를 계산하는 함수
void getIndexSubMap(int x, int y, int &subIndexX, int &subIndexY, int &indexX,
                      int &indexY) {
    // 맵 범위 안에 있을 경우
    if (sizeX() > x && x >= 0 && sizeY() > y && y >= 0) {
        // 하위 맵의 오프셋을 추가
        x += subMapOffsetX; 
        y += subMapOffsetY; 
        // 하위 맵의 X, Y 인덱스를 계산
        subIndexX = x / SUBMAP_SIZE; 
        subIndexY = y / SUBMAP_SIZE; 
        // 하위 맵 내의 세부 인덱스를 계산
        indexX = x % SUBMAP_SIZE; 
        indexY = y % SUBMAP_SIZE; 
        return; 
    }
    // 맵의 X 범위를 초과했을 경우
    if (sizeX() <= x) {
        mapSizeX = x + 1; 
    } else if (x < 0) {
        // X 값이 0보다 작을 경우 맵을 확장하고 오프셋을 조정
        mapSizeX -= x; 
        mapOffsetX += x * resolution; 
        subMapOffsetX += x; 
        x = 0; 
    }
    // 맵의 Y 범위를 초과했을 경우
    if (sizeY() <= y) {
        mapSizeY = y + 1; 
    } else if (y < 0) {
        // Y 값이 0보다 작을 경우 맵을 확장하고 오프셋을 조정
        mapSizeY -= y; 
        mapOffsetY += y * resolution; 
        subMapOffsetY += y; 
        y = 0; 
    }
    // 하위 맵의 오프셋을 추가
    x += subMapOffsetX; 
    y += subMapOffsetY; 
    // 하위 맵의 X, Y 인덱스를 계산 (부호를 고려)
    subIndexX = x / SUBMAP_SIZE - signbit(x); 
    subIndexY = y / SUBMAP_SIZE - signbit(y); 

    // 맵 확장을 위한 변수 초기화
    int growX = 0, growY = 0; 
    if (subMap.size() <= subIndexX) {
        growX = subIndexX - subMap.size() + 1; 
    } else if (subIndexX < 0) {
        growX = subIndexX; 
    }
    if (subMap[0].size() <= subIndexY) {
        growY = subIndexY - subMap[0].size() + 1; 
    } else if (subIndexY < 0) {
        growY = subIndexY; 
    }
    // 확장이 필요할 경우
    if (growX != 0 || growY != 0) {
        growSubMaps(growX, growY); 
        if (subIndexX < 0) {
            // X 오프셋을 조정
            x = subMapOffsetX - 1; 
            subIndexX = x / SUBMAP_SIZE; 
        }
        if (subIndexY < 0) {
            // Y 오프셋을 조정
            y = subMapOffsetY - 1; 
            subIndexY = y / SUBMAP_SIZE; 
        }
    }
    // 하위 맵 내의 세부 인덱스를 계산
    indexX = x % SUBMAP_SIZE; 
    indexY = y % SUBMAP_SIZE; 
}

// 하위 맵을 확장하는 함수
void growSubMaps(int growX, int growY) {
    // 새로 생성할 하위 맵의 크기를 저장할 변수
    int newSizeX, newSizeY;
    if (subMap.size() == 0) {
        // 기존에 하위 맵이 없으면, 새 크기는 growX와 growY의 절댓값
        newSizeX = abs(growX); // growX의 절댓값을 X 크기로 설정
        newSizeY = abs(growY); // growY의 절댓값을 Y 크기로 설정
    } else {
        // 기존 하위 맵이 있을 경우 growX와 growY 값을 계산
        if (growX != 0)
            // growX가 양수인지 음수인지 판단 후, 기존 맵 크기와 비교해 최대값 사용
            growX = growX / abs(growX) * max(int(subMap.size()), abs(growX));
        if (growY != 0)
            // growY가 양수인지 음수인지 판단 후, 기존 맵 크기와 비교해 최대값 사용
            growY = growY / abs(growY) * max(int(subMap[0].size()), abs(growY));
        // 새 하위 맵 크기는 기존 크기에 growX, growY를 더한 값
        newSizeX = subMap.size() + abs(growX);
        newSizeY = subMap[0].size() + abs(growY);
    }
    // 새로운 하위 맵을 생성 (X축 크기를 기준으로 초기화)
    vector<vector<int **>> newSubMap(newSizeX); // 정수형 하위 맵
    vector<vector<double **>> newSubMapHeight(newSizeX); // 높이 정보 맵
    vector<vector<double **>> newSubMapHeightTop(newSizeX); // 상단 높이 정보 맵
    vector<vector<double **>> newSubMapSlope(newSizeX); // 기울기 정보 맵

    for (int i = 0; i < newSizeX; i++) {
        // 각 X축 벡터에 대해 Y축 크기를 설정하며 nullptr로 초기화
        newSubMap[i] = vector<int **>(newSizeY, nullptr);
        newSubMapHeight[i] = vector<double **>(newSizeY, nullptr);
        newSubMapHeightTop[i] = vector<double **>(newSizeY, nullptr);
        newSubMapSlope[i] = vector<double **>(newSizeY, nullptr);
    }

    // 기존 하위 맵이 음수 방향으로 확장될 경우의 오프셋 계산
    int offsetIndexX = growX < 0 ? -growX : 0; // 음수일 경우 X축 오프셋
    int offsetIndexY = growY < 0 ? -growY : 0; // 음수일 경우 Y축 오프셋

    // 기존 하위 맵 데이터를 새로운 하위 맵으로 복사
    for (int x = 0; x < subMap.size(); x++) {
        for (int y = 0; y < subMap[0].size(); y++) {
            // 새로운 하위 맵의 적절한 위치에 기존 데이터를 복사
            newSubMap[x + offsetIndexX][y + offsetIndexY] = subMap[x][y];
            newSubMapHeight[x + offsetIndexX][y + offsetIndexY] = subMapHeight[x][y];
            newSubMapHeightTop[x + offsetIndexX][y + offsetIndexY] = subMapHeightTop[x][y];
            newSubMapSlope[x + offsetIndexX][y + offsetIndexY] = subMapSlope[x][y];
        }
    }

    // 새로운 하위 맵에 맞게 오프셋 값 업데이트
    subMapOffsetX += offsetIndexX * SUBMAP_SIZE; // X축 오프셋 계산
    subMapOffsetY += offsetIndexY * SUBMAP_SIZE; // Y축 오프셋 계산

    // 새로운 하위 맵 데이터를 원래 변수에 저장
    subMap = newSubMap; // 정수형 하위 맵 갱신
    subMapHeight = newSubMapHeight; // 높이 맵 갱신
    subMapHeightTop = newSubMapHeightTop; // 상단 높이 맵 갱신
    subMapSlope = newSubMapSlope; // 기울기 맵 갱신
}

// 정수형 하위 맵을 생성하는 함수
int **generateSubMapInt() {
    // SUBMAP_SIZE 크기의 2차원 정수 배열 생성
    int **map = new int *[SUBMAP_SIZE];
    for (int x = 0; x < SUBMAP_SIZE; x++) {
        // 각 행에 대해 배열 생성
        map[x] = new int[SUBMAP_SIZE];
        for (int y = 0; y < SUBMAP_SIZE; y++) {
            // 모든 셀 값을 -1로 초기화
            map[x][y] = -1;
        }
    }
    return map; // 생성된 맵 반환
}

// 실수형 하위 맵을 생성하는 함수
double **generateSubMapDouble() {
    // SUBMAP_SIZE 크기의 2차원 실수 배열 생성
    double **map = new double *[SUBMAP_SIZE];
    for (int x = 0; x < SUBMAP_SIZE; x++) {
        // 각 행에 대해 배열 생성
        map[x] = new double[SUBMAP_SIZE];
        for (int y = 0; y < SUBMAP_SIZE; y++) {
            // 모든 셀 값을 NAN(Not a Number)으로 초기화
            map[x][y] = NAN;
        }
    }
    return map; // 생성된 맵 반환
}



