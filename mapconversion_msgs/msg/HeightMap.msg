std_msgs/Header header  // 메시지의 메타데이터를 포함하는 헤더 정보
nav_msgs/MapMetaData info  // 맵의 메타데이터, 해상도, 크기, 원점 위치 등의 정보를 포함
float32[] top  // 맵의 각 셀에 대한 상단(천장) 높이값을 저장하는 배열
float32[] bottom  // 맵의 각 셀에 대한 하단(바닥) 높이값을 저장하는 배열
