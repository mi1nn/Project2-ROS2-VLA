# 검출된 물체 하나. 좌표계는 카메라 기준, 단위 mm.
string     class_name      # class_names.json 의 이름. 예: "cup_ramen"
float32    score           # 0.0 ~ 1.0
float64[3] camera_xyz      # 카메라 좌표계 (x, y, z), mm
float64[]  masking_map     # polygon 마스킹맵
int32[2]   centroid_px     # seg mask 무게중심 픽셀 (x, y). 로깅/디버깅용