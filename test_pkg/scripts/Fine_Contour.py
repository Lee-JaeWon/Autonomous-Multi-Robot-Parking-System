import numpy as np
import cv2
import matplotlib.pyplot as plt
MIN_AREA = 80
MAX_AREA = 300
# MIN_WIDTH, MIN_HEIGHT = 2, 8
MIN_WIDTH, MIN_HEIGHT = 10, 40

MIN_RATIO, MAX_RATIO = 0.1, 1.0




MAX_DIAG_MULTIPLYER = 10 # 5 , cx 거리가 윤곽선 대각선 길이의 5배 안으로 있을 때
MAX_ANGLE_DIFF = 10.0 # 12.0 # cx 아크탄젠트 각도 리미트
MAX_AREA_DIFF = 1.0# 0.5 면적 차이 
MAX_WIDTH_DIFF = 1.0 # 0.8 너비 차이
MAX_HEIGHT_DIFF = 0.5 # 0.2 높이 차이
MIN_N_MATCHED = 3 # 3 3개 이상 되어야 번호판 인정


def find_contour(contours,img):
    print("hello")
    height = img.shape[0]
    width = img.shape[1]
    contours_dict = []
    # 사각형 범위 추출
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        contours_dict.append({
            'contour': contour, # data type : list
            'x': x,
            'y': y,
            'w': w,
            'h': h,
            'cx': x + (w / 2), # center x
            'cy': y + (h / 2) # center y
        })
    possible_contours = []
    cnt = 0
    for d in contours_dict:
        area = d['w'] * d['h']
        ratio = d['w'] / d['h']
        # 넓이 , 비율 구하기
        if (area < MAX_AREA)and (area > MIN_AREA) and (d['w'] > MIN_WIDTH) and (d['h'] > MIN_HEIGHT) and (MIN_RATIO < ratio < MAX_RATIO):
            # 설정한 값보다 큰 경우에만 인덱스 값 저장한다.
            d['idx'] = cnt
            cnt += 1
            possible_contours.append(d)
            
    matched_result_idx = []
    for d1 in possible_contours:
        matched_contours_idx = []
        for d2 in possible_contours:
            if d1['idx'] == d2['idx']: # 같은 컨투어 일 때 continue
                continue
            
            dx = abs(d1['cx'] - d2['cx'])
            dy = abs(d1['cy'] - d2['cy'])
            # center distance difference
            
            diagonal_length1 = np.sqrt(d1['w'] ** 2 + d1['h'] ** 2)
            # 대각선 길이
            distance = np.linalg.norm(np.array([d1['cx'], d1['cy']]) - np.array([d2['cx'], d2['cy']]))
            # 벡터 a 와 b 사이의 거리를 구한다.
            
            if dx == 0: angle_diff = 90
            else: angle_diff = np.degrees(np.arctan(dy / dx)) # theta 구해주기
            
            area_diff_ratio = abs(d1['w'] * d1['h'] - d2['w'] * d2['h']) / (d1['w'] * d1['h'])
            
            width_diff_ratio = abs(d1['w'] - d2['w']) / d1['w']
            height_diff_ratio = abs(d1['h'] - d2['h']) / d1['h']

            if distance < diagonal_length1 * MAX_DIAG_MULTIPLYER \
            and angle_diff < MAX_ANGLE_DIFF and area_diff_ratio < MAX_AREA_DIFF \
            and width_diff_ratio < MAX_WIDTH_DIFF and height_diff_ratio < MAX_HEIGHT_DIFF:
                matched_contours_idx.append(d2['idx'])
        matched_contours_idx.append(d1['idx'])

        if len(matched_contours_idx) < MIN_N_MATCHED: continue
        # 개수가 3개 보다 작으면 continue
        
        matched_result_idx.append(matched_contours_idx)

        return matched_result_idx
        matched_result = []
        for idx_list in matched_result_idx: 
            matched_result.append(np.take(possible_contours, idx_list))

        temp_result = np.zeros((height, width), dtype=np.uint8)
        for r in matched_result:
            for d in r:
        #         cv2.drawContours(temp_result, d['contour'], -1, (255, 255, 255))
                cv2.rectangle(temp_result, pt1=(d['x'], d['y']), pt2=(d['x']+d['w'], d['y']+d['h']), color=(255, 255, 255), thickness=1)
        plt.imshow(temp_result)
        plt.show()