#!/usr/bin/env python3


import cv2
import sys,os
import numpy as np
import matplotlib.pyplot as plt
import pytesseract
from cv_bridge import CvBridge

plt.style.use('dark_background')
sys.path.append(os.getcwd())
# img = cv2.imread('./images/car_plate_test5.jpg')
img = cv2.imread('/home/hyoseok/catkin_ws/src/test_pkg/images/TEST.jpg')

def Detection(data):
    k_cnt = 0
    n_cnt = 0
    for c in data:
        if ord('가') <= ord(c) <= ord('힣'):
            k_cnt += 1
        elif (c.isdigit()):
            n_cnt += 1

    if (k_cnt == 1 and n_cnt ==7):

        return True
    else:
        return False


def find_number(img_ori = img):
    # bridge = CvBridge()
    # # cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
    # img_ori = bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
    
    # img_ori = cv2.imread('/home/hyoseok/catkin_ws/src/test_pkg/images/3.png')
    height, width, channel = img_ori.shape
    gray = cv2.cvtColor(img_ori, cv2.COLOR_BGR2GRAY)
    # Convert Image to GrayScale

    structuringElement = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    imgTopHat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, structuringElement)
    imgBlackHat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, structuringElement)
    imgGrayscalePlusTopHat = cv2.add(gray, imgTopHat)
    gray = cv2.subtract(imgGrayscalePlusTopHat, imgBlackHat)
    # plt.imshow(gray)
    # plt.show()
    #노이즈 줄이기 위해
    # 이미지 구별하기 쉽게  (0, 255)

    img_blurred = cv2.GaussianBlur(gray, ksize=(5, 5), sigmaX=0)

    img_thresh = cv2.adaptiveThreshold(
        img_blurred, 
        maxValue=255.0, 
        adaptiveMethod=cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
        thresholdType=cv2.THRESH_BINARY_INV, 
        blockSize=19, 
        C=9
    )
    # 
    # plt.figure(figsize=(12, 10))
    # plt.imshow(img_thresh, cmap='gray')
    # plt.show()

    #윤곽선

    ## 윤곽선 찾기
    contours, _ = cv2.findContours(img_thresh, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)

    # temp_result = np.zeros((height, width, channel), dtype=np.uint8)
    # 전체 윤곽선 그리기
    # cv2.drawContours(temp_result, contours=contours, contourIdx=-1, color=(255, 255, 255))
    # plt.imshow(temp_result)
    # plt.show()





        
    # 컨투어의 사각형 범위 찾기

    temp_result = np.zeros((height, width, channel), dtype=np.uint8)

    contours_dict = []
    # 사각형 범위 추출
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(temp_result, pt1=(x, y), pt2=(x+w, y+h), color=(255, 255, 255), thickness=1)
        # 사각형 그리기
        # insert to dict
        contours_dict.append({
            'contour': contour, # data type : list
            'x': x,
            'y': y,
            'w': w,
            'h': h,
            'cx': x + (w / 2), # center x
            'cy': y + (h / 2) # center y
        })

    # plt.imshow(temp_result)
    # plt.show()



    # MIN_AREA = 100
    MIN_AREA = 50
    MAX_AREA = 500
    # MIN_WIDTH, MIN_HEIGHT = 2, 8
    # MIN_WIDTH, MIN_HEIGHT = 10, 40
    MIN_WIDTH, MIN_HEIGHT = 2, 10

    MIN_RATIO, MAX_RATIO = 0.1, 1.0
    # MIN_RATIO, MAX_RATIO = 0.1, 0.75


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
            
    # visualize possible contours
    temp_result = np.zeros((height, width, channel), dtype=np.uint8)

    for d in possible_contours:
    #     cv2.drawContours(temp_result, d['contour'], -1, (255, 255, 255))
        cv2.rectangle(temp_result, pt1=(d['x'], d['y']), pt2=(d['x']+d['w'], d['y']+d['h']), color=(255, 255, 255), thickness=1)

    # plt.imshow(temp_result)
    # plt.show()




    # Parameters
    MAX_DIAG_MULTIPLYER = 10 # 5 ,6  , cx 거리가 윤곽선 대각선 길이의 5배 안으로 있을 때
    MAX_ANGLE_DIFF = 20.0 # 12.0 , 45 # cx 아크탄젠트 각도 리미트
    MAX_AREA_DIFF = 1.0# 0.5 면적 차이 
    MAX_WIDTH_DIFF = 1.0 # 0.8 너비 차이
    MAX_HEIGHT_DIFF = 1.0 # 0.2 높이 차이
    MIN_N_MATCHED = 5 # 3 3개 이상 되어야 번호판 인정
    MIN_CENTER_DIFF = 20

    def find_chars(contour_list):
        matched_result_idx = []
        
        for d1 in contour_list:
            matched_contours_idx = []
            for d2 in contour_list:
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
                

                if (distance < diagonal_length1 * MAX_DIAG_MULTIPLYER) and (MIN_CENTER_DIFF < distance)  \
                and (angle_diff < MAX_ANGLE_DIFF) and (area_diff_ratio < MAX_AREA_DIFF) \
                and (width_diff_ratio < MAX_WIDTH_DIFF) and (height_diff_ratio < MAX_HEIGHT_DIFF):
                    matched_contours_idx.append(d2['idx'])

            # append 현재 contour
            matched_contours_idx.append(d1['idx'])

            if len(matched_contours_idx) < MIN_N_MATCHED: continue
            # 개수가 3개 보다 작으면 continue
            
            matched_result_idx.append(matched_contours_idx)

            unmatched_contour_idx = []
            for d4 in contour_list:
                # matched contoure에 속하지 않은 애들 한 번 더 비교
                if d4['idx'] not in matched_contours_idx:
                    unmatched_contour_idx.append(d4['idx'])
            unmatched_contour = np.take(possible_contours, unmatched_contour_idx)
            # np.take(a,idx) a에서 idx와 같은 인덱스의 값만 추출
            # 매치 안 된 컨투어 데이터 추출
            # recursive
            recursive_contour_list = find_chars(unmatched_contour)
            
            for idx in recursive_contour_list: matched_result_idx.append(idx)

            break

        return matched_result_idx
        
    result_idx = find_chars(possible_contours)

    matched_result = []
    for idx_list in result_idx: matched_result.append(np.take(possible_contours, idx_list))
    # 가능한 contour 에서 result_idx와 같은 데이터 추출

    # visualize possible contours
    temp_result = np.zeros((height, width, channel), dtype=np.uint8)


    for r in matched_result:
        for d in r:
    #         cv2.drawContours(temp_result, d['contour'], -1, (255, 255, 255))
            cv2.rectangle(temp_result, pt1=(d['x'], d['y']), pt2=(d['x']+d['w'], d['y']+d['h']), color=(255, 255, 255), thickness=1)
            cv2.rectangle(img_ori, pt1=(d['x'], d['y']), pt2=(d['x']+d['w'], d['y']+d['h']), color=(0, 255, 0), thickness=1)
            
#####
    # plt.imshow(temp_result)
    # plt.imshow(img_ori)
    # plt.show()


    ############################################################################################################################################

    #똑바로 돌리기

    PLATE_WIDTH_PADDING = 1.5 # 1.3
    PLATE_HEIGHT_PADDING = 3.0 # 1.5
    MIN_PLATE_RATIO = 3  
    MAX_PLATE_RATIO = 10

    plate_imgs = []
    plate_infos = []

    for i, matched_chars in enumerate(matched_result):
        sorted_chars = sorted(matched_chars, key=lambda x: x['cx'])
        # cx 순으로 정렬
        plate_cx = (sorted_chars[0]['cx'] + sorted_chars[-1]['cx']) / 2
        plate_cy = (sorted_chars[0]['cy'] + sorted_chars[-1]['cy']) / 2
        # 번호판 중심 좌표
        plate_width = (sorted_chars[-1]['x'] + sorted_chars[-1]['w'] - sorted_chars[0]['x']) * PLATE_WIDTH_PADDING
        
        sum_height = 0
        for d in sorted_chars:
            sum_height += d['h']

        plate_height = int(sum_height / len(sorted_chars) * PLATE_HEIGHT_PADDING)
        
        triangle_height = sorted_chars[-1]['cy'] - sorted_chars[0]['cy']
        triangle_hypotenus = np.linalg.norm(
            np.array([sorted_chars[0]['cx'], sorted_chars[0]['cy']]) - 
            np.array([sorted_chars[-1]['cx'], sorted_chars[-1]['cy']]))
        # 빗변 거리 구하기
        
        angle = np.degrees(np.arcsin(triangle_height / triangle_hypotenus))
        # 각도 구하기

        rotation_matrix = cv2.getRotationMatrix2D(center=(plate_cx, plate_cy), angle=angle, scale=1.0)

        img_rotated = cv2.warpAffine(img_thresh, M=rotation_matrix, dsize=(width, height))
        # print("회전한 이미지")
        # plt.imshow(img_rotated)
        # plt.show()
        

        img_cropped = cv2.getRectSubPix(
            img_rotated, 
            patchSize=(int(plate_width), int(plate_height)), 
            center=(int(plate_cx), int(plate_cy))
        )
##
        # plt.imshow(img_cropped,cmap='gray')
        # plt.show()
        
        if img_cropped.shape[1] / img_cropped.shape[0] < MIN_PLATE_RATIO or img_cropped.shape[1] / img_cropped.shape[0] < MIN_PLATE_RATIO > MAX_PLATE_RATIO:
            continue
        
        plate_imgs.append(img_cropped)
        plate_infos.append({
            'x': int(plate_cx - plate_width / 2),
            'y': int(plate_cy - plate_height / 2),
            'w': int(plate_width),
            'h': int(plate_height)
        })
        
        
    #최종확인
    longest_idx, longest_text = -1, 0
    plate_chars = []

    for i, plate_img in enumerate(plate_imgs):

        plate_img = cv2.resize(plate_img, dsize=(0, 0), fx=1.6, fy=1.6)
        _, plate_img = cv2.threshold(plate_img, thresh=0.0, maxval=255.0, type=cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(plate_img, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)
        

        plate_min_x, plate_min_y = plate_img.shape[1], plate_img.shape[0]
        plate_max_x, plate_max_y = 0, 0

        # print(Fine_Contour.find_contour(contours=contours,img = temp_result))
        
        for a,contour in enumerate(contours):
            x, y, w, h = cv2.boundingRect(contour)    
            area = w * h
            ratio = w / h

            if area > MIN_AREA \
            and w > MIN_WIDTH and h > MIN_HEIGHT \
            and MIN_RATIO < ratio < MAX_RATIO:
                # cv2.rectangle(plate_img, pt1=(x, y), pt2=(x+w, y+h), color=(255, 255, 255), thickness=1)
                if x < plate_min_x: plate_min_x = x
                if y < plate_min_y: plate_min_y = y
                if x + w > plate_max_x: plate_max_x = x + w
                if y + h > plate_max_y: plate_max_y = y + h
                
        # plt.imshow(plate_img,cmap='gray')
        # plt.show()

        
        img_result = plate_img[plate_min_y:plate_max_y, plate_min_x:plate_max_x]
        # print("필터 처리 전")
        # plt.imshow(img_result,cmap='gray')
        # plt.show() 
        
        img_result = cv2.GaussianBlur(img_result, ksize=(5, 5), sigmaX=0)
        _, img_result = cv2.threshold(img_result, thresh=0.0, maxval=255.0, type=cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        img_result = cv2.copyMakeBorder(img_result, top=10, bottom=10, left=10, right=10, borderType=cv2.BORDER_CONSTANT, value=(0,0,0))
        # print("필터 처리 후")
        # plt.imshow(img_result,cmap='gray')
        # plt.show() 
        
        # chars = pytesseract.image_to_string(img_result, lang='kor', config='--psm 7 --oem 0')
        # print('1 : ',chars)
        chars2 = pytesseract.image_to_string(img_result, lang='kor',config='--psm 7')
        # print('2 : ',chars2)
        # chars3 = pytesseract.image_to_string(img_result, lang='kor')
        # print('3 : ',chars3)
        
        result_chars = ''
        has_digit = False
        for c in chars2:
            if ord('가') <= ord(c) <= ord('힣') or c.isdigit():
                if c.isdigit():
                    has_digit = True
                result_chars += c
        
        # print(result_chars)        
        plate_chars.append(result_chars)
        if has_digit and len(result_chars) > longest_text:
            longest_idx = i
            longest_text = len(result_chars)

#####
        # plt.subplot(len(plate_imgs), 1, i+1)
        # plt.imshow(img_result, cmap = 'gray')
        # plt.show()
        
    # print("plate number : ",plate_chars[longest_idx])

    detected = False
    res = plate_chars[longest_idx]     
    detected = Detection(res)
    return res, img_ori,detected



# img = cv2.imread('./images/car_plate_test5.jpg')
img = cv2.imread('/home/hyoseok/catkin_ws/src/test_pkg/images/car_plate_test5.jpg')

Number , img ,dect= find_number(img)
print(Number)
print(dect)

# cv2.imshow("car_number_plate",img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
