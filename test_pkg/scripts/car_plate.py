import cv2
import numpy as np
import pytesseract
# 이미지 파일 읽기

img = cv2.imread('/home/hyoseok/catkin_ws/src/test_pkg/images/test.jpeg')
# 이미지 전처리
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blur = cv2.GaussianBlur(gray, (5,5), 0)
chars = pytesseract.image_to_string(blur, lang='kor', config='--psm 7')
print(chars)
cv2.imshow("test",gray)
cv2.waitKey(5000)
cv2.destroyAllWindows()