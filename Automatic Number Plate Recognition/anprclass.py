# anprclass.py 

from skimage.segmentation import clear_border
import pytesseract
import numpy as np
import imutils
import cv2
import os
from abc import ABC, abstractmethod

class ANPR(ABC):
    # Hàm khởi tạo, thiết lập các thông số ban đầu.
    def __init__(self, algo_id, morph_op, minAR=2, maxAR=5, debug=False, save=False):
        self.min_aspect_ratio = minAR
        self.max_aspect_ratio = maxAR
        self.debug_mode = debug
        self.save_image = save
        self.algorithm_id = algo_id
        self.morph_operator = morph_op

    # Hiển thị các bước xử lý nếu chế độ debug được bật.
    def display_debug(self, title, image):
        if self.debug_mode:
            cv2.imshow(title, image)
            cv2.waitKey(0)

    # Lưu ảnh kết quả vào thư mục tương ứng.
    def save_result(self, filename, image):
        if not self.save_image or self.debug_mode:
            return
        
        target_dir = os.path.join(r"D:\testbiensoxe\result", str(self.algorithm_id))
        if not os.path.exists(target_dir):
            os.makedirs(target_dir)

        full_path = os.path.join(target_dir, filename)
        cv2.imwrite(full_path, image)
        print(f"Đã lưu kết quả vào: {full_path}")
        
    # Áp dụng Blackhat/Tophat để làm nổi bật vùng ký tự.
    def apply_morphological_transform(self, gray_image):
        rect_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (13, 5))
        
        if self.morph_operator == 'bh': # Blackhat cho chữ đen nền trắng
            transformed = cv2.morphologyEx(gray_image, cv2.MORPH_BLACKHAT, rect_kernel)
            square_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            light_mask = cv2.morphologyEx(gray_image, cv2.MORPH_CLOSE, square_kernel)
            light_mask = cv2.threshold(light_mask, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
            return (transformed, light_mask)
        elif self.morph_operator == 'th': # Tophat cho chữ trắng nền đen
            transformed = cv2.morphologyEx(gray_image, cv2.MORPH_TOPHAT, rect_kernel)
            square_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            dark_mask = cv2.morphologyEx(gray_image, cv2.MORPH_CLOSE, square_kernel)
            dark_mask = cv2.threshold(dark_mask, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
            return (transformed, dark_mask)

    # Phương thức trừu tượng, nơi chứa logic xử lý ảnh lõi của từng thuật toán.
    @abstractmethod
    def _process_image_for_plate(self, transformed_image):
        pass

    # Quy trình chính để tìm các đường viền có khả năng là biển số.
    def find_plate_candidates(self, gray_image):
        # 1. Áp dụng biến đổi hình thái để lấy ảnh đã xử lý và mặt nạ sáng/tối.
        (transformed_image, brightness_mask) = self.apply_morphological_transform(gray_image)
        self.display_debug("Blackhat/Tophat", transformed_image)
        self.display_debug("Brightness Mask", brightness_mask)

        # 2. Gọi phương thức xử lý lõi (của Sobel, Canny, hoặc Edgeless).
        thresh_image = self._process_image_for_plate(transformed_image)
        self.display_debug("Initial Threshold", thresh_image)

        # 3. Dọn dẹp ảnh nhị phân (KHÔI PHỤC LOGIC GỐC).
        thresh_image = cv2.erode(thresh_image, None, iterations=3)
        thresh_image = cv2.dilate(thresh_image, None, iterations=3)
        thresh_image = cv2.bitwise_and(thresh_image, thresh_image, mask=brightness_mask)
        thresh_image = cv2.dilate(thresh_image, None, iterations=2)
        thresh_image = cv2.erode(thresh_image, None, iterations=1)
        self.display_debug("Cleaned Threshold", thresh_image)
        
        # 4. Tìm các đường viền và giữ lại những đường viền lớn nhất.
        contours = cv2.findContours(thresh_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:5]
        return contours

    # Lọc và chọn ra biển số chính xác từ các ứng cử viên.
    def locate_license_plate(self, filename_prefix, gray_image, candidates, clear_border_flag=False):
        for candidate in candidates:
            (x, y, w, h) = cv2.boundingRect(candidate)
            aspect_ratio = w / float(h)

            if self.min_aspect_ratio <= aspect_ratio <= self.max_aspect_ratio:
                plate_image = gray_image[y:y + h, x:x + w]
                roi_image = cv2.threshold(plate_image, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
                if clear_border_flag:
                    roi_image = clear_border(roi_image)
                self.save_result(f'roi_{filename_prefix}.png', plate_image)
                return (roi_image, candidate)
        return (None, None)

    # Hàm tổng hợp: điều phối toàn bộ quá trình.
    def find_and_ocr_plate(self, filename_prefix, image, psm=7, clear_border_flag=False):
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        candidates = self.find_plate_candidates(gray_image)
        (roi_image, plate_contour) = self.locate_license_plate(filename_prefix, gray_image, candidates, clear_border_flag)
        
        if roi_image is not None:
            options = f"--psm {psm} -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"
            plate_text = pytesseract.image_to_string(roi_image, config=options)
            return (plate_text, plate_contour)
        return (None, None)

# Lớp con cài đặt thuật toán Sobel.
class SobelANPR(ANPR):
    def _process_image_for_plate(self, transformed_image):
        grad_x = cv2.Sobel(transformed_image, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=-1)
        grad_x = np.absolute(grad_x)
        (min_val, max_val) = (np.min(grad_x), np.max(grad_x))
        grad_x = 255 * ((grad_x - min_val) / (max_val - min_val))
        grad_x = grad_x.astype("uint8")
        
        grad_x = cv2.GaussianBlur(grad_x, (5, 5), 0)
        grad_x = cv2.morphologyEx(grad_x, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (13, 5)))
        return cv2.threshold(grad_x, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

# Lớp con cài đặt thuật toán Canny.
class CannyANPR(ANPR):
    def _process_image_for_plate(self, transformed_image):
        canny_image = cv2.Canny(transformed_image, 50, 150)
        canny_image = cv2.morphologyEx(canny_image, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (13, 5)))
        return cv2.threshold(canny_image, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

# Lớp con cài đặt thuật toán "Edgeless".
class EdgelessANPR(ANPR):
    def _process_image_for_plate(self, transformed_image):
        gaussian_image = cv2.GaussianBlur(transformed_image, (5,5), 0)
        gaussian_image = cv2.morphologyEx(gaussian_image, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (13, 5)))
        return cv2.threshold(gaussian_image, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]