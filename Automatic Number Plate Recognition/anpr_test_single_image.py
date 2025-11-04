# anpr_test_single_image.py
from anprclass import CannyANPR, EdgelessANPR, SobelANPR
import matplotlib.pyplot as plt
import imutils
import sys
import cv2
import os

def cleanup_text(text):
    # Hàm này giúp loại bỏ các ký tự không phải ASCII để hiển thị không bị lỗi font
    return "".join([c for c in text if ord(c) < 128]).strip()

# --- KHU VỰC THIẾT LẬP ---

# 1. Đặt đường dẫn đến ảnh của bạn ở đây
image_path = r"D:\testbiensoxe\test\alo3.jpg" # <<-- THAY ĐỔI ĐƯỜNG DẪN NÀY

# 2. Chọn thuật toán (1: Sobel, 2: Canny, 3: Edgeless)
algorithm_choice = 3 

# 3. Các thiết lập khác
save_results = True      # True nếu muốn lưu ảnh kết quả
debug_mode = True        # True để xem các bước xử lý trung gian
morphology_type = 'bh'   # 'bh' cho chữ đen nền trắng, 'th' cho chữ trắng nền đen
psm_mode = 7
clear_border_pixels = False

# --- KẾT THÚC KHU VỰC THIẾT LẬP ---

anpr = None

# Khởi tạo đối tượng ANPR dựa trên lựa chọn

if algorithm_choice == 1:
    anpr = SobelANPR(algo_id=algorithm_choice, morph_op=morphology_type, debug=debug_mode, save=save_results)
elif algorithm_choice == 2:
    anpr = CannyANPR(algo_id=algorithm_choice, morph_op=morphology_type, debug=debug_mode, save=save_results)
elif algorithm_choice == 3:
    anpr = EdgelessANPR(algo_id=algorithm_choice, morph_op=morphology_type, debug=debug_mode, save=save_results)
else:
    print('Lựa chọn thuật toán không hợp lệ')
    sys.exit()

# Lấy tên file gốc để đặt tên cho file output
filename_prefix = os.path.splitext(os.path.basename(image_path))[0]

# Đọc ảnh từ biến 'image_path'
originimage = cv2.imread(image_path)
if originimage is None:
    print(f"Lỗi: Không thể đọc ảnh từ đường dẫn: {image_path}")
    sys.exit()

# Tiền xử lý ảnh
image = imutils.resize(originimage, width=400, height=400)
image = cv2.bilateralFilter(image, 3, 105, 105)

(lpText, lpCnt) = anpr.find_and_ocr_plate(
    filename_prefix=filename_prefix, 
    image=image, 
    psm=psm_mode, 
    clear_border_flag=clear_border_pixels
)

# Vẽ kết quả lên ảnh
if lpText is not None and lpCnt is not None:
    box = cv2.boxPoints(cv2.minAreaRect(lpCnt))
    box = box.astype("int")
    cv2.drawContours(image, [box], -1, (0, 255, 0), 2)

    (x, y, w, h) = cv2.boundingRect(lpCnt)
    cv2.putText(image, cleanup_text(lpText), (x, y - 15),
        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

    print(f"[INFO] Biển số nhận dạng được: {cleanup_text(lpText)}")
else:
    print("[INFO] Không tìm thấy biển số trong ảnh.")

if save_results:
    # 'luu_ket_qua' -> 'save_result'
    anpr.save_result(f"Final_{filename_prefix}.jpg", image)

# HIỂN THỊ KẾT QUẢ BẰNG MATPLOTLIB
result_image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

plt.figure(figsize=(10, 8))
plt.imshow(result_image_rgb)
plt.title("Kết quả nhận dạng biển số")
plt.axis('off')
plt.show()