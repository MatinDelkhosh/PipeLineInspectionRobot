import cv2
import numpy as np
import os
from google.colab.patches import cv2_imshow

def detect_strongest_circle(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # افزایش کنتراست تصویر
    gray = cv2.equalizeHist(gray)
    
    # بلور برای کاهش نویز
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    # تنظیمات HoughCircles
    circles = cv2.HoughCircles(
        blurred, 
        cv2.HOUGH_GRADIENT, 
        dp=1.5,  # افزایش دقت
        minDist=30,  # فاصله حداقل بین دایره‌ها
        param1=80,  # آستانه برای تشخیص لبه
        param2=30,  # آستانه تصمیم‌گیری
        minRadius=20,  # حداقل شعاع
        maxRadius=200  # حداکثر شعاع
    )
    
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        strongest_circle = circles[0]
        x, y, r = strongest_circle
        
        cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
        cv2.circle(frame, (x, y), 2, (0, 0, 255), 3)
        return (x, y), frame

    return None, frame

def main(image_folder):
    images = sorted([os.path.join(image_folder, f) for f in os.listdir(image_folder) if f.endswith(('.png', '.jpg', '.jpeg'))])
    print("لیست تصاویر موجود:")
    print(images)

    for image_path in images:
        frame = cv2.imread(image_path)
        
        if frame is None:
            print(f"خطا در باز کردن تصویر {image_path}")
            continue

        print(f"در حال پردازش تصویر: {image_path}")

        center, output_frame = detect_strongest_circle(frame)

        if center:
            print(f"تصویر {image_path}: قوی‌ترین دایره در موقعیت {center} شناسایی شد.")
        else:
            print(f"تصویر {image_path}: هیچ دایره‌ای شناسایی نشد.")

        cv2_imshow(output_frame)

if __name__ == "__main__":
    image_folder = "content"
    main(image_folder)
