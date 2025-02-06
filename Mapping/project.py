import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import threading
import time
def plotting_pipe():
# لیست ذخیره نقاط
    points = []

    # ثابت قطر لوله
    DIAMETER = 30
    RADIUS = DIAMETER / 2

    def add_point(new_point):
        """
        افزودن نقطه جدید به لیست نقاط
        """
        global points
        points.append(new_point)

    def plot_pipe():
        """
        رسم لوله و مسیر بر اساس نقاط
        """
        plt.ion()  # فعال کردن حالت تعاملی
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        while True:
            if len(points) >= 2:  # حداقل دو نقطه برای رسم لوله نیاز است
                ax.clear()

                # رسم پوسته لوله
                theta = np.linspace(0, 2 * np.pi, 100)
                for i in range(len(points) - 1):
                    start, end = points[i], points[i + 1]

                    # ساخت مقطع لوله بین دو نقطه
                    x_center = np.linspace(start[0], end[0], 50)
                    y_center = np.linspace(start[1], end[1], 50)
                    z_center = np.linspace(start[2], end[2], 50)

                    for j in range(len(x_center)):
                        x_circle = x_center[j] + RADIUS * np.cos(theta)
                        y_circle = y_center[j] + RADIUS * np.sin(theta)
                        z_circle = np.full_like(x_circle, z_center[j])
                        ax.plot(x_circle, y_circle, z_circle, color="blue", alpha=0.3)

                # رسم مسیر نقاط
                xs, ys, zs = zip(*points)
                ax.plot(xs, ys, zs, color="red", linewidth=3, label="Pipe Path")
                ax.scatter(xs, ys, zs, color="black", s=50, label="Points")

                # تنظیمات نمودار
                ax.set_xlabel("X")
                ax.set_ylabel("Y")
                ax.set_zlabel("Z")
                ax.set_title("Pipe Profile with Defined Diameter")
                ax.legend()
                plt.draw()
            plt.pause(0.01)  # تأخیر برای بروزرسانی

    def simulate_real_time_input():
        """
        شبیه‌سازی دریافت نقاط در زمان واقعی
        """
        add_point((15, 10, 6))  # نقطه اول
        print("نقطه اضافه شد: (15, 10, 6)")
        add_point((10, 8, 4))  # نقطه دوم
        print("نقطه اضافه شد: (10, 8, 4)")
        add_point((5, 3, 10))  # نقطه سوم
        print("نقطه اضافه شد: (5, 3, 10)")

    if __name__ == "__main__":
        # اجرای شبیه‌سازی دریافت داده‌ها در یک نخ جداگانه
        input_thread = threading.Thread(target=simulate_real_time_input, daemon=True)
        input_thread.start()

        # رسم پروفیل لوله
        plot_pipe()
    ر