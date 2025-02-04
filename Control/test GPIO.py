import RPi.GPIO as GPIO

# تنظیم GPIO
GPIO.setmode(GPIO.BCM)

# تعریف پین‌ها
PWM_PIN_5 = 5
PIN_6 = 6
PIN_13 = 13
PWM_PIN_19 = 19

# تنظیم پین‌های دیجیتال
GPIO.setup(PIN_6, GPIO.OUT)
GPIO.setup(PIN_13, GPIO.OUT)

# تنظیم پین‌های PWM
GPIO.setup(PWM_PIN_5, GPIO.OUT)
GPIO.setup(PWM_PIN_19, GPIO.OUT)

pwm5 = GPIO.PWM(PWM_PIN_5, 1000) # فرکانس 1000 هرتز
pwm19 = GPIO.PWM(PWM_PIN_19, 1000) # فرکانس 1000 هرتز

# مقداردهی اولیه
pwm5.start(0) # PWM صفر درصد برای پین 5
GPIO.output(PIN_6, GPIO.HIGH) # مقدار 1 برای پین 6
GPIO.output(PIN_13, GPIO.HIGH) # مقدار 0 برای پین 13
pwm19.start(10) # PWM 10 درصد برای پین 19

try:
    while True:
        pass # برنامه در حالت اجرا باقی می‌ماند

except KeyboardInterrupt:
    pwm5.stop()
    pwm19.stop()
    GPIO.cleanup()
    print("GPIO cleanup done")