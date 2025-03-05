from gpiozero import LED, Button
from time import sleep

blink_speed = .25

lblinker = LED(17)
lbutton = Button(27)
lcount = 0

rblinker = LED(18)
rbutton = Button(23)
rcount = 0
def toggle_lblinker():
    global lcount
    lcount += 1
    if lcount % 2 == 0:
        lblinker.off()
    else:
        lblinker.blink(on_time=blink_speed, off_time=blink_speed)

def toggle_rblinker():
    global rcount
    rcount += 1
    if rcount % 2 == 0:
        rblinker.off()
    else:
        rblinker.blink(on_time=blink_speed, off_time=blink_speed)

while True:
    lbutton.when_pressed = toggle_lblinker
    rbutton.when_pressed = toggle_rblinker

