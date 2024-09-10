#! /usr/bin/env python3

# from stretch_core.keyboard import KBHit
import stretch_body.gamepad_controller as gc

# kb = KBHit()
# while True:
#     if kb.kbhit(): # Returns True if any key pressed
#         activation = kb.getch() # Enter == "\n"
#         if (activation == "\n"):
#             break

xbox_controller = gc.GamePadController()
xbox_controller.start()

while True:
    controller_state = xbox_controller.get_state()
    if controller_state['right_pad_pressed']: # Returns True if any key pressed
        break


