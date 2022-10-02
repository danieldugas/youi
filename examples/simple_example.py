import time

from youi import LeGUI

G = LeGUI()

# we can connect buttons to callbacks which are run immediately when the button is clicked.
# def do_something():
#     print("Click!")
# G.get_button(idx=0).on_click(do_something)

sequence = []
for i in range(100):
    # checks the status of the first toggle created in the live GUI. (If there isn't one yet, it will be created)
    if G.get_toggle(idx=0).is_enabled():
        print("toggle is enabled")
        sequence.append(1)
    else:
        print("toggle is disabled")
        sequence.append(0)

    # gets the first button created in the live GUI. (If there isn't one yet, it will be created)
    if G.get_button(idx=0).is_clicked():
        print("button was clicked recently.")

    # gets the first axes created in the live GUI. (if they don't exist, creates an axes widget)
#     ax = G.get_axes(idx=0)
#     ax.plot(sequence)

    time.sleep(0.5)




