import uuid
import rospy

from common_srvs.srv import QuestionAnswer, QuestionAnswerRequest

def call_service(srv_name, srv_type, request):
    rospy.wait_for_service(srv_name)
    # try:
    service_proxy = rospy.ServiceProxy(srv_name, srv_type)
    resp1 = service_proxy(request)
    # except rospy.ServiceException as e:
    #     print("Service call failed: %s"%e)
    return resp1

class LeWidget(object):
    def __init__(self):
        self.name = ""
        self.uid = uuid.uuid4()

class LeButton(LeWidget):
    def __init__(self):
        super(LeButton, self).__init__()
        response = call_service('add_button', QuestionAnswer, QuestionAnswerRequest(str(self.uid)))
        
    def on_click(self, func, args=None, kwargs=None):
        # set up a rosservice for calling the callback when the button is pressed
        raise NotImplementedError

    def is_clicked(self):
        response = call_service('is_button_clicked', QuestionAnswer, QuestionAnswerRequest(str(self.uid)))
        if response.answer == "true":
            return True


class LeGUI(object):
    def __init__(self, path=None):
        # starts pyqt5 app in separate thread
        self.widgets = []
        pass

    def find_widget(self, idx, name, type_):
        count = 0
        for widget in self.widgets:
            match_idx = idx is None or idx == count
            match_name = name is None or name == widget.name
            match_type = isinstance(widget, type_)
            if match_idx and match_name and match_type:
                return widget
            if match_type:
                count += 1
        return None

    def get_button(self, idx=None, name=None):
        button = self.find_widget(idx=idx, name=name, type_=LeButton)
        if button is None:
            print("Creating button")
            button = LeButton()
            self.widgets.append(button)
        return button

#     # we can connect buttons to callbacks which are run immediately when the button is clicked.
# def do_something():
#     print("Click!")
# G.get_button(idx=0).on_click(do_something)

# sequence = []
# for i in range(100):
#     # checks the status of the first toggle created in the live GUI. (If there isn't one yet, it will be created)
#     if G.get_toggle(idx=0).is_enabled():
#         print("toggle is enabled")
#         sequence.append(1)
#     else:
#         print("toggle is disabled")
#         sequence.append(0)

#     # gets the first button created in the live GUI. (If there isn't one yet, it will be created)
#     if G.get_button(idx=0).is_clicked():
#         print("button was clicked recently.")

#     # gets the first axes created in the live GUI. (if they don't exist, creates an axes widget)
#     ax = G.get_axes(idx=0)
#     ax.plot(sequence)

#     time.sleep(1.0)

if __name__ == "__main__":
    import time

    G = LeGUI()
    G.get_button(idx=0).is_clicked()

    for i in range(100):

        # gets the first button created in the live GUI. (If there isn't one yet, it will be created)
        if G.get_button(idx=0).is_clicked():
            print("button was clicked recently.")

        time.sleep(1.0)
    

