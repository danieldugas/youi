import uuid
import rospy
import json

from common_srvs.srv import QuestionAnswer, QuestionAnswerRequest

class ConnectionHandler(object):
    """ Object for keeping track of """
    def __init__(self, verbose=0):
        self.connected_server = None
        self.verbose = verbose

    def send_and_receive(self, channel, data):
        response = self.call_service(channel, QuestionAnswer, QuestionAnswerRequest(data))
        if response is None:
            return None
        return response.answer

    def call_service(self, srv_name, srv_type, request):
        try:
            rospy.wait_for_service(srv_name, timeout=1.)
        except rospy.ROSException:
            if self.connected_server is not None:
                if self.verbose > 0: 
                    print("Server disconnected")
            self.connected_server = None
            return None
        if self.connected_server is None:
            self.connected_server = "unknown"
            if self.verbose > 0: 
                print("Connected to server {}".format(self.connected_server))
        # try:
        service_proxy = rospy.ServiceProxy(srv_name, srv_type)
        resp1 = service_proxy(request)
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s"%e)
        return resp1


class LeWidgetClient(object):
    def __init__(self, button_response, connection_handler):
        self.uid = button_response["uid"]
        self.label = button_response["label"]
        self.connection_handler = connection_handler

class LeButtonClient(LeWidgetClient):
    def __init__(self, button_response, connection_handler):
        super(LeButtonClient, self).__init__(button_response, connection_handler)
        
    def on_click(self, func, args=None, kwargs=None):
        # set up a rosservice for calling the callback when the button is pressed
        raise NotImplementedError

    def is_clicked(self):
        resp_str = self.connection_handler.send_and_receive('is_button_clicked', str(self.uid))
        if resp_str == "true":
            return True
        return False

class NotFoundButton(object):
    def is_clicked(self):
        return None

class LeGUI(object):
    def __init__(self, path=None, verbose=0):
        # TODO starts pyqt5 app in separate thread
        self.verbose = verbose
        self.connection_handler = ConnectionHandler(verbose=verbose)

    def get_button(self, idx=None, name=None, uid=None):
        button_query = {"idx": idx, "name": name, "uid": uid, "typestr": "LeButton"}
        button_query_json = json.dumps(button_query)
        resp_str = self.connection_handler.send_and_receive('get_widget', button_query_json)
        if resp_str is None:
            if self.verbose > 0:
                print("No LeGUI server found when getting button")
            return NotFoundButton()
        button_response = json.loads(resp_str)
        # If a button isn't found it will be created, unless a uid was specified
        if not button_response["is_found"]:
            if self.verbose > 0:
                print("Requested button not found")
            return NotFoundButton()
        button = LeButtonClient(button_response, self.connection_handler)
        # print("found button {}".format(button.uid))
        return button

if __name__ == "__main__":
    import time

    G = LeGUI(verbose=1)
    G.get_button(idx=0).is_clicked()

    for i in range(100):

        # gets the first button created in the live GUI. (If there isn't one yet, it will be created)
        B = G.get_button(idx=0)
        if B.is_clicked():
            print("button {} was clicked recently.".format(B.label))

        time.sleep(1.0)
    

