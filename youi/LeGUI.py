import uuid
import json

HIBYE = True

if HIBYE:
    import hibye
else:
    from common_srvs.srv import QuestionAnswer, QuestionAnswerRequest
    import rospy

class ConnectionHandler(object):
    """ Object for keeping track of server and sending / receiving commands """
    def __init__(self, verbose=0):
        self.connected_server = None
        self.verbose = verbose

    def send_and_receive(self, channel, data):
        if HIBYE:
            response = self.call_service(channel, None, data)
        else:
            response = self.call_service(channel, QuestionAnswer, QuestionAnswerRequest(data))
            if response is not None:
                response = response.answer
        return response

    def call_service(self, srv_name, srv_type, request):
        found_service = True
        if HIBYE:
            service_proxy = hibye.wait_for_service(srv_name, timeout=1.)
            if service_proxy is None:
                found_service = False
        else:
            try:
                rospy.wait_for_service(srv_name, timeout=1.)
            except rospy.ROSException:
                found_service = False
        if not found_service:
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
        if HIBYE:
            resp1 = hibye.call_service(srv_name, request, s=service_proxy)
        else:
            service_proxy = rospy.ServiceProxy(srv_name, srv_type)
            resp1 = service_proxy(request)
            # except rospy.ServiceException as e:
            #     print("Service call failed: %s"%e)
        return resp1


class LeWidgetClient(object):
    def __init__(self, widget_response, connection_handler):
        self.uid = widget_response["uid"]
        self.label = widget_response["label"]
        self.connection_handler = connection_handler

    def widget_typestr():
        raise NotImplementedError

class LeButtonClient(LeWidgetClient):
    def __init__(self, widget_response, connection_handler):
        super(LeButtonClient, self).__init__(widget_response, connection_handler)

    def on_click(self, func, args=None, kwargs=None):
        # set up a rosservice for calling the callback when the button is pressed
        # idea: start a new thread which checks is_clicked often and executes callback
        # thread needs to be added to LeGUI, also how do we deal with buttons being deleted?
        raise NotImplementedError

    def is_clicked(self):
        resp_str = self.connection_handler.send_and_receive('is_button_clicked', str(self.uid))
        if resp_str == "true":
            return True
        return False

    def widget_typestr():
        return "LeButton"

class LeToggleClient(LeWidgetClient):
    def __init__(self, widget_response, connection_handler):
        super(LeToggleClient, self).__init__(widget_response, connection_handler)

    def is_enabled(self):
        resp_str = self.connection_handler.send_and_receive('is_toggle_enabled', str(self.uid))
        if resp_str == "true":
            return True
        return False

    def widget_typestr():
        return "LeToggle"

class NotFoundWidget(object):
    def is_clicked(self):
        return None

class LeGUI(object):
    def __init__(self, path=None, verbose=0):
        # TODO starts pyqt5 app in separate thread
        self.verbose = verbose
        self.connection_handler = ConnectionHandler(verbose=verbose)

    def get_button(self, idx=None, name=None, uid=None):
        return self.get_widgetclient(LeButtonClient, idx=idx, name=name, uid=uid)

    def get_toggle(self, idx=None, name=None, uid=None):
        return self.get_widgetclient(LeToggleClient, idx=idx, name=name, uid=uid)

    def get_widgetclient(self, widgetclient_type, idx=None, name=None, uid=None):
        widget_typestr = widgetclient_type.widget_typestr()
        query_dict = {"idx": idx, "name": name, "uid": uid, "typestr": widget_typestr}
        query_json = json.dumps(query_dict)
        resp_str = self.connection_handler.send_and_receive('get_widget', query_json)
        if resp_str is None:
            if self.verbose > 0:
                print("No LeGUI server found when getting {}".widget_typestr)
            return NotFoundWidget()
        widget_response = json.loads(resp_str)
        # If a widget isn't found it will be created, unless a uid was specified
        if not widget_response["is_found"]:
            if self.verbose > 0:
                print("Requested {} not found".format(widget_typestr))
            return NotFoundWidget()
        widgetclient = widgetclient_type(widget_response, self.connection_handler)
        # print("found widget {}".format(widgetclient.uid))
        return widgetclient

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


