import sys
from tkinter import Button
from bson import UUID_SUBTYPE
import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

import rospy

from common_srvs.srv import QuestionAnswer, QuestionAnswerResponse

class WidgetClient(object):
    def __init__(self, mainwindow, uid):
        self.uid = uid
        self.mainwindow = mainwindow

class ButtonClient(WidgetClient):
    def __init__(self, mainwindow, uid):
        super(ButtonClient, self).__init__(mainwindow, uid)
        self.was_clicked = False
        # we can't create the button here because this may not be the main thread

    def on_click(self):
        self.was_clicked = True

    def get_click_status(self):
        status = self.was_clicked
        self.was_clicked = False
        return status

class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)


class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        # Create the maptlotlib FigureCanvas object,
        # which defines a single set of axes as self.axes.
        # sc = MplCanvas(self, width=5, height=4, dpi=100)
        # sc.axes.plot([0,1,2,3,4], [10,1,20,3,40])
        # self.setCentralWidget(sc)

        # Adds a button
        # btn = QtWidgets.QPushButton('Edit', self)
        # btn.clicked.connect(self.plot)
        # self.statusBar().addPermanentWidget(btn)


        self.show()

    def add_button(self):
        btn = QtWidgets.QPushButton('Button', self)
        return btn


class LeGUIServer(object):
    def __init__(self, path=None):
        rospy.init_node('LeGUIServer')
        self.widget_clients = []
        self.message_queue = []
        

        # starts pyqt5 app in separate thread
        app = QtWidgets.QApplication(sys.argv)
        self.mainwindow = MainWindow()

        s = rospy.Service('add_button', QuestionAnswer, self.add_button_service_call)
        s = rospy.Service('is_button_clicked', QuestionAnswer, self.is_button_clicked_service_call)

        # loop in main thread for checking incoming messages
        timer = QtCore.QTimer()
        timer.timeout.connect(self.check_messages)
        timer.start(1000)

        app.exec_()

    def check_messages(self):
        print("checking")
        if len(self.message_queue) > 0:
            message = self.message_queue.pop()
            self.handle(message)
        return True

    def handle(self, message):
        if message["action"] == "create_button":
            print("creating button")
            message["button_client"].qt_btn = QtWidgets.QPushButton('Button', self.mainwindow)
            message["button_client"].qt_btn.clicked.connect(message["button_client"].on_click)
            message["button_client"].qt_btn.show()
            

    def find_widget_client(self, uid, type_):
        for widget in self.widget_clients:
            match_uid = uid == widget.uid
            match_type = isinstance(widget, type_)
            if match_uid and match_type:
                return widget
        return None

    def add_button_service_call(self, req):
        uid = req.question
        new_button_client = ButtonClient(self.mainwindow, uid)
        self.widget_clients.append(new_button_client)
        self.message_queue.append({"action": "create_button", "button_client": new_button_client})
        return QuestionAnswerResponse("")

    def is_button_clicked_service_call(self, req):
        uid = req.question
        button = self.find_widget_client(uid, ButtonClient)
        if button is None:
            raise ValueError("Button with requested uid {} not found.".format(uid))
        answer_string = "true" if button.get_click_status() else "false"
        return QuestionAnswerResponse(answer_string)
        



if __name__ == "__main__":
    LeGUIServer()
    