# TODO: click to add button
import sys
import time
import uuid
import json
from tkinter import Button
from bson import UUID_SUBTYPE
import matplotlib
from matplotlib import widgets
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QWidget, QPushButton, QMainWindow
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout
from PyQt5.QtWidgets import QMenu
from PyQt5.QtWidgets import QInputDialog
from PyQt5.QtGui import QPalette, QColor

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

import rospy

from common_srvs.srv import QuestionAnswer, QuestionAnswerResponse

class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(MplCanvas, self).__init__(fig)

class MainLayoutWidget(QWidget):
    def __init__(self, color):
        super(MainLayoutWidget, self).__init__()
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor(color))
        self.setPalette(palette)
    

class LeVBox(QWidget):
    """ An empty widget with vertical layout, can be right clicked to add objects inside """
    def __init__(self, leguiserver, color=None):
        super(LeVBox, self).__init__()
        self.leguiserver = leguiserver

        if color is not None:
            self.setAutoFillBackground(True)
            palette = self.palette()
            palette.setColor(QPalette.Window, QColor(color))
            self.setPalette(palette)

    # def mousePressEvent(self, QMouseEvent):
    #     if QMouseEvent.button() == Qt.LeftButton:
    #         print("Left Button Clicked")
    #     elif QMouseEvent.button() == Qt.RightButton:
    #         #do what you want here
    #         print("Right Button Clicked")

    def contextMenuEvent(self, event):
        contextMenu = QMenu(self)
        addAct = contextMenu.addAction("Add")
        action = contextMenu.exec_(self.mapToGlobal(event.pos()))
        if action == addAct:
            self.addLeButton()
            # self.close()

    def addLeButton(self, uid=None):
        if uid is None:
            uid = str(uuid.uuid4())
        editable_button = LeButton(uid, 'Button {}'.format(self.leguiserver.count_widgets(type_=LeButton)), self)
        self.leguiserver.all_editable_widgets.append(editable_button)
        editable_button.clicked.connect(editable_button.on_click)
        self.layout().addWidget(editable_button)
        return editable_button


class LeButton(QPushButton):
    def __init__(self, uid, text, parent):
        icon = None
        super(LeButton, self).__init__(text=text) 
        self.parent = parent
        self.uid = uid
        self.was_clicked = False

    def contextMenuEvent(self, event):
        contextMenu = QMenu(self)
        deleteAct = contextMenu.addAction("Delete")
        settextAct = contextMenu.addAction("Set Text")
        action = contextMenu.exec_(self.mapToGlobal(event.pos()))
        if action == deleteAct:
            self.parent.leguiserver.all_editable_widgets.remove(self)
            self.close()
        if action == settextAct:
            text, ok = QInputDialog.getText(self, 'Text Input Dialog', 'Enter new text:')
            if ok:
                self.setText(text)
            # Delete self TODO
            # set text TODO
    
    def on_click(self):
        self.was_clicked = True

    def get_click_status(self):
        status = self.was_clicked
        self.was_clicked = False
        return status

    def get_label(self):
        return self.text()


class LeGUIServer(object):
    def __init__(self, path=None):
        rospy.init_node('LeGUIServer')
        self.all_editable_widgets = []
        self.message_queue = []
        self.edit_mode_enabled = False
        self.currently_selected_editablevbox = None
        
        # Create and populate main window
        app = QtWidgets.QApplication(sys.argv)
        self.mainwindow = QMainWindow()
        self.mainwindow.show()
        btn = QPushButton('Edit', self.mainwindow)
        btn.clicked.connect(self.toggle_edit_mode)
        self.mainwindow.statusBar().addPermanentWidget(btn)
        layout = QVBoxLayout()


        editable = LeVBox(self, 'blue')
        editable.setLayout(QVBoxLayout())
        editable.addLeButton()
        editable.addLeButton()
        self.currently_selected_editablevbox = editable

        editable2 = LeVBox(self, 'cyan')
        editable2.setLayout(QVBoxLayout())
        editable2.layout().addWidget(MplCanvas(self, width=5, height=4, dpi=100))


        parent = MainLayoutWidget('red')
        parent.setLayout(QHBoxLayout())
        parent.layout().addWidget(editable)

        parent.layout().addWidget(editable2)
        self.mainwindow.setCentralWidget(parent)

        s = rospy.Service('get_widget', QuestionAnswer, self.get_widget_service_call)
        s = rospy.Service('is_button_clicked', QuestionAnswer, self.is_button_clicked_service_call)

        # loop in main thread for checking incoming messages
        timer = QtCore.QTimer()
        timer.timeout.connect(self.check_messages)
        timer.start(1000)

        app.exec_()
    
    def toggle_edit_mode(self):
        self.edit_mode_enabled = not self.edit_mode_enabled

    def check_messages(self):
        print("checking")
        if len(self.message_queue) > 0:
            message = self.message_queue.pop()
            self.handle(message)
        return True

    def handle(self, message):
        if message["action"] == "create_lewidget":
            if message["typestr"] == "LeButton":
                print("creating button from external request")
                uid = message["uid"]
                editable_button = self.currently_selected_editablevbox.addLeButton(uid)
            

    def find_widget(self, name=None, idx=None, uid=None, type_=None):
        count = 0
        for widget in self.all_editable_widgets:
            match_uid = uid is None or uid == widget.uid
            match_name = name is None or name == widget.name
            match_idx = idx is None or idx == count
            match_type = type_ is None or isinstance(widget, type_)
            if match_type:
                count += 1
            if match_uid and match_type and match_name and match_idx:
                return widget
        return None

    def count_widgets(self, type_=None):
        count = 0
        for widget in self.all_editable_widgets:
            match_type = type_ is None or isinstance(widget, type_)
            if match_type:
                count += 1
        return count

    def wait_find_widget(self, name=None, idx=None, uid=None, type_=None, timeout=None):
        time_waited = 0
        wait_increment = 0.1
        while True:
            found = self.find_widget(name=name, idx=idx, uid=uid, type_=type_)
            if found is not None:
                return found
            time.sleep(wait_increment)
            time_waited += wait_increment
            if timeout is not None and time_waited > timeout:
                print("Timed out when looking for button")
                return None

    def get_widget_service_call(self, req):
        widget_request = json.loads(req.question)
        if widget_request["typestr"] == "LeButton":
            type_ = LeButton
        else:
            raise NotImplementedError
        uid = widget_request["uid"]
        widget = self.find_widget(name=widget_request["name"], idx=widget_request["idx"], uid=uid, type_=type_)
        is_found = True
        if widget is None:
            if uid is None:
                uid = str(uuid.uuid4())
                self.message_queue.append({"action": "create_lewidget", "typestr": widget_request["typestr"], "uid": uid})
            else:
                is_found = False
        else:
            uid = widget.uid
        response_dict = {"is_found": is_found, "uid": uid, "label": widget.get_label()}
        response_str = json.dumps(response_dict)
        return QuestionAnswerResponse(response_str)

    def is_button_clicked_service_call(self, req):
        uid = req.question
        button = self.wait_find_widget(uid=uid, type_=LeButton, timeout=3.)
        if button is None:
            print("Button with requested uid {} not found.".format(uid))
            return QuestionAnswerResponse("not_found")
        answer_string = "true" if button.get_click_status() else "false"
        return QuestionAnswerResponse(answer_string)
        



if __name__ == "__main__":
    LeGUIServer()
    