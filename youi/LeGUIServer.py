# TODO: click to add button
from plistlib import UID
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
from PyQt5.QtWidgets import QWidget, QPushButton, QMainWindow, QLabel, QSlider, QCheckBox
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout
from PyQt5.QtWidgets import QMenu
from PyQt5.QtWidgets import QInputDialog
from PyQt5.QtGui import QPalette, QColor

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

import rospy

from common_srvs.srv import QuestionAnswer, QuestionAnswerResponse


    
class LeLayout(QWidget):
    DEPTH_COLORS = ["white", "gold", "cyan", "green", "black", "red"]
    """ An empty widget with vertical layout, can be right clicked to add objects inside """
    def __init__(self, uid, label, parent, leguiserver):
        raise NotImplementedError

    # def mousePressEvent(self, QMouseEvent):
    #     if QMouseEvent.button() == Qt.LeftButton:
    #         print("Left Button Clicked")
    #     elif QMouseEvent.button() == Qt.RightButton:
    #         #do what you want here
    #         print("Right Button Clicked")

    def contextMenuEvent(self, event):
        contextMenu = QMenu(self)
        empty = contextMenu.addAction(self.get_label())
        addBtnAct = contextMenu.addAction("Add Button")
        addPPlotAct = contextMenu.addAction("Add Pyplot")
        addCheckboxAct = contextMenu.addAction("Add Checkbox")
        addSliderAct = contextMenu.addAction("Add Slider")
        addLabelAct = contextMenu.addAction("Add Label")
        addVBoxAct = contextMenu.addAction("Add Vertical Layout")
        addHBoxAct = contextMenu.addAction("Add Horizontal Layout")
        rmSelfAct = contextMenu.addAction("Delete Layout")
        action = contextMenu.exec_(self.mapToGlobal(event.pos()))
        if action == addBtnAct:
            self.addLeWidget(LeButton, uid=None)
        if action == addVBoxAct:
            self.addLeWidget(LeVBox, uid=None)
        if action == addHBoxAct:
            self.addLeWidget(LeHBox, uid=None)
        if action == addPPlotAct:
            self.addLeWidget(LePyplot, uid=None)
        if action == addCheckboxAct:
            self.addLeWidget(LeCheckBox, uid=None)
        if action == addSliderAct:
            self.addLeWidget(LeSlider, uid=None)
        if action == addLabelAct:
            self.addLeWidget(LeLabel, uid=None)
        if action == rmSelfAct:
            self.parent.removeLeWidget(self)

    def addLeWidget(self, type_, uid=None):
        if uid is None:
            uid = str(uuid.uuid4())
        lewidget = type_(uid, '{} {}'.format(type_.__name__, self.leguiserver.count_widgets(type_=type_)), self, self.leguiserver)
        self.leguiserver.all_editable_widgets.append(lewidget)
        self.lewidgets.append(lewidget)
        self.layout().addWidget(lewidget)
        return lewidget

    def removeLeWidget(self, lewidget):
        self.leguiserver.all_editable_widgets.remove(lewidget)
        self.lewidgets.remove(lewidget)
        lewidget.close()

    def removeAllLeWidgets(self):
        for lewidget in self.lewidgets[:]:
            self.removeLeWidget(lewidget)

    def set_color(self, color):
        if color is not None:
            self.setAutoFillBackground(True)
            palette = self.palette()
            palette.setColor(QPalette.Window, QColor(color))
            self.setPalette(palette)
    
    def get_depth(self):
        return self.parent.get_depth() + 1

    def set_label(self, label):
        self.labelwidget = QLabel(text=label)
        self.layout().addWidget(self.labelwidget)

    def get_label(self):
        return self.labelwidget.text()

    def close(self):
        self.removeAllLeWidgets()
        super(LeLayout, self).close()


class LeMainLayout(LeLayout):
    def __init__(self, leguiserver):
        super(LeLayout, self).__init__()
        self.uid = "0"
        self.parent = None
        self.leguiserver = leguiserver
        self.lewidgets = []
        
        self.setLayout(QHBoxLayout())
        self.setAutoFillBackground(True)
        palette = self.palette()
        palette.setColor(QPalette.Window, QColor("white"))
        self.setPalette(palette)

    def contextMenuEvent(self, event):
        contextMenu = QMenu(self)
        addVBoxAct = contextMenu.addAction("Add Vertical Layout")
        addHBoxAct = contextMenu.addAction("Add Horizontal Layout")
        action = contextMenu.exec_(self.mapToGlobal(event.pos()))
        if action == addVBoxAct:
            self.addLeWidget(LeVBox, uid=None)
        if action == addHBoxAct:
            self.addLeWidget(LeHBox, uid=None)

    def get_depth(self):
        return 0

class LeVBox(LeLayout):
    def __init__(self, uid, label, parent, leguiserver):
        super(LeLayout, self).__init__()
        self.uid = uid
        self.parent = parent
        self.leguiserver = leguiserver
        self.setLayout(QVBoxLayout())
        self.lewidgets = []
        
        self.set_label(label)
        self.labelwidget.setAlignment(QtCore.Qt.AlignCenter)
        self.set_color(LeLayout.DEPTH_COLORS[self.get_depth()])

class LeHBox(LeLayout):
    def __init__(self, uid, label, parent, leguiserver):
        super(LeLayout, self).__init__()
        self.uid = uid
        self.parent = parent
        self.leguiserver = leguiserver
        self.setLayout(QHBoxLayout())
        self.lewidgets = []
        
        print("New Layout, depth {}".format(self.get_depth()))
        self.set_label(label)
        self.set_color(LeLayout.DEPTH_COLORS[self.get_depth()])       

class LeButton(QPushButton):
    def __init__(self, uid, label, parent, leguiserver):
        icon = None
        super(LeButton, self).__init__() 
        self.leguiserver = leguiserver
        self.parent = parent
        self.uid = uid
        self.set_label(label)
        # Button specific
        self.was_clicked = False

    def contextMenuEvent(self, event):
        contextMenu = QMenu(self)
        deleteAct = contextMenu.addAction("Delete")
        settextAct = contextMenu.addAction("Set Label")
        action = contextMenu.exec_(self.mapToGlobal(event.pos()))
        if action == deleteAct:
            self.parent.removeLeWidget(self)
        if action == settextAct:
            text, ok = QInputDialog.getText(self, 'Text Input Dialog', 'Enter new label:')
            if ok:
                self.set_label(text)
    
    def get_label(self):
        return self.text()

    def set_label(self, label):
        self.setText(label)

    # Button specific methods
    def on_click(self):
        self.was_clicked = True

    def get_click_status(self):
        status = self.was_clicked
        self.was_clicked = False
        return status

class LePyplot(FigureCanvasQTAgg):
    def __init__(self, uid, label, parent, leguiserver, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super(LePyplot, self).__init__(fig)
        # LeWidget generic init
        self.leguiserver = leguiserver
        self.parent = parent
        self.uid = uid
        self.set_label(label)

    def contextMenuEvent(self, event):
        contextMenu = QMenu(self)
        deleteAct = contextMenu.addAction("Delete")
        settextAct = contextMenu.addAction("Set Label")
        action = contextMenu.exec_(self.mapToGlobal(event.pos()))
        if action == deleteAct:
            self.parent.removeLeWidget(self)
        if action == settextAct:
            text, ok = QInputDialog.getText(self, 'Text Input Dialog', 'Enter new label:')
            if ok:
                self.set_label(text)
    
    def get_label(self):
        return self.axes.get_title()

    def set_label(self, label):
        self.axes.set_title(label)


class LeCheckBox(QCheckBox):
    def __init__(self, uid, label, parent, leguiserver):
        super(LeCheckBox, self).__init__()
        self.leguiserver = leguiserver
        self.parent = parent
        self.uid = uid
        self.set_label(label)

    def contextMenuEvent(self, event):
        contextMenu = QMenu(self)
        deleteAct = contextMenu.addAction("Delete")
        settextAct = contextMenu.addAction("Set Label")
        action = contextMenu.exec_(self.mapToGlobal(event.pos()))
        if action == deleteAct:
            self.parent.removeLeWidget(self)
        if action == settextAct:
            text, ok = QInputDialog.getText(self, 'Text Input Dialog', 'Enter new label:')
            if ok:
                self.set_label(text)
    
    def get_label(self):
        return self.text()

    def set_label(self, label):
        self.setText(label)

class LeLabel(QLabel):
    def __init__(self, uid, label, parent, leguiserver):
        super(LeLabel, self).__init__()
        self.leguiserver = leguiserver
        self.parent = parent
        self.uid = uid
        self.set_label(label)

    def contextMenuEvent(self, event):
        contextMenu = QMenu(self)
        deleteAct = contextMenu.addAction("Delete")
        settextAct = contextMenu.addAction("Set Label")
        action = contextMenu.exec_(self.mapToGlobal(event.pos()))
        if action == deleteAct:
            self.parent.removeLeWidget(self)
        if action == settextAct:
            text, ok = QInputDialog.getText(self, 'Text Input Dialog', 'Enter new label:')
            if ok:
                self.set_label(text)
    
    def get_label(self):
        return self.text()

    def set_label(self, label):
        self.setText(label)

class LeSlider(QSlider):
    def __init__(self, uid, label, parent, leguiserver):
        super(LeSlider, self).__init__()
        self.leguiserver = leguiserver
        self.parent = parent
        self.uid = uid
        self.set_label(label)

    def contextMenuEvent(self, event):
        contextMenu = QMenu(self)
        deleteAct = contextMenu.addAction("Delete")
        settextAct = contextMenu.addAction("Set Label")
        action = contextMenu.exec_(self.mapToGlobal(event.pos()))
        if action == deleteAct:
            self.parent.removeLeWidget(self)
        if action == settextAct:
            text, ok = QInputDialog.getText(self, 'Text Input Dialog', 'Enter new label:')
            if ok:
                self.set_label(text)
    
    def get_label(self):
        return self.label

    def set_label(self, label):
        self.label = label

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
        # set window size
        self.mainwindow.resize(800, 600)
        self.mainwindow.show()
        btn = QPushButton('Edit', self.mainwindow)
        btn.clicked.connect(self.toggle_edit_mode)
        self.mainwindow.statusBar().addPermanentWidget(btn)
        btn = QPushButton('Save', self.mainwindow)
        self.mainwindow.statusBar().addPermanentWidget(btn)
        layout = QVBoxLayout()

        mainlayout = LeMainLayout(self)
        self.mainlayout = mainlayout
        self.mainwindow.setCentralWidget(mainlayout)

        if True:
            editable = self.mainlayout.addLeWidget(LeVBox)
            editable.addLeWidget(LeButton)
            editable.addLeWidget(LeButton)
            editable.addLeWidget(LeSlider)
            editable.addLeWidget(LeCheckBox)
            editable.addLeWidget(LeLabel)
            self.currently_selected_editablevbox = editable

            editable2 = self.mainlayout.addLeWidget(LeVBox)
            editable2.addLeWidget(LePyplot)



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
    