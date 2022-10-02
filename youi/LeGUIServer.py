# TODO: click to add button
import sys
import time
import uuid
import json
from tkinter import Button
import matplotlib
from matplotlib import widgets
matplotlib.use('Qt5Agg')

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QWidget, QPushButton, QMainWindow, QLabel, QSlider, QCheckBox, QLineEdit
from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout
from PyQt5.QtWidgets import QMenu
from PyQt5.QtWidgets import QInputDialog
from PyQt5.QtGui import QPalette, QColor

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

HIBYE = True
if HIBYE:
    import hibye
else:
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
        addValueAct = contextMenu.addAction("Add Value")
        addSliderAct = contextMenu.addAction("Add Slider")
        addTextFieldAct = contextMenu.addAction("Add TextField")
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
            self.addLeWidget(LeToggle, uid=None)
        if action == addSliderAct:
            self.addLeWidget(LeSlider, uid=None)
        if action == addTextFieldAct:
            self.addLeWidget(LeTextField, uid=None)
        if action == addValueAct:
            self.addLeWidget(LeValue, uid=None)
        if action == rmSelfAct:
            self.parent.removeLeWidget(self)

    def get_currently_selected_layout(self):
        for lewidget in self.lewidgets[::-1]:
            if isinstance(lewidget, LeLayout):
                return lewidget.get_currently_selected_layout()
        return self

    def addLeWidget(self, type_, uid=None):
        if uid is None:
            uid = str(uuid.uuid4())
        lewidget = type_(uid, '{} {}'.format(type_.__name__, self.leguiserver.count_widgets(type_=type_)), self, self.leguiserver)
        self.leguiserver.all_editable_widgets.append(lewidget)
        self.lewidgets.append(lewidget)
        self.layout().addWidget(lewidget.get_qwidget())
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

    def get_qwidget(self):
        return self

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

    def addLeWidget(self, type_, uid=None):
        # if type is not a LeLayout, and we don't have any children layouts, create one
        if not issubclass(type_, LeLayout):
            surrogate_layout = self.get_currently_selected_layout()
            if surrogate_layout == self:
                surrogate_layout = self.addLeWidget(LeVBox, uid=None)
            return surrogate_layout.addLeWidget(type_, uid=uid)
        return super(LeMainLayout, self).addLeWidget(type_, uid=uid)

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

class LeWidget(object):
    def __init__(self, uid, label, parent, leguiserver):
        self.create_qwidget()
        self.leguiserver = leguiserver
        self.parent = parent
        self.uid = uid
        self.set_label(label)
        self.get_qwidget().contextMenuEvent = self.contextMenuEvent

    def contextMenuEvent(self, event):
        contextMenu = QMenu(self.get_qwidget())
        empty = contextMenu.addAction(self.get_label())
        deleteAct = contextMenu.addAction("Delete")
        settextAct = contextMenu.addAction("Set Label")
        action = contextMenu.exec_(self.get_qwidget().mapToGlobal(event.pos()))
        if action == deleteAct:
            self.parent.removeLeWidget(self)
        if action == settextAct:
            text, ok = QInputDialog.getText(self.get_qwidget(), 'Text Input Dialog', 'Enter new label:')
            if ok:
                self.set_label(text)

    def create_qwidget(self):
        raise NotImplementedError

    def get_qwidget(self):
        raise NotImplementedError

    def get_label(self):
        raise NotImplementedError

    def set_label(self, label):
        raise NotImplementedError

    def close(self):
        self.get_qwidget().close()

class LeButton(LeWidget):
    def create_qwidget(self):
        self.qwidget = QPushButton()
        self.qwidget.clicked.connect(self.on_click)
        self.was_clicked = False

    def get_qwidget(self):
        return self.qwidget

    def get_label(self):
        return self.qwidget.text()

    def set_label(self, label):
        self.qwidget.setText(label)

    def on_click(self):
        self.was_clicked = True

    def get_click_status(self):
        status = self.was_clicked
        self.was_clicked = False
        return status

class LePyplot(LeWidget):
    def create_qwidget(self):
        fig = Figure(figsize=(5, 4), dpi=100)
        self.axes = fig.add_subplot(111)
        self.qwidget = FigureCanvasQTAgg(fig)

    def get_qwidget(self):
        return self.qwidget

    def get_label(self):
        return self.axes.get_title()

    def set_label(self, label):
        self.axes.set_title(label)


class LeToggle(LeWidget):
    def create_qwidget(self):
        self.qwidget = QCheckBox()

    def get_qwidget(self):
        return self.qwidget

    def get_label(self):
        return self.qwidget.text()

    def set_label(self, label):
        self.qwidget.setText(label)

    def get_enabled_status(self):
        status = self.qwidget.isChecked()
        return status

class LeValue(LeWidget):
    def create_qwidget(self):
        self.qwidget = QWidget()
        self.qwidget.setLayout(QHBoxLayout())
        self.labelqwidget = QLabel()
        self.valueqwidget = QLabel(text="None")
        self.qwidget.layout().addWidget(self.labelqwidget)
        self.qwidget.layout().addWidget(QLabel(text=":"))
        self.qwidget.layout().addWidget(self.valueqwidget)

    def get_qwidget(self):
        return self.qwidget

    def get_label(self):
        return self.labelqwidget.text()

    def set_label(self, label):
        self.labelqwidget.setText(label)

class LeSlider(LeWidget):
    def create_qwidget(self):
        self.qwidget = QWidget()
        self.qwidget.setLayout(QHBoxLayout())
        self.labelqwidget = QLabel()
        self.sliderqwidget = QSlider()
        show_value_qwidget = QLabel()
        self.sliderqwidget.valueChanged.connect(lambda: show_value_qwidget.setText(str(self.get_value())))
        self.show_min_qwidget = QLabel()
        self.show_max_qwidget = QLabel()
        slider_layout = QWidget()
        slider_layout.setLayout(QVBoxLayout())
        slider_layout.layout().addWidget(self.show_max_qwidget)
        slider_layout.layout().addWidget(self.sliderqwidget)
        slider_layout.layout().addWidget(self.show_min_qwidget)
        self.qwidget.layout().addWidget(self.labelqwidget)
        self.qwidget.layout().addWidget(QLabel(text=":"))
        self.qwidget.layout().addWidget(show_value_qwidget)
        self.qwidget.layout().addWidget(slider_layout)
        self.n_ticks = 100
        self.set_minimum(0.)
        self.set_maximum(1.)
        self.sliderqwidget.setMinimum(0)
        self.sliderqwidget.setMaximum(self.n_ticks)
        show_value_qwidget.setText(str(self.get_value()))


    def get_qwidget(self):
        return self.qwidget

    def get_label(self):
        return self.labelqwidget.text()

    def set_label(self, label):
        self.labelqwidget.setText(label)

    def set_minimum(self, min):
        self.min = min
        self.show_min_qwidget.setText(str(min))

    def set_maximum(self, max):
        self.max = max
        self.show_max_qwidget.setText(str(max))

    def set_n_ticks(self, n):
        self.n_ticks = n
        self.sliderqwidget.setMaximum(n)

    def get_value(self):
        return self.sliderqwidget.value() * 1.0 / self.n_ticks * (self.max - self.min) + self.min


class LeTextField(LeWidget):
    def create_qwidget(self):
        self.qwidget = QWidget()
        self.qwidget.setLayout(QHBoxLayout())
        self.labelqwidget = QLabel()
        self.fieldqwidget = QLineEdit()
        self.qwidget.layout().addWidget(self.labelqwidget)
        self.qwidget.layout().addWidget(self.fieldqwidget)

    def get_qwidget(self):
        return self.qwidget

    def get_label(self):
        return self.labelqwidget.text()

    def set_label(self, label):
        self.labelqwidget.setText(label)

class LeGUIServer(object):
    def __init__(self, path=None):
        if HIBYE:
            self.service_server = hibye.ServiceServer(run_in_main_thread=True, verbose=True)
        else:
            rospy.init_node('LeGUIServer')
        self.all_editable_widgets = []
        self.message_queue = []
        self.edit_mode_enabled = False

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
            editable.addLeWidget(LeToggle)
            editable.addLeWidget(LeValue)
            editable.addLeWidget(LeSlider)
            editable.addLeWidget(LeTextField)

            editable2 = self.mainlayout.addLeWidget(LeVBox)
            editable2.addLeWidget(LePyplot)



        if HIBYE:
            self.service_server.advertise("get_widget", self.get_widget_service_call)
            self.service_server.advertise("is_button_clicked", self.is_button_clicked_service_call)
            self.service_server.advertise("is_toggle_enabled", self.is_toggle_enabled_service_call)
        else:
            s = rospy.Service('get_widget', QuestionAnswer, self.get_widget_service_call)
            s = rospy.Service('is_button_clicked', QuestionAnswer, self.is_button_clicked_service_call)

        # loop in main thread for checking incoming messages
        timer = QtCore.QTimer()
        timer.timeout.connect(self.check_messages)
        timer.start(500)

        app.exec_()


    def get_currently_selected_layout(self):
        return self.mainlayout.get_currently_selected_layout()

    def toggle_edit_mode(self):
        self.edit_mode_enabled = not self.edit_mode_enabled

    def check_messages(self):
        print("checking")
        if HIBYE:
            self.service_server.run_once()
        if len(self.message_queue) > 0:
            message = self.message_queue.pop()
            self.handle(message)
        return True

    def handle(self, message):
        if message["action"] == "create_lewidget":
            if message["typestr"] == "LeButton":
                print("creating button from external request")
                uid = message["uid"]
                editable_button = self.get_currently_selected_layout().addLeWidget(LeButton, uid=uid)


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
        if HIBYE:
            widget_request_str = req
        else:
            widget_request_str = req.question
        widget_request = json.loads(widget_request_str)
        if widget_request["typestr"] == "LeButton":
            type_ = LeButton
        elif widget_request["typestr"] == "LeToggle":
            type_ = LeToggle
        else:
            raise NotImplementedError("Requested widget of unknown type {}".format(widget_request["typestr"]))
        uid = widget_request["uid"]
        widget = self.find_widget(name=widget_request["name"], idx=widget_request["idx"], uid=uid, type_=type_)
        is_found = True
        if widget is None:
            if uid is None:
                uid = str(uuid.uuid4())
                label = None
                self.message_queue.append({"action": "create_lewidget", "typestr": widget_request["typestr"], "uid": uid})
            else:
                is_found = False
        else:
            uid = widget.uid
            label = widget.get_label()
        response_dict = {"is_found": is_found, "uid": uid, "label": label}
        response_str = json.dumps(response_dict)
        if HIBYE:
            resp = response_str
        else:
            resp = QuestionAnswerResponse(response_str)
        return resp

    def is_button_clicked_service_call(self, req):
        def predicate(button):
            return button.get_click_status()
        return self.is_widget_in_given_state_service_call(req, LeButton, predicate)

    def is_toggle_enabled_service_call(self, req):
        def predicate(toggle):
            return toggle.get_enabled_status()
        return self.is_widget_in_given_state_service_call(req, LeToggle, predicate)

    def is_widget_in_given_state_service_call(self, req, widget_type, predicate):
        """
        req: the request string containing the widget uid
        widget_type: type of widget (LeButton, etc)
        predicate: a callable (function) which returns a boolean
        """
        if HIBYE:
            uid = req
        else:
            uid = req.question
        widget = self.wait_find_widget(uid=uid, type_=widget_type, timeout=3.)
        if widget is None:
            print("{} with requested uid {} not found.".format(widget_type, uid))
            answer_string = "not_found"
        else:
            answer_string = "true" if predicate(widget) else "false"
        if HIBYE:
            resp = answer_string
        else:
            resp = QuestionAnswerResponse(answer_string)
        return resp




if __name__ == "__main__":
    LeGUIServer()
