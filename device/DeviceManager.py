import cv2

from PySide6 import QtCore
from PySide6.QtCore import QRunnable, QMutexLocker
from PySide6.QtGui import QImage

from RebarAlignment import *
from RebarTwist import *
from ShapeRecognition import *
from TuningView import *
from ManualTuningView import *
from ManualGripTuningView import *
from AligningView import *
from GripView import *

from MvCameraControl_class import *
from RunnableStreaming import *
from RunnableVideo import *

import time

class RebarLoadStatusManager(RunnableVideo):
    """ A class to show rebar load status demo
    """

    def __init__(self, manager, output, id, path):
        super(RebarLoadStatusManager, self).__init__(output, id, path)

    def process_frame(self, frame):
        cv2.waitKey(20)
        return frame

class DeviceScanner:
    """ A class to scan connected devices
    """

    def __init__(self):
        self.__device_list = None
        self.__scanned_serial_ids = []

    def get_device_list(self):
        return self.__device_list

    def get_scanned_serial_ids(self):
        return self.__scanned_serial_ids

    def num_found_device(self):
        return len(self.__scanned_serial_ids)

    def scan(self):
        self.__device_list = None
        self.__scanned_serial_ids = []

        device_list = MV_CC_DEVICE_INFO_LIST()
        tlayer_type = MV_GIGE_DEVICE

        ret = MvCamera.MV_CC_EnumDevices(tlayer_type, device_list)
        if ret != 0:
            print("enum devices fail! ret[0x%x]" % ret)
            return

        if device_list.nDeviceNum == 0:
            print("find no streaming device!")
            return

        for i in range(0, device_list.nDeviceNum):
            mvcc_dev_info = cast(device_list.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
            scanned_serial_id = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chSerialNumber:
                if per == 0:
                    break
                scanned_serial_id = scanned_serial_id + chr(per)

            if len(scanned_serial_id) > 0:
                self.__scanned_serial_ids.append(scanned_serial_id)

        self.__device_list = device_list


class DeviceManager:
    """ A class to manage all registered devices
    """

    def __init__(self, caller):
        self.caller_ = caller

        self.__device_scanner = DeviceScanner()

        self.__scanned_device_ids = []

        self.__dict_running_device_objs = {}

        self.__unknown_device_ids = []
        self.__dict_unknown_device_objs = {}

        self.__dict_registered_device_objs = {}

        self.__registered_device_ids = ["TE28010025", "TE28010026", "TE28010027", "TE28010028", "", ""]
        self.__registered_device_names = ["Rebar Alignment", "Rebar Twist", "Shape Recognition", "Rebar Load Status", "Aligning View", "Shaping View"]

        self.__current_coil_thickness = 10

        self.__start_out_cell_visible = True
        self.__start_out_cell_size = 50
        self.__start_out_cell_x = 542
        self.__start_out_cell_y = 425

        self.__start_in_cell_visible = True
        self.__start_in_cell_size = 50
        self.__start_in_cell_x = 542
        self.__start_in_cell_y = 475

        self.__end_out_cell_visible = False
        self.__end_out_cell_size = 40
        self.__end_out_cell_x = 572
        self.__end_out_cell_y = 740

        self.__end_in_cell_visible = False
        self.__end_in_cell_size = 40
        self.__end_in_cell_x = 572
        self.__end_in_cell_y = 780

        self.__alignment_interval = 1

        self.__aligning_view_fps = 30
        self.__aligning_view_expo = 1000
        self.__aligning_view_led_status = False

        self.__shaping_view_fps = 30
        self.__shaping_view_expo = 1000
        self.__shaping_view_led_status = False

        self.__grip_delta_x = 0
        self.__grip_delta_y = 0

        self.__grip_robot_delta_x = 0
        self.__grip_robot_delta_y = 0

        self.__list_current_grip_point = [0, 0, 0, 0, 0, 0]
        self.__list_80_rebar_out_data_length = []
        self.__list_80_rebar_in_data_length = []

        self.list_result_alignment_= []

        self.mutex_ = QtCore.QMutex()

    def check_running_device_by_id(self, id):
        ''' check running device

        :param id: device id
        :return:
        '''
        ids = list(self.__dict_running_device_objs.keys())
        if ids.index(id) > -1:
            return True
        else:
            return False

    def set_saved_device_info(self, list_device_id, list_device_name):
        ''' save device list of id and name to member private variables

        :param list_device_id: list of device ids
        :param list_device_name: list of device names
        :return:
        '''
        self.__registered_device_ids = list_device_id
        self.__registered_device_names = list_device_name

    def save_tuning_view_value(self):
        ''' save tuning view value(end in cell size, end out cell x, ...)

        :return:
        '''
        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            if self.__current_coil_thickness is self.__dict_running_device_objs[ids[0]].current_coil_thickness():
                self.__end_in_cell_size = self.__dict_running_device_objs[ids[0]].end_in_cell_size()
                self.__end_in_cell_x = self.__dict_running_device_objs[ids[0]].end_in_cell_x()
                self.__end_in_cell_y = self.__dict_running_device_objs[ids[0]].end_in_cell_y()
                self.__end_out_cell_size = self.__dict_running_device_objs[ids[0]].end_out_cell_size()
                self.__end_out_cell_x = self.__dict_running_device_objs[ids[0]].end_out_cell_x()
                self.__end_out_cell_y = self.__dict_running_device_objs[ids[0]].end_out_cell_y()

                str_log = "IN-CELLS(" + str(self.__end_in_cell_size) + ", " + str(self.__end_in_cell_x) + ", " + str(self.__end_in_cell_y) + "), "
                str_log += "OUT-CELLS(" + str(self.__end_in_cell_size) + ", " + str(self.__end_in_cell_x) + ", " + str(self.__end_in_cell_y) + ")"
                self.caller_.setLog(str_log)

    def current_coil_thickness(self):
        ''' get current coil thickness

        :return:
        '''
        return self.__current_coil_thickness

    def change_current_coil_thickness(self, id, value):
        ''' change current coil thickness temporarily because of OPC server.
            This doesn't update base coil thickness(from tuning)
        :param id: device id
        :param value: coil thickness(10, 13, 16)
        :return:
        '''
        ids = list(self.__dict_running_device_objs.keys())
        if id not in ids:
            return
        self.__dict_running_device_objs[id].change_current_coil_thickness(value)

    def update_current_coil_thickness(self, value):
        ''' update current coil thickness

        :param value: coil thickness
        :return:
        '''
        self.__current_coil_thickness = value
        index_id = self.registered_device_names().index("Aligning View")
        if index_id > -1:
            id_aligning_view = self.registered_device_ids()[index_id]
            self.change_current_coil_thickness(id_aligning_view, value)

    def update_device_id_using_name(self, id, name):
        ''' when sw start, ini file gives device id using name.
        it is just used at the first time.

        :param id: device id
        :param name: device name
        :return:
        '''
        index = self.__registered_device_names.index(name)
        if index > -1:
            self.__registered_device_ids[index] = id

    def update_led_status_at(self, id, value):
        ''' update led status using device id

        :param id: device id
        :param value: led status(true/false)
        :return:
        '''
        ids = list(self.__dict_running_device_objs.keys())
        if id not in ids:
            return

        self.__dict_running_device_objs[id].set_led_on(value)

    def update_led_status_by_name(self, name, value):
        ''' update led status using device name

        :param name: device name
        :param value: led status
        :return:
        '''
        if name == "Aligning View":
            self.set_aligning_view_led_status(value)
        elif name == "Shaping View":
            self.set_shaping_view_led_status(value)

        index = self.__registered_device_names.index(name)
        if index > -1:
            self.update_led_status_at(self.__registered_device_ids[index], value)

    def update_exposure_at(self, id, value):
        ''' update exposure using device id

        :param id: device id
        :param value: exposure value(1~20000)
        :return:
        '''
        ids = list(self.__dict_running_device_objs.keys())
        if id not in ids:
            return

        self.__dict_running_device_objs[id].set_exposure(value)

    def update_exposure_by_name(self, name, value):
        ''' update exposure using device name

        :param name: device name
        :param value: exposure value(1~20000)
        :return:
        '''
        if name == "Aligning View":
            self.set_aligning_view_expo(value)
        elif name == "Shaping View":
            self.set_shaping_view_expo(value)

        index = self.__registered_device_names.index(name)
        if index > -1:
            self.update_exposure_at(self.__registered_device_ids[index], value)

    def update_fps_at(self, id, value):
        ''' update fps using device id

        :param id: device id
        :param value: fps value(1~90)
        :return:
        '''
        ids = list(self.__dict_running_device_objs.keys())
        if id not in ids:
            return

        self.__dict_running_device_objs[id].set_fps(value)

    def update_fps_by_name(self, name, value):
        ''' update fps using device name

        :param name: device name
        :param value: fps value(1~90)
        :return:
        '''
        if name == "Aligning View":
            self.set_aligning_view_fps(value)
        elif name == "Shaping View":
            self.set_shaping_view_fps(value)

        index = self.__registered_device_names.index(name)
        if index > -1:
            self.update_fps_at(self.__registered_device_ids[index], value)

    def update_vision_status_at(self, id, value):
        ''' update vision status using device id

        :param id: device id
        :param value: true/false
        :return:
        '''
        ids = list(self.__dict_running_device_objs.keys())
        if id not in ids:
            return

        self.__dict_running_device_objs[id].set_vision_processing(value)

    def update_vision_status_by_name(self, name, value):
        ''' update vision status using device name

        :param name: device name
        :param value: true/false
        :return:
        '''
        index = self.__registered_device_names.index(name)
        if index > -1:
            self.update_vision_status_at(self.__registered_device_ids[index], value)

    def start_in_cell_visible(self):
        ''' getter of start in cell visible

        :return:
        '''
        return self.__start_in_cell_visible

    def update_start_in_cell_visible(self, value):
        ''' setter of start in cell visible

        :param value: true/false
        :return:
        '''
        self.__start_in_cell_visible = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_start_in_cell_visible(value)

    def start_in_cell_size(self):
        ''' getter of start in cell size

        :return:
        '''
        return self.__start_in_cell_size

    def update_start_in_cell_size(self, value):
        ''' setter of start in cell size

        :param value: 0~200
        :return:
        '''
        self.__start_in_cell_size = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_start_in_cell_size(value)

    def start_in_cell_x(self):
        ''' getter of start in cell x

        :return:
        '''
        return self.__start_in_cell_x

    def update_start_in_cell_x(self, value):
        ''' setter of start in cell x

        :param value: 0~1280
        :return:
        '''
        self.__start_in_cell_x = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_start_in_cell_x(value)

    def start_in_cell_y(self):
        ''' getter of start in cell y

        :return:
        '''
        return self.__start_in_cell_y

    def update_start_in_cell_y(self, value):
        ''' setter of start in cell y

        :param value: 0~1024
        :return:
        '''
        self.__start_in_cell_y = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_start_in_cell_y(value)

    def start_out_cell_visible(self):
        ''' getter of start out cell visible

        :return:
        '''
        return self.__start_out_cell_visible

    def update_start_out_cell_visible(self, value):
        ''' setter of start out cell visible

        :param value: true/false
        :return:
        '''
        self.__start_out_cell_visible = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_start_out_cell_visible(value)

    def start_out_cell_size(self):
        ''' getter of start out cell size

        :return:
        '''
        return self.__start_out_cell_size

    def update_start_out_cell_size(self, value):
        ''' setter of start out cell size

        :param value: 0~200
        :return:
        '''
        self.__start_out_cell_size = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_start_out_cell_size(value)

    def start_out_cell_x(self):
        ''' getter of start out cell x

        :return:
        '''
        return self.__start_out_cell_x

    def update_start_out_cell_x(self, value):
        ''' setter of start out cell x

        :param value: 0~1280
        :return:
        '''
        self.__start_out_cell_x = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_start_out_cell_x(value)

    def start_out_cell_y(self):
        ''' getter of start out cell y

        :return:
        '''
        return self.__start_out_cell_y

    def update_start_out_cell_y(self, value):
        ''' setter of start out cell y

        :param value: 0~1024
        :return:
        '''
        self.__start_out_cell_y = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_start_out_cell_y(value)

    def end_in_cell_visible(self):
        ''' getter of end in cell visible

        :return:
        '''
        return self.__end_in_cell_visible

    def update_end_in_cell_visible(self, value):
        ''' setter of end in cell visible

        :param value: true/false
        :return:
        '''
        self.__end_in_cell_visible = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_end_in_cell_visible(value)

    def end_in_cell_size(self):
        ''' getter of end in cell size

        :return:
        '''
        return self.__end_in_cell_size

    def update_end_in_cell_size(self, value):
        ''' setter of end in cell size

        :param value: 0~200
        :return:
        '''
        self.__end_in_cell_size = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_end_in_cell_size(value)

    def end_in_cell_x(self):
        ''' getter of end in cell x

        :return:
        '''
        return self.__end_in_cell_x

    def update_end_in_cell_x(self, value):
        ''' setter of end in cell x

        :param value: 0~1280
        :return:
        '''
        self.__end_in_cell_x = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_end_in_cell_x(value)

    def end_in_cell_y(self):
        ''' getter of end in cell y

        :return:
        '''
        return self.__end_in_cell_y

    def update_end_in_cell_y(self, value):
        ''' setter of end in cell y

        :param value: 0~1024
        :return:
        '''
        self.__end_in_cell_y = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_end_in_cell_y(value)

    def end_out_cell_visible(self):
        ''' getter of end out cell visible

        :return:
        '''
        return self.__end_out_cell_visible

    def update_end_out_cell_visible(self, value):
        ''' setter of end out cell visible

        :param value: true/false
        :return:
        '''
        self.__end_out_cell_visible = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_end_out_cell_visible(value)

    def end_out_cell_size(self):
        ''' getter of end out cell size

        :return:
        '''
        return self.__end_out_cell_size

    def update_end_out_cell_size(self, value):
        ''' setter of end out cell size

        :param value: 0~200
        :return:
        '''
        self.__end_out_cell_size = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_end_out_cell_size(value)

    def end_out_cell_x(self):
        ''' getter of end out cell x

        :return:
        '''
        return self.__end_out_cell_x

    def update_end_out_cell_x(self, value):
        ''' setter of end out cell x

        :param value: 0~1280
        :return:
        '''
        self.__end_out_cell_x = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_end_out_cell_x(value)

    def end_out_cell_y(self):
        ''' getter of end out cell y

        :return:
        '''
        return self.__end_out_cell_y

    def update_end_out_cell_y(self, value):
        ''' setter of end out cell y

        :param value: 0~1024
        :return:
        '''
        self.__end_out_cell_y = value

        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_end_out_cell_y(value)

    def alignment_interval(self):
        ''' getter of alignment interval

        :return:
        '''
        return self.__alignment_interval

    def update_alignment_interval(self, value):
        ''' setter of alignment interval

        :param value: 1(CONST.ALIGNMENT_INTERVAL_NOT_SEND), 2(CONST.ALIGNMENT_INTERVAL_SEND_ONCE_EVERY_THREE_TIMES), 3(CONST.ALIGNMENT_INTERVAL_SEND_ALWAYS)
        :return:
        '''
        self.__alignment_interval = value
        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_alignment_interval(value)

    def update_manual_current_out_cell_x(self, value):
        ''' update manual current out cell x

        :param value: 0~1280
        :return:
        '''
        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_manual_current_out_cell_x(value)

    def update_manual_current_out_cell_y(self, value):
        ''' update manual current out cell y

        :param value: 0~1024
        :return:
        '''
        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_manual_current_out_cell_y(value)

    def update_manual_current_in_cell_x(self, value):
        ''' update manual current in cell x

        :param value: 0~1280
        :return:
        '''
        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_manual_current_in_cell_x(value)

    def update_manual_current_in_cell_y(self, value):
        ''' update manual current in cell y

        :param value: 0~1024
        :return:
        '''
        ids = list(self.__dict_running_device_objs.keys())
        if len(ids) == 1:
            self.__dict_running_device_objs[ids[0]].update_manual_current_in_cell_y(value)

    def set_record_on(self, id, value):
        ''' activate/deactivate record function

        :param id: device id
        :param value: true/false
        :return:
        '''
        self.__dict_running_device_objs[id].set_record_on(value)

    def stop_device(self, id):
        ''' stop device

        :param id: device id
        :return:
        '''
        keys = self.__dict_running_device_objs.keys()
        if id in keys:
            self.__dict_running_device_objs[id].set_streaming(False)

    def get_list_running_device_objs(self):
        ''' return list of running device objs' keys

        :return:
        '''
        return list(self.__dict_running_device_objs.keys())

    def count_running_device_objs(self):
        ''' count running device objs using keys

        :return:
        '''
        return len(self.__dict_running_device_objs.keys())

    def get_device_scanner(self):
        ''' getter of device scanner

        :return:
        '''
        return self.__device_scanner

    def get_scanned_device_ids(self):
        ''' getter of scanned_device_ids

        :return:
        '''
        return self.__scanned_device_ids

    def get_grip_delta_values(self):
        ''' get the [grip delta x, grip delta y]

        :return:
        '''
        return [self.__grip_delta_x, self.__grip_delta_y]

    def get_grip_robot_delta_values(self):
        ''' get the [grip robot delta x, grip robot delta y]

        :return:
        '''
        return [self.__grip_robot_delta_x, self.__grip_robot_delta_y]

    def activate_alignment_value_checking(self):
        ''' (it is existed for Manual Tuning View)
        trigger for checking an alignment value

        :return:
        '''
        index = self.__registered_device_names.index("Aligning View")
        id = self.__registered_device_ids[index]
        self.__dict_running_device_objs[id].set_alignment_value_checking(True)

    def start_manual_grip_tuning(self):
        ''' robot-vision calibration start

        :return:
        '''
        index = self.__registered_device_names.index("Shaping View")
        id = self.__registered_device_ids[index]
        self.__dict_running_device_objs[id].set_started(True)

    def end_manual_grip_tuning(self):
        ''' start to analyze grip delta values for robot-vision calibration

        :return:
        '''
        index = self.__registered_device_names.index("Shaping View")
        id = self.__registered_device_ids[index]
        self.__dict_running_device_objs[id].set_ended(True)

    def generate_test_data_for_manual_grip_tuning(self, delta_x, delta_y):
        ''' generate test data and pass delta values for calculating grip robot delta values

        :param delta_x: the result delta x of robot-vision calibration
        :param delta_y: the result delta y of robot-vision calibration
        :return:
        '''
        index = self.__registered_device_names.index("Shaping View")
        id = self.__registered_device_ids[index]
        self.__dict_running_device_objs[id].set_delta_x(delta_x)
        self.__dict_running_device_objs[id].set_delta_y(delta_y)
        self.__dict_running_device_objs[id].set_tested(True)

    def get_tuning_values(self):
        ''' get all tuning values(end in cell visible,
        start out cell x, alignment interval, current coil thickness, ...)
        as list

        :return:
        '''
        ret = [self.__end_in_cell_visible,
               self.__end_in_cell_size,
               self.__end_in_cell_x,
               self.__end_in_cell_y,
               self.__end_out_cell_visible,
               self.__end_out_cell_size,
               self.__end_out_cell_x,
               self.__end_out_cell_y,
               self.__start_in_cell_visible,
               self.__start_in_cell_size,
               self.__start_in_cell_x,
               self.__start_in_cell_y,
               self.__start_out_cell_visible,
               self.__start_out_cell_size,
               self.__start_out_cell_x,
               self.__start_out_cell_y,
               self.__alignment_interval,
               self.__current_coil_thickness
               ]

        return ret

    def registered_device_ids(self):
        ''' getter of registered device ids

        :return:
        '''
        return self.__registered_device_ids

    def registered_device_names(self):
        ''' getter of registered device names

        :return:
        '''
        return self.__registered_device_names

    def aligning_view_fps(self):
        ''' getter of aligning view fps

        :return:
        '''
        return self.__aligning_view_fps

    def set_aligning_view_fps(self, value):
        ''' setter of aligning view fps

        :param value: 1~90
        :return:
        '''
        self.__aligning_view_fps = value

    def aligning_view_expo(self):
        ''' getter of aligning view exposure

        :return:
        '''
        return self.__aligning_view_expo

    def set_aligning_view_expo(self, value):
        ''' setter of aligning view exposure

        :param value: 1~20000
        :return:
        '''
        self.__aligning_view_expo = value

    def aligning_view_led_status(self):
        ''' getter of aligning view led status

        :return:
        '''
        return self.__aligning_view_led_status

    def set_aligning_view_led_status(self, value):
        ''' setter of aligning view led status

        :param value: true/false
        :return:
        '''
        self.__aligning_view_led_status = value

    def shaping_view_fps(self):
        ''' getter of shaping view fps

        :return:
        '''
        return self.__shaping_view_fps

    def set_shaping_view_fps(self, value):
        ''' setter of shaping view fps

        :param value: 1~90
        :return:
        '''
        self.__shaping_view_fps = value

    def shaping_view_expo(self):
        ''' getter of shaping view exposure

        :return:
        '''
        return self.__shaping_view_expo

    def set_shaping_view_expo(self, value):
        ''' setter of shaping view exposure

        :param value: 1~20000
        :return:
        '''
        self.__shaping_view_expo = value

    def shaping_view_led_status(self):
        ''' getter of shaping view led status

        :return:
        '''
        return self.__shaping_view_led_status

    def set_shaping_view_led_status(self, value):
        ''' setter of shaping view led status

        :param value: true/false
        :return:
        '''
        self.__shaping_view_led_status = value

    def list_current_grip_point(self):
        ''' getter of list current grip point
        grip point is handled as list([x, y, z, rx, ry, rz])
        robot x is matching for vision x
        robot z is matching for vision y

        :return:
        '''
        return self.__list_current_grip_point

    def set_list_current_grip_point(self, value):
        ''' setter of list current grip point
        grip point is handled as list([x, y, z, rx, ry, rz])
        robot x is matching for vision x
        robot z is matching for vision y

        :param value: list[x, y, z, rx, ry, rz]
        :return:
        '''
        self.__list_current_grip_point = value

    def list_result_alignment(self):
        ''' getter of list result alignment
        s -> roller
        list result alignment = [s1, s2, s3, s4, s5, s6, s7, s8, out_coil_cell, in_coil_cell]

        :return:
        '''
        return self.list_result_alignment_

    def set_list_result_alignment(self, value):
        ''' setter of list result alignment
        s -> roller
        list result alignment = [s1, s2, s3, s4, s5, s6, s7, s8, out_coil_cell, in_coil_cell]

        :param value: [s1, s2, s3, s4, s5, s6, s7, s8, out_coil_cell, in_coil_cell]
        :return:
        '''
        self.list_result_alignment_ = value

    def grip_get_background(self):
        ''' (deprecated) extract current frame as image

        :return:
        '''
        index = self.__registered_device_names.index("Shaping View")
        id = self.__registered_device_ids[index]
        self.__dict_running_device_objs[id].set_background_checking(True)

    def grip_get_coordinates(self):
        ''' (deprecated) By comparing bg, calculate grip points

        :return:
        '''
        index = self.__registered_device_names.index("Shaping View")
        id = self.__registered_device_ids[index]
        self.__dict_running_device_objs[id].set_coordinates_checking(True)

    def get_runnable_device(self, output, name, id):
        ''' return device obj which can be put in the thread pool

        :param output: frontend area for showing video frames
        :param name: device name to run
        :param id: device id
        :return:
        '''
        if name == "Rebar Twist":
            path = os.path.abspath("./device/CameraB/3.avi").replace("\\", "/")
            device = RebarTwistManager(self, output, id, path)
            device.signal_.streaming_done.connect(self.streaming_done)

            self.__dict_running_device_objs[id] = device

            return device
        elif name == "Rebar Alignment":
            path = os.path.abspath("./device/CameraA/test_vid/t80.mp4").replace("\\", "/")
            device = RebarAlignmentManager(self, output, id, path)
            device.signal_.streaming_done.connect(self.streaming_done)

            self.__dict_running_device_objs[id] = device

            return device
        elif name == "Shape Recognition":
            path = os.path.abspath("./device/CameraC/o-shape.mp4").replace("\\", "/")
            device = ShapeRecognitionManager(self, output, id, path)
            device.signal_.streaming_done.connect(self.streaming_done)

            self.__dict_running_device_objs[id] = device

            return device
        elif name == "Rebar Load Status":
            path = os.path.abspath("./device/CameraD/rebar_load_status.mp4").replace("\\", "/")
            device = RebarLoadStatusManager(self, output, id, path)
            device.signal_.streaming_done.connect(self.streaming_done)

            self.__dict_running_device_objs[id] = device

            return device
        elif name == "Aligning View":
            device_list = self.__device_scanner.get_device_list()
            scanned_serial_ids = self.__device_scanner.get_scanned_serial_ids()

            index_cam = scanned_serial_ids.index(id)
            device_info = device_list.pDeviceInfo[index_cam]

            device = AligningView(device_info, output, id, self.__current_coil_thickness, self.__end_in_cell_visible, self.__end_in_cell_size, self.__end_in_cell_x, self.__end_in_cell_y, self.__end_out_cell_visible, self.__end_out_cell_size, self.__end_out_cell_x, self.__end_out_cell_y, self.__start_in_cell_visible, self.__start_in_cell_size, self.__start_in_cell_x, self.__start_in_cell_y, self.__start_out_cell_visible, self.__start_out_cell_size, self.__start_out_cell_x, self.__start_out_cell_y)

            device.set_fps(self.aligning_view_fps())
            device.set_exposure(self.aligning_view_expo())
            device.set_led_on(self.aligning_view_led_status())
            device.update_alignment_interval(self.alignment_interval())

            if len(self.__list_80_rebar_out_data_length) > 2:
                device.set_base_out_data_length(np.median(self.__list_80_rebar_out_data_length))
                # device.set_base_out_data_length(235)

            if len(self.__list_80_rebar_in_data_length) > 2:
                device.set_base_in_data_length(np.median(self.__list_80_rebar_in_data_length))
                # device.set_base_in_data_length(230)

            if device.base_out_data_length() > 0:
                str_log = "BASE OUT DATA LENGTH : " + str(device.base_out_data_length())
            else:
                str_log = "BASE OUT DATA LENGTH : -1"

            str_log = str_log + ", "

            if device.base_in_data_length() > 0:
                str_log = str_log + "BASE IN DATA LENGTH : " + str(device.base_in_data_length())
            else:
                str_log = str_log + "BASE IN DATA LENGTH : -1"

            self.caller_.setLog(str_log)

            device.signal_.streaming_done.connect(self.streaming_aligning_view_done)
            device.signal_aligning_view_.alignment_work_done.connect(self.alignment_work_done)

            self.__dict_running_device_objs[id] = device

            return device
        elif name == "Shaping View":
            device_list = self.__device_scanner.get_device_list()
            scanned_serial_ids = self.__device_scanner.get_scanned_serial_ids()

            index_cam = scanned_serial_ids.index(id)
            device_info = device_list.pDeviceInfo[index_cam]

            device = GripView(device_info, output, id)
            device.set_fps(self.shaping_view_fps())
            device.set_exposure(self.shaping_view_expo())
            device.set_led_on(self.shaping_view_led_status())

            device.signal_.streaming_done.connect(self.streaming_done)
            device.signal_grip_view_.grip_dists_occurred.connect(self.grip_dists_occurred)

            self.__dict_running_device_objs[id] = device

            return device
        elif name == "Tuning View":
            device_list = self.__device_scanner.get_device_list()
            scanned_serial_ids = self.__device_scanner.get_scanned_serial_ids()

            index_cam = scanned_serial_ids.index(id)
            device_info = device_list.pDeviceInfo[index_cam]

            device = TuningView(device_info, output, id, self.__current_coil_thickness, self.__end_in_cell_visible, self.__end_in_cell_size, self.__end_in_cell_x, self.__end_in_cell_y, self.__end_out_cell_visible, self.__end_out_cell_size, self.__end_out_cell_x, self.__end_out_cell_y, self.__start_in_cell_visible, self.__start_in_cell_size, self.__start_in_cell_x, self.__start_in_cell_y, self.__start_out_cell_visible, self.__start_out_cell_size, self.__start_out_cell_x, self.__start_out_cell_y)

            device.set_fps(self.aligning_view_fps())
            device.set_exposure(self.aligning_view_expo())
            device.set_led_on(self.aligning_view_led_status())

            device.update_start_out_cell_visible(self.start_out_cell_visible())
            device.update_start_out_cell_size(self.start_out_cell_size())
            device.update_start_in_cell_visible(self.start_in_cell_visible())
            device.update_alignment_interval(self.alignment_interval())

            device.signal_.streaming_done.connect(self.streaming_done)
            device.signal_tuning_view_.alignment_work_done.connect(self.alignment_work_done)
            device.signal_tuning_view_.data_80_rebar_occurred.connect(self.data_80_rebar_occurred)

            self.__dict_running_device_objs[id] = device

            return device
        elif name == "Manual Tuning View":
            device_list = self.__device_scanner.get_device_list()
            scanned_serial_ids = self.__device_scanner.get_scanned_serial_ids()

            index_cam = scanned_serial_ids.index(id)
            device_info = device_list.pDeviceInfo[index_cam]

            device = ManualTuningView(device_info, output, id, self.__current_coil_thickness, self.__end_in_cell_visible, self.__end_in_cell_size, self.__end_in_cell_x, self.__end_in_cell_y, self.__end_out_cell_visible, self.__end_out_cell_size, self.__end_out_cell_x, self.__end_out_cell_y, self.__start_in_cell_visible, self.__start_in_cell_size, self.__start_in_cell_x, self.__start_in_cell_y, self.__start_out_cell_visible, self.__start_out_cell_size, self.__start_out_cell_x, self.__start_out_cell_y)

            device.set_fps(self.aligning_view_fps())
            device.set_exposure(self.aligning_view_expo())
            device.set_led_on(self.aligning_view_led_status())

            device.update_start_out_cell_visible(self.start_out_cell_visible())
            device.update_start_out_cell_size(self.start_out_cell_size())
            device.update_start_in_cell_visible(self.start_in_cell_visible())
            device.update_alignment_interval(self.alignment_interval())

            device.signal_.streaming_done.connect(self.streaming_done)
            device.signal_tuning_view_.alignment_work_done.connect(self.alignment_work_done)

            self.__dict_running_device_objs[id] = device

            return device
        elif name == "Manual Grip Tuning View":
            device_list = self.__device_scanner.get_device_list()
            scanned_serial_ids = self.__device_scanner.get_scanned_serial_ids()

            index_cam = scanned_serial_ids.index(id)
            device_info = device_list.pDeviceInfo[index_cam]

            device = ManualGripTuningView(device_info, output, id, self.__shaping_view_fps, self.__shaping_view_expo)

            device.signal_manual_grip_tuning_view_.delta_calculated.connect(self.grip_delta_calculated)
            device.signal_manual_grip_tuning_view_.robot_delta_calculated.connect(self.grip_robot_delta_calculated)
            device.signal_.streaming_done.connect(self.streaming_done)

            self.__dict_running_device_objs[id] = device

            return device
        elif name == "Unknown Device":
            device_list = self.__device_scanner.get_device_list()
            scanned_serial_ids = self.__device_scanner.get_scanned_serial_ids()

            index_cam = scanned_serial_ids.index(id)
            device_info = device_list.pDeviceInfo[index_cam]

            device = RunnableRGBStreaming(device_info, output, "Unknown Device", id)
            device.signal_.streaming_done.connect(self.streaming_done)

            self.__dict_running_device_objs[id] = device

            return device

    def streaming_aligning_view_done(self, id):
        ''' there are something to do for aligning view before streaming done
        deleting recorded files for monitoring services is necessary

        :param id: device id
        :return:
        '''
        # delete ] Alignment Processing, ] Alignment Done
        files = os.listdir()
        for str_file_name in files:
            if (str_file_name.find("] Alignment Processing.mp4") > -1) or (str_file_name.find("] Alignment Done.mp4") > -1):
                try:
                    os.remove(str_file_name)
                except OSError as error:
                    print(error)

        self.streaming_done(id)

    def streaming_done(self, id):
        ''' streaming is done
        pop from the thread pool is also occurred

        :param id: device id
        :return:
        '''
        self.__dict_running_device_objs.pop(id)
        self.caller_.streamingDone.emit(id)

    def data_80_rebar_occurred(self, out_total, in_total):
        ''' this is used for tuning view
        collect 800mm~880mm data length as base length
        it will be used for prediction

        :param out_total: out-coil data length
        :param in_total: in-coil data length
        :return:
        '''
        self.__list_80_rebar_out_data_length.append(out_total)
        self.__list_80_rebar_in_data_length.append(in_total)

        str_log = "BASE DATA COLLECTING(OUT): " + str(self.__list_80_rebar_out_data_length) + ""
        self.caller_.setLog(str_log)

        str_log = "BASE DATA COLLECTING(IN): " + str(self.__list_80_rebar_in_data_length) + ""
        self.caller_.setLog(str_log)

    def alignment_work_done(self, cells, values):
        ''' when alignment work done, there are some processes to do.
        uploading current status and sending the alignment data to OPC server.

        :param cells: the last poistion of rebar end point
        :param values: alignment values(string) for server
        :return:
        '''
        str_result = "CELLS(OUT, IN) : (" + str(cells[0]) + ", " + str(cells[1]) + "), Alignment values : " + values
        self.caller_.setLog(str_result)

        self.caller_.upload_status()

        if self.caller_.opc_manager_.get_opc_server_state() > 0:
            self.caller_.opc_manager_.send_using_str(values)
            self.caller_.setBackendAlignState("DataSent")

    def grip_dists_occurred(self, dist_x, dist_y):
        ''' when vision calculates the distance for robot grip,
        the distance is converted robot unit distance and it is sent to the robot client

        :param dist_x:
        :param dist_y:
        :return:
        '''
        if dist_x != 9999 and dist_y != 9999:
            robot_dist_x = 0
            robot_dist_y = 0

            if self.__grip_delta_x != 0:
                robot_dist_x = dist_x / self.__grip_delta_x

            if self.__grip_delta_y != 0:
                robot_dist_y = dist_y / self.__grip_delta_y * -1

            # Robocon only requests y value of robot unit
            # self.__list_current_grip_point[0] = robot_dist_x
            self.__list_current_grip_point[2] = robot_dist_y
            print("robot_dist_x : ", robot_dist_x, ", robot_dist_y : ", robot_dist_y)
        else:
            self.__list_current_grip_point[0] = 9999
            self.__list_current_grip_point[2] = 9999

        str_log = "X for the robot grip : " + str(self.__list_current_grip_point[0])
        self.caller_.setLog(str_log)
        str_log = "Y for the robot grip : " + str(self.__list_current_grip_point[2])
        self.caller_.setLog(str_log)

        self.caller_.send_data_to_robot()

    def grip_delta_calculated(self, list_data):
        ''' when grip delta is calculated by ManualGripTuningView
        It is saved to member private variables

        :param list_data: list of grip delta([x, y])
        :return:
        '''
        self.__grip_delta_x = float(list_data[0])
        self.__grip_delta_y = float(list_data[1])
        self.caller_.gripDeltaUpdated.emit()

    def save_grip_delta_value(self, delta_x, delta_y):
        '''  grip delta is also edited by User,
        so final values should be saved at the specific function(this)

        :param delta_x: the result x of robot-vision calibration
        :param delta_y: the result y of robot-vision calibration
        :return:
        '''
        self.__grip_delta_x = delta_x
        self.__grip_delta_y = delta_y

        str_log = "The result of robot-vision unit is (" + str(self.__grip_delta_x) + ", " + str(self.__grip_delta_y) + ")"
        self.caller_.setLog(str_log)

    def grip_robot_delta_calculated(self, list_data):
        ''' when grip robot delta is calculated by ManualGripTuningView
        grip robot delta is for test

        :param list_data: list of grip robot delta([x, y])
        :return:
        '''
        self.__grip_robot_delta_x = float(list_data[0])
        self.__grip_robot_delta_y = float(list_data[1])
        self.caller_.gripRobotDeltaUpdated.emit()

    def scan_devices(self):
        ''' scan devices

        :return:
        '''
        self.__scanned_device_ids = []
        self.__unknown_device_ids = []

        self.__device_scanner.scan()
        self.__scanned_device_ids = self.__device_scanner.get_scanned_serial_ids()

        for i, serial_id in enumerate(self.__scanned_device_ids):
            if serial_id not in self.__registered_device_ids:
                self.__unknown_device_ids.append(serial_id)

        # for demonstration - video versions are also included
        self.__scanned_device_ids.append("TE28010025")
        self.__scanned_device_ids.append("TE28010026")
        self.__scanned_device_ids.append("TE28010027")
        self.__scanned_device_ids.append("TE28010028")

        return len(self.__scanned_device_ids)
