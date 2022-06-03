# This Python file uses the following encoding: utf-8
import os
import sys

from PySide6.QtCore import QObject, Slot, QThreadPool, Signal, Property, QMutexLocker, QUrl, QPoint, QRunnable, Qt, \
    QEvent
from PySide6.QtGui import QGuiApplication, QImage
from PySide6.QtQml import qmlRegisterType

from PySide6 import QtCore
from PySide6.QtQuick import QQuickItem, QQuickPaintedItem, QQuickView

from server.RobotManager import RobotManager
from server.OPCManager import GSAManager
from server.FileUploadManager import FileUploadManager
from device.DeviceManager import DeviceManager
import configparser


class BackendManager(QQuickItem):
    """ A class to manage whole backend components
    """

    def __init__(self):
        super(BackendManager, self).__init__()
        self.pool = QThreadPool().globalInstance()
        # self.pool.setMaxThreadCount(10) # can limit the number of threads
        self.mutex = QtCore.QMutex()

        self.device_manager_ = DeviceManager(self)
        self.opc_manager_ = GSAManager(self)
        self.robot_manager_ = RobotManager(self)

        self.log_ = ""

        self.__backendAlignState = None
        self.__backendRobotState = None

        self.current_coil_thickness_ = -1
        self.__current_barcode = None
        self.__current_rebar_shape = None

        self.__pf_comp_no = -1
        self.__pf_eqp_no = -1

    def getBackendAlignState(self):
        ''' component function for Property

        :return:
        '''
        locker = QMutexLocker(self.mutex)
        return self.__backendAlignState

    def setBackendAlignState(self, value):
        ''' component function for Property

        :param string value: state for aligning view
        :return:
        '''
        self.__backendAlignState = value
        self.backendAlignStateChanged.emit()

    def getBackendRobotState(self):
        ''' component function for Property

        :return:
        '''
        locker = QMutexLocker(self.mutex)
        return self.__backendRobotState

    def setBackendRobotState(self, value):
        ''' component function for Property

        :param value: state for shaping view
        :return:
        '''
        self.__backendRobotState = value
        self.backendRobotStateChanged.emit()

    def getLog(self):
        ''' getter of self.log_

        :return:
        '''
        locker = QMutexLocker(self.mutex)
        return self.log_

    def setLog(self, value):
        ''' setter of self.log_

        :param value:
        :return:
        '''
        self.log_ = value
        self.logChanged.emit()

    def current_coil_thickness(self):
        ''' getter of self.current_coil_thickness_

        :return:
        '''
        return self.current_coil_thickness_

    def change_current_coil_thickness(self, value):
        '''
        change current coil thickness when data received from OPC server.

        this is not set_current_coil_thickness.

        it leads the temporary change of grid cell map size during production,
        base coil thickness is not changed

        :param value:
        :return:
        '''
        value = int(value)
        index_id = self.device_manager_.registered_device_names().index("Aligning View")
        if index_id > -1:
            id_aligning_view = self.device_manager_.registered_device_ids()[index_id]
            self.device_manager_.change_current_coil_thickness(id_aligning_view, value)
            self.current_coil_thickness_ = value

    def current_barcode(self):
        ''' getter of self.__current_barcode

        :return:
        '''
        return self.__current_barcode

    def set_current_barcode(self, value):
        ''' setter of self.__current_barcode

        :param value:
        :return:
        '''
        self.__current_barcode = value

    def current_rebar_shape(self):
        ''' getter of self.__current_rebar_shape

        :return:
        '''
        return self.__current_rebar_shape

    def set_current_rebar_shape(self, value):
        ''' setter of self.__current_rebar_shape

        :param value:
        :return:
        '''
        self.__current_rebar_shape = value

    def pf_comp_no(self):
        ''' getter of self.__pf_comp_no

        :return:
        '''
        return self.__pf_comp_no

    def set_pf_comp_no(self, value):
        ''' setter of self.__pf_comp_no

        :param value:
        :return:
        '''
        self.__pf_comp_no = value

    def pf_eqp_no(self):
        ''' getter of self.__pf_eqp_no

        :return:
        '''
        return self.__pf_eqp_no

    def set_pf_eqp_no(self, value):
        ''' setter of self.__pf_eqp_no

        :param value:
        :return:
        '''
        self.__pf_eqp_no = value

    def getDeviceIDList(self):
        ''' get scanned device ids from device_manager_

        :return:
        '''
        return self.device_manager_.get_scanned_device_ids()

    def send_data_to_robot(self):
        ''' send grip point to the connected robot

        :return:
        '''
        self.robot_manager_.send_grip_point(self.device_manager_.list_current_grip_point())
        self.device_manager_.set_list_current_grip_point([0, 0, 0, 0, 0, 0])
        self.setBackendRobotState("DataSent")

    def load_ini(self):
        ''' load setting information from the gsa2.ini file

        :return:
        '''
        config = configparser.ConfigParser()
        config.read('gsa2.ini')

        if 'Monitoring' in config:
            temp_pf_comp_no = int(config['Monitoring']['pf_comp_no']) if 'pf_comp_no' in config['Monitoring'] else -1
            temp_pf_eqp_no = int(config['Monitoring']['pf_eqp_no']) if 'pf_eqp_no' in config['Monitoring'] else -1
            self.set_pf_comp_no(temp_pf_comp_no)
            self.set_pf_eqp_no(temp_pf_eqp_no)

        if 'AligningView' in config:
            av_fps = config['AligningView']['FPS'] if 'FPS' in config['AligningView'] else self.device_manager_.aligning_view_fps()
            av_exposure = config['AligningView']['Exposure'] if 'Exposure' in config['AligningView'] else self.device_manager_.aligning_view_expo()
            av_led_on = config['AligningView']['LED_ON'] if 'LED_ON' in config['AligningView'] else False

            av_id = config['AligningView']['ID'] if 'ID' in config['AligningView'] else ""

            av_base_coil_thickness = config['AligningView']['BASE_COIL_THICKNESS'] if 'BASE_COIL_THICKNESS' in config['AligningView'] else self.device_manager_.current_coil_thickness()

            av_start_out_cell_visible = config['AligningView']['START_OUT_CELL_VISIBLE'] if 'START_OUT_CELL_VISIBLE' in config['AligningView'] else self.device_manager_.start_out_cell_visible()
            av_start_out_cell_size = config['AligningView']['START_OUT_CELL_SIZE'] if 'START_OUT_CELL_SIZE' in config['AligningView'] else self.device_manager_.start_out_cell_size()
            av_start_out_cell_x = config['AligningView']['START_OUT_CELL_X'] if 'START_OUT_CELL_X' in config['AligningView'] else self.device_manager_.start_out_cell_x()
            av_start_out_cell_y = config['AligningView']['START_OUT_CELL_Y'] if 'START_OUT_CELL_Y' in config['AligningView'] else self.device_manager_.start_out_cell_y()

            av_start_in_cell_visible = config['AligningView']['START_IN_CELL_VISIBLE'] if 'START_IN_CELL_VISIBLE' in config['AligningView'] else self.device_manager_.start_in_cell_visible()
            av_start_in_cell_size = config['AligningView']['START_IN_CELL_SIZE'] if 'START_IN_CELL_SIZE' in config['AligningView'] else self.device_manager_.start_in_cell_size()
            av_start_in_cell_x = config['AligningView']['START_IN_CELL_X'] if 'START_IN_CELL_X' in config['AligningView'] else self.device_manager_.start_in_cell_x()
            av_start_in_cell_y = config['AligningView']['START_IN_CELL_Y'] if 'START_IN_CELL_Y' in config['AligningView'] else self.device_manager_.start_in_cell_y()

            av_end_out_cell_visible = config['AligningView']['END_OUT_CELL_VISIBLE'] if 'END_OUT_CELL_VISIBLE' in config['AligningView'] else self.device_manager_.end_out_cell_visible()
            av_end_out_cell_size = config['AligningView']['END_OUT_CELL_SIZE'] if 'END_OUT_CELL_SIZE' in config['AligningView'] else self.device_manager_.end_out_cell_size()
            av_end_out_cell_x = config['AligningView']['END_OUT_CELL_X'] if 'END_OUT_CELL_X' in config['AligningView'] else self.device_manager_.end_out_cell_x()
            av_end_out_cell_y = config['AligningView']['END_OUT_CELL_Y'] if 'END_OUT_CELL_Y' in config['AligningView'] else self.device_manager_.end_out_cell_y()

            av_end_in_cell_visible = config['AligningView']['END_IN_CELL_VISIBLE'] if 'END_IN_CELL_VISIBLE' in config['AligningView'] else self.device_manager_.end_in_cell_visible()
            av_end_in_cell_size = config['AligningView']['END_IN_CELL_SIZE'] if 'END_IN_CELL_SIZE' in config['AligningView'] else self.device_manager_.end_in_cell_size()
            av_end_in_cell_x = config['AligningView']['END_IN_CELL_X'] if 'END_IN_CELL_X' in config['AligningView'] else self.device_manager_.end_in_cell_x()
            av_end_in_cell_y = config['AligningView']['END_IN_CELL_Y'] if 'END_IN_CELL_Y' in config['AligningView'] else self.device_manager_.end_in_cell_y()

            av_alignment_interval = config['AligningView']['ALIGNMENT_INTERVAL'] if 'ALIGNMENT_INTERVAL' in config['AligningView'] else self.device_manager_.alignment_interval()

            self.device_manager_.set_aligning_view_fps(float(av_fps))
            self.device_manager_.set_aligning_view_expo(float(av_exposure))
            self.device_manager_.set_aligning_view_led_status(bool(av_led_on))

            self.device_manager_.update_device_id_using_name(av_id, "Aligning View")

            self.device_manager_.update_current_coil_thickness(int(av_base_coil_thickness))

            self.device_manager_.update_start_out_cell_visible(str(av_start_out_cell_visible) == "True")
            self.device_manager_.update_start_out_cell_size(int(av_start_out_cell_size))
            self.device_manager_.update_start_out_cell_x(int(av_start_out_cell_x))
            self.device_manager_.update_start_out_cell_y(int(av_start_out_cell_y))

            self.device_manager_.update_start_in_cell_visible(str(av_start_in_cell_visible) == "True")
            self.device_manager_.update_start_in_cell_size(int(av_start_in_cell_size))
            self.device_manager_.update_start_in_cell_x(int(av_start_in_cell_x))
            self.device_manager_.update_start_in_cell_y(int(av_start_in_cell_y))

            self.device_manager_.update_end_out_cell_visible(str(av_end_out_cell_visible) == "True")
            self.device_manager_.update_end_out_cell_size(int(av_end_out_cell_size))
            self.device_manager_.update_end_out_cell_x(int(av_end_out_cell_x))
            self.device_manager_.update_end_out_cell_y(int(av_end_out_cell_y))

            self.device_manager_.update_end_in_cell_visible(str(av_end_in_cell_visible) == "True")
            self.device_manager_.update_end_in_cell_size(int(av_end_in_cell_size))
            self.device_manager_.update_end_in_cell_x(int(av_end_in_cell_x))
            self.device_manager_.update_end_in_cell_y(int(av_end_in_cell_y))

            self.device_manager_.update_alignment_interval((int(av_alignment_interval)))

        if 'ShapingView' in config:
            sv_fps = config['ShapingView']['FPS'] if 'FPS' in config['ShapingView'] else self.device_manager_.shaping_view_fps()
            sv_exposure = config['ShapingView']['Exposure'] if 'Exposure' in config['ShapingView'] else self.device_manager_.shaping_view_expo()

            sv_id = config['ShapingView']['ID'] if 'ID' in config['ShapingView'] else ""

            self.device_manager_.set_shaping_view_fps(float(sv_fps))
            self.device_manager_.set_shaping_view_expo(float(sv_exposure))

            self.device_manager_.update_device_id_using_name(sv_id, "Shaping View")

        if 'Server' in config:
            opc_ip = config['Server']['OPC_IP'] if 'OPC_IP' in config['Server'] else self.opc_manager_.ip()
            opc_port = config['Server']['OPC_PORT'] if 'OPC_PORT' in config['Server'] else self.opc_manager_.port()
            opc_auto = config['Server']['OPC_AUTO_CONNECT'] if 'OPC_AUTO_CONNECT' in config['Server'] else self.opc_manager_.auto_connection()

            robot_ip = config['Server']['ROBOT_IP'] if 'ROBOT_IP' in config['Server'] else self.robot_manager_.ip()
            robot_port = config['Server']['ROBOT_PORT'] if 'ROBOT_PORT' in config['Server'] else self.robot_manager_.port()
            robot_auto = config['Server']['ROBOT_AUTO_CONNECT'] if 'ROBOT_AUTO_CONNECT' in config['Server'] else self.robot_manager_.auto_connection()

            self.opc_manager_.set_ip(opc_ip)
            self.opc_manager_.set_port(opc_port)
            self.opc_manager_.set_auto_connection(str(opc_auto) == "True")

            self.robot_manager_.set_ip(robot_ip)
            self.robot_manager_.set_port(robot_port)
            self.robot_manager_.set_auto_connection(str(robot_auto) == "True")

    def upload_status(self):
        ''' check monitoring files for uploading.
        if there are 'Alignment Done' files, then the file name is changed to 'Upload'.
        And, the name of Upload is uploaded by FileUploadManager

        :return:
        '''

        files = os.listdir()
        path_current = os.getcwd() + "/"
        path_current = path_current.replace("\\", "/")
        for str_filename in files:
            if str_filename.find("] Alignment Done") > -1:
                list_str_filename = str_filename.split("] ")

                file_name = str_filename
                file_name_updated = list_str_filename[0] + "] Upload.mp4"
                os.rename(path_current+file_name, path_current+file_name_updated)

                file_path = path_current + file_name_updated
                barcode_current = self.current_barcode()
                time_start = list_str_filename[0][1:]
                file_upload_manager = FileUploadManager(self, file_name_updated, file_path, barcode_current, time_start, self.pf_comp_no(), self.pf_eqp_no())
                self.pool.start(file_upload_manager)
                break

    '''
    Signals & Properties are for interaction between backend and frontend
    '''
    logChanged = Signal()
    backendStateChanged = Signal()
    backendAlignStateChanged = Signal()
    backendRobotStateChanged = Signal()
    deviceIDListChanged = Signal()

    streamingDone = Signal(str)
    gripDeltaUpdated = Signal()
    gripRobotDeltaUpdated = Signal()

    log = Property(str, getLog, setLog, notify=logChanged)
    backendAlignState = Property(str, getBackendAlignState, setBackendAlignState, notify=backendAlignStateChanged)
    backendRobotState = Property(str, getBackendRobotState, setBackendRobotState, notify=backendRobotStateChanged)
    deviceIDList = Property("QVariantList", fget=getDeviceIDList, notify=deviceIDListChanged)

    @Slot()
    def q_init(self):
        ''' init process

        :return:
        '''
        # init - load ini files
        self.load_ini()

        # sync to frontend
        # below functions are defined at the main.qml
        self.copy_saved_device_info_py2qml(self.device_manager_.registered_device_ids(), self.device_manager_.registered_device_names())

        self.copy_saved_fps_expo_py2qml(
            [self.device_manager_.aligning_view_fps(), self.device_manager_.aligning_view_expo()],
            [self.device_manager_.shaping_view_fps(), self.device_manager_.shaping_view_expo()]
        )

    @Slot()
    def q_save_ini(self):
        ''' save current setting information to the gsa2.ini file

        :return:
        '''
        config = configparser.ConfigParser()
        config.read('gsa2.ini')

        if not 'Monitoring' in config:
            config['Monitoring'] = {}

        if not 'AligningView' in config:
            config['AligningView'] = {}

        if not 'ShapingView' in config:
            config['ShapingView'] = {}

        if not 'Server' in config:
            config['Server'] = {}

        config['Monitoring']['pf_comp_no'] = str(self.pf_comp_no())
        config['Monitoring']['pf_eqp_no'] = str(self.pf_eqp_no())

        config['AligningView']['FPS'] = str(self.device_manager_.aligning_view_fps())
        config['AligningView']['Exposure'] = str(self.device_manager_.aligning_view_expo())
        config['AligningView']['LED_ON'] = str(self.device_manager_.aligning_view_led_status())

        index_id = self.device_manager_.registered_device_names().index("Aligning View")
        if index_id > -1:
            config['AligningView']['ID'] = str(self.device_manager_.registered_device_ids()[index_id])

        config['AligningView']['BASE_COIL_THICKNESS'] = str(self.device_manager_.current_coil_thickness())

        config['AligningView']['START_OUT_CELL_VISIBLE'] = str(self.device_manager_.start_out_cell_visible())
        config['AligningView']['START_OUT_CELL_SIZE'] = str(self.device_manager_.start_out_cell_size())
        config['AligningView']['START_OUT_CELL_X'] = str(self.device_manager_.start_out_cell_x())
        config['AligningView']['START_OUT_CELL_Y'] = str(self.device_manager_.start_out_cell_y())

        config['AligningView']['START_IN_CELL_VISIBLE'] = str(self.device_manager_.start_in_cell_visible())
        config['AligningView']['START_IN_CELL_SIZE'] = str(self.device_manager_.start_in_cell_size())
        config['AligningView']['START_IN_CELL_X'] = str(self.device_manager_.start_in_cell_x())
        config['AligningView']['START_IN_CELL_Y'] = str(self.device_manager_.start_in_cell_y())

        config['AligningView']['END_OUT_CELL_VISIBLE'] = str(self.device_manager_.end_out_cell_visible())
        config['AligningView']['END_OUT_CELL_SIZE'] = str(self.device_manager_.end_out_cell_size())
        config['AligningView']['END_OUT_CELL_X'] = str(self.device_manager_.end_out_cell_x())
        config['AligningView']['END_OUT_CELL_Y'] = str(self.device_manager_.end_out_cell_y())

        config['AligningView']['END_IN_CELL_VISIBLE'] = str(self.device_manager_.end_in_cell_visible())
        config['AligningView']['END_IN_CELL_SIZE'] = str(self.device_manager_.end_in_cell_size())
        config['AligningView']['END_IN_CELL_X'] = str(self.device_manager_.end_in_cell_x())
        config['AligningView']['END_IN_CELL_Y'] = str(self.device_manager_.end_in_cell_y())

        config['AligningView']['ALIGNMENT_INTERVAL'] = str(self.device_manager_.alignment_interval())

        config['ShapingView']['FPS'] = str(self.device_manager_.shaping_view_fps())
        config['ShapingView']['Exposure'] = str(self.device_manager_.shaping_view_expo())

        index_id = self.device_manager_.registered_device_names().index("Shaping View")
        if index_id > -1:
            config['ShapingView']['ID'] = str(self.device_manager_.registered_device_ids()[index_id])

        config['Server']['OPC_IP'] = str(self.opc_manager_.ip())
        config['Server']['OPC_PORT'] = str(self.opc_manager_.port())
        config['Server']['OPC_AUTO_CONNECT'] = str(self.opc_manager_.auto_connection())

        config['Server']['ROBOT_IP'] = str(self.robot_manager_.ip())
        config['Server']['ROBOT_PORT'] = str(self.robot_manager_.port())
        config['Server']['ROBOT_AUTO_CONNECT'] = str(self.robot_manager_.auto_connection())

        with open('gsa2.ini', 'w') as configfile:
            config.write(configfile)

    @Slot()
    def q_run_server(self):
        ''' run OPC server

        :return:
        '''
        self.opc_manager_.run()

    @Slot()
    def q_stop_server(self):
        ''' stop OPC server

        :return:
        '''
        self.opc_manager_.stop()

    @Slot()
    def q_run_robot_server(self):
        ''' run robot server

        :return:
        '''
        self.robot_manager_.run()

    @Slot()
    def q_stop_robot_server(self):
        ''' stop robot server

        :return:
        '''
        self.robot_manager_.stop()

    @Slot()
    def q_send_data(self):
        ''' send alignment data to OPC server using opc_manager_

        :return:
        '''
        list_data = self.device_manager_.list_result_alignment()
        self.opc_manager_.send(list_data[0], list_data[1], list_data[2], list_data[3], list_data[4], list_data[5],
                               list_data[6], list_data[7], list_data[8], list_data[9], list_data[10])

    @Slot(QObject, str, str, result=int)
    def q_run_device(self, output, name, id):
        ''' run device on the runnable(pool)

        :param output: output id in qml
        :param name: device name
        :param id: device id
        :return: 0 is success, 1 is fail
        '''
        device = self.device_manager_.get_runnable_device(output, name, id)
        if device is None: return -1
        self.pool.start(device)
        return 0

    @Slot(str)
    def q_stop_device(self, id):
        '''stop device using device_manager_

        :param id: device id
        :return:
        '''
        self.device_manager_.stop_device(id)

    @Slot()
    def q_stop_all_devices(self):
        ''' stop all running devices using device_manager_

        :return:
        '''
        list_ids = self.device_manager_.get_list_running_device_objs()
        for index, device_id in enumerate(list_ids):
            self.device_manager_.stop_device(device_id)

    @Slot()
    def q_scan_device(self):
        ''' scan device using device_manager_

        :return:
        '''
        num_found_device = self.device_manager_.scan_devices()
        str_log = "Find " + str(num_found_device) + " device(s)"
        self.setLog(str_log)
        self.deviceIDListChanged.emit()



    @Slot(str)
    def q_start_vision_by_name(self, name):
        ''' start vision using device_manager_

        :param name: device name
        :return:
        '''
        self.device_manager_.update_vision_status_by_name(name, True)

    @Slot(str)
    def q_stop_vision_by_name(self, name):
        ''' stop vision using device_manager_

        :param name: device name
        :return:
        '''
        self.device_manager_.update_vision_status_by_name(name, False)

    @Slot(result=int)
    def q_count_running_device_objs(self):
        ''' count running device using device_manager_

        :return: the number of running devices
        '''
        return self.device_manager_.count_running_device_objs()

    @Slot(result=int)
    def q_count_uploaded_files(self):
        ''' count uploaded files

        :return: the number of uploaded files
        '''
        count = 0

        files = os.listdir()
        for str_file_name in files:
            if str_file_name.find("] Upload.mp4") > -1:
                count = count + 1

        return count

    @Slot()
    def q_remove_uploaded_files(self):
        ''' remove uploaded files

        :return:
        '''
        files = os.listdir()
        for str_file_name in files:
            if str_file_name.find("] Upload.mp4") > -1:
                try:
                    os.remove(str_file_name)
                    log = "Uploaded file is deleted : " + str_file_name
                    self.setLog(log)
                except OSError as error:
                    print(error)

    @Slot(int)
    def q_get_opc_server_state(self):
        ''' return opc server state

        :return:
        '''
        return self.opc_manager_.get_opc_server_state()

    @Slot(result=list)
    def q_get_scanned_device_ids(self):
        ''' get scanned device ids using device_manager_

        :return:
        '''
        return self.device_manager_.get_scanned_device_ids()

    @Slot()
    def q_get_alignment_value(self):
        ''' calculate alignment value for manual tuning view using device_manager_
        value is emitted, and it shows its alignment at the log area

        :return:
        '''
        self.device_manager_.activate_alignment_value_checking()

    @Slot(result=list)
    def q_get_tuning_values(self):
        ''' get tuning values(start&end coils position, coil thickness, ...)

        :return:
        '''
        return self.device_manager_.get_tuning_values()

    @Slot(str, result=bool)
    def q_get_led_status_by_name(self, name):
        ''' get led status by name using device manager

        :param name:
        :return:
        '''

        if name == "Aligning View" or name == "Tuning View" or name == "Manual Tuning View":
            return self.device_manager_.aligning_view_led_status()
        elif name == "Shaping View":
            return self.device_manager_.shaping_view_led_status()

        return False

    @Slot(result=list)
    def q_get_grip_delta_values(self):
        ''' grip delta : vision unit per one robot movement

        :return:
        '''
        return self.device_manager_.get_grip_delta_values()

    @Slot(result=list)
    def q_get_grip_robot_delta_values(self):
        ''' grip robot delta : calculated robot unit distance by vision unit

        :return:
        '''
        return self.device_manager_.get_grip_robot_delta_values()


    @Slot(result=int)
    def q_get_current_coil_thickness(self):
        ''' get current coil thickness in the backend

        :return:
        '''
        return self.current_coil_thickness()

    @Slot(result=str)
    def q_get_current_rebar_shape(self):
        ''' get current rebar shape in the backend

        :return:
        '''
        return self.current_rebar_shape()

    @Slot(result=list)
    def q_get_opc_server_option(self):
        ''' get opc server's saved ip and port number

        :return: [ip, port]
        '''
        list_options = [str(self.opc_manager_.ip()), str(self.opc_manager_.port())]

        if self.opc_manager_.auto_connection() is True:
            list_options.append("True")
        else:
            list_options.append("False")

        return list_options

    @Slot(result=list)
    def q_get_robot_server_option(self):
        ''' get robot server's saved ip, port number, and auto connection option

        :return: [ip, port, auto_connection]
        '''
        list_options = [str(self.robot_manager_.ip()), str(self.robot_manager_.port())]

        if self.robot_manager_.auto_connection() is True:
            list_options.append("True")
        else:
            list_options.append("False")

        return list_options

    @Slot(str, bool)
    def q_update_record_on_at(self, id, value):
        ''' update recording status using device_manager_

        :param id: device id
        :param value: true/false
        :return:
        '''
        self.device_manager_.set_record_on(id, value)

    @Slot(str, str, bool)
    def q_update_opc_server_option(self, ip, port, auto_connection):
        ''' update opc server's ip, port and auto connection option

        :param ip: opc server's ip
        :param port: opc server's ip
        :param auto_connection: auto connection when sw runs
        :return:
        '''
        self.opc_manager_.set_ip(ip)
        self.opc_manager_.set_port(port)
        self.opc_manager_.set_auto_connection(auto_connection)

    @Slot(str, str, bool)
    def q_update_robot_server_option(self, ip, port, auto_connection):
        ''' update robot server's ip, port and auto connection option

        :param ip: robot server's ip
        :param port: robot server's port
        :param auto_connection: auto connection when sw runs
        :return:
        '''
        self.robot_manager_.set_ip(ip)
        self.robot_manager_.set_port(port)
        self.robot_manager_.set_auto_connection(auto_connection)

    @Slot(str, bool)
    def q_update_led_status_at(self, id, value):
        ''' update led status using device_manager_

        :param id: device id
        :param value: true/false
        :return:
        '''
        self.device_manager_.update_led_status_at(id, value)

    @Slot(str, bool)
    def q_update_led_status_by_name(self, name, value):
        ''' update led status by device name using device_manager_

        :param name: device name
        :param value: true/false
        :return:
        '''
        self.device_manager_.update_led_status_by_name(name, value)

    @Slot(str, float)
    def q_update_fps_at(self, id, value):
        ''' update fps using device_manager_

        :param id: device id
        :param value: fps(1~90)
        :return:
        '''
        self.device_manager_.update_fps_at(id, value)

    @Slot(str, float)
    def q_update_fps_by_name(self, name, value):
        ''' update fps by name using device_manager_

        :param name: device name
        :param value: fps(1~90)
        :return:
        '''
        self.device_manager_.update_fps_by_name(name, value)

    @Slot(str, float)
    def q_update_exposure_at(self, id, value):
        ''' update exposure using device_manager_

        :param id: device id
        :param value: exposure(1~20000)
        :return:
        '''
        self.device_manager_.update_exposure_at(id, value)

    @Slot(str, float)
    def q_update_exposure_by_name(self, name, value):
        ''' update exposure by name using device_manager_

        :param name: device name
        :param value: exposure(1~20000)
        :return:
        '''
        self.device_manager_.update_exposure_by_name(name, value)

    @Slot(int)
    def q_update_current_coil_thickness(self, value):
        ''' update current coil thicknes using device_manager_

        :param value: 10/13/16
        :return:
        '''
        self.device_manager_.update_current_coil_thickness(value)

    @Slot(bool)
    def q_update_start_in_cell_visible(self, value):
        ''' update start in cell visible using device_manager_

        :param value: true/false
        :return:
        '''
        self.device_manager_.update_start_in_cell_visible(value)

    @Slot(int)
    def q_update_start_in_cell_size(self, value):
        ''' update start in cell size using device_manager_

        :param value: cell size(0~200) for in-cell qt the start point
        :return:
        '''
        self.device_manager_.update_start_in_cell_size(value)

    @Slot(int)
    def q_update_start_in_cell_x(self, value):
        ''' update start in cell x using device_manager_

        :param value: x value for in-cell at the start point
        :return:
        '''
        self.device_manager_.update_start_in_cell_x(value)

    @Slot(int)
    def q_update_start_in_cell_y(self, value):
        ''' update start in cell y using device_manager_

        :param value: y value for in-cell at the start point
        :return:
        '''
        self.device_manager_.update_start_in_cell_y(value)

    @Slot(bool)
    def q_update_start_out_cell_visible(self, value):
        ''' update start out cell visible using device_manager_

        :param value: true/false
        :return:
        '''
        self.device_manager_.update_start_out_cell_visible(value)

    @Slot(int)
    def q_update_start_out_cell_size(self, value):
        ''' update start out cell size using device_manager_

        :param value: cell size(0~200) for out-cell qt the start point
        :return:
        '''
        self.device_manager_.update_start_out_cell_size(value)

    @Slot(int)
    def q_update_start_out_cell_x(self, value):
        ''' update start out cell x using device_manager_

        :param value: x value for out-cell at the start point
        :return:
        '''
        self.device_manager_.update_start_out_cell_x(value)

    @Slot(int)
    def q_update_start_out_cell_y(self, value):
        ''' update start out cell y using device_manager_

        :param value: y value for out-cell at the start point
        :return:
        '''
        self.device_manager_.update_start_out_cell_y(value)

    @Slot(bool)
    def q_update_end_in_cell_visible(self, value):
        ''' update end in cell visible using device_manager_

        :param value: true/false
        :return:
        '''
        self.device_manager_.update_end_in_cell_visible(value)

    @Slot(int)
    def q_update_end_in_cell_size(self, value):
        ''' update end in cell size using device_manager_

        :param value: cell size(0~200) for in-cell at the end point
        :return:
        '''
        self.device_manager_.update_end_in_cell_size(value)

    @Slot(int)
    def q_update_end_in_cell_x(self, value):
        ''' update end in cell x using device_manager_

        :param value: x value for in-cell at the end point
        :return:
        '''
        self.device_manager_.update_end_in_cell_x(value)

    @Slot(int)
    def q_update_end_in_cell_y(self, value):
        ''' update end in cell y using device_manager_

        :param value: y value for in-cell at the end point
        :return:
        '''
        self.device_manager_.update_end_in_cell_y(value)

    @Slot(bool)
    def q_update_end_out_cell_visible(self, value):
        ''' update end out cell visible using device_manager_

        :param value: true/false
        :return:
        '''
        self.device_manager_.update_end_out_cell_visible(value)

    @Slot(int)
    def q_update_end_out_cell_size(self, value):
        ''' update end out cell size using device_manager_

        :param value: cell size(0~200) for out-cell at the end point
        :return:
        '''
        self.device_manager_.update_end_out_cell_size(value)

    @Slot(int)
    def q_update_end_out_cell_x(self, value):
        ''' update end out cell x using device_manager_

        :param value: x value for out-cell at the end point
        :return:
        '''
        self.device_manager_.update_end_out_cell_x(value)

    @Slot(int)
    def q_update_end_out_cell_y(self, value):
        ''' update end out cell y using device_manager_

        :param value: y value for out-cell at the end point
        :return:
        '''
        self.device_manager_.update_end_out_cell_y(value)

    @Slot(int)
    def q_update_manual_current_out_cell_x(self, value):
        ''' update manual current out cell x for calculating an alignment value at the ManualTuning

        :param value: out cell's x value
        :return:
        '''
        self.device_manager_.update_manual_current_out_cell_x(value)

    @Slot(int)
    def q_update_manual_current_out_cell_y(self, value):
        ''' update manual current out cell y for calculating an alignment value at the ManualTuning

        :param value: out cell's y value
        :return:
        '''
        self.device_manager_.update_manual_current_out_cell_y(value)

    @Slot(int)
    def q_update_manual_current_in_cell_x(self, value):
        ''' update manual current in cell x for calculating an alignment value at the ManualTuning

        :param value: in cell's x value
        :return:
        '''
        self.device_manager_.update_manual_current_in_cell_x(value)

    @Slot(int)
    def q_update_manual_current_in_cell_y(self, value):
        ''' update manual current in cell y for calculating an alignment value at the ManualTuning

        :param value: in cell's y value
        :return:
        '''
        self.device_manager_.update_manual_current_in_cell_y(value)

    @Slot(int)
    def q_update_alignment_interval(self, value):
        ''' update alignment interval,
        this interval is directly connected to the interval of Server communication

        :param value: None/once every three times/every time
        :return:
        '''
        self.device_manager_.update_alignment_interval(value)

    @Slot()
    def q_start_manual_grip_tuning(self):
        ''' open robot-vision calibration using device_manager_

        :return:
        '''
        self.device_manager_.start_manual_grip_tuning()

    @Slot()
    def q_end_manual_grip_tuning(self):
        ''' finish robot-vision calibration using device_manager_

        :return:
        '''
        self.device_manager_.end_manual_grip_tuning()

    @Slot(float, float)
    def q_generate_test_data_for_manual_grip_tuning(self, delta_x, delta_y):
        ''' generate test data for robot-vision calibration

        :param delta_x: current vision unit x for robot unit distance
        :param delta_y: current vision unit y for robot unit distance
        :return:
        '''
        self.device_manager_.generate_test_data_for_manual_grip_tuning(delta_x, delta_y)

    @Slot(float, float)
    def q_save_grip_delta_value(self, delta_x, delta_y):
        ''' save current vision unit for robot unit distance

        :param delta_x: current vision unit x for robot unit distance
        :param delta_y: current vision unit y for robot unit distance
        :return:
        '''
        self.device_manager_.save_grip_delta_value(delta_x, delta_y)

    @Slot()
    def q_save_tuning_view_value(self):
        ''' save tuning view value(end out cell position, ...) from device to device_manager_
        because coil thickness can be changed temporarily by OPC server

        :return:
        '''
        self.device_manager_.save_tuning_view_value()

    @Slot(list, list)
    def q_copy_saved_device_info_qml2py(self, list_device_id, list_device_name):
        ''' copy saved device id & name from qml(frontend) to py(backend)

        :param list_device_id: list of device ids
        :param list_device_name: list of device names
        :return:
        '''
        self.device_manager_.set_saved_device_info(list_device_id, list_device_name)

    @Slot()
    def q_grip_get_background(self):
        ''' (deprecated) extract current frame to image as background

        :return:
        '''
        self.device_manager_.grip_get_background()

    @Slot()
    def q_grip_get_coordinates(self):
        ''' (deprecated, for test) calculate grip point in the current frame

        :return:
        '''
        self.device_manager_.grip_get_coordinates()



class CVOutput(QQuickPaintedItem):
    """ A class to show video frames as qml component
    """

    def __init__(self, parent=None):
        super(CVOutput, self).__init__(parent)
        self.source_ = QImage()
        self.busy_ = False

    def get_source(self):
        return self.source_

    def set_source(self, source):
        self.source_ = source
        self.sourceChanged.emit()

    def paint(self, painter):
        if self.source_.isNull(): return

        size_background = self.size().toSize()

        source = self.source_.scaled(size_background, Qt.KeepAspectRatio)

        x = size_background.width() / 2 - source.width() / 2
        y = size_background.height() / 2 - source.height() / 2

        painter.drawImage(QPoint(x, y), source)

    sourceChanged = Signal()
    source = Property(QImage, get_source, set_source, notify=sourceChanged)



class QQuickViewFunctionExtender(QObject):
    """ A class to extend QQuickView's functions
    """
    def __init__(self, target):
        super(QQuickViewFunctionExtender, self).__init__()
        self.__view = target

    def eventFilter(self, obj, event):
        if event.type() == QEvent.Close:
            qml_ready_to_quit = view.rootObject().property("qmlReadyToQuit")
            if not qml_ready_to_quit:
                event.ignore()
                view.rootObject().setProperty("triggerQuit", True)

        return QObject.eventFilter(self, obj, event)

def qt_message_handler(mode, context, message):
    ''' print out logs from console.log(qml)

    :param mode:
    :param context:
    :param message:
    :return:
    '''
    if mode == QtCore.QtInfoMsg:
        mode = 'Info'
    elif mode == QtCore.QtWarningMsg:
        mode = 'Warning'
    elif mode == QtCore.QtCriticalMsg:
        mode = 'critical'
    elif mode == QtCore.QtFatalMsg:
        mode = 'fatal'
    else:
        mode = 'Debug'

    print("%s: %s (%s:%d, %s)" % (mode, message, context.file, context.line, context.file))


if __name__ == "__main__":
    QtCore.qInstallMessageHandler(qt_message_handler)

    app = QGuiApplication(sys.argv)

    view = QQuickView()
    extender = QQuickViewFunctionExtender(view)

    qmlRegisterType(BackendManager, 'BackendManager', 1, 0, 'BackendManager')
    qmlRegisterType(CVOutput, 'CVOutput', 1, 0, 'CVOutput')

    view.installEventFilter(extender)
    view.setSource(QUrl.fromLocalFile("main.qml"))

    view.setMinimumWidth(1544)
    view.setMaximumWidth(1544)
    view.setMinimumHeight(872)
    view.setMaximumHeight(872)

    view.engine().quit.connect(app.quit)

    view.setMinimumWidth(1544)
    view.setMaximumWidth(1544)
    view.setMinimumHeight(872)
    view.setMaximumHeight(872)

    if view.status() == QQuickView.Error:
        sys.exit(-1)
    view.show()

    sys.exit(app.exec())
