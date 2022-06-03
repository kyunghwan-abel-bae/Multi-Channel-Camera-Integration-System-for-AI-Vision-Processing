import cv2
import time
import numpy as np

from PySide6.QtCore import QRunnable, QObject, Signal
from MvCameraControl_class import *
from PySide6.QtGui import QImage


class NoDataError(Exception):
    """ A class to declare the Exception
    """
    def __init__(self, value):
        self.value = value


class RunnableStreamingSignal(QObject):
    """ A class to declare signals for RunnableStreaming
    """
    streaming_done = Signal(str)


class RunnableStreaming(QRunnable):
    """ A base class is for devices.
    it supports a mono color streaming
    By using this base class, devices can be managed by a threadpool
    """
    def __init__(self, device_info, output, device_name, device_id, is_landscape=True):
        super(RunnableStreaming, self).__init__()
        self.__device_info = device_info
        self.__output = output

        self.__device_name = device_name
        self.__device_id = device_id

        self.__is_streaming = False
        self.__is_vision_processing = True
        self.__is_option_changed = False

        self.__record_on = False
        self.__led_on = False
        self.__fps = 30
        self.__exposure = 1000

        self.__is_landscape = is_landscape

        self.__ratio_record = 0.5
        self.__resolution = (640, 512)
        if self.__is_landscape is False:
            self.__resolution = (512, 640)

        self.__vid_writer = cv2.VideoWriter("Video.avi", cv2.VideoWriter_fourcc(*'MJPG'), 30, self.__resolution, isColor=False)

        self.signal_ = RunnableStreamingSignal()

    def is_streaming(self):
        ''' getter of is streaming

        :return:
        '''
        return self.__is_streaming

    def set_streaming(self, value):
        ''' setter of is streaming

        :param value:
        :return:
        '''
        self.__is_streaming = value

    def is_vision_processing(self):
        ''' getter of is vision processing

        :return:
        '''
        return self.__is_vision_processing

    def set_vision_processing(self, value):
        ''' setter of is vision processing

        :param value:
        :return:
        '''
        self.__is_vision_processing = value

    def is_option_changed(self):
        ''' getter of is option changed
        when basic stream handling options(record, led, fps, exposure, ...) are changed,
        this function is used for batch processes

        :return:
        '''
        return self.__is_option_changed

    def set_option_changed(self, value):
        ''' setter of is option changed
        when basic stream handling options(record, led, fps, exposure, ...) are changed,
        this function is used for batch processes

        :param value:
        :return:
        '''
        self.__is_option_changed = value

    def record_on(self):
        ''' getter of record on

        :return:
        '''
        return self.__record_on

    def set_record_on(self, value):
        ''' setter of record on

        :param value:
        :return:
        '''
        self.__record_on = value
        self.set_option_changed(True)

    def led_on(self):
        ''' getter of led on

        :return:
        '''
        return self.__led_on

    def set_led_on(self, value):
        ''' setter of led on

        :param value:
        :return:
        '''
        self.__led_on = value
        self.set_option_changed(True)

    def fps(self):
        ''' getter of fps

        :return:
        '''
        return self.__fps

    def set_fps(self, value):
        ''' setter of fps

        :param value:
        :return:
        '''
        self.__fps = value
        self.set_option_changed(True)

    def exposure(self):
        ''' getter of exposure

        :return:
        '''
        return self.__exposure

    def set_exposure(self, value):
        ''' setter of exposure

        :param value:
        :return:
        '''
        self.__exposure = value
        self.set_option_changed(True)

    def is_landscape(self):
        ''' getter of is landscape

        :return:
        '''
        return self.__is_landscape

    def resolution(self):
        ''' getter of resolution

        :return:
        '''
        return self.__resolution

    def ratio_record(self):
        ''' getter of ratio record

        :return:
        '''
        return self.__ratio_record

    def get_mono_array_from(self, arg_data, arg_width, arg_height):
        ''' calcuate a mono version of np.array

        :param arg_data:
        :param arg_width:
        :param arg_height:
        :return:
        '''
        data = np.frombuffer(arg_data, dtype=np.uint8, offset=0)
        data_mono_arr = data.reshape(arg_height, arg_width)
        num_array = np.zeros([arg_height, arg_width, 1], "uint8")
        num_array[:, :, 0] = data_mono_arr
        return num_array

    def show_output(self, qimg):
        ''' show frame at the frontend side

        :param qimg:
        :return:
        '''
        if self.__output is None: return
        self.__output.setProperty("source", qimg)

    def process_frame(self, frame):
        ''' main point to handle frame
        most of devices put their own functions here

        :param frame:
        :return:
        '''
        ret = 1
        return frame

    def run_stream(self, cam=0, pdata=0, ndatasize=0):
        ''' main point of RunnableVideo

        :return:
        '''
        st_frame_info = MV_FRAME_OUT_INFO_EX()
        pdata = (c_ubyte * ndatasize)()

        memset(byref(st_frame_info), 0, sizeof(st_frame_info))

        current_record_on_status = self.record_on()
        current_led_status = self.led_on()
        current_fps = self.fps()
        current_exposure = self.exposure()

        cam.MV_CC_SetEnumValue("LineSelector", MV_TRIGGER_SOURCE_LINE1)
        cam.MV_CC_SetBoolValue("StrobeEnable", current_led_status)
        cam.MV_CC_SetFloatValue("AcquisitionFrameRate", current_fps)
        cam.MV_CC_SetFloatValue("ExposureTime", current_exposure)

        try:
            while True:
                if self.is_streaming() == False:
                    break

                if self.is_option_changed():
                    if current_record_on_status != self.record_on():
                        current_record_on_status = self.record_on()

                    if current_led_status != self.led_on():
                        current_led_status = self.led_on()
                        print("current_led_status : ", current_led_status)
                        cam.MV_CC_SetBoolValue("StrobeEnable", current_led_status)

                    if current_fps != self.fps():
                        current_fps = self.fps()
                        cam.MV_CC_SetFloatValue("AcquisitionFrameRate", current_fps)

                    if current_exposure != self.exposure():
                        current_exposure = self.exposure()
                        cam.MV_CC_SetFloatValue("ExposureTime", current_exposure)

                    self.set_option_changed(False)

                ret = cam.MV_CC_GetOneFrameTimeout(pdata, ndatasize, st_frame_info, 500)

                if ret == 0:
                    frame = cast(pdata, POINTER(c_ubyte))

                    if PixelType_Gvsp_Mono8 == st_frame_info.enPixelType:
                        num_array = self.get_mono_array_from(pdata, st_frame_info.nWidth, st_frame_info.nHeight)
                    else:
                        num_array = None

                    if self.is_vision_processing() is True:
                        self.process_frame(num_array)

                    if num_array is None:
                        qim = QImage()
                    else:
                        qim = QImage(num_array.data, num_array.shape[1], num_array.shape[0], QImage.Format_Grayscale8)
                        if self.record_on():
                            if current_record_on_status == False:
                                self.__vid_writer.release()

                                now = time.strftime("%Y-%m-%d %H_%M_%S")
                                vid_name = "[" + now + "] " + self.__device_name + ".avi"
                                self.__vid_writer = cv2.VideoWriter(vid_name, cv2.VideoWriter_fourcc(*'MJPG'), 30,
                                                             self.resolution(), isColor=False)
                            else:
                                resized_num_array = cv2.resize(num_array, dsize=self.resolution(), fx=self.ratio_record(), fy=self.ratio_record(), interpolation=cv2.INTER_LINEAR)
                                self.__vid_writer.write(resized_num_array)

                    self.show_output(qim.copy())
                else:
                    msg_error = "No Data - " + str(ret)
                    raise NoDataError(msg_error)

        except NoDataError as e:
            print(e.value)

    def run(self):
        ''' start point of RunnableVideo

        :return:
        '''

        self.set_streaming(True)

        cam = MvCamera()
        st_device_list = cast(self.__device_info, POINTER(MV_CC_DEVICE_INFO)).contents

        ret = cam.MV_CC_CreateHandle(st_device_list)
        if ret != 0:
            print("create handle fail! ret[0x%x]" % ret)
            return

        # ch:打开设备 | en:Open device
        ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            print("open device fail! ret[0x%x]" % ret)
            return

        if st_device_list.nTLayerType == MV_GIGE_DEVICE:
            packet_size = cam.MV_CC_GetOptimalPacketSize()
            if int(packet_size) > 0:
                ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", packet_size)
                if ret != 0:
                    print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
            else:
                print("Warning: Get Packet Size fail! ret[0x%x]" % packet_size)

        # set trigger mode as off
        ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if ret != 0:
            print("set trigger mode fail! ret[0x%x]" % ret)
            return

        # Get payload size
        st_param = MVCC_INTVALUE()
        memset(byref(st_param), 0, sizeof(MVCC_INTVALUE))

        ret = cam.MV_CC_GetIntValue("PayloadSize", st_param)
        if ret != 0:
            return

        payload_size = st_param.nCurValue

        ret = cam.MV_CC_StartGrabbing()
        if ret != 0:
            return

        data_buf = (c_ubyte * payload_size)()

        self.run_stream(cam, byref(data_buf), payload_size)

        ret = cam.MV_CC_StopGrabbing()
        if ret != 0:
            print("stop grabbing failed")
            del data_buf
            return

        ret = cam.MV_CC_CloseDevice()
        if ret != 0:
            print("close device failed")
            del data_buf
            return

        ret = cam.MV_CC_DestroyHandle()
        if ret != 0:
            print("destroy handle failed")
            del data_buf
            return

        del data_buf

        self.set_streaming(False)

        self.signal_.streaming_done.emit(self.__device_id)