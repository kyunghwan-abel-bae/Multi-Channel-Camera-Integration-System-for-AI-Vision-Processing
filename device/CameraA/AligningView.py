import cv2
import numpy as np

from RunnableRGBStreaming import *

import cvlib as cv
from PySide6.QtCore import QRunnable
from PySide6.QtGui import QImage
from cvlib.object_detection import draw_bbox
import cv2
import numpy as np
from matplotlib import pyplot as plt

# import cv2 as cv
import argparse
import sys
import os.path
from PIL import Image
import pandas as pd
import seaborn as sns
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
from sklearn.linear_model import LinearRegression
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import time
import xml.etree.ElementTree as etree  # XML 로드 모듈
import math

from sklearn.linear_model import Ridge, LinearRegression
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import PolynomialFeatures, StandardScaler

def constant(func):
    ''' decorator for below class _Const

    :param func:
    :return:
    '''
    def func_set(self, value):
        raise TypeError

    def func_get(self):
        return func()
    return property(func_get, func_set)


class _Const(object):
    """ A class to define const variables

    """
    @constant
    def START_OUT_CELL():
        ''' the first point of out-point tracking
        a point at the (out-point in start area + threshold) using size
        and len(coords_out) == 0

        :return:
        '''
        return 0

    @constant
    def START_IN_CELL():
        ''' the first point of in-point tracking
        a point at the (in-point in start area + threshold) using size
        and len(coords_in_x) == 0

        :return:
        '''
        return 1

    @constant
    def START_OUT_CELL_INIT():
        ''' the out-point tracking is back to the first point
        a point at the (out-point in start area + threshold) using size
        and By comparing current size and last size(coords_out_size[-1] > current size + threshold),
        sw finds out it is back to the start point

        :return:
        '''
        return 2

    @constant
    def START_IN_CELL_INIT():
        ''' the in-point tracking is back to the fist point
        a point at the (in-point in start area + threshold) using size
        and By comparing current size and last size(coords_in_size[-1] > current size + threshold),
        sw finds out it is back to the start point

        :return:
        '''
        return 3

    @constant
    def PROGRESS_OUT_CELL():
        ''' tracking is progressing as out-point

        :return:
        '''
        return 4

    @constant
    def PROGRESS_IN_CELL():
        ''' tracking is progressing as in-point

        :return:
        '''
        return 5

    @constant
    def ALIGNMENT_INTERVAL_NOT_SEND():
        ''' SW don't calculate and send an alignment value

        :return:
        '''
        return 1

    @constant
    def ALIGNMENT_INTERVAL_SEND_ONCE_EVERY_THREE_TIMES():
        ''' SW calculates and sends an alignment value once every three times

        :return:
        '''
        return 2

    @constant
    def ALIGNMENT_INTERVAL_SEND_ALWAYS():
        ''' SW calulates and sends an alignment values every time

        :return:
        '''
        return 3

    @constant
    def ALIGNMENT_END_POINTS_MINIMUM_COUNT():
        ''' Minimum count for validation the number of rebar end points
         At least, end point > minimum count

        :return:
        '''
        return 15

    @constant
    def ALIGNMENT_END_POINTS_MAXIMUM_DIFFERENCE_COUNT_RATIO():
        ''' Ratio for validation the number of out-point, in-point
        When the difference of comparing between out-point counts and in-point counts is too big(80%),
        then something is wrong.(like tracking is failed)

        :return:
        '''
        return 1.8


class Measurement_Map:
    """ A class for a grid cell
    """
    def __init__(self):
        self.in_center = (0, 0)
        self.out_center = (0, 0)
        self.cell_size = 0
        self.InCell = [[0 for cols in range(4)] for rows in range(86)]
        self.OutCell = [[0 for cols in range(4)] for rows in range(93)]
        for i in range(0, 86):
            self.InCell[i] = [0, 0, 0, 0]
        for i in range(0, 93):
            self.OutCell[i] = [0, 0, 0, 0]

    def set_end_in_cell(self, size, x, y):
        ''' change grid (in-)cell's position and size

        :param size: cell size
        :param x: position x
        :param y: position y
        :return:
        '''
        x = x - (size / 2)
        y = y - (size / 2)
        self.InCell[0] = [x - size, y - size, 3 * size, 2 * size]
        self.InCell[1] = [x + (9 * size), y, size, size]
        self.InCell[2] = [x + (8 * size), y - size, size, size]
        self.InCell[3] = [x + (8 * size), y, size, size]
        self.InCell[4] = [x + (7 * size), y - (2 * size), size, size]
        self.InCell[5] = [x + (7 * size), y - (size), size, size]
        self.InCell[6] = [x + (7 * size), y, size, size]
        self.InCell[7] = [x + (6 * size), y - (3 * size), size, size]
        self.InCell[8] = [x + (6 * size), y - (2 * size), size, size]
        self.InCell[9] = [x + (6 * size), y - (size), size, size]
        self.InCell[10] = [x + (6 * size), y, size, size]
        self.InCell[11] = [x + (5 * size), y - (4 * size), size, size]
        self.InCell[12] = [x + (5 * size), y - (3 * size), size, size]
        self.InCell[13] = [x + (5 * size), y - (2 * size), size, size]
        self.InCell[14] = [x + (5 * size), y - (size), size, size]
        self.InCell[15] = [x + (5 * size), y, size, size]
        self.InCell[16] = [x + (4 * size), y - (5 * size), size, size]
        self.InCell[17] = [x + (4 * size), y - (4 * size), size, size]
        self.InCell[18] = [x + (4 * size), y - (3 * size), size, size]
        self.InCell[19] = [x + (4 * size), y - (2 * size), size, size]
        self.InCell[20] = [x + (4 * size), y - (size), size, size]
        self.InCell[21] = [x + (4 * size), y, size, size]
        self.InCell[22] = [x + (3 * size), y - (6 * size), size, size]
        self.InCell[23] = [x + (3 * size), y - (5 * size), size, size]
        self.InCell[24] = [x + (3 * size), y - (4 * size), size, size]
        self.InCell[25] = [x + (3 * size), y - (3 * size), size, size]
        self.InCell[26] = [x + (3 * size), y - (2 * size), size, size]
        self.InCell[27] = [x + (3 * size), y - (size), size, size]
        self.InCell[28] = [x + (3 * size), y, size, size]
        self.InCell[29] = [x + (2 * size), y - (6 * size), size, size]
        self.InCell[30] = [x + (2 * size), y - (5 * size), size, size]
        self.InCell[31] = [x + (2 * size), y - (4 * size), size, size]
        self.InCell[32] = [x + (2 * size), y - (3 * size), size, size]
        self.InCell[33] = [x + (2 * size), y - (2 * size), size, size]
        self.InCell[34] = [x + (2 * size), y - (size), size, size]
        self.InCell[35] = [x + (2 * size), y, size, size]
        self.InCell[36] = [x + (size), y - (6 * size), size, size]
        self.InCell[37] = [x + (size), y - (5 * size), size, size]
        self.InCell[38] = [x + (size), y - (4 * size), size, size]
        self.InCell[39] = [x + (size), y - (3 * size), size, size]
        self.InCell[40] = [x + (size), y - (2 * size), size, size]
        self.InCell[41] = [x, y - (6 * size), size, size]
        self.InCell[42] = [x, y - (5 * size), size, size]
        self.InCell[43] = [x, y - (4 * size), size, size]
        self.InCell[44] = [x, y - (3 * size), size, size]
        self.InCell[45] = [x, y - (2 * size), size, size]
        self.InCell[46] = [x - (size), y - (6 * size), size, size]
        self.InCell[47] = [x - (size), y - (5 * size), size, size]
        self.InCell[48] = [x - (size), y - (4 * size), size, size]
        self.InCell[49] = [x - (size), y - (3 * size), size, size]
        self.InCell[50] = [x - (size), y - (2 * size), size, size]
        self.InCell[51] = [x - (2 * size), y - (6 * size), size, size]
        self.InCell[52] = [x - (2 * size), y - (5 * size), size, size]
        self.InCell[53] = [x - (2 * size), y - (4 * size), size, size]
        self.InCell[54] = [x - (2 * size), y - (3 * size), size, size]
        self.InCell[55] = [x - (2 * size), y - (2 * size), size, size]
        self.InCell[56] = [x - (2 * size), y - (size), size, size]
        self.InCell[57] = [x - (2 * size), y, size, size]
        self.InCell[58] = [x - (3 * size), y - (6 * size), size, size]
        self.InCell[59] = [x - (3 * size), y - (5 * size), size, size]
        self.InCell[60] = [x - (3 * size), y - (4 * size), size, size]
        self.InCell[61] = [x - (3 * size), y - (3 * size), size, size]
        self.InCell[62] = [x - (3 * size), y - (2 * size), size, size]
        self.InCell[63] = [x - (3 * size), y - (size), size, size]
        self.InCell[64] = [x - (3 * size), y, size, size]
        self.InCell[65] = [x - (4 * size), y - (5 * size), size, size]
        self.InCell[66] = [x - (4 * size), y - (4 * size), size, size]
        self.InCell[67] = [x - (4 * size), y - (3 * size), size, size]
        self.InCell[68] = [x - (4 * size), y - (2 * size), size, size]
        self.InCell[69] = [x - (4 * size), y - (size), size, size]
        self.InCell[70] = [x - (4 * size), y, size, size]
        self.InCell[71] = [x - (5 * size), y - (4 * size), size, size]
        self.InCell[72] = [x - (5 * size), y - (3 * size), size, size]
        self.InCell[73] = [x - (5 * size), y - (2 * size), size, size]
        self.InCell[74] = [x - (5 * size), y - (size), size, size]
        self.InCell[75] = [x - (5 * size), y, size, size]
        self.InCell[76] = [x - (6 * size), y - (3 * size), size, size]
        self.InCell[77] = [x - (6 * size), y - (2 * size), size, size]
        self.InCell[78] = [x - (6 * size), y - (size), size, size]
        self.InCell[79] = [x - (6 * size), y, size, size]
        self.InCell[80] = [x - (7 * size), y - (2 * size), size, size]
        self.InCell[81] = [x - (7 * size), y - (size), size, size]
        self.InCell[82] = [x - (7 * size), y, size, size]
        self.InCell[83] = [x - (8 * size), y - (size), size, size]
        self.InCell[84] = [x - (8 * size), y, size, size]
        self.InCell[85] = [x - (9 * size), y, size, size]

    def set_end_out_cell(self, size, x, y):
        ''' change grid (out-)cell's position and size

        :param size: cell size
        :param x: position x
        :param y: position y
        :return:
        '''
        x = x - (size / 2)
        y = y - (size / 2)
        self.OutCell[0] = [x - size, y - size, 3 * size, 3 * size]
        self.OutCell[1] = [x + (9 * size), y, size, size]
        self.OutCell[2] = [x + (8 * size), y - size, size, size]
        self.OutCell[3] = [x + (8 * size), y, size, size]
        self.OutCell[4] = [x + (8 * size), y + (size), size, size]
        self.OutCell[5] = [x + (7 * size), y - (2 * size), size, size]
        self.OutCell[6] = [x + (7 * size), y - (size), size, size]
        self.OutCell[7] = [x + (7 * size), y, size, size]
        self.OutCell[8] = [x + (7 * size), y + size, size, size]
        self.OutCell[9] = [x + (6 * size), y - (3 * size), size, size]
        self.OutCell[10] = [x + (6 * size), y - (2 * size), size, size]
        self.OutCell[11] = [x + (6 * size), y - (size), size, size]
        self.OutCell[12] = [x + (6 * size), y, size, size]
        self.OutCell[13] = [x + (6 * size), y + (size), size, size]
        self.OutCell[14] = [x + (5 * size), y - (4 * size), size, size]
        self.OutCell[15] = [x + (5 * size), y - (3 * size), size, size]
        self.OutCell[16] = [x + (5 * size), y - (2 * size), size, size]
        self.OutCell[17] = [x + (5 * size), y - (size), size, size]
        self.OutCell[18] = [x + (5 * size), y, size, size]
        self.OutCell[19] = [x + (5 * size), y + size, size, size]
        self.OutCell[20] = [x + (4 * size), y - (5 * size), size, size]
        self.OutCell[21] = [x + (4 * size), y - (4 * size), size, size]
        self.OutCell[22] = [x + (4 * size), y - (3 * size), size, size]
        self.OutCell[23] = [x + (4 * size), y - (2 * size), size, size]
        self.OutCell[24] = [x + (4 * size), y - (size), size, size]
        self.OutCell[25] = [x + (4 * size), y, size, size]
        self.OutCell[26] = [x + (4 * size), y + (size), size, size]
        self.OutCell[27] = [x + (3 * size), y - (5 * size), size, size]
        self.OutCell[28] = [x + (3 * size), y - (4 * size), size, size]
        self.OutCell[29] = [x + (3 * size), y - (3 * size), size, size]
        self.OutCell[30] = [x + (3 * size), y - (2 * size), size, size]
        self.OutCell[31] = [x + (3 * size), y - (size), size, size]
        self.OutCell[32] = [x + (3 * size), y, size, size]
        self.OutCell[33] = [x + (3 * size), y + (size), size, size]
        self.OutCell[34] = [x + (2 * size), y - (5 * size), size, size]
        self.OutCell[35] = [x + (2 * size), y - (4 * size), size, size]
        self.OutCell[36] = [x + (2 * size), y - (3 * size), size, size]
        self.OutCell[37] = [x + (2 * size), y - (2 * size), size, size]
        self.OutCell[38] = [x + (2 * size), y - (size), size, size]
        self.OutCell[39] = [x + (2 * size), y, size, size]
        self.OutCell[40] = [x + (2 * size), y + (size), size, size]
        self.OutCell[41] = [x + size, y - (5 * size), size, size]
        self.OutCell[42] = [x + size, y - (4 * size), size, size]
        self.OutCell[43] = [x + size, y - (3 * size), size, size]
        self.OutCell[44] = [x + size, y - (2 * size), size, size]
        self.OutCell[45] = [x, y - (5 * size), size, size]
        self.OutCell[46] = [x, y - (4 * size), size, size]
        self.OutCell[47] = [x, y - (3 * size), size, size]
        self.OutCell[48] = [x, y - (2 * size), size, size]
        self.OutCell[49] = [x - size, y - (5 * size), size, size]
        self.OutCell[50] = [x - size, y - (4 * size), size, size]
        self.OutCell[51] = [x - size, y - (3 * size), size, size]
        self.OutCell[52] = [x - size, y - (2 * size), size, size]
        self.OutCell[53] = [x - (2 * size), y - (5 * size), size, size]
        self.OutCell[54] = [x - (2 * size), y - (4 * size), size, size]
        self.OutCell[55] = [x - (2 * size), y - (3 * size), size, size]
        self.OutCell[56] = [x - (2 * size), y - (2 * size), size, size]
        self.OutCell[57] = [x - (2 * size), y - (size), size, size]
        self.OutCell[58] = [x - (2 * size), y, size, size]
        self.OutCell[59] = [x - (2 * size), y + (size), size, size]
        self.OutCell[60] = [x - (3 * size), y - (5 * size), size, size]
        self.OutCell[61] = [x - (3 * size), y - (4 * size), size, size]
        self.OutCell[62] = [x - (3 * size), y - (3 * size), size, size]
        self.OutCell[63] = [x - (3 * size), y - (2 * size), size, size]
        self.OutCell[64] = [x - (3 * size), y - (size), size, size]
        self.OutCell[65] = [x - (3 * size), y, size, size]
        self.OutCell[66] = [x - (3 * size), y + (size), size, size]
        self.OutCell[67] = [x - (4 * size), y - (5 * size), size, size]
        self.OutCell[68] = [x - (4 * size), y - (4 * size), size, size]
        self.OutCell[69] = [x - (4 * size), y - (3 * size), size, size]
        self.OutCell[70] = [x - (4 * size), y - (2 * size), size, size]
        self.OutCell[71] = [x - (4 * size), y - (size), size, size]
        self.OutCell[72] = [x - (4 * size), y, size, size]
        self.OutCell[73] = [x - (4 * size), y + (size), size, size]
        self.OutCell[74] = [x - (5 * size), y - (4 * size), size, size]
        self.OutCell[75] = [x - (5 * size), y - (3 * size), size, size]
        self.OutCell[76] = [x - (5 * size), y - (2 * size), size, size]
        self.OutCell[77] = [x - (5 * size), y - (size), size, size]
        self.OutCell[78] = [x - (5 * size), y, size, size]
        self.OutCell[79] = [x - (5 * size), y + (size), size, size]
        self.OutCell[80] = [x - (6 * size), y - (3 * size), size, size]
        self.OutCell[81] = [x - (6 * size), y - (2 * size), size, size]
        self.OutCell[82] = [x - (6 * size), y - (size), size, size]
        self.OutCell[83] = [x - (6 * size), y, size, size]
        self.OutCell[84] = [x - (6 * size), y + (size), size, size]
        self.OutCell[85] = [x - (7 * size), y - (2 * size), size, size]
        self.OutCell[86] = [x - (7 * size), y - (size), size, size]
        self.OutCell[87] = [x - (7 * size), y, size, size]
        self.OutCell[88] = [x - (7 * size), y + (size), size, size]
        self.OutCell[89] = [x - (8 * size), y - (size), size, size]
        self.OutCell[90] = [x - (8 * size), y, size, size]
        self.OutCell[91] = [x - (8 * size), y + (size), size, size]
        self.OutCell[92] = [x - (9 * size), y, size, size]

    def check_end_in_cell_map(self, x, y):
        ''' return matching cell number for in-cell based on param x,y
        셀 매핑, x,y 좌표 전달을 통해서 셀 번호 도출
        :param x:
        :param y:
        :return:
        '''
        no = 9999  # 에러 셀 번호 9999, 정상 0~85
        for i in range(0, 86):
            if (x >= self.InCell[i][0] and self.InCell[i][0] + self.InCell[i][2] >= x and y >= self.InCell[i][1] and
                    self.InCell[i][1] + self.InCell[i][3] >= y):
                no = i
                break
        return no

    def check_end_out_cell_map(self, x, y):
        ''' return matching cell number for out-cell based parm x,y
        :param x:
        :param y:
        :return:
        '''
        no = 9999  # 에러 셀 번호 9999, 정상 0~92
        for i in range(0, 93):
            if (x >= self.OutCell[i][0] and self.OutCell[i][0] + self.OutCell[i][2] >= x and y >= self.OutCell[i][1] and
                    self.OutCell[i][1] + self.OutCell[i][3] >= y):
                no = i
                break
        return no

    def show_end_in_cell_map(self, image):
        ''' show grid in-cell

        :param image: frame
        :return:
        '''
        cv2.rectangle(image, (int(self.InCell[0][0]), int(self.InCell[0][1])),
                      (int(self.InCell[0][0] + self.InCell[0][2]), int(self.InCell[0][1] + self.InCell[0][3])),
                      (0, 255, 0), 2, 1)
        cv2.putText(image, 'IN', (
        int(self.InCell[0][0] + (self.InCell[0][2] / 2)), int(self.InCell[0][1] + (self.InCell[0][3])) - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 180, 50), 1)
        for i in range(1, 86):
            cv2.rectangle(image, (int(self.InCell[i][0]), int(self.InCell[i][1])),
                          (int(self.InCell[i][0] + self.InCell[i][2]), int(self.InCell[i][1] + self.InCell[i][3])),
                          (0, 0, 255), 1, 1)
            cv2.putText(image, str(int(i)), (
            int(self.InCell[i][0] + (self.InCell[i][2] / 2)), int(self.InCell[i][1] + (self.InCell[i][3] / 2))),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 180, 50), 1)
        return image

    def show_end_out_cell_map(self, image):
        ''' show grid out-cell

        :param image: frame
        :return:
        '''
        cv2.rectangle(image, (int(self.OutCell[0][0]), int(self.OutCell[0][1])),
                      (int(self.OutCell[0][0] + self.OutCell[0][2]), int(self.OutCell[0][1] + self.OutCell[0][3])),
                      (0, 255, 0), 2, 1)
        cv2.putText(image, 'OUT', (
        int(self.OutCell[0][0] + (self.OutCell[0][2] / 2)), int(self.OutCell[0][1] + (self.OutCell[0][3])) - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 180, 50), 1)
        for i in range(1, 93):
            cv2.rectangle(image, (int(self.OutCell[i][0]), int(self.OutCell[i][1])),
                          (int(self.OutCell[i][0] + self.OutCell[i][2]), int(self.OutCell[i][1] + self.OutCell[i][3])),
                          (0, 0, 255), 1, 1)
            cv2.putText(image, str(int(i)), (
            int(self.OutCell[i][0] + (self.OutCell[i][2] / 2)), int(self.OutCell[i][1] + (self.OutCell[i][3] / 2))),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 180, 50), 1)
        return image


class Compare_Model:
    """ A class to calculate alignment value
    """
    def __init__(self):
        self.model_name = os.path.abspath("./device/CameraA/standard_model.xml").replace("\\", "/")
        self.model = self.model_name

    def change_model(self, name):
        self.model_name = name
        self.model = self.directory_name + self.model_name

    def change_directory(self, directory):
        self.directory_name = directory
        self.model = self.directory_name + self.model_name

    def compare_in_cell_map(self, in_no):
        global s1, s3, s5, s7, in_coil_cell
        xmlD = etree.parse(self.model)
        root = xmlD.getroot()

        # 매칭 에러 시 E 리턴
        s1 = 'E'
        s3 = 'E'
        s5 = 'E'
        s7 = 'E'

        for coil in root.findall('Coil'):
            number = coil.get('Number')
            if number == str(in_no):
                s1 = coil.find('S1').text
                s3 = coil.find('S3').text
                s5 = coil.find('S5').text
                s7 = coil.find('S7').text
                in_coil_cell = str(in_no)
                break

    def compare_out_cell_map(self, out_no):
        global s2, s4, s6, s8, out_coil_cell
        xmlD = etree.parse(self.model)
        root = xmlD.getroot()

        # 매칭 에러 시 E 리턴
        s2 = 'E'
        s4 = 'E'
        s6 = 'E'
        s8 = 'E'

        for coil in root.findall('Coil'):
            number = coil.get('Number')
            if number == str(out_no):
                s2 = coil.find('S2').text
                s4 = coil.find('S4').text
                s6 = coil.find('S6').text
                s8 = coil.find('S8').text
                out_coil_cell = str(out_no)
                break


CONST = _Const()

s1 = '0'  # Roller no1
s2 = '0'  # Roller no2
s3 = '0'  # Roller no3
s4 = '0'  # Roller no4
s5 = '0'  # Roller no5
s6 = '0'  # Roller no6
s7 = '0'  # Roller no7
s8 = '0'  # Roller no8
out_coil_cell = 0  # Out Coil Cell No.(0~92)
in_coil_cell = 0   # In Coil Cell No.(0~92)
grap_info = 'N' # Graping Information('Y'or'N')

confThreshold = 0.5  # Confidence threshold
nmsThreshold = 0.4   # Non-maximum suppression threshold

inpWidth = 416     # Width of network's input image
inpHeight = 416    # Height of network's input image

prevTime = 0

parser = argparse.ArgumentParser(description='Object Detection using YOLO in OPENCV')
parser.add_argument('--image', help='Path to image file.')
video_path = os.path.abspath("./device/CameraA/test_vid/t80.mp4").replace("\\", "/")
parser.add_argument('--video', default=video_path, help='Path to video file.')
args = parser.parse_args()

classesFile = os.path.abspath("./device/CameraA/custom_steel_cfg/steel_plate.names").replace("\\", "/")

classes = None
with open(classesFile, 'rt') as f:
    classes = f.read().rstrip('\n').split('\n')

modelConfiguration = os.path.abspath("./device/CameraA/custom_steel_cfg/rebar.cfg").replace("\\", "/")
modelWeights = os.path.abspath("./device/CameraA/weights/yolov4-rebar_final.weights").replace("\\", "/")

net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

bbox_coor_x = []
bbox_coor_y = []

bbox_rl = []

def getOutputsNames(net):
    layersNames = net.getLayerNames()
    return [layersNames[i[0] - 1] for i in net.getUnconnectedOutLayers()]

def drawPred(frame, left, top, right, bottom, label):
    cv2.rectangle(frame, (left, top - 30), (left + 50, top + 30), (255, 255, 255), -1)
    cv2.putText(frame, label, (left, top), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2)

    cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 1)

def onChange(pos):
    pass

class AligningViewSignal(QObject):
    """ A class to declare signals for AligningView
    """
    alignment_work_done = Signal(list, str)


class AligningView(RunnableRGBStreaming):
    """ A class about real time rebar alignment based prediction
    """
    def __init__(self, device_info, output, serial_number, current_coil_thickness, end_in_cell_visible, end_in_cell_size, end_in_cell_x, end_in_cell_y, end_out_cell_visible, end_out_cell_size, end_out_cell_x, end_out_cell_y, start_in_cell_visible, start_in_cell_size, start_in_cell_x, start_in_cell_y, start_out_cell_visible, start_out_cell_size, start_out_cell_x, start_out_cell_y):#, end_in_cell_visible, end_in_cell_size, end_in_cell_x, end_in_cell_y, end_out_cell_visible, end_out_cell_size, end_out_cell_x, end_out_cell_y, start_in_cell_visible, start_in_cell_size, start_in_cell_x, start_in_cell_y, start_out_cell_visible, start_out_cell_size, start_out_cell_x, start_out_cell_y):
        super(AligningView, self).__init__(device_info, output, "AligningView", serial_number)

        self.__current_coil_thickness = current_coil_thickness

        self.__start_in_cell_visible = start_in_cell_visible
        self.__start_in_cell_size = start_in_cell_size
        self.__start_in_cell_size_threshold = 0.05

        # x, y = (left, top)
        self.__start_in_cell_x = start_in_cell_x
        self.__start_in_cell_y = start_in_cell_y

        self.__start_out_cell_visible = start_out_cell_visible
        self.__start_out_cell_size = start_out_cell_size
        self.__start_out_cell_size_threshold = 0.05

        # x, y = (left, top)
        self.__start_out_cell_x = start_out_cell_x
        self.__start_out_cell_y = start_out_cell_y

        self.__end_in_cell_visible = end_in_cell_visible
        self.__end_in_cell_size = end_in_cell_size
        self.__end_in_cell_x = end_in_cell_x
        self.__end_in_cell_y = end_in_cell_y

        self.__end_out_cell_visible = end_out_cell_visible
        self.__end_out_cell_size = end_out_cell_size
        self.__end_out_cell_x = end_out_cell_x
        self.__end_out_cell_y = end_out_cell_y

        self.__alignment_interval = 1

        self.__coords_x = []
        self.__coords_y = []

        self.__dict_current_coords = {}

        self.__list_coords_30_history = []

        self.__coords_in_x = []
        self.__coords_in_y = []
        self.__coords_in_size = []
        self.__coords_out_x = []
        self.__coords_out_y = []
        self.__coords_out_size = []

        self.__list_end_out_in_coords = []
        self.__list_end_out_coords = []
        self.__list_end_in_coords = []
        self.__interval_count = 3

        self.__base_out_data_length = 0
        self.__base_in_data_length = 0

        self.__ratio_size_40_rebar_by_start_rebar = 1.5

        self.__is_alignment_value_checking = False

        self.__aligning_vid_writer = cv2.VideoWriter("Video.mp4", 0x00000021, 30, (640, 512), isColor=False)

        self.__vid_now = time.strftime("%Y-%m-%d %H_%M_%S")
        self.__vid_file_name = ""
        self.__vid_original_file_name = ""

        self.__resolution = super().resolution()
        self.__ratio_record = 0.25

        self.signal_aligning_view_ = AligningViewSignal()

    def base_out_data_length(self):
        ''' getter of base out data length

        :return:
        '''
        return self.__base_out_data_length

    def set_base_out_data_length(self, value):
        ''' setter of base out data length

        :param value:
        :return:
        '''
        self.__base_out_data_length = int(value)

    def base_in_data_length(self):
        ''' getter of base in data length

        :return:
        '''
        return self.__base_in_data_length

    def set_base_in_data_length(self, value):
        ''' setter of base in data length

        :param value:
        :return:
        '''
        self.__base_in_data_length = int(value)

    def update_alignment_interval(self, value):
        ''' setter of alignment interval

        :param value:
        :return:
        '''
        self.__alignment_interval = value

    def does_start_out_cell_contain(self, x, y):
        ''' checking that (x,y) is found at the start out-point

        :param x:
        :param y:
        :return:
        '''
        if x < self.__start_out_cell_x:
            return False

        if x > (self.__start_out_cell_x + self.__start_out_cell_size):
            return False

        if y < self.__start_out_cell_y:
            return False

        if y > (self.__start_out_cell_y + self.__start_out_cell_size):
            return False

        return True

    def does_start_in_cell_contain(self, x, y):
        ''' checking that (x,y) is found at the start in-point

        :param x:
        :param y:
        :return:
        '''
        if x < self.__start_in_cell_x:
            return False

        if x > (self.__start_in_cell_x + self.__start_in_cell_size):
            return False

        if y < self.__start_in_cell_y:
            return False

        if y > (self.__start_in_cell_y + self.__start_in_cell_size):
            return False

        return True

    def check_status_in_start_point(self, x, y, size):
        ''' check the point is in the start point
        there are some status in start point based several conditions

        :param x:
        :param y:
        :param size:
        :return:
        '''
        if self.does_start_out_cell_contain(x, y) == True:
            if size <= (self.__start_out_cell_size + self.__start_out_cell_size * self.__start_out_cell_size_threshold) \
                    and len(self.__coords_out_x) == 0:
                return CONST.START_OUT_CELL
            elif (size <= (self.__start_out_cell_size + self.__start_out_cell_size * self.__start_out_cell_size_threshold)) \
                    and (self.__coords_out_size[-1] > size * self.__ratio_size_40_rebar_by_start_rebar):
                return CONST.START_OUT_CELL_INIT
            else:
                return -1  # injected rebar might cover start points OR injected rebar might cover another starting point
        elif self.does_start_in_cell_contain(x, y) == True:
            if size <= (self.__start_in_cell_size + self.__start_in_cell_size * self.__start_in_cell_size_threshold) \
                    and len(self.__coords_in_x) == 0:
                return CONST.START_IN_CELL
            elif (size <= (self.__start_in_cell_size + self.__start_in_cell_size * self.__start_in_cell_size_threshold)) \
                    and (self.__coords_in_size[-1] > size * self.__ratio_size_40_rebar_by_start_rebar):
                return CONST.START_IN_CELL_INIT
            else:
                return -1  # injected rebar might cover start points OR injected rebar might cover another starting point

        return -1

    def check_status_in_progress(self, x, y, size):
        ''' check the point in progress step

        :param x:
        :param y:
        :param size: size param is used for threshold
        :return:
        '''
        dist_out_len = -1
        dist_in_len = -1

        # maximum value in integer
        dist_out = int(2147483647)
        dist_in = int(2147483647)

        if len(self.__coords_out_x) > 0:
            dist_out_x = x - self.__coords_out_x[-1]
            dist_out_y = y - self.__coords_out_y[-1]
            dist_out = math.sqrt(dist_out_x ** 2 + dist_out_y ** 2)

        if len(self.__coords_in_x) > 0:
            dist_in_x = x - self.__coords_in_x[-1]
            dist_in_y = y - self.__coords_in_y[-1]
            dist_in = math.sqrt(dist_in_x ** 2 + dist_in_y ** 2)

        if (dist_out < size * 1.5) and (dist_out < dist_in):
            return CONST.PROGRESS_OUT_CELL
        elif (dist_in < size * 1.5) and (dist_in < dist_out):
            return CONST.PROGRESS_IN_CELL
        else:
            return -1

    def check_noise(self, w, h):
        ''' check noise data using width & height size ratio

        :param w:
        :param h:
        :return:
        '''
        if (w > (h * 1.5)) or (h > (w * 1.5)):  # width or height can not be bigger than each other
            return -1

    def current_coil_thickness(self):
        ''' getter of current coil thickness

        :return:
        '''
        return self.__current_coil_thickness

    def change_current_coil_thickness(self, value):
        ''' change current coil thickness temporarily because of OPC server.
            This doesn't update base coil thickness(from tuning)
        :param id: device id
        :param value: coil thickness(10, 13, 16)
        :return:
        '''
        if self.__current_coil_thickness == value:
            return

        ratio = 1.0
        if self.__current_coil_thickness == 10:
            if value == 13:
                ratio = 1.3
            elif value == 16:
                ratio = 1.6
        elif self.__current_coil_thickness == 13:
            if value == 10:
                ratio = 1 / 1.3
            elif value == 16:
                ratio = 1.6 / 1.3
        elif self.__current_coil_thickness == 16:
            if value == 10:
                ratio = 1 / 1.6
            elif value == 13:
                ratio = 1.3 / 1.6

        self.__end_out_cell_size = int(self.__end_out_cell_size * ratio)
        self.__end_in_cell_size = int(self.__end_in_cell_size * ratio)

        self.__current_coil_thickness = value

    def alignment_interval(self):
        ''' getter of alignment interval

        :return:
        '''
        return self.__alignment_interval

    def onChange(self, pos):
        pass

    def get_alignment_values(self, index_out, index_in):
        ''' calculate alignment value

        :param index_out: out-coil cell no.
        :param index_in:  in-coil cell no.
        :return:
        '''
        comp = Compare_Model()
        comp.compare_in_cell_map(index_in)
        comp.compare_out_cell_map(index_out)

        local_s1 = s1
        local_s2 = s2
        local_s3 = s3
        local_s4 = s4
        local_s5 = s5
        local_s6 = s6
        local_s7 = s7
        local_s8 = s8

        # Using below comments, alignment values can be adjusted

        # local_s1 = int(local_s1) / 5
        # local_s2 = int(local_s2) / 5
        # local_s3 = int(local_s3) / 5
        # local_s4 = int(local_s4) / 5
        # local_s5 = int(local_s5) / 5
        # local_s6 = int(local_s6) / 5
        # local_s7 = int(local_s7) / 5
        # local_s8 = int(local_s8) / 5

        # local_s1 = int(local_s1)
        # local_s2 = int(local_s2)
        # local_s3 = int(local_s3)
        # local_s4 = int(local_s4)
        # local_s5 = int(local_s5)
        # local_s6 = int(local_s6)
        # local_s7 = int(local_s7)
        # local_s8 = int(local_s8)

        ret = "%s|%s|%s|%s|%s|%s|%s|%s|%s|%s" % (
        local_s1, local_s2, local_s3, local_s4, local_s5, local_s6, local_s7, local_s8, out_coil_cell, in_coil_cell)
        return ret

    def regress_coords(self, coords, predict_point):
        ''' predict coords

        :param coords: base data for making a model(prediction)
        :param predict_point: predict point using data length
        :return:
        '''
        list_ret = [-1, -1]

        if len(coords) == 0:
            return list_ret

        y = coords.copy()
        X = [item for item in range(1, len(y) + 1)]

        model = Ridge(alpha=1, random_state=42)
        model = Pipeline([
            ("poly_features", PolynomialFeatures(degree=2, include_bias=False)),
            ("std_scaler", StandardScaler()),
            ("regul_reg", model),
        ])

        X_for_model = np.array(X)
        X_for_model = X_for_model.reshape(len(X), 1)
        model.fit(X_for_model, y)

        list_ret = model.predict([[predict_point]])[0]

        return list_ret if len(list_ret) == 2 else [-1, -1]

    def filter_coords(self, coords):
        ''' filter coords when rebar is cut and bended

        :param coords: base data for making a model(prediction):
        :return:
        '''
        if len(coords) == 0:
            return [[]]

        y = coords.copy()
        X = [item for item in range(1, len(y) + 1)]

        model = Ridge(alpha=1, random_state=42)
        model = Pipeline([
            ("poly_features", PolynomialFeatures(degree=10, include_bias=False)),
            ("std_scaler", StandardScaler()),
            ("regul_reg", model),
        ])

        X_for_model = np.array(X)
        X_for_model = X_for_model.reshape(len(X), 1)

        model.fit(X_for_model, y)

        max_X = max(X)
        min_X = min(X)

        list_predicted_sequence_frame = model.predict(X_for_model)
        list_predicted_sequence_frame_x = [item[0] for item in list_predicted_sequence_frame]
        list_predicted_sequence_frame_y = [item[1] for item in list_predicted_sequence_frame]

        coef = model.named_steps['regul_reg'].coef_

        y_result = []
        y_result_x = []
        y_result_y = []
        for X_item in X:
            for index in range(len(coef[0])):
                if index is 0:
                    y_x = coef[0][index]
                    y_y = coef[1][index]
                else:
                    y_x = y_x + (coef[0][index] * (X_item ** index))
                    y_y = y_y + (coef[1][index] * (X_item ** index))

                y_result_x.append(y_x)
                y_result_y.append(y_y)

        # slopes
        dpy = [y_result_x[i + 1] - y_result_x[i] for i in range(len(y_result_x) - 1)]

        # slope values are too big to handle, so I adapted a normalization
        # Plus, it is easy to understand the number as percentage.
        dpy_max = max(dpy)
        dpy_min = min(dpy)
        dpy_dist = dpy_max - dpy_min

        dpy_norm = [(item - dpy_min) / dpy_dist for item in dpy]

        trim_point = -1
        min_dist = int(2147483646)
        first_dpy_norm = dpy_norm[0]
        for index, item in enumerate(dpy_norm):
            dist = abs(first_dpy_norm - item)
            if dist >= 0.05:  # 5%(value can be changed)
                trim_point = index
                break

        y_trimmed = coords.copy()

        trim_elem = trim_point
        if trim_elem > 0:
            del y_trimmed[trim_elem:]

        return y_trimmed

    def collect_coords(self, frame, outs):
        ''' organize coords data

        :param frame: it used for fining coords location
        :param outs: saved coords per frame
        :return:
        '''
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]

        frame_original = frame.copy()

        classIds = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]

                if confidence > confThreshold:
                    center_x = int(detection[0] * frameWidth)
                    center_y = int(detection[1] * frameHeight)

                    width = int(detection[2] * frameWidth)
                    height = int(detection[3] * frameHeight)
                    left = int(center_x - width / 2)
                    top = int(center_y - height / 2)

                    if self.check_noise(width, height) is -1:
                        continue

                    # coords in start_point
                    # Plus check new rebar here
                    in_start_point = self.check_status_in_start_point(center_x, center_y, width)
                    in_progress = self.check_status_in_progress(center_x, center_y, width)

                    # point in start point
                    if in_start_point > -1:
                        if in_start_point is CONST.START_OUT_CELL:
                            self.__coords_out_x.append(center_x)
                            self.__coords_out_y.append(center_y)
                            self.__coords_out_size.append(width)

                            # Uncomment below when you need to collect video data and coords data in AligningView
                            # self.__vid_now = time.strftime("%Y-%m-%d %H_%M_%S")
                            # self.__vid_file_name = "[" + self.__vid_now + "] AligningView.avi"
                            # self.__tuning_vid_writer = cv2.VideoWriter(self.__vid_file_name,
                            #                                            cv2.VideoWriter_fourcc(*'MJPG'), 30,
                            #                                            (640, 512), isColor=True)

                            # self.__vid_original_file_name = "[" + self.__vid_now + "] AligningView(Original).avi"
                            # self.__tuning_original_vid_writer = cv2.VideoWriter(self.__vid_original_file_name,
                            #                                            cv2.VideoWriter_fourcc(*'MJPG'), 30,
                            #                                            (640, 512), isColor=True)

                            drawPred(frame, left, top, (left+width), (top+height), "OUT")
                        elif in_start_point is CONST.START_IN_CELL:
                            self.__coords_in_x.append(center_x)
                            self.__coords_in_y.append(center_y)
                            self.__coords_in_size.append(width)

                            # Uncomment below when you need to collect video data and coords data in AligningView
                            # self.__vid_now = time.strftime("%Y-%m-%d %H_%M_%S")
                            # self.__vid_file_name = "[" + self.__vid_now + "] AligningView.avi"
                            # self.__vid_original_file_name = "[" + self.__vid_now + "] AligningView(Original).avi"
                            # self.__tuning_vid_writer = cv2.VideoWriter(self.__vid_file_name,
                            #                                            cv2.VideoWriter_fourcc(*'MJPG'), 30,
                            #                                            (640, 512), isColor=True)

                            # self.__vid_original_file_name = "[" + self.__vid_now + "] AligningView(Original).avi"
                            # self.__tuning_original_vid_writer = cv2.VideoWriter(self.__vid_original_file_name,
                            #                                                     cv2.VideoWriter_fourcc(*'MJPG'), 30,
                            #                                                     (640, 512), isColor=True)

                            drawPred(frame, left, top, (left+width), (top+height), "IN")
                        elif in_start_point is CONST.START_OUT_CELL_INIT:
                            # Uncomment below when you need to collect video data and coords data in AligningView
                            # self.__tuning_vid_writer.release()
                            # self.__tuning_original_vid_writer.release()

                            if len(self.__coords_out_x) > 0 and len(self.__coords_out_y) > 0 and len(
                                    self.__coords_in_x) > 0 and len(self.__coords_in_y) > 0:
                                list_out_coords = [list(pair) for pair in zip(self.__coords_out_x, self.__coords_out_y)]
                                list_in_coords = [list(pair) for pair in zip(self.__coords_in_x, self.__coords_in_y)]

                                self.__dict_current_coords = {"out": list_out_coords, "in": list_in_coords}

                                num_list_out_coords = len(list_out_coords)
                                num_list_in_coords = len(list_in_coords)
                                ratio = 1
                                if num_list_out_coords > num_list_in_coords:
                                    ratio = num_list_out_coords / num_list_in_coords
                                else:
                                    ratio = num_list_in_coords / num_list_out_coords

                                # Uncomment below when you need to collect video data and coords data in AligningView
                                # if num_list_out_coords > CONST.ALIGNMENT_END_POINTS_MINIMUM_COUNT \
                                #         and num_list_in_coords > CONST.ALIGNMENT_END_POINTS_MINIMUM_COUNT \
                                #         and ratio < CONST.ALIGNMENT_END_POINTS_MAXIMUM_DIFFERENCE_COUNT_RATIO:
                                #     txt_name = "[" + self.__vid_now + "] AligningView Data.txt"
                                #     f = open(txt_name, 'w')
                                #     f.write(str(self.__dict_current_coords))
                                #     f.close()
                                # else:
                                #     try:
                                #         os.remove(self.__vid_file_name)
                                #         os.remove(self.__vid_original_file_name)
                                #         print("file removed")
                                #     except OSError as error:
                                #         print(error)

                            self.__coords_out_x = []
                            self.__coords_out_y = []
                            self.__coords_out_size = []
                            self.__coords_in_x = []
                            self.__coords_in_y = []
                            self.__coords_in_size = []

                            self.__coords_out_x.append(center_x)
                            self.__coords_out_y.append(center_y)
                            self.__coords_out_size.append(width)

                            drawPred(frame, left, top, (left+width), (top+height), "OUT")

                        elif in_start_point is CONST.START_IN_CELL_INIT:
                            # Uncomment below when you need to collect video data and coords data in AligningView
                            # self.__tuning_vid_writer.release()
                            # self.__tuning_original_vid_writer.release()

                            if len(self.__coords_out_x) > 0 and len(self.__coords_out_y) > 0 and len(
                                    self.__coords_in_x) > 0 and len(self.__coords_in_y) > 0:
                                list_out_coords = [list(pair) for pair in zip(self.__coords_out_x, self.__coords_out_y)]
                                list_in_coords = [list(pair) for pair in zip(self.__coords_in_x, self.__coords_in_y)]

                                self.__dict_current_coords = {"out": list_out_coords, "in": list_in_coords}

                                num_list_out_coords = len(list_out_coords)
                                num_list_in_coords = len(list_in_coords)
                                ratio = 1
                                if num_list_out_coords > num_list_in_coords:
                                    ratio = num_list_out_coords / num_list_in_coords
                                else:
                                    ratio = num_list_in_coords / num_list_out_coords

                                # Uncomment below when you need to collect video data and coords data in AligningView
                                # if len(list_out_coords) > CONST.ALIGNMENT_END_POINTS_MINIMUM_COUNT \
                                #         and len(list_in_coords) > CONST.ALIGNMENT_END_POINTS_MINIMUM_COUNT \
                                #         and ratio < CONST.ALIGNMENT_END_POINTS_MAXIMUM_DIFFERENCE_COUNT_RATIO:
                                    # txt_name = "[" + self.__vid_now + "] AligningView Data.txt"
                                    # f = open(txt_name, 'w')
                                    # f.write(str(self.__dict_current_coords))
                                    # f.close()
                                # else:
                                #     try:
                                #         os.remove(self.__vid_file_name)
                                #         print("file removed")
                                #     except OSError as error:
                                #         print(error)

                            self.__coords_out_x = []
                            self.__coords_out_y = []
                            self.__coords_out_size = []
                            self.__coords_in_x = []
                            self.__coords_in_y = []
                            self.__coords_in_size = []

                            self.__coords_in_x.append(center_x)
                            self.__coords_in_y.append(center_y)
                            self.__coords_in_size.append(width)

                            drawPred(frame, left, top, (left+width), (top+height), "IN")
                    else:
                        # point in progress
                        # Uncomment below when you need to collect video data and coords data in AligningView
                        # frame_vid = cv2.resize(frame, dsize=self.__resolution, fx=self.__ratio_record, fy=self.__ratio_record, interpolation=cv2.INTER_LINEAR)
                        # frame_vid_original = cv2.resize(frame_original, dsize=self.__resolution, fx=self.__ratio_record, fy=self.__ratio_record, interpolation=cv2.INTER_LINEAR)
                        # self.__tuning_vid_writer.write(frame_vid)
                        # self.__tuning_original_vid_writer.write(frame_vid_original)

                        if in_progress > -1:
                            if in_progress is CONST.PROGRESS_OUT_CELL:
                                self.__coords_out_x.append(center_x)
                                self.__coords_out_y.append(center_y)
                                self.__coords_out_size.append(width)
                                drawPred(frame, left, top, (left + width), (top + height), "OUT")
                            elif in_progress is CONST.PROGRESS_IN_CELL:
                                self.__coords_in_x.append(center_x)
                                self.__coords_in_y.append(center_y)
                                self.__coords_in_size.append(width)
                                drawPred(frame, left, top, (left + width), (top + height), "IN")

                    if in_start_point > -1 or in_progress > -1:
                        classIds.append(classId)
                        confidences.append(float(confidence))

                        boxes.append([left, top, width, height])

                        boxes_sep = [left, top, width, height]

                        l = boxes_sep[0]
                        t = boxes_sep[1]
                        w = boxes_sep[2]
                        h = boxes_sep[3]

                        bbox_coor_x.append(l + w / 2)
                        bbox_coor_y.append(t + h / 2)

    def get_index_end_cells(self, out_x, out_y, in_x, in_y):
        ''' calculate cell numbers using coords

        :param out_x:
        :param out_y:
        :param in_x:
        :param in_y:
        :return:
        '''
        Map = Measurement_Map()

        Map.set_end_in_cell(self.__end_in_cell_size, self.__end_in_cell_x, self.__end_in_cell_y)
        Map.set_end_out_cell(self.__end_out_cell_size, self.__end_out_cell_x, self.__end_out_cell_y)

        In_index = Map.check_end_in_cell_map(in_x, in_y)
        Out_index = Map.check_end_out_cell_map(out_x, out_y)

        return [Out_index, In_index]

    def process_frame(self, frame):
        ''' main point to handle frame

        :param frame:
        :return:
        '''
        Map = Measurement_Map()

        Map.set_end_in_cell(self.__end_in_cell_size, self.__end_in_cell_x, self.__end_in_cell_y)
        Map.set_end_out_cell(self.__end_out_cell_size, self.__end_out_cell_x, self.__end_out_cell_y)

        blob = cv2.dnn.blobFromImage(frame, 1 / 255, (inpWidth, inpHeight), [0, 0, 0], 1, crop=False)
        net.setInput(blob)
        outs = net.forward(getOutputsNames(net))

        self.collect_coords(frame, outs)
        if len(self.__vid_file_name) > 0:
            resized_frame = cv2.resize(frame, dsize=self.__resolution, fx=self.__ratio_record,
                                           fy=self.__ratio_record, interpolation=cv2.INTER_LINEAR)
            self.__aligning_vid_writer.write(resized_frame)

        if len(self.__dict_current_coords.keys()) > 0:
            if self.alignment_interval() is CONST.ALIGNMENT_INTERVAL_SEND_ALWAYS:
                current_time = time.strftime("%Y-%m-%d %H_%M_%S")

                coords_out_30_history = self.__dict_current_coords['out']
                coords_in_30_history = self.__dict_current_coords['in']

                if len(coords_out_30_history) > 30 and len(coords_in_30_history) > 30:
                    filtered_out_coords = self.filter_coords(coords_out_30_history)
                    filtered_in_coords = self.filter_coords(coords_in_30_history)

                    predict_out_point = len(filtered_out_coords)
                    if self.base_out_data_length() > predict_out_point:
                        predict_out_point = self.base_out_data_length()

                    predict_in_point = len(filtered_in_coords)
                    if self.base_in_data_length() > predict_in_point:
                        predict_in_point = self.base_in_data_length()

                    end_out_coords = self.regress_coords(filtered_out_coords, predict_out_point)
                    end_in_coords = self.regress_coords(filtered_in_coords, predict_in_point)

                    list_index_end_cells = self.get_index_end_cells(end_out_coords[0], end_out_coords[1],
                                                                    end_in_coords[0], end_in_coords[1])
                    str_alignment_values = self.get_alignment_values(list_index_end_cells[0], list_index_end_cells[1])

                    self.__aligning_vid_writer.release()
                    self.__vid_file_name = ""

                    files = os.listdir()
                    path_current = os.getcwd() + "\\"
                    for str_filename in files:
                        if str_filename.find("] Alignment Processing") > -1:
                            list_str_filename = str_filename.split(" A")
                            str_filename_stdt = list_str_filename[0]
                            str_filename_updated = str_filename_stdt + " Alignment Done.mp4"
                            os.rename(path_current + str_filename, path_current + str_filename_updated)
                            break

                    if list_index_end_cells[0] != 0 or list_index_end_cells[1] != 0:
                        self.__vid_file_name = "[" + current_time + "] Alignment Processing.mp4"
                        self.__aligning_vid_writer =  cv2.VideoWriter(self.__vid_file_name,
                                                                       0x00000021, 30,
                                                                       self.resolution(), isColor=True)

                    self.signal_aligning_view_.alignment_work_done.emit(list_index_end_cells, str_alignment_values)

                self.__list_end_out_in_coords = []
                self.__list_coords_30_history = []
                self.__dict_current_coords = {}
            elif self.alignment_interval() is CONST.ALIGNMENT_INTERVAL_SEND_ONCE_EVERY_THREE_TIMES:
                if self.__interval_count >= 3:
                    self.__interval_count = 0

                    current_time = time.strftime("%Y-%m-%d %H_%M_%S")

                    coords_out_30_history = self.__dict_current_coords['out']
                    coords_in_30_history = self.__dict_current_coords['in']

                    if len(coords_out_30_history) > 30 and len(coords_in_30_history) > 30:
                        filtered_out_coords = self.filter_coords(coords_out_30_history)
                        filtered_in_coords = self.filter_coords(coords_in_30_history)

                        predict_out_point = len(filtered_out_coords)
                        if self.base_out_data_length() > predict_out_point:
                            predict_out_point = self.base_out_data_length()

                        predict_in_point = len(filtered_in_coords)
                        if self.base_in_data_length() > predict_in_point:
                            predict_in_point = self.base_in_data_length()

                        end_out_coords = self.regress_coords(filtered_out_coords, predict_out_point)
                        end_in_coords = self.regress_coords(filtered_in_coords, predict_in_point)

                        list_index_end_cells = self.get_index_end_cells(end_out_coords[0], end_out_coords[1],
                                                                        end_in_coords[0], end_in_coords[1])
                        str_alignment_values = self.get_alignment_values(list_index_end_cells[0],
                                                                         list_index_end_cells[1])

                        self.__aligning_vid_writer.release()
                        self.__vid_file_name = ""

                        files = os.listdir()
                        path_current = os.getcwd() + "\\"
                        for str_filename in files:
                            if str_filename.find("] Alignment Processing") > -1:
                                list_str_filename = str_filename.split(" A")
                                str_filename_stdt = list_str_filename[0]
                                str_filename_updated = str_filename_stdt + " Alignment Done.mp4"
                                os.rename(path_current + str_filename, path_current + str_filename_updated)
                                break

                        if list_index_end_cells[0] != 0 or list_index_end_cells[1] != 0:
                            self.__vid_file_name = "[" + current_time + "] Alignment Processing.mp4"
                            self.__aligning_vid_writer = cv2.VideoWriter(self.__vid_file_name,
                                                                         0x00000021, 30,
                                                                         self.resolution(), isColor=True)

                        self.signal_aligning_view_.alignment_work_done.emit(list_index_end_cells, str_alignment_values)

                else:
                    self.__interval_count = self.__interval_count + 1

                self.__list_end_out_in_coords = []
                self.__list_coords_30_history = []
                self.__dict_current_coords = {}
            else:
                self.__list_end_out_in_coords = []
                self.__list_coords_30_history = []
                self.__dict_current_coords = {}

        return frame