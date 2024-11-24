# coding:UTF-8
from cmr_imu.i_data_processor import IDataProcessor

"""
    JY901S数据处理器 Data Processor
"""


class JY901SDataProcessor(IDataProcessor):
    onVarChanged = []
    def onOpen(self, deviceModel):
        pass

    def onClose(self):
        pass

    @staticmethod
    def onUpdate(*args):
        for fun in JY901SDataProcessor.onVarChanged:
            fun(*args)