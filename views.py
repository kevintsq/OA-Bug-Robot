import json
import math
import sys

from PyQt5.QtChart import QValueAxis, QSplineSeries, QChart, QChartView, QPolarChart, QScatterSeries
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel, QProgressBar, QDoubleSpinBox, QPushButton, \
    QComboBox
from PyQt5.QtGui import QPainter, QImage, QPixmap, QIcon
from PyQt5.QtCore import Qt

PI2 = math.pi * 2
TAG_IDS = ["8B", "AC", "C3", "D2"]


class ControlPanel(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        controllerLabel = QLabel("Controller", self)
        self.controllerDropdown = QComboBox(self)
        self.controllerButton = QPushButton(self)
        self.controllerButton.setText("Connect")

        auditoryLabel = QLabel("Auditory", self)
        self.auditoryDropdown = QComboBox(self)
        self.auditoryButton = QPushButton(self)
        self.auditoryButton.setText("Connect")

        olfactoryLabel = QLabel("Olfactory", self)
        self.olfactoryDropdown = QComboBox(self)
        self.olfactoryButton = QPushButton(self)
        self.olfactoryButton.setText("Connect")

        motionLabel = QLabel("Motion", self)
        self.motionDropdown = QComboBox(self)
        self.motionButton = QPushButton(self)
        self.motionButton.setText("Connect")

        batteryVoltageLabel = QLabel("Voltage", self)
        self.batteryVoltage = QLabel(self)

        batteryCurrentLabel = QLabel("Current", self)
        self.batteryCurrent = QLabel(self)

        batteryTemperatureLabel = QLabel("Temperature", self)
        self.batteryTemperature = QLabel(self)

        batteryRemainingLabel = QLabel("Remaining", self)
        self.batteryRemaining = QProgressBar(self)

        xSpeedLabel = QLabel('xSpeed', self)
        self.xSpeedSpinBox = QDoubleSpinBox(self)
        self.xSpeedSpinBox.setDecimals(1)
        self.xSpeedSpinBox.setSingleStep(0.1)
        self.xSpeedSpinBox.setMinimum(-2)
        zSpeedLabel = QLabel('zSpeed', self)
        self.zSpeedSpinBox = QDoubleSpinBox(self)
        self.zSpeedSpinBox.setDecimals(1)
        self.zSpeedSpinBox.setSingleStep(0.1)
        self.zSpeedSpinBox.setMinimum(-2)
        self.goButton = QPushButton("Go", self)
        self.stopButton = QPushButton("Stop", self)
        self.startButton = QPushButton("Start the Algorithm", self)

        layout = QGridLayout()
        # row, column, rowspan, columnspan
        layout.addWidget(controllerLabel, 0, 0)
        layout.addWidget(self.controllerDropdown, 0, 1)
        layout.addWidget(self.controllerButton, 0, 2)
        layout.addWidget(auditoryLabel, 1, 0)
        layout.addWidget(self.auditoryDropdown, 1, 1)
        layout.addWidget(self.auditoryButton, 1, 2)
        layout.addWidget(olfactoryLabel, 2, 0)
        layout.addWidget(self.olfactoryDropdown, 2, 1)
        layout.addWidget(self.olfactoryButton, 2, 2)
        layout.addWidget(motionLabel, 3, 0)
        layout.addWidget(self.motionDropdown, 3, 1)
        layout.addWidget(self.motionButton, 3, 2)

        layout.addWidget(batteryVoltageLabel, 4, 0)
        layout.addWidget(self.batteryVoltage, 4, 1, 1, 2)
        layout.addWidget(batteryCurrentLabel, 5, 0)
        layout.addWidget(self.batteryCurrent, 5, 1, 1, 2)
        layout.addWidget(batteryTemperatureLabel, 6, 0)
        layout.addWidget(self.batteryTemperature, 6, 1, 1, 2)
        layout.addWidget(batteryRemainingLabel, 7, 0)
        layout.addWidget(self.batteryRemaining, 7, 1, 1, 2)

        layout.addWidget(xSpeedLabel, 8, 0)
        layout.addWidget(self.xSpeedSpinBox, 8, 1, 1, 2)
        layout.addWidget(zSpeedLabel, 9, 0)
        layout.addWidget(self.zSpeedSpinBox, 9, 1, 1, 2)
        layout.addWidget(self.goButton, 10, 0, 1, 2)
        layout.addWidget(self.stopButton, 10, 2)
        layout.addWidget(self.startButton, 11, 0, 1, 3)
        self.setLayout(layout)


class ChartView(QChartView):
    def __init__(self, parent, titleName, titleUnit, seriesNames, *args):
        super().__init__(parent, *args)
        self.setRenderHint(QPainter.Antialiasing)
        self.xAxis = QValueAxis()
        self.xAxis.setTitleText("Time (s)")
        self.xOriginal = 32
        self.xMax = self.xOriginal
        self.xAxis.setMax(self.xMax)
        self.setXaxisRange = False
        self.yAxis = QValueAxis()
        self.yAxis.setTitleText(f"{titleName} ({titleUnit})")
        self.yMax = 0
        self.yMin = 0
        self.chart = QChart()
        self.chart.addAxis(self.xAxis, Qt.AlignBottom)
        self.chart.addAxis(self.yAxis, Qt.AlignLeft)
        self.chart.setAnimationOptions(QChart.SeriesAnimations)  # AllAnimations
        self.series = {}
        for name in seriesNames:
            s = QSplineSeries()
            s.setUseOpenGL(True)
            s.setName(name)
            self.chart.addSeries(s)
            # must be put after chart is available
            s.attachAxis(self.xAxis)
            s.attachAxis(self.yAxis)
            s.pointAdded.connect(self.redraw)
            self.series[name] = s
        # self.series.setPointLabelsVisible(True)
        self.setChart(self.chart)

    def redraw(self, index):
        self.yAxis.setRange(self.yMin - 2, self.yMax + 2)
        self.xAxis.setRange(self.xMax - self.xOriginal, self.xMax)

    def update(self, timeStamp, key, value):
        if value > self.yMax:
            self.yMax = value
        if timeStamp > self.xMax:
            self.xMax = timeStamp
        self.series[key].append(timeStamp, value)


class PolarView(QChartView):
    def __init__(self, parent, pointNames, *args):
        super().__init__(parent, *args)
        self.setRenderHint(QPainter.Antialiasing)
        self.polarAxis = QValueAxis()
        self.chart = QPolarChart()
        self.chart.addAxis(self.polarAxis, QPolarChart.PolarOrientationRadial)
        self.points = {}
        for name in pointNames:
            p = QScatterSeries()
            p.setUseOpenGL(True)
            p.setMarkerSize(20)
            p.setName(name)
            self.chart.addSeries(p)
            p.attachAxis(self.polarAxis)
            p.setPointLabelsVisible(True)
            self.points[name] = p
        self.setChart(self.chart)

    def updatePoints(self, tagId, angle):
        self.points[tagId].clear()
        if angle < 0:
            angle += 360
        self.points[tagId].append(angle / 360, 0.75)


class CameraView(QLabel):
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        # self.setAlignment(Qt.AlignCenter)

    def updateImage(self, image):
        image = QImage(image.data, image.shape[1], image.shape[0], QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap(image)
        self.setPixmap(pixmap)
        #    .scaled(self.parent().width() // 4, self.parent().height() // 2, Qt.KeepAspectRatio))


class CompassView(QChartView):
    def __init__(self, parent, *args):
        super().__init__(parent, *args)
        self.setRenderHint(QPainter.Antialiasing)
        self.polarAxis = QValueAxis()
        self.points = QScatterSeries()
        self.points.setUseOpenGL(True)
        self.points.setName("Yaw")
        self.chart = QPolarChart()
        self.chart.addSeries(self.points)
        self.chart.addAxis(self.polarAxis, QPolarChart.PolarOrientationRadial)
        self.points.attachAxis(self.polarAxis)
        self.points.setPointLabelsVisible(True)
        self.setChart(self.chart)
        self.pointsCount = 0

    def updateCompass(self, data):
        self.points.clear()
        # mag = data["magneticIntensity"]
        # yaw = -math.atan2(mag["y"], mag["x"])
        # if yaw < 0:
        #     yaw += PI2
        # self.points.append(yaw / PI2, 0.75)
        yaw = data["azimuth"]["yaw"]
        if yaw < 0:
            yaw += 360
        self.points.append(yaw / 360, 0.75)


class MainWindow(QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle("Robot Studio")
        self.setWindowState(Qt.WindowMaximized)
        self.ethanolChartView = ChartView(self, "Ethanol Concentration", "raw", ["value"])
        # self.h2ChartView = ChartView(self, "H2", "raw")
        # self.tVocCharView = ChartView(self, "tVOC", "ppb")
        # self.co2ChartView = ChartView(self, "CO2eq", "ppm")
        # self.cameraView = CameraView(self)
        self.distanceView = ChartView(self, "Distance from Other Robots", "m", TAG_IDS)
        self.elevationView = ChartView(self, "Elevation to Other Robots", "cm", TAG_IDS)
        self.soundDirectionView = PolarView(self, TAG_IDS)
        self.compassView = CompassView(self)
        self.controlPanel = ControlPanel(self)

        layout = QGridLayout()
        layout.addWidget(self.ethanolChartView, 0, 0)
        # layout.addWidget(self.h2ChartView, 0, 1)
        layout.addWidget(self.compassView, 0, 1)
        layout.addWidget(self.controlPanel, 0, 2)
        # layout.addWidget(self.cameraView, 0, 3)
        # layout.addWidget(self.tVocCharView, 1, 0)
        # layout.addWidget(self.co2ChartView, 1, 1)
        layout.addWidget(self.soundDirectionView, 1, 0)
        layout.addWidget(self.distanceView, 1, 1)
        layout.addWidget(self.elevationView, 1, 2)
        self.setLayout(layout)

    def updateCharts(self, data):
        timeStamp = data["timeStamp"] // 50
        self.ethanolChartView.update(timeStamp, "value", data["value"])
        # self.h2ChartView.series.append(timeStamp, data["H2"])
        # self.tVocCharView.series.append(timeStamp, data["tVOC"])
        # self.co2ChartView.series.append(timeStamp, data["CO2"])

    def updateSoundSphere(self, data):
        tagId = data["tagId"]
        timeStamp = data["timeStamp"]
        if timeStamp % 10 == 0:
            self.soundDirectionView.updatePoints(tagId, data["azimuth"])
        if timeStamp % 20 == 0:
            timeStamp //= 20
            self.distanceView.update(timeStamp, tagId, data["distance"])
            self.elevationView.update(timeStamp, tagId, data["elevation"])

    # def updateImage(self, data):
    #     self.cameraView.updateImage(data)

    def updateCompass(self, data):
        self.compassView.updateCompass(data)


def test():
    IPC_TYPE = "SOCKET"

    if IPC_TYPE == "SOCKET":
        import socket

        HOST = ''
        PORT = 8080
        TRUNK_SIZE = 1024
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((HOST, PORT))
            s.listen(1)
            connection, address = s.accept()
            with connection:
                print(f'Connected by {address}.')
                buffer = b''
                while True:
                    while b'\r' not in buffer:
                        data = connection.recv(TRUNK_SIZE)
                        if not data:
                            print(f"Connection closed from {address}.")
                            sys.exit(0)
                        buffer += data
                    data_list = buffer.split(b'\r')
                    # print(data_list)
                    last_one = data_list[-1]
                    if last_one == b'\n' or last_one == b'':
                        buffer = b''
                    else:
                        buffer = last_one
                    if len(data_list) > 1:
                        try:
                            result = json.loads(data_list[-2])
                        except:
                            print("Bad format.", file=sys.stderr)
                            continue
                        if result["type"] == "auditory":
                            mainWindow.updateSoundSphere(result)
                        elif result["type"] == "olfactory":
                            if result["timeStamp"] % 50 == 0:
                                print(result)
                                mainWindow.updateCharts(result)
                        elif result["type"] == "motion":
                            mainWindow.updateCompass(result)
                        else:
                            print("Bad format.", file=sys.stderr)
    elif IPC_TYPE == "PIPE":
        import subprocess

        with subprocess.Popen("./client", stdout=subprocess.PIPE, text=True) as p:
            while True:
                data = p.stdout.readline()
                if not data:
                    print("Connection closed.")
                    break
                print(data, end='')


if __name__ == "__main__":
    from threading import Thread

    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon("plane.svg"))
    app.beep()
    mainWindow = MainWindow()


    def run():
        mainWindow.show()
        sys.exit(app.exec())


    test_thread = Thread(target=test, args=(), daemon=True)
    test_thread.start()
    run()
