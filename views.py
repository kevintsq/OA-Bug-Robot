import math
import sys

from PyQt5.QtChart import QValueAxis, QSplineSeries, QChart, QChartView, QPolarChart, QScatterSeries
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel, QProgressBar, QDoubleSpinBox, QPushButton, \
    QComboBox, QSizePolicy, QLineEdit, QSpinBox
from PyQt5.QtGui import QPainter, QImage, QPixmap, QIcon
from PyQt5.QtCore import Qt, pyqtSignal


PI2 = math.pi * 2
TAG_IDS = ["8B", "AC", "C3", "D2"]


class SmallControl(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.leftButton = QPushButton("Left", self)
        self.leftButton.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.rightButton = QPushButton("Right", self)
        self.rightButton.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.frontButton = QPushButton("Front", self)
        self.frontButton.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self.backButton = QPushButton("Back", self)
        self.backButton.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        layout = QGridLayout()
        layout.addWidget(self.leftButton, 1, 0, 2, 2)
        layout.addWidget(self.rightButton, 1, 4, 2, 2)
        layout.addWidget(self.frontButton, 0, 2, 2, 2)
        layout.addWidget(self.backButton, 2, 2, 2, 2)
        self.setLayout(layout)


class AutoUpdatedComboBox(QComboBox):
    update_signal = pyqtSignal()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def showPopup(self):
        self.update_signal.emit()
        super().showPopup()


class ControlPanel(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        controllerLabel = QLabel("Controller", self)
        self.controllerDropdown = AutoUpdatedComboBox(self)
        self.controllerButton = QPushButton("Connect", self)
        self.controllerDropdown.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)

        visionLabel = QLabel("Vision", self)
        self.visionDropdown = AutoUpdatedComboBox(self)
        self.visionButton = QPushButton("Connect", self)
        self.visionDropdown.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)

        auditoryLabel = QLabel("Auditory", self)
        self.auditoryDropdown = AutoUpdatedComboBox(self)
        self.auditoryButton = QPushButton("Connect", self)
        self.auditoryDropdown.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)

        olfactoryLabel = QLabel("Olfactory", self)
        self.olfactoryDropdown = AutoUpdatedComboBox(self)
        self.olfactoryButton = QPushButton("Connect", self)
        self.olfactoryDropdown.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)

        motionLabel = QLabel("Motion", self)
        self.motionDropdown = AutoUpdatedComboBox(self)
        self.motionButton = QPushButton("Connect", self)
        self.motionDropdown.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)

        # batteryVoltageLabel = QLabel("Voltage", self)
        # self.batteryVoltage = QLabel(self)

        # batteryCurrentLabel = QLabel("Current", self)
        # self.batteryCurrent = QLabel(self)

        # batteryTemperatureLabel = QLabel("Temperature", self)
        # self.batteryTemperature = QLabel(self)

        # batteryRemainingLabel = QLabel("Remaining", self)
        # self.batteryRemaining = QProgressBar(self)

        olfactoryThreshLabel = QLabel('Olfactory Thresh')
        self.olfactoryThresh = QSpinBox(self)
        self.olfactoryThresh.setSingleStep(25)
        self.olfactoryThresh.setMinimum(0)
        self.olfactoryThresh.setMaximum(1023)

        pumpOpenSecLabel = QLabel('Pump Open Seconds')
        self.pumpOpenSec = QDoubleSpinBox(self)
        self.pumpOpenSec.setDecimals(2)
        self.pumpOpenSec.setMinimum(0.35)
        self.pumpOpenSec.setSingleStep(0.05)

        pumpCloseSecLabel = QLabel('Pump Close Seconds')
        self.pumpCloseSec = QSpinBox(self)

        self.pumpDebugButton = QPushButton("Debug Pump", self)
        self.pumpDebugButton.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)

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
        self.smallControl = SmallControl(self)

        self.cmd_box = QLineEdit(self)
        self.cmd_box.setPlaceholderText("Enter valid Python expression to run")
        self.exec_button = QPushButton("Execute", self)

        layout = QGridLayout()
        # row, column, rowspan, columnspan
        layout.addWidget(controllerLabel, 0, 0)
        layout.addWidget(self.controllerDropdown, 0, 1)
        layout.addWidget(self.controllerButton, 0, 2)
        layout.addWidget(visionLabel, 1, 0)
        layout.addWidget(self.visionDropdown, 1, 1)
        layout.addWidget(self.visionButton, 1, 2)
        layout.addWidget(auditoryLabel, 2, 0)
        layout.addWidget(self.auditoryDropdown, 2, 1)
        layout.addWidget(self.auditoryButton, 2, 2)
        layout.addWidget(olfactoryLabel, 3, 0)
        layout.addWidget(self.olfactoryDropdown, 3, 1)
        layout.addWidget(self.olfactoryButton, 3, 2)
        layout.addWidget(motionLabel, 4, 0)
        layout.addWidget(self.motionDropdown, 4, 1)
        layout.addWidget(self.motionButton, 4, 2)

        # layout.addWidget(batteryVoltageLabel, 5, 0)
        # layout.addWidget(self.batteryVoltage, 5, 1, 1, 2)
        # layout.addWidget(batteryCurrentLabel, 6, 0)
        # layout.addWidget(self.batteryCurrent, 6, 1, 1, 2)
        # layout.addWidget(batteryTemperatureLabel, 7, 0)
        # layout.addWidget(self.batteryTemperature, 7, 1, 1, 2)
        # layout.addWidget(batteryRemainingLabel, 8, 0)
        # layout.addWidget(self.batteryRemaining, 8, 1, 1, 2)

        layout.addWidget(olfactoryThreshLabel, 5, 0)
        layout.addWidget(self.olfactoryThresh, 5, 1)
        layout.addWidget(pumpOpenSecLabel, 6, 0)
        layout.addWidget(self.pumpOpenSec, 6, 1)
        layout.addWidget(pumpCloseSecLabel, 7, 0)
        layout.addWidget(self.pumpCloseSec, 7, 1)
        layout.addWidget(self.pumpDebugButton, 5, 2, 3, 1)

        layout.addWidget(xSpeedLabel, 9, 0)
        layout.addWidget(self.xSpeedSpinBox, 9, 1)
        layout.addWidget(zSpeedLabel, 10, 0)
        layout.addWidget(self.zSpeedSpinBox, 10, 1)
        layout.addWidget(self.goButton, 11, 0)
        layout.addWidget(self.stopButton, 11, 1)
        layout.addWidget(self.startButton, 12, 0, 1, 2)
        layout.addWidget(self.smallControl, 9, 2, 4, 1)
        layout.addWidget(self.cmd_box, 13, 0, 1, 2)
        layout.addWidget(self.exec_button, 13, 2)
        self.setLayout(layout)

    def setConnectionButtonsEnabled(self, enabled):
        self.controllerButton.setEnabled(enabled)
        self.visionButton.setEnabled(enabled)
        self.auditoryButton.setEnabled(enabled)
        self.olfactoryButton.setEnabled(enabled)
        self.motionButton.setEnabled(enabled)

    def setControlButtonsEnabled(self, enabled):
        self.goButton.setEnabled(enabled)
        self.stopButton.setEnabled(enabled)
        self.startButton.setEnabled(enabled)
        self.smallControl.frontButton.setEnabled(enabled)
        self.smallControl.backButton.setEnabled(enabled)
        self.smallControl.leftButton.setEnabled(enabled)
        self.smallControl.rightButton.setEnabled(enabled)


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

    def updateCompass(self, yaw):
        self.points.clear()
        # mag = data["magneticIntensity"]
        # yaw = -math.atan2(mag["y"], mag["x"])
        # if yaw < 0:
        #     yaw += PI2
        # self.points.append(yaw / PI2, 0.75)
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

    # def updateImage(self, data):
    #     self.cameraView.updateImage(data)


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


    run()
