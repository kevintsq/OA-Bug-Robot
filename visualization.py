import math
import sys

from PyQt5.QtChart import QValueAxis, QSplineSeries, QChart, QChartView, QPolarChart, QScatterSeries
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QLabel
from PyQt5.QtGui import QPainter, QImage, QPixmap, QIcon
from PyQt5.QtCore import Qt


PI2 = math.pi * 2


class ChartView(QChartView):
    def __init__(self, parent, titleName, titleUnit, *args):
        super().__init__(parent, *args)
        self.setRenderHint(QPainter.Antialiasing)
        self.xAxis = QValueAxis()
        self.xAxis.setTitleText("Time (s)")
        self.xOriginal = 32
        self.xMax = self.xOriginal
        self.xAxis.setMax(self.xMax)
        self.setXaxisRange = False
        self.yAxis = QValueAxis()
        self.yAxis.setTitleText(f"{titleName} Concentration ({titleUnit})")
        self.yMax = 0
        self.yMin = 0
        self.firstTime = True
        self.series = QSplineSeries()
        self.series.setUseOpenGL(True)
        self.series.setName(titleName)
        self.chart = QChart()
        self.chart.addSeries(self.series)
        self.chart.addAxis(self.xAxis, Qt.AlignBottom)
        self.chart.addAxis(self.yAxis, Qt.AlignLeft)
        self.chart.setAnimationOptions(QChart.SeriesAnimations)  # AllAnimations
        # must be put after chart is available
        self.series.attachAxis(self.xAxis)
        self.series.attachAxis(self.yAxis)
        # self.series.setPointLabelsVisible(True)
        self.series.pointAdded.connect(self.redraw)
        self.setChart(self.chart)

    def redraw(self, index):
        y = self.series.at(index).y()
        if y > self.yMax:
            self.yMax = y
        if y < self.yMin:
            self.yMin = y
        elif self.firstTime:
            self.firstTime = False
            self.yMin = y
        self.yAxis.setRange(self.yMin - 2, self.yMax + 2)
        if self.series.count() > self.xOriginal:
            self.xMax += 1
            self.xAxis.setRange(self.xMax - self.xOriginal, self.xMax)


class PolarView(QChartView):
    def __init__(self, parent, *args):
        super().__init__(parent, *args)
        self.setRenderHint(QPainter.Antialiasing)
        self.polarAxis = QValueAxis()
        self.points = QScatterSeries()
        self.points.setUseOpenGL(True)
        self.points.setName("Sound Directions")
        self.chart = QPolarChart()
        self.chart.addSeries(self.points)
        self.chart.addAxis(self.polarAxis, QPolarChart.PolarOrientationRadial)
        self.points.attachAxis(self.polarAxis)
        self.points.setPointLabelsVisible(True)
        self.setChart(self.chart)
        self.pointsCount = 0

    def updatePoints(self, data):
        self.points.clear()
        for i in data:
            if i["tag"] == "dynamic":
                angle = -math.atan2(i["y"], i["x"])
                if angle < 0:
                    angle += PI2
                self.points.append(angle / PI2, 0.75)


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
        self.setWindowTitle("When ready, press the button on the robot to start...")
        self.setWindowState(Qt.WindowMaximized)
        self.ethanolChartView = ChartView(self, "Ethanol", "raw")
        self.h2ChartView = ChartView(self, "H2", "raw")
        self.tVocCharView = ChartView(self, "tVOC", "ppb")
        self.co2ChartView = ChartView(self, "CO2eq", "ppm")
        # self.cameraView = CameraView(self)
        self.soundDirectionView = PolarView(self)
        self.compassView = CompassView(self)

        layout = QGridLayout()
        layout.addWidget(self.ethanolChartView, 0, 0)
        layout.addWidget(self.h2ChartView, 0, 1)
        layout.addWidget(self.compassView, 0, 2)
        # layout.addWidget(self.cameraView, 0, 3)
        layout.addWidget(self.tVocCharView, 1, 0)
        layout.addWidget(self.co2ChartView, 1, 1)
        layout.addWidget(self.soundDirectionView, 1, 2)
        self.setLayout(layout)

    def updateCharts(self, data):
        timeStamp = data["timeStamp"]
        self.ethanolChartView.series.append(timeStamp, data["Ethanol"])
        self.h2ChartView.series.append(timeStamp, data["H2"])
        self.tVocCharView.series.append(timeStamp, data["tVOC"])
        self.co2ChartView.series.append(timeStamp, data["CO2"])

    def updateSoundSphere(self, data):
        self.soundDirectionView.updatePoints(data)

    def updateImage(self, data):
        self.cameraView.updateImage(data)

    def updateCompass(self, data):
        self.compassView.updateCompass(data)


QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
app = QApplication(sys.argv)
app.setWindowIcon(QIcon("plane.svg"))
app.beep()
mainWindow = MainWindow()


def run():
    mainWindow.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    run()
