package org.firstinspires.ftc.teamcode.TeamElementDetection.Pipeline;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TeamElementDetection.Detection;

public class OwnPipeline extends OpenCvPipeline {
    private Telemetry telemetry;
    private Detection detection;
    public static double mindist = 141;
    public static double low_h = 10;
    public static double low_s = 150;
    public static double low_v = 180;
    public static double high_h = 30;
    public static double high_s = 255;
    public static double high_v = 255;
    // Минимальная и максимальная площадь контура
    private static final double MIN_CONTOUR_AREA = 30000.0/4;
    private static final double MAX_CONTOUR_AREA = 110000.0/4;

    // Цветовые границы для фильтрации в HSV
    private static final Scalar LOWER_HSV_BOUND = new Scalar(low_h, low_s, low_v);
    private static final Scalar UPPER_HSV_BOUND = new Scalar(high_h, high_s, high_v);

    // Цвета для отображения
    private static final Scalar BLACK = new Scalar(0, 0, 0);
    private static final Scalar WHITE = new Scalar(255, 255, 255);
    private static final Scalar RED = new Scalar(0, 0, 255);
    private static final Scalar GREEN = new Scalar(0, 255, 0);
    private static final Scalar BLUE = new Scalar(255, 0, 0);

    private Point cameraCenter = new Point(320, 180); // Центр камеры
    private boolean detected = false; // Флаг, показывающий, был ли найден объект
    public OwnPipeline(Telemetry telemetry, Detection detection) {
        this.telemetry = telemetry;
        this.detection = detection;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat darker = new Mat();
        Core.convertScaleAbs(input, darker, 0.5, 0);

        Mat hsv = new Mat();
        Imgproc.cvtColor(darker, hsv, Imgproc.COLOR_RGB2HSV);

        Mat mask = new Mat();
        Core.inRange(hsv, LOWER_HSV_BOUND, UPPER_HSV_BOUND, mask);

        Imgproc.GaussianBlur(mask, mask, new Size(15, 15), 0);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        detected = false;
        Point nearestCenter = new Point();
        double nearestAngle = 0;
        double minDistance = Double.MAX_VALUE;

        for (MatOfPoint contour : contours) {
            double contourArea = Imgproc.contourArea(contour);
            if (contourArea < MIN_CONTOUR_AREA || contourArea > MAX_CONTOUR_AREA) {
                continue;
            }

            detected = true;
            RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));

            drawRotatedRect(darker, rotatedRect);
            Point objectCenter = rotatedRect.center;
            double distance = calculateDistance(new Point(cameraCenter.x / 2, cameraCenter.y / 2), objectCenter);
            double angle = adjustedAngle(rotatedRect);
            if (distance < minDistance) {
                minDistance = distance;
                nearestCenter = objectCenter;
                nearestAngle = angle;
            }

            Imgproc.circle(darker, objectCenter, 5, GREEN, -1);
            annotateText(darker, String.format("Angle: %.2f", angle), new Point(objectCenter.x - 20, objectCenter.y - 20), BLUE);
        }
        detection.status(detected);
        if (detected) {
            drawNearestLines(darker, new Point(cameraCenter.x, cameraCenter.y), nearestCenter);
            Imgproc.circle(darker, nearestCenter, 5, GREEN, -1);
            annotateText(darker, String.format("Nearest: %.2f, %.2f, %.2f", nearestCenter.x, nearestCenter.y, nearestAngle), new Point(nearestCenter.x - 40, nearestCenter.y - 40), GREEN);

            telemetry.addData("x:", nearestCenter.x - (cameraCenter.x));
            telemetry.addData("y:", nearestCenter.y - (cameraCenter.y));
            telemetry.addData("Angle:", nearestAngle);
            detection.updateDistances(nearestCenter.x - (cameraCenter.x), nearestCenter.y - (cameraCenter.y), nearestAngle);
        } else {

            telemetry.addData("Detected", detected);
        }

        telemetry.update();

        Mat output = new Mat();
        Imgproc.resize(darker, output, input.size());

        return output;
    }

    private void drawRotatedRect(Mat input, RotatedRect rect) {
        Point[] rectPoints = new Point[4];
        rect.points(rectPoints);
        for (int j = 0; j < 4; j++) {
            Imgproc.line(input, rectPoints[j], rectPoints[(j + 1) % 4], BLACK, 2);
        }
    }

    private double calculateDistance(Point p1, Point p2) {
        return Math.hypot(p1.x - p2.x, p1.y - p2.y);
    }

    private double adjustedAngle(RotatedRect rect) {
        double angle = rect.angle;
        if (rect.size.width < rect.size.height) {
            angle += 90;
        }
        return angle - 90;
    }

    private void drawNearestLines(Mat input, Point cameraCenter, Point nearestCenter) {
        Imgproc.line(input, cameraCenter, new Point(cameraCenter.x, nearestCenter.y), BLUE, 4);
        Imgproc.line(input, new Point(cameraCenter.x, nearestCenter.y), nearestCenter, BLUE, 4);
    }

    private void annotateText(Mat input, String text, Point position, Scalar color) {
        Imgproc.putText(input, text, position, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }
}
