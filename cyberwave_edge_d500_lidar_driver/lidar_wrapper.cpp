#include "ldlidar_driver.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>

extern "C" {
    typedef struct {
        float angle;
        uint16_t distance;
        uint8_t intensity;
        double x;
        double y;
    } Point;

    typedef struct {
        Point* points;
        int count;
        double stamp;
    } ScanResult;

    ldlidar::LDLidarDriver* driver_ptr = nullptr;

    uint64_t GetSystemTimeStamp(void) {
        auto tp = std::chrono::high_resolution_clock::now();
        auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
        return (uint64_t)tmp.count();
    }

    bool lidar_start(const char* product, const char* port) {
        if (driver_ptr) return true;
        driver_ptr = new ldlidar::LDLidarDriver();
        driver_ptr->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp));
        driver_ptr->EnableFilterAlgorithnmProcess(true);

        ldlidar::LDType type = ldlidar::LDType::LD_19;
        if (std::string(product) == "LD06") type = ldlidar::LDType::LD_06;

        if (!driver_ptr->Start(type, port, 230400, ldlidar::COMM_SERIAL_MODE)) {
            return false;
        }
        return driver_ptr->WaitLidarCommConnect(3500);
    }

    ScanResult lidar_get_scan() {
        ScanResult res = {nullptr, 0, 0.0};
        if (!driver_ptr) return res;

        static ldlidar::Points2D laser_scan_points;
        if (driver_ptr->GetLaserScanData(laser_scan_points, 1500) == ldlidar::LidarStatus::NORMAL) {
            res.count = laser_scan_points.size();
            res.points = (Point*)malloc(sizeof(Point) * res.count);
            res.stamp = (double)laser_scan_points.front().stamp / 1e9;
            for (int i = 0; i < res.count; ++i) {
                res.points[i].angle = laser_scan_points[i].angle;
                res.points[i].distance = laser_scan_points[i].distance;
                res.points[i].intensity = laser_scan_points[i].intensity;
                res.points[i].x = laser_scan_points[i].x;
                res.points[i].y = laser_scan_points[i].y;
            }
        }
        return res;
    }

    void lidar_free_scan(ScanResult res) {
        if (res.points) free(res.points);
    }

    void lidar_stop() {
        if (driver_ptr) {
            driver_ptr->Stop();
            delete driver_ptr;
            driver_ptr = nullptr;
        }
    }
}
