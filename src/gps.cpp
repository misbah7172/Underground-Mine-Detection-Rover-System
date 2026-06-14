#include "gps.h"
#include "config.h"

GPSModule::GPSModule(HardwareSerial& serial) : hwSerial(serial), lastFeedMillis(0) {}

void GPSModule::begin(unsigned long baud) {
    hwSerial.begin(baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void GPSModule::feed() {
    while (hwSerial.available()) {
        gps.encode(hwSerial.read());
    }
}

GPSData GPSModule::getData() {
    GPSData d;
    d.valid = gps.location.isValid();
    if (d.valid) {
        double lat = gps.location.lat();
        double lon = gps.location.lng();

        // add to sliding window buffer for temporal filtering
        latBuf[idx] = lat;
        lonBuf[idx] = lon;
        idx++;
        if (idx >= windowSize) { idx = 0; bufferFull = true; }

        // compute average excluding outliers (simple method: median then average of within threshold)
        size_t count = bufferFull ? windowSize : idx;
        double latArr[16]; double lonArr[16];
        for (size_t i=0;i<count;i++) { latArr[i]=latBuf[i]; lonArr[i]=lonBuf[i]; }

        // compute median
        auto median = [&](double* arr, size_t n)->double {
            // simple selection sort copy
            double tmp[16]; for (size_t i=0;i<n;i++) tmp[i]=arr[i];
            for (size_t i=0;i<n;i++) {
                size_t minj=i;
                for (size_t j=i+1;j<n;j++) if (tmp[j]<tmp[minj]) minj=j;
                double v = tmp[i]; tmp[i]=tmp[minj]; tmp[minj]=v;
            }
            if (n==0) return 0.0;
            if (n%2==1) return tmp[n/2];
            return 0.5*(tmp[n/2-1]+tmp[n/2]);
        };

        double latMed = median(latArr, count);
        double lonMed = median(lonArr, count);

        // average points within threshold (e.g., 30 meters)
        const double OUTLIER_METERS = 30.0;
        double sumLat=0.0, sumLon=0.0; int validCount=0;
        for (size_t i=0;i<count;i++) {
            // quick approximate filter using degree difference ~111000 m per deg
            double dlat = (latArr[i]-latMed)*111000.0;
            double dlon = (lonArr[i]-lonMed)*111000.0*cos(latMed*M_PI/180.0);
            double dist = sqrt(dlat*dlat + dlon*dlon);
            if (dist <= OUTLIER_METERS) { sumLat += latArr[i]; sumLon += lonArr[i]; validCount++; }
        }
        if (validCount>0) {
            d.latitude = sumLat / validCount;
            d.longitude = sumLon / validCount;
        } else {
            d.latitude = lat;
            d.longitude = lon;
        }

        d.altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
        d.speedKph = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
        d.courseDeg = gps.course.isValid() ? gps.course.deg() : NAN;
        d.hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9;
        d.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
    } else {
        d.latitude = d.longitude = d.altitude = d.speedKph = d.hdop = 0.0;
        d.courseDeg = NAN;
        d.satellites = 0;
    }
    return d;
}

bool GPSModule::hasFix() {
    return gps.location.isValid();
}
