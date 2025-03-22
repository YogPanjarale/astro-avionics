# data is currentlly time (sec ) , altitude (m) , vertical velocity (m/s), vertical Acceleration (m/s^2) , roll rate 

#  bmp is a sensor which gives the altitude of the plane in meters
# bmpVel is velocity calculate by time difference of altitude
# imu_vel is the velocity cacluated by vertical acceleration
# imualt is the altitude calculated by the integration of imu_vel

# we make new csv with the following columns
# time(ms) , altitude(m) , accel_y(m/s^2) , bmpVel(m/s) , imu_vel(m/s) , imualt(m) , 

import csv

def create_data():
    with open('flight_data_filtered.csv', 'r') as file:
        reader = csv.reader(file)
        with open('flight_data_cleaned.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerow(['time(ms)', 'altitude(m)', 'accel_y(m/s^2)', 'bmpVel(m/s)', 'imu_vel(m/s)', 'imualt(m)'])
            previous_altitude = None
            previous_time = None
            imualt = 0
            imu_vel = 0
            for row in reader:
                current_time = float(row[0]) *1000  # convert sec to ms
                altitude = float(row[1]) 
                accel_y = float(row[3])
                
                if previous_time is not None:
                    time_diff = (current_time - previous_time) / 1000  # convert ms to s
                    bmpVel = (altitude - previous_altitude) / time_diff
                    imu_vel += accel_y * time_diff
                    imualt += imu_vel * time_diff 
                else:
                    bmpVel = 0
                # make sure we are using three decimal places
                current_time = round(current_time, 3)
                altitude = round(altitude, 2)
                accel_y = round(accel_y, 3)
                bmpVel = round(bmpVel, 3)
                imu_vel = round(imu_vel, 3)
                imualt = round(imualt, 2)

                writer.writerow([current_time, altitude, accel_y, bmpVel, imu_vel, imualt])
                
                previous_altitude = altitude
                previous_time = current_time
    print('Data cleaned and written to flight_data_cleaned.csv')

if __name__ == '__main__':
    create_data()