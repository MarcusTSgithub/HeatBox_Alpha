import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd
import serial
from datetime import datetime
from datetime import timedelta


SERIAL_PORT = '/dev/cu.usbmodem14201'
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

Sensors = ["DHT22_1", "DHT22_2", "DHT22_3", "DHT22_4"]

time_data = []
time_data_norm = []
temp1_data = []
temp2_data = []
temp3_data = []
temp4_data = []
power_data = []
power_data_crnt = []
temp_setpoint_data = []
setPoint = []
chosen_temp_sensor = [9]  # [1 = DHT22_1, 2 = DHT22_2, ]
ambTemp = []
stabTempDiff = []
pid_values = []
save_data = False


def read_and_process_data():
    line = ser.readline().decode('utf-8').strip()
    sensorVals = line.split('\t')
    if sensorVals[0] == 'pid values:':
        pid_values.append(float(sensorVals[1]))
        pid_values.append(float(sensorVals[2]))
        pid_values.append(float(sensorVals[3]))
        print(f'PID values: Kp = {pid_values[0]}, Ki: {pid_values[1]}, Kd: {pid_values[2]}')
        return False
    if sensorVals[0] == 'DHT22 Heat Dim Test: Temperature setpoint at':
        setPoint.append(float(sensorVals[1]))
        print(f'Set Point Temperature = {setPoint[0]}')
        return False
    if sensorVals[0] == 'Values for python script. Ambient Temperature:':
        ambTemp.append(float(sensorVals[1]))
        stabTempDiff.append(float(sensorVals[3]))
        print(f'Ambient Temperature = {ambTemp[0]}, Stabilize Temperature Difference = {stabTempDiff[0]}')
        return False
    if len(sensorVals) == 0:
        return False
    if sensorVals[0] == 'Dim Power:':
        time_data.append((float(sensorVals[11]))/1000)  # time value/refresh rate tick
        hhmmss = timedelta(seconds=int((float(sensorVals[11]))/1000))
        temp1_data.append(float(sensorVals[3]))
        temp2_data.append(float(sensorVals[5]))
        temp3_data.append(float(sensorVals[7]))
        temp4_data.append(float(sensorVals[9]))
        power_data.append(float(sensorVals[1]))
        power_data_crnt.insert(0, float(sensorVals[1]))
        temp_setpoint_data.append(float(sensorVals[12]))
        chosen_temp_sensor.append(int(sensorVals[14]))
        print(f'Time: {hhmmss}, Temp 1: {sensorVals[3]}, Temp 2: {sensorVals[5]}, Temp 3: {sensorVals[7]}, Temp 4: {sensorVals[9]}, Power: {sensorVals[1]}')
        return True
    else:
        return False


def update_plot(frame):
    read_and_process_data()
    ax1.cla()
    ax2.cla()
    ax1.plot(time_data, temp_setpoint_data, label='Set Point', color='black')
    ax1.plot(time_data, temp1_data, label='Temp 1', color='red')
    ax1.plot(time_data, temp2_data, label='Temp 2', color='orange')
    ax1.plot(time_data, temp3_data, label='Temp 3', color='blue')
    ax1.plot(time_data, temp4_data, label='Temp 4', color='green')
    ax2.plot(time_data, power_data, label='Dim Power', color='black')

    ax1.set_ylabel('Sensor values')
    ax2.set_ylabel('Dim Power (0-255)')
    ax2.set_xlabel('Time (s)')
    leg1 = ax1.legend(fontsize=8, loc="center left", bbox_to_anchor=(0.79, 0.715), bbox_transform=fig.transFigure)
    leg1_1_text = 'Set Point: ' + str(temp_setpoint_data[len(temp_setpoint_data)-1])
    if chosen_temp_sensor[len(chosen_temp_sensor)-1] == 1:
        leg1_2_text = 'Temp 1: ' + str(temp1_data[len(temp1_data)-1]) + " (C)"
        leg1_3_text = 'Temp 2: ' + str(temp2_data[len(temp2_data)-1])
        leg1_4_text = 'Temp 3: ' + str(temp3_data[len(temp3_data)-1])
        leg1_5_text = 'Temp 4: ' + str(temp4_data[len(temp4_data)-1])
    elif chosen_temp_sensor[len(chosen_temp_sensor)-1] == 2:
        leg1_2_text = 'Temp 1: ' + str(temp1_data[len(temp1_data)-1])
        leg1_3_text = 'Temp 2: ' + str(temp2_data[len(temp2_data)-1]) + " (C)"
        leg1_4_text = 'Temp 3: ' + str(temp3_data[len(temp3_data)-1])
        leg1_5_text = 'Temp 4: ' + str(temp4_data[len(temp4_data)-1])
    elif chosen_temp_sensor[len(chosen_temp_sensor)-1] == 3:
        leg1_2_text = 'Temp 1: ' + str(temp1_data[len(temp1_data)-1])
        leg1_3_text = 'Temp 2: ' + str(temp2_data[len(temp2_data)-1])
        leg1_4_text = 'Temp 3: ' + str(temp3_data[len(temp3_data)-1]) + " (C)"
        leg1_5_text = 'Temp 4: ' + str(temp4_data[len(temp4_data)-1])
    elif chosen_temp_sensor[len(chosen_temp_sensor)-1] == 4:
        leg1_2_text = 'Temp 1: ' + str(temp1_data[len(temp1_data)-1])
        leg1_3_text = 'Temp 2: ' + str(temp2_data[len(temp2_data)-1])
        leg1_4_text = 'Temp 3: ' + str(temp3_data[len(temp3_data)-1])
        leg1_5_text = 'Temp 4: ' + str(temp4_data[len(temp4_data)-1]) + " (C)"
    leg1.get_texts()[0].set_text(leg1_1_text)
    leg1.get_texts()[1].set_text(leg1_2_text)
    leg1.get_texts()[2].set_text(leg1_3_text)
    leg1.get_texts()[3].set_text(leg1_4_text)
    leg1.get_texts()[4].set_text(leg1_5_text)
    leg2 = ax2.legend(fontsize=8, loc="center left", bbox_to_anchor=(0.79, 0.285), bbox_transform=fig.transFigure)
    leg2_text = 'Power: ' + str(power_data[len(power_data)-1])
    leg2.get_texts()[0].set_text(leg2_text)


def on_close_excel(event):
    print('Plot window closed.')
    if save_data:
        now = datetime.now()
        d = now.strftime("%d%m")
        t = now.strftime("%H%M")
        file_name = "Heat_Test_" + d + "_" + t + ".xlsx"
        df = pd.DataFrame({'Time (s)': time_data, 'Temperature 1 (dht22)': temp1_data, 'Temperature 2 (dht22)': temp2_data, 'Temperature 3 (dht22)': temp3_data, 'Temperature 4 (dht22)': temp4_data, 'Lamp Power (0-255)': power_data})
        df_pid = pd.DataFrame({'Kp': [pid_values[0]], 'Ki': [pid_values[1]], 'Kd': [pid_values[2]]})
        df_other = pd.DataFrame({'Set Point': [setPoint[0]], 'Ambient Temperature': [ambTemp[0]], 'Stab Temp Diff': [stabTempDiff[0]]})
        dataSize = df.size
        if dataSize > 600:  # If test duration is longer than (600/4)*refreshTime = 300 sec = 5 min
            with pd.ExcelWriter(file_name) as excel_writer:
                df.to_excel(excel_writer, sheet_name='Sheet1', float_format="%.2f", index=False)
                df_pid.to_excel(excel_writer, sheet_name='Sheet1', float_format="%.2f", index=False, startcol=6)
                df_other.to_excel(excel_writer, sheet_name='Sheet1', float_format="%.2f", index=False, startcol=9)
            print("Data saved to: " + file_name)
        else:
            print("Test duration shorter than 5 minutes, did not save to excel file.")
    else:
        print("Did not save data.")


while not read_and_process_data():
    print('initiation phase')
print('Initiation phase done!')

prompt = input("Do you want to save the data (y/n)?")
if prompt.lower() == "y" or prompt.lower() == "yes":
    print("Data will be saved.")
    save_data = True
elif prompt.lower() == "n" or prompt.lower() == "no":
    print("Data will not be saved.")
    save_data = False
else:
    raise Exception("input was not y or n, please rerun the program and give a valid input.")


fig, (ax1, ax2) = plt.subplots(2, 1)
fig.subplots_adjust(right=0.79)
ax1.set_ylabel('Sensor values')
ax2.set_ylabel('Dim Power (0-255)')
ax2.set_xlabel('Time (s)')

fig.canvas.mpl_connect('close_event', on_close_excel)
ani = FuncAnimation(fig, update_plot, interval=50)
plt.show()

