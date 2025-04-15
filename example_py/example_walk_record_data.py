#!/usr/bin/python

import sys
import time
import math
import csv

sys.path.append('../lib/python/amd64')
import robot_interface as sdk


if __name__ == '__main__':

    HIGHLEVEL = 0xee
    LOWLEVEL  = 0xff

    udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)

    cmd = sdk.HighCmd()
    state = sdk.HighState()
    udp.InitCmdData(cmd)

    motiontime = 0
    buffer = []
    BUFFER_SIZE = 100

    with open("go1_full_data_log.csv", mode="w", newline="") as file:
        writer = csv.writer(file)

        # HEADER
        header = [
            "timestamp", "motiontime",
            "cmd_mode", "cmd_gaitType", "cmd_velocity_x", "cmd_velocity_y",
            "cmd_yawSpeed", "cmd_bodyHeight", "cmd_euler_x", "cmd_euler_y", "cmd_euler_z",
            "imu_r", "imu_p", "imu_y",
            "imu_gyro_x", "imu_gyro_y", "imu_gyro_z",
            "imu_acc_x", "imu_acc_y", "imu_acc_z",
        ]

        # # Foot sensor info
        for i in range(4):
            header += [f"footForce_{i}", f"footForceEst_{i}"]

        # Motor state headers
        for i in range(12):
            header += [f"m{i}_q", f"m{i}_dq", f"m{i}_tau"]

        writer.writerow(header)

        while True:
            time.sleep(0.002)
            motiontime = motiontime + 1
            timestamp = time.time()

            udp.Recv()
            udp.GetRecv(state)

            # print(state.imu.rpy[0])
            # print(motiontime, state.motorState[0].q, state.motorState[1].q, state.motorState[2].q)
            # print(state.imu.rpy[0])

            cmd.mode = 0      # 0:idle, default stand      1:forced stand     2:walk continuously
            cmd.gaitType = 0
            cmd.speedLevel = 0
            cmd.footRaiseHeight = 0
            cmd.bodyHeight = 0
            cmd.euler = [0, 0, 0]
            cmd.velocity = [0, 0]
            cmd.yawSpeed = 0.0
            cmd.reserve = 0

            # cmd.mode = 2
            # cmd.gaitType = 1
            # # cmd.position = [1, 0]
            # # cmd.position[0] = 2
            # cmd.velocity = [-0.2, 0] # -1  ~ +1
            # cmd.yawSpeed = 0
            # cmd.bodyHeight = 0.1

            if(motiontime > 0 and motiontime < 1000):
                cmd.mode = 1
                cmd.euler = [-0.3, 0, 0]
            if(motiontime > 1000 and motiontime < 2000):
                cmd.mode = 1
                cmd.euler = [0.3, 0, 0]
            if(motiontime > 2000 and motiontime < 3000):
                cmd.mode = 1
                cmd.euler = [0, -0.2, 0]
            if(motiontime > 3000 and motiontime < 4000):
                cmd.mode = 1
                cmd.euler = [0, 0.2, 0]
            if(motiontime > 4000 and motiontime < 5000):
                cmd.mode = 1
                cmd.euler = [0, 0, -0.2]
            if(motiontime > 5000 and motiontime < 6000):
                cmd.mode = 1
                cmd.euler = [0.2, 0, 0]
            if(motiontime > 6000 and motiontime < 7000):
                cmd.mode = 1
                cmd.bodyHeight = -0.2
            if(motiontime > 7000 and motiontime < 8000):
                cmd.mode = 1
                cmd.bodyHeight = 0.1
            if(motiontime > 8000 and motiontime < 9000):
                cmd.mode = 1
                cmd.bodyHeight = 0.0
            if(motiontime > 9000 and motiontime < 11000):
                cmd.mode = 5
            if(motiontime > 11000 and motiontime < 13000):
                cmd.mode = 6
            if(motiontime > 13000 and motiontime < 14000):
                cmd.mode = 0
            if(motiontime > 14000 and motiontime < 18000):
                cmd.mode = 2
                cmd.gaitType = 2
                cmd.velocity = [0.4, 0] # -1  ~ +1
                cmd.yawSpeed = 2
                cmd.footRaiseHeight = 0.1
                # printf("walk\n")
            if(motiontime > 18000 and motiontime < 20000):
                cmd.mode = 0
                cmd.velocity = [0, 0]
            if(motiontime > 20000 and motiontime < 24000):
                cmd.mode = 2
                cmd.gaitType = 1
                cmd.velocity = [0.2, 0] # -1  ~ +1
                cmd.bodyHeight = 0.1
                # printf("walk\n")
            # Data row
            row = [
                timestamp, motiontime,
                cmd.mode, cmd.gaitType, cmd.velocity[0], cmd.velocity[1],
                cmd.yawSpeed, cmd.bodyHeight, cmd.euler[0], cmd.euler[1], cmd.euler[2],
                state.imu.rpy[0], state.imu.rpy[1], state.imu.rpy[2],
                state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2],
                state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2]
            ]
            # # Foot sensors
            for i in range(4):
                row += [state.footForce[i], state.footForceEst[i]]
            # Motor states
            for i in range(12):
                motor = state.motorState[i]
                row += [motor.q, motor.dq, motor.tauEst]
            buffer.append(row)
            # Write to CSV
            if len(buffer) >= BUFFER_SIZE:
                writer.writerows(buffer)
                buffer.clear()

            udp.SetSend(cmd)
            udp.Send()
