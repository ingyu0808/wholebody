#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import swift
import qpsolvers as qp


def step_robot(r: rtb.ERobot, Tep, qd_prev=None):
    wTe = r.fkine(r.q)
    eTep = np.linalg.inv(wTe) @ Tep

    et = np.sum(np.abs(eTep[:3, -1]))
    Y = 0.01
    Q = np.eye(r.n + 6)
    Q[: r.n, : r.n] *= Y
    Q[:3, :3] *= 1.0 / et
    Q[r.n :, r.n :] = (1.0 / et) * np.eye(6)

    v, _ = rtb.p_servo(wTe, Tep, 1.5)
    v[3:] *= 1.3

    Aeq = np.c_[r.jacobe(r.q), np.eye(6)]
    beq = v.reshape((6,))

    Ain = np.zeros((r.n + 6, r.n + 6))
    bin = np.zeros(r.n + 6)
    ps = 0.1
    pi = 0.9
    Ain[: r.n, : r.n], bin[: r.n] = r.joint_velocity_damper(ps, pi, r.n)

    c = np.concatenate(
        (np.zeros(3), -r.jacobm(start=r.links[5]).reshape((r.n - 3,)), np.zeros(6))
    )

    kε = 0.5
    bTe = r.fkine(r.q, include_base=False).A
    θε = math.atan2(bTe[1, -1], bTe[0, -1])
    ε = kε * θε
    c[0] = -ε

    if qd_prev is not None:
        lambda_smooth = 5.0
        Q[:r.n, :r.n] += lambda_smooth * np.eye(r.n)
        c[:r.n] += -lambda_smooth * qd_prev

    lb = -np.r_[r.qdlim[: r.n], 10 * np.ones(6)]
    ub = np.r_[r.qdlim[: r.n], 10 * np.ones(6)]

    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='quadprog')
    qd = qd[: r.n]

    # ✅ 속도 변화량 제한: ±0.01
    if qd_prev is not None:
        max_delta = 0.01
        delta = qd - qd_prev
        delta = np.clip(delta, -max_delta, max_delta)
        qd = qd_prev + delta

    if et > 0.5:
        qd *= 0.7 / et
    else:
        qd *= 1.4

    return (et < 0.02), qd


def get_target_position(ax_goal):
    x_target = float(input("목표 위치 X 좌표를 입력하세요: "))
    y_target = float(input("목표 위치 Y 좌표를 입력하세요: "))
    z_target = float(input("목표 위치 Z 좌표를 입력하세요: "))

    wTep = sm.SE3(x_target, y_target, z_target) * sm.SE3.Rz(np.pi)
    wTep.A[:3, :3] = np.diag([-1, 1, -1])

    ax_goal.T = wTep  # 시뮬레이터 시각화용 목표 위치 반영
    return wTep


def main():
    rclpy.init()
    node = rclpy.create_node('frankie_qp_controller')
    qd_pub = node.create_publisher(Float64MultiArray, '/joint_velocity_cmd', 10)

    # ✅ Swift 시뮬레이터 초기화
    env = swift.Swift()
    env.launch(realtime=True)

    # 목표 좌표 시각화용 축 추가
    ax_goal = sg.Axes(0.1)
    env.add(ax_goal)

    # 로봇 초기화 및 시뮬레이터에 추가
    frankie = rtb.models.FrankieOmni()
    frankie.q = frankie.qr
    env.add(frankie)

    wTep = get_target_position(ax_goal)
    dt = 0.025  # 시뮬레이터 프레임 시간
    qd_prev = np.zeros(frankie.n)

    while rclpy.ok():
        arrived = False

        while not arrived:
            arrived, qd = step_robot(frankie, wTep.A, qd_prev=qd_prev)
            frankie.qd = qd
            qd_prev = qd.copy()

            msg = Float64MultiArray()
            msg.data = qd[-7:].tolist()
            qd_pub.publish(msg)

            env.step(dt)
            rclpy.spin_once(node, timeout_sec=dt)

        print("🚀 목표에 도착했습니다! 새로운 목표를 입력하세요.")
        wTep = get_target_position(ax_goal)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
