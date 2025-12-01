#!/usr/bin/env python3

import time
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class TicTacToeDemo(Node):
    def __init__(self):
        super().__init__('tic_tac_toe_demo')

        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # 6 arm joints + 2 gripper joints
        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_base_to_mors_1',   # left jaw
            'joint_base_to_mors_2'    # right jaw
        ]

        # Starting joint states
        self.current = [0.0] * 8

        # Home pose (arm straight + gripper open)
        self.home_pose = [
            0.0, -0.6, 1.2, 0.0, 0.0, 0.0,
            0.01, -0.01        # gripper open
        ]

        # Board
        self.board = [" "] * 9

        # Cell locations (with gripper positions included)
        self.cells = {
            '1': [0.7, -0.4, 0.9, 0.0, 0.0, 0.0, 0.01, -0.01],
            '2': [0.0, -0.4, 0.9, 0.0, 0.0, 0.0, 0.01, -0.01],
            '3': [-0.7, -0.4, 0.9, 0.0, 0.0, 0.0, 0.01, -0.01],
            '4': [0.7, 0.0,  0.9, 0.0, 0.0, 0.0, 0.01, -0.01],
            '5': [0.0, 0.0,  0.9, 0.0, 0.0, 0.0, 0.01, -0.01],
            '6': [-0.7,0.0,  0.9, 0.0, 0.0, 0.0, 0.01, -0.01],
            '7': [0.7, 0.4,  0.9, 0.0, 0.0, 0.0, 0.01, -0.01],
            '8': [0.0, 0.4,  0.9, 0.0, 0.0, 0.0, 0.01, -0.01],
            '9': [-0.7,0.4,  0.9, 0.0, 0.0, 0.0, 0.01, -0.01],
        }

        self.print_board()

    # ---------------------------------------------------------
    def print_board(self):
        print("\n==== TIC TAC TOE ====")
        print(f" {self.board[0]} | {self.board[1]} | {self.board[2]}")
        print("---+---+---")
        print(f" {self.board[3]} | {self.board[4]} | {self.board[5]}")
        print("---+---+---")
        print(f" {self.board[6]} | {self.board[7]} | {self.board[8]}")
        print("======================\n")

    # ---------------------------------------------------------
    def move_to(self, target, duration=1.2):
        """Smooth interpolation for all 8 joints."""
        rate = 30
        steps = int(duration * rate)
        start = self.current[:]

        for i in range(steps):
            a = (i + 1) / steps
            pos = [
                start[j] + a * (target[j] - start[j])
                for j in range(8)
            ]

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = pos
            self.pub.publish(msg)

            time.sleep(1.0 / rate)

        self.current = target[:]

    # ---------------------------------------------------------
    def gripper_open(self):
        open_pose = self.current[:]
        open_pose[6] = 0.01
        open_pose[7] = -0.01
        self.move_to(open_pose, duration=0.7)

    def gripper_close(self):
        close_pose = self.current[:]
        close_pose[6] = -0.01
        close_pose[7] = 0.01
        self.move_to(close_pose, duration=0.7)

    # ---------------------------------------------------------
    def check_winner(self):
        wins = [
            [0,1,2],
            [3,4,5],
            [6,7,8],
            [0,3,6],
            [1,4,7],
            [2,5,8],
            [0,4,8],
            [2,4,6]
        ]

        clean = []
        for c in self.board:
            if "X" in c:
                clean.append("X")
            elif "O" in c:
                clean.append("O")
            else:
                clean.append("")

        for a, b, c in wins:
            if clean[a] == clean[b] == clean[c] != "":
                return clean[a]

        return None

    # ---------------------------------------------------------
    # PICK → MOVE TO CELL → PLACE → HOME
    # ---------------------------------------------------------
    def perform_pick_place(self, cell_pose):
        # 1. Go to home and simulate picking
        self.move_to(self.home_pose)
        self.gripper_open()
        time.sleep(0.2)
        self.gripper_close()   # pretend picking token

        # 2. Move to target cell
        self.move_to(cell_pose)

        # 3. Simulate placing token
        self.gripper_open()
        time.sleep(0.3)

        # 4. Return home
        self.move_to(self.home_pose)

    # ---------------------------------------------------------
    def user_move(self, cell):
        idx = int(cell) - 1

        if "X" in self.board[idx] or "O" in self.board[idx]:
            print("❌ Cell filled!")
            return False

        pose = self.cells[cell]
        self.perform_pick_place(pose)

        self.board[idx] = "X"
        return True

    # ---------------------------------------------------------
    def computer_move(self):
        empty = [i for i in range(9) if ("X" not in self.board[i] and "O" not in self.board[i])]
        if not empty:
            return

        cell_id = random.choice(empty) + 1
        pose = self.cells[str(cell_id)]

        self.perform_pick_place(pose)

        self.board[cell_id - 1] = "O"

    # ---------------------------------------------------------
    def game_loop(self):
        while rclpy.ok():

            self.print_board()

            # Check winner before next turn
            winner = self.check_winner()
            if winner:
                print(f"🏆 WINNER: {winner}")
                break

            if all(("X" in c or "O" in c) for c in self.board):
                print("😐 DRAW!")
                break

            user_in = input("Enter cell (1–9), or q: ").strip()
            if user_in == "q":
                break

            if user_in not in self.cells:
                print("Invalid input!")
                continue

            if not self.user_move(user_in):
                continue

            # After user move
            winner = self.check_winner()
            if winner == "X":
                self.print_board()
                print("🏆 YOU WIN!")
                break

            # Computer move
            self.computer_move()

            winner = self.check_winner()
            if winner == "O":
                self.print_board()
                print("🤖 COMPUTER WINS!")
                break


def main(args=None):
    rclpy.init(args=args)
    node = TicTacToeDemo()
    node.game_loop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
