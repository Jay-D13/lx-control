#!/usr/bin/env python3

from typing import Tuple
import numpy as np

class PIDController():
    def __init__(self):

        # We will initialize some variables that might be useful
        self.prev_e_heading = 0.0
        self.prev_e_offset = 0.0
        self.prev_int_heading = 0.0
        self.prev_int_offset = 0.0

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0


    def HeadingControl(self,
                       v_ref: float,
                       theta_ref: float,
                       theta_curr: float,
                       delta_t: float
    ) -> Tuple[float, float]:
        """
        PID performing heading control.
        Args:
            v_ref:      reference velocity.
            theta_ref:  reference heading pose.
            theta_curr: the current estimated heading.
            delta_t:    time interval since last call.
        Returns:
            v:          linear velocity of the Duckiebot
            omega:      angular velocity of the Duckiebot
        """

        # TODO: implement a PID controller to track the reference heading
        # feel free to make use of the global variables:
        # self.kp, self.ki, and self. kd, which are
        # set either by the notebook or from noVNC
        # as well as self_prev_int_heading to track the integral term
        # self.prev_e_heading the previous error. But note that you
        # should be the one to update them also.
        
        # e = theta_ref - theta_curr
        
        # if e > np.pi:
        #     e -= 2 * np.pi
        # elif e < -np.pi:
        #     e += 2 * np.pi
        
        e = np.arctan2(np.sin(theta_ref - theta_curr), np.cos(theta_ref - theta_curr))
        # print()
        # print("Theta ref:", theta_ref)
        # print("Theta curr:", theta_curr)
        # print("delta time:", delta_t)
        # print("Error:", e)
        # print(f"kp: {self.kp}, ki: {self.ki}, kd: {self.kd}")
        
        de = (e - self.prev_e_heading) / delta_t
        
        self.prev_int_heading += e * delta_t
                
        omega = self.kp * e + self.ki * self.prev_int_heading + self.kd * de

        self.prev_e_heading = e 
        v = v_ref
        return v, omega

    def OffsetControl(self,
                      v_ref: float,
                      y_ref: float,
                      y_curr: float,
                      delta_t: float
                      ) -> Tuple[float, float]:
        """
        PID performing lateral offset control.
        Args:
            v_ref:      linear Duckiebot speed.
            y_ref:      reference heading pose.
            y_curr:     the current estimated "y" coordinate (offset)
            delta_t:    time interval since last call.
        Returns:
            v:          linear velocity of the Duckiebot
            omega:      angular velocity of the Duckiebot
        """

        # TODO: implement a PID controller to track the reference lateral offset
        # feel free to make use of the global variables:
        # self.kp, self.ki, and self. kd, which are
        # set either by the notebook or from noVNC
        # as well as self_prev_int_offset to track the integral term
        # self.prev_e_offset the previous error. But note that you
        # should be the one to update them also.
        
        e = y_ref - y_curr
        
        if delta_t <= 0:
            de = 0.0
        else:
            de = (e - self.prev_e_offset) / delta_t
        
        self.prev_int_offset += e * delta_t
                
        omega = self.kp * e + self.ki * self.prev_int_offset + self.kd * de

        self.prev_e_offset = e 
        v = v_ref
        return v, omega

    def SetGains(self, kp: float, ki: float, kd: float) -> None:
        # Set the PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd