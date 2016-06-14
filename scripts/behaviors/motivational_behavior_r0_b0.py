#!/usr/bin/env python
# coding=utf-8

__author__ = "Minglong Li"

import sys
sys.path.append("..")
from basic_support.motivational_behavior import MotivationalBehavior

class MotivationalBehaviorR0B0(MotivationalBehavior):
    def __init__(self):
        MotivationalBehavior.__init__(self, node_name = 'motivational_behavior_r0_b0')

    def run(self):
        MotivationalBehavior.start_motive(self)

if __name__ == '__main__':
    motivational_behavior_r0_b0 = MotivationalBehaviorR0B0()
    motivational_behavior_r0_b0.start()

