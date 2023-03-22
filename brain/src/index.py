#!/usr/bin/python3

import numpy
import superclasses
from conditions import *
from actions import * 
from utilities import Root, And, Or

if __name__=="__main__":
    root = Root(
        And([
            ifAnchorDetected(),
            Or([
                ifFoundMatchingToyAndBox(),
                doExplore()])
        ])
    )
    while True:
        root.tick()
