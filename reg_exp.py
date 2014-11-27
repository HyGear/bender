# -*- coding: utf-8 -*-
"""
Created on Wed Nov 26 16:34:43 2014

@author: YA2148
"""

import re

f = open('proe.pts.1','r')

for line in f:
    if re.search('(^ +\-?[0-9].[0.9]+)',line):
        print line


f.close()