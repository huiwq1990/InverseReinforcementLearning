import numpy as np
'''
Created on 2013-10-21

@author: hg
'''
map = np.array( [[1, 1, 1, 1, 1], 
                     [1, 0, 0, 1, 1],
                     [1, 0, 1, 1, 1],
                     [1, 1, 1, 0, 0]])
print map.shape
print np.array( map.shape )

box_size = np.array( [2,2] )
print np.prod( np.ceil(  map.shape/box_size ))

print  map.shape[-4:]

b = np.array([[ 0,  1,  2,  3],
       [10, 11, 12, 13],
       [20, 21, 22, 23],
       [30, 31, 32, 33],
       [40, 41, 42, 43]])

print  b[1:4, :] 
print b[-1,-1]