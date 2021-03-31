import numpy as np

x = []
f1 = 22100. / 256

f2 = np.array( [ -9., -8., -7., -6., -5., -4., -3., -2., -1., 0., 1., 2.  ] )
f2 = np.power( 2, f2/12 ) * 440

step = f2/f1
step *= 2**24
notes = [ "C ", "C#", "D ", "D#", "E ", "F ", "F#", "G ", "G#", "A ", "A#", "B " ]

print("f1: {}, f2: {}, step: {}".format(f1, f2, step))

step = step.tolist()
for i, x in enumerate(step):
    print( "/*{}*/ {},".format(notes[i], round(x)) )