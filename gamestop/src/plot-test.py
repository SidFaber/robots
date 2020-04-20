import numpy as np
import matplotlib.pyplot as plt
# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import


points = [
    (0.20146225295673453, -0.1096445739227643, 0.0711905630892654),
    (0.20083105745900365, -0.06206024353830996, 0.07304811662710058), 
    (0.20116300565782286, -0.012204132798658264, 0.07121232890491759), 
    (0.20129160079959105, 0.03855885271510831, 0.07122808075965263), 
    (0.20137776039420066, 0.08850486523355054, 0.08066877137725027), 
    (0.23141795475586974, -0.11002418141564359, 0.06952962346534379), 
    (0.2313277657667629, -0.061421206257017455, 0.07004997680580922), 
    (0.23128049934661415, -0.012121719357722447, 0.07120942411659081), 
    (0.2314798420430706, 0.038430272375649985, 0.07060907880199503), 
    (0.23196185536027114, 0.08914086504546631, 0.07166829806945432)]


# setup the figure and axes
fig = plt.figure(figsize=(3, 3))
ax = fig.add_subplot(121, projection='3d')

# fake data
_x = []
_y = []
_z = []
bottom = None
for point in points:
    _x.append (point[0] * 100)
    _y.append (point[1] * 100)
    _z.append (point[2] * 100)
    if bottom is None:
        bottom = point[2] * 100
    else:
        bottom = min (bottom, point[2] * 100)
for i in range (0, len(_z)):
    _z[i] -= bottom

ax.bar3d(_x, _y, bottom, 3, 5, _z, shade=True)

plt.show()
