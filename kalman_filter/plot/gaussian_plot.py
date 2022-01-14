import numpy as np
import scipy.stats
import matplotlib.pyplot as plt

#### modify these values ###
mu = 10
sigma = 15

# generate histogram values
x = mu + sigma*np.random.randn(10000)

# the more bins, the more histogram bars
bins = 50
# histogram of the data
n, bins, patches = plt.hist(x, bins, density=1, facecolor='green', alpha=0.75, edgecolor='black', linewidth=1.2)

# generate a normal continuous random distribution from same values
y = scipy.stats.norm.pdf( bins, mu, sigma)
l = plt.plot(bins, y, 'r-', linewidth=2, label='Inline label')
# Compute the max value
ymax = y.max()

# plot
plt.xlabel('x variable')
plt.ylabel('Probability')
plt.title(r'$\mathrm{Histogram\:}\ \mu='+str(mu)+',\ \sigma='+str(sigma)+'$')
viewport = [mu-4*sigma, mu+4*sigma, 0, ymax*1.1]
plt.axis(viewport)
plt.grid(True)
plt.show()