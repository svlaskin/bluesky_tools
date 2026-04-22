import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import lognorm

# parameters
t0 = 20.0  # seconds
sigmas = [0.05, 0.1, 0.2, 0.4, 0.6]

target_mean = 30  # desired mean loiter time (including shift)

# quantile grid (stable tails)
q = np.linspace(0.001, 0.999, 1000)

plt.figure(figsize=(8, 5))

for sigma in sigmas:
    # recompute mu so mean stays fixed
    mu = np.log(target_mean - t0) - 0.5 * sigma**2

    t = t0 + lognorm.ppf(q, s=sigma, scale=np.exp(mu))
    pdf = lognorm.pdf(t - t0, s=sigma, scale=np.exp(mu))

    plt.plot(t, pdf, label=fr"$\sigma = {sigma}$")

plt.xlabel("Loiter time (s)")
plt.ylabel("Density")
plt.title("Shifted Lognormal Loiter Time Distribution")
plt.legend()
plt.tight_layout()
plt.show()
