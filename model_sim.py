import numpy as np
import matplotlib.pyplot as plt

t = np.linspace(0, 5, 500) # Simulation time
u = 1.0 # Step input (100% throttle @ t=0)

# Model Parameters
tau_normal = 1.0
K_normal = 1.0

tau_sport = 0.1
K_sport = 1.5

# Step response solutions for v: tau * v' + v = K * u
v_normal = K_normal * u * (1 - np.exp(-t / tau_normal))
v_sport = K_sport * u * (1 - np.exp(-t / tau_sport))

# Analytical derivatives for a: v' = (K*u / tau) * exp(-t / tau)
a_normal = (K_normal * u / tau_normal) * np.exp(-t / tau_normal)
a_sport = (K_sport * u / tau_sport) * np.exp(-t / tau_sport)

# 3. Electrical Current Draw Model (Amperes)
# Base const loads: ESP32 BLE + DRV8833 idle + 2x Motor No-Load currents
i_quiescent = 0.13 + 0.02 + (2 * 0.07) # ESTIMATED based on idle measurements

# Proportional torque/acceleration scaling constant to simulate transient spikes
K_torque = 0.5  

i_normal = i_quiescent + (K_torque * a_normal)
i_sport = i_quiescent + (K_torque * a_sport)

# Plotting Combined Model
fig, ax1 = plt.subplots(figsize=(9, 5))

# Primary Y-Axis
ax1.plot(t, v_normal, label='Normal Velocity (τ=1.0, K=1.0)', color='blue', linewidth=2.5)
ax1.plot(t, v_sport, label='Sport Velocity (τ=0.1, K=1.5)', color='red', linewidth=2.5)
ax1.set_xlabel("Time (seconds)", fontsize=10, fontweight='bold', color='#e1e1e6')
ax1.set_ylabel("Velocity (Normalized)", fontsize=10, fontweight='bold', color='#e1e1e6')
ax1.tick_params(axis='y')

# Secondary Y-axis
ax2 = ax1.twinx()
ax2.plot(t, i_normal, label='Normal Current (No Resistance)', color='blue', linestyle='--', linewidth=1.8, alpha=0.8)
ax2.plot(t, i_sport, label='Sport Current (No Resistance)', color='red', linestyle='--', linewidth=1.8, alpha=0.8)
ax2.set_ylabel("Total System Current (No Outside Forces)", fontsize=10, fontweight='bold', color='#e1e1e6')
ax2.tick_params(axis='y')

# Simulated OC trip threshold
ax2.axhline(y=1, color='darkorange', linestyle=':', linewidth=1.5, label='Fault Threshold (1A)')

plt.title("VCU State Dynamics: Step Response & Transient Current Spikes", fontsize=12, fontweight='bold')
ax1.grid(True, linestyle=':', alpha=0.5)

# Combine legends from both axes
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=9)

plt.tight_layout()
plt.show()