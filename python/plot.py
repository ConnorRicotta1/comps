import matplotlib.pyplot as plt

# Sample data (replace with your actual values)
demand_ratio = [10, 20, 30, 40, 50]
unbalanced = [5.22, 7.34, 22.62, 27.82, 42.06]
balanced = [8.1, 11.39, 11.08, 11.65, 11.85]

# Create the p
# Create the plot
plt.figure(figsize=(8, 5))
plt.plot(demand_ratio, unbalanced, marker='o', linestyle='-', color='blue', label='Original')
plt.plot(demand_ratio, balanced, marker='s', linestyle='--', color='green', label='Improved')

# Add labels and title
plt.xlabel('Demand Ratio (%)', fontsize=14)
plt.ylabel('Delay per Car (s)', fontsize=14)
plt.title('Original Vs. Improved Delay Function')
plt.legend()
plt.grid(True)

# Show the plot
plt.tight_layout()
plt.show()
