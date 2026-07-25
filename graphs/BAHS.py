import matplotlib.pyplot as plt

# Data from the BAHS 2025 Press Release
years = ['2014-15', '2015-16', '2016-17', '2017-18', '2018-19', 
         '2019-20', '2020-21', '2021-22', '2022-23', '2023-24', '2024-25']
production = [146.3, 155.5, 165.4, 176.3, 187.7, 198.4, 210.0, 222.1, 230.6, 239.3, 247.9]

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(years, production, marker='o', linestyle='-', color='#1f77b4', linewidth=2)

# Professional Formatting
plt.xlabel('Financial Year', fontsize=12, fontweight='bold')
plt.ylabel('Milk Production (Million Tonnes)', fontsize=12, fontweight='bold')
plt.xticks(rotation=45)
plt.grid(True, linestyle='--', alpha=0.7)
plt.tight_layout()

# Save as PDF for high-quality LaTeX integration
plt.savefig('milk_production_graph.pdf')
plt.show()