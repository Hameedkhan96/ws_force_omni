import csv

# Function to convert .txt file to .csv with headers
def txt_to_csv(txt_file, csv_file):
    # Open the CSV file in 'w' mode to clear the file content (empty it)
    open(csv_file, 'w').close()  # This will create/empty the file before writing new data
    
    # Now open the .txt file for reading and the .csv file for writing
    with open(txt_file, 'r') as infile:
        with open(csv_file, 'w', newline='') as outfile:
            writer = csv.writer(outfile)
            
            # Write the headers for the CSV file
            writer.writerow(['xd', 'yd', 'zd', 'x', 'y', 'z','phi', 'the', 'psi', 'fx'])  # Column labels
            # Read each line from the .txt file and write the data
            for line in infile:
                # Split each line by comma and write to the CSV
                writer.writerow(line.strip().split(','))

# File paths
txt_file = 'Results.txt'  # Input .txt file
csv_file = 'Results_converted.csv'  # Output CSV file

# Call the function to perform conversion
txt_to_csv(txt_file, csv_file)

print(f"Conversion complete. The data has been written to {csv_file}")