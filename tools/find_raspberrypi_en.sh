#!/bin/bash

# Function to retrieve available interfaces from ifconfig output
get_available_interfaces() {
    # Extract interface names using regular expression
    ifconfig -a | awk '/^[a-zA-Z0-9]/ {gsub(":$", "", $1); print $1}'
}

# Get available interfaces
available_interfaces=$(get_available_interfaces)

# Display available interfaces with numbered list
echo "Available interfaces:"
echo "$available_interfaces" | cat -n

# Prompt user to select an interface
read -p "Enter the number of the interface you want to use: " interface_number

# Check if the selected number is valid
if ! [[ "$interface_number" =~ ^[0-9]+$ ]] || [ "$interface_number" -lt 1 ] || [ "$interface_number" -gt $(echo "$available_interfaces" | wc -l) ]; then
    echo "Error: Invalid interface number."
    exit 1
fi

# Get the selected interface
selected_interface=$(echo "$available_interfaces" | sed -n "${interface_number}p")

# Function to extract Raspberry Pi's IP address from arp-scan output
get_raspberry_pi_ip() {
    while read -r line; do
        # Extract IPv4 address from the line
        ip=$(echo "$line" | awk '{print $1}')
        # Extract manufacturer information from the line
        manufacturer=$(echo "$line" | awk '{$1=""; $2=""; print $0}')
        # If the manufacturer information contains "Raspberry Pi", output its IP address and exit
        if [[ "$manufacturer" == *"Raspberry Pi"* ]]; then
            echo "$ip"
            return
        fi
    done
}

# Get Raspberry Pi's IP address from arp-scan output
raspberry_pi_ip=$(sudo arp-scan --interface "$selected_interface" --localnet | get_raspberry_pi_ip)

# Set the obtained IP address to an environment variable
export RASPBERRY_PI_IP="$raspberry_pi_ip"

# Display the Raspberry Pi's IP address for confirmation
echo "The IP address of Raspberry Pi is $RASPBERRY_PI_IP."

