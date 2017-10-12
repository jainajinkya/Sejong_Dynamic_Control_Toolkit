# Sejong Dynamic Control Toolkit

- robotics dynamic control libraries
- various whole-body controllers


## Ubuntu Installation
Upgrade cmake >= 3.0

````
# Download the latest cmake at https://cmake.org/download/
chmod +x cmake-3.9.4-Linux-x86_64.sh  # Make the script executable
sudo ./cmake-3.9.4-Linux-x86_64.sh  # Install at some directory
sudo mv cmake-3.9.4-Linux-x86_64 /opt/ # Move extracted files to /opt/
sudo ln -s /opt/cmake-3.9.4-Linux-x86_64/bin/* /usr/local/bin/ # Create a symbolic link
````

Install numpy and Matplotlib
````
python -mpip install -U pip
pip install numpy
python -mpip install -U matplotlib
````