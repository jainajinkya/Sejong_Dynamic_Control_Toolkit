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

## Compiling The Code
````
cd [cloned directory]
mkdir build
cd build
cmake ..
make -j
````

## Contributing to the code
1) Fork the github
2) Clone from your fork
3) Add upstream to your local git remote

````git remote -v
git remote add upstream "other"
````

4) To merge your fork with upstream:
````
git fetch upstream
git merge upstream/master
````

5) When committing, push to your fork eg:
````
git add --all
git commit -m "message"
git push origin master
````

6) Test the compilation and resolve the conflicts
````
cd build
cmake ..
make -j
````

7) In github, do a pull request


