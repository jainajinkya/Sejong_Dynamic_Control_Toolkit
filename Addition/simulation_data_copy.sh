#! /bin/bash
PATH_PACKAGE="/Users/donghyunkim/locomotion_package"

# Move experiment data to the other folder to plot data
cp $PATH_PACKAGE/experiment_data/* $PATH_PACKAGE/experiment_data_check/

# Remove every data for the next experiment
rm $PATH_PACKAGE/experiment_data/*

exit 0
